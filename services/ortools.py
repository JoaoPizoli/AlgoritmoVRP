# roteirizador_excedente.py
# pip install ortools

from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Set, Optional, Any
import math

from ortools.constraint_solver import pywrapcp, routing_enums_pb2


# =========================
# 1) MODELOS DE DADOS
# =========================

@dataclass(frozen=True)
class Truck:
    """
    Um caminhão do dia.
    - cities: lista de cidades que esse caminhão pode atender (rota do dia).
      Se uma rota tiver 2 caminhões, você cria 2 Truck com a MESMA lista.
    """
    id: str
    cities: Tuple[str, ...]
    capacity_kg: int = 19_000


@dataclass
class Order:
    """
    Um pedido/entrega.
    - city: cidade do pedido (tem que existir em alguma lista de cidades daquele dia)
    - weight_kg: peso do pedido
    - x, y: coordenadas só pra exemplo (no real você usa matriz OSRM/tempo/distância)
    - alt_truck_ids: caminhões alternativos viáveis (apenas para EXCEDENTES na etapa 2)
    - priority_penalty: se quiser, aumente isso para impedir que pedidos importantes virem excedente
    """
    id: str
    city: str
    weight_kg: int
    x: float
    y: float
    alt_truck_ids: List[str] = field(default_factory=list)
    priority_penalty: int = 0


@dataclass
class SolveResult:
    objective: int
    dropped_orders: List[str]                 # excedentes (não alocados)
    served_by_truck: Dict[str, str]           # order_id -> truck_id
    routes: Dict[str, List[str]]              # truck_id -> [order_id...]
    loads_by_truck: Dict[str, int]            # truck_id -> carga total alocada (kg)


@dataclass
class PedidoRealocado:
    """Informações de um pedido que foi realocado de uma rota excedente para outro caminhão."""
    id: str
    cidade: str
    caminhao_original: str          # Caminhão da rota base (que lotou)
    caminhao_realocado: str         # Caminhão que recebeu o pedido
    distancia_adicional_km: float   # Km adicionais que a realocação causou
    peso_kg: int
    motivo: str = "excedente_capacidade"


# =========================
# 2) MATRIZ DE CUSTO (EXEMPLO)
# =========================

def euclidean_cost(a: Tuple[float, float], b: Tuple[float, float]) -> int:
    # custo inteiro (OR-Tools usa int). Multiplico por 100 pra mais resolução.
    return int(round(math.hypot(a[0] - b[0], a[1] - b[1]) * 100))

def build_cost_matrix(points: List[Tuple[float, float]]) -> List[List[int]]:
    n = len(points)
    m = [[0] * n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            if i != j:
                m[i][j] = euclidean_cost(points[i], points[j])
    return m


# =========================
# 3) REGRAS DO SEU DIA (LISTAS DE CIDADES)
# =========================

def route_key(truck: Truck) -> Tuple[str, ...]:
    # Uma rota é definida pelo conjunto de cidades (ordem não importa aqui)
    return tuple(sorted(truck.cities))

def group_trucks_by_route(trucks: List[Truck]) -> Dict[Tuple[str, ...], List[str]]:
    """
    Agrupa caminhões que têm a MESMA lista de cidades (rota com 2 caminhões, por exemplo).
    Retorna: route_key -> [truck_id, truck_id, ...] na ordem que você cadastrou.
    """
    groups: Dict[Tuple[str, ...], List[str]] = {}
    for t in trucks:
        groups.setdefault(route_key(t), []).append(t.id)
    return groups

def validate_disjoint_routes(trucks: List[Truck]) -> None:
    """
    Regra do usuário: entre listas diferentes, NÃO pode ter cidade repetida.
    (Se 2 caminhões têm a MESMA lista, isso é o MESMO grupo, permitido.)
    """
    groups = group_trucks_by_route(trucks)
    seen: Set[str] = set()
    for rk in groups.keys():
        for c in rk:
            if c in seen:
                raise ValueError(
                    f"Cidade repetida entre listas diferentes: '{c}'. "
                    f"Isso viola sua regra (cidades não repetem entre rotas do dia)."
                )
            seen.add(c)

def city_to_base_trucks(trucks: List[Truck]) -> Dict[str, List[str]]:
    """
    Para cada cidade, quais caminhões são 'base' (rota dona).
    Se uma rota tem 2 caminhões, a cidade aponta pros 2.
    """
    validate_disjoint_routes(trucks)
    groups = group_trucks_by_route(trucks)

    mapping: Dict[str, List[str]] = {}
    for rk, truck_ids in groups.items():
        for city in rk:
            mapping[city] = truck_ids[:]  # 1 ou mais caminhões (prioridade = ordem cadastrada)
    return mapping


# =========================
# 4) SOLVER OR-TOOLS
# =========================

def solve_vrp(
    orders: List[Order],
    trucks: List[Truck],
    depot_xy: Tuple[float, float],
    cost_matrix: Optional[List[List[int]]],
    allowed_trucks_per_order: Dict[str, List[str]],
    drop_penalty_per_order: Dict[str, int],
    vehicle_fixed_costs: Optional[Dict[str, int]] = None,
    base_trucks_per_order: Optional[Dict[str, Set[str]]] = None,
    move_penalty_if_outside_base: int = 0,
    time_limit_seconds: int = 10,
) -> SolveResult:
    """
    Resolve VRP com:
      - capacidade por caminhão (19t)
      - allowed vehicles por pedido (SetAllowedVehiclesForIndex)
      - pedidos opcionais com penalidade (AddDisjunction) => EXCEDENTE
      - custo fixo por veículo (SetFixedCostOfVehicle) => prioriza abrir o 1º antes do 2º
      - penalidade de remanejamento (move_penalty) se pedido for atendido fora do conjunto base
    """

    # Mapeia truck_id -> vehicle_index do OR-Tools
    truck_id_to_vid = {t.id: i for i, t in enumerate(trucks)}
    vid_to_truck_id = {i: t.id for i, t in enumerate(trucks)}

    # Nós do problema: 0 = depósito; 1..N = pedidos
    # OBS: Se você passar cost_matrix (ex.: OSRM), as coordenadas viram apenas metadados.
    points = [depot_xy] + [(o.x, o.y) for o in orders]
    if cost_matrix is None:
        cost_matrix = build_cost_matrix(points)
    else:
        n = len(points)
        if len(cost_matrix) != n or any(len(row) != n for row in cost_matrix):
            raise ValueError(
                f"cost_matrix inválida: esperado {n}x{n}, recebido {len(cost_matrix)}x"
                f"{(len(cost_matrix[0]) if cost_matrix else 0)}"
            )

    demands = [0] + [o.weight_kg for o in orders]
    capacities = [t.capacity_kg for t in trucks]

    manager = pywrapcp.RoutingIndexManager(len(points), len(trucks), 0)
    routing = pywrapcp.RoutingModel(manager)

    # (A) Prioridade de caminhões (encher 1º, depois 2º) via custo fixo de abertura
    if vehicle_fixed_costs:
        for truck_id, cost in vehicle_fixed_costs.items():
            routing.SetFixedCostOfVehicle(cost, truck_id_to_vid[truck_id])

    # (B) Capacidade (kg)
    def demand_cb(from_index: int) -> int:
        node = manager.IndexToNode(from_index)
        return demands[node]

    demand_cb_i = routing.RegisterUnaryTransitCallback(demand_cb)
    routing.AddDimensionWithVehicleCapacity(demand_cb_i, 0, capacities, True, "Capacity")
    cap_dim = routing.GetDimensionOrDie("Capacity")

    # (C) Penalidade por remanejamento (fora da base) — implementada no custo do arco por veículo
    # Pré-computa: is_base[order_node][vid] (order_node = 1..N)
    is_base = None
    if base_trucks_per_order is not None:
        is_base = [[False] * len(trucks) for _ in range(len(orders) + 1)]
        for i, o in enumerate(orders, start=1):
            base_set = base_trucks_per_order.get(o.id, set())
            for tid in base_set:
                is_base[i][truck_id_to_vid[tid]] = True

    for vid in range(len(trucks)):
        def make_arc_cb(vehicle_id: int):
            def arc_cost(from_index: int, to_index: int) -> int:
                from_node = manager.IndexToNode(from_index)
                to_node = manager.IndexToNode(to_index)
                base_cost = cost_matrix[from_node][to_node]

                # Se estamos indo PARA um pedido (to_node != 0),
                # e esse pedido está sendo atendido fora da base, soma penalidade.
                if move_penalty_if_outside_base > 0 and is_base is not None and to_node != 0:
                    if not is_base[to_node][vehicle_id]:
                        base_cost += move_penalty_if_outside_base

                return base_cost
            return arc_cost

        cb = routing.RegisterTransitCallback(make_arc_cb(vid))
        routing.SetArcCostEvaluatorOfVehicle(cb, vid)

    # (D) Restrições por pedido: quais caminhões podem atender + pode virar excedente (disjunction)
    for node, o in enumerate(orders, start=1):
        idx = manager.NodeToIndex(node)
        allowed = allowed_trucks_per_order[o.id]
        allowed_vids = [truck_id_to_vid[t] for t in allowed]
        routing.SetAllowedVehiclesForIndex(allowed_vids, idx)

        penalty = drop_penalty_per_order[o.id]
        routing.AddDisjunction([idx], penalty)

    # (E) Busca
    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    params.time_limit.FromSeconds(time_limit_seconds)

    solution = routing.SolveWithParameters(params)
    if solution is None:
        raise RuntimeError("Sem solução (mesmo com disjunction). Verifique dados/penalidades/restrições.")

    # (F) Extrai resultado
    dropped: List[str] = []
    served_by_truck: Dict[str, str] = {}
    routes: Dict[str, List[str]] = {t.id: [] for t in trucks}
    loads: Dict[str, int] = {t.id: 0 for t in trucks}

    # Nó droppado: NextVar(index) == index
    for index in range(routing.Size()):
        if routing.IsStart(index) or routing.IsEnd(index):
            continue
        if solution.Value(routing.NextVar(index)) == index:
            node = manager.IndexToNode(index)
            order = orders[node - 1]
            dropped.append(order.id)

    # Rotas por veículo
    for vid in range(len(trucks)):
        truck_id = vid_to_truck_id[vid]
        index = routing.Start(vid)

        # Se o caminhão nem saiu do depósito, pula
        if solution.Value(routing.NextVar(index)) == routing.End(vid):
            continue

        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            if node != 0:
                order = orders[node - 1]
                if order.id not in dropped:
                    routes[truck_id].append(order.id)
                    served_by_truck[order.id] = truck_id
            index = solution.Value(routing.NextVar(index))

        # carga total alocada (cumul no fim)
        end_index = routing.End(vid)
        loads[truck_id] = solution.Value(cap_dim.CumulVar(end_index))

    return SolveResult(
        objective=solution.ObjectiveValue(),
        dropped_orders=dropped,
        served_by_truck=served_by_truck,
        routes=routes,
        loads_by_truck=loads,
    )


def _safe_int(x: Any, default: int = 0) -> int:
    try:
        return int(round(float(x)))
    except Exception:
        return default


def _normalize_square_matrix_to_int(matrix: List[List[Any]], fallback: int = 10_000_000) -> List[List[int]]:
    if not matrix:
        return []
    n = len(matrix)
    out: List[List[int]] = []
    for i in range(n):
        row = matrix[i]
        if len(row) != n:
            raise ValueError(f"Matriz de custo precisa ser quadrada (linha {i} tem {len(row)}, esperado {n}).")
        out_row: List[int] = []
        for v in row:
            if v is None:
                out_row.append(fallback)
            else:
                out_row.append(_safe_int(v, default=fallback))
        out.append(out_row)
    return out


def resolver_vrp_ortools(dados_normalizados: Dict[str, Any], logica_negocio: Dict[str, Any], matriz_distancias: List[List[Any]]):
    """Wrapper compatível com o fluxo do [app.py](app.py).

    Espera:
      - dados_normalizados: retorno de normalizar_dados_entrada()
      - logica_negocio: retorno de aplicar_regras_alocacao()
      - matriz_distancias: matriz NxN (OSRM) em metros

    Retorna:
      Dict JSON-friendly com chaves: rotas, excedentes
    """
    from config.settings import CAPACIDADE_VEICULO_KG, PENALIDADE_DE_DROP, TEMPO_LIMITE_SOLVER_SEGUNDOS

    num_caminhoes = int(dados_normalizados.get('num_caminhoes') or 0)
    if num_caminhoes <= 0:
        raise ValueError("num_caminhoes precisa ser > 0")

    prefs = dados_normalizados.get('prefs') or {}

    # Monta caminhões (ids = "0", "1", ... para casar com o resto do projeto)
    trucks: List[Truck] = []
    for i in range(num_caminhoes):
        cfg = prefs.get(str(i), {})
        cities = tuple(cfg.get('cidades') or ())
        trucks.append(Truck(id=str(i), cities=cities, capacity_kg=int(CAPACIDADE_VEICULO_KG)))

    # Clientes (0 = depósito) no formato usado pelos formatters/map_service
    lista_osrm = dados_normalizados.get('lista_osrm') or []
    if not lista_osrm:
        raise ValueError("dados_normalizados['lista_osrm'] vazio")

    depot_point = lista_osrm[0]
    clientes: List[Dict[str, Any]] = [
        {
            'id': 'DEPÓSITO',
            'lat': float(depot_point.get('lat')),
            'lon': float(depot_point.get('lon')),
            'peso': 0,
            'cidade': 'DEPÓSITO',
        }
    ]

    pedidos_metadata = dados_normalizados.get('pedidos_metadata') or []
    for meta in pedidos_metadata:
        p = meta.get('original_data') or {}
        clientes.append(
            {
                'id': p.get('id') or f"{meta.get('cidade_original', meta.get('cidade', 'PEDIDO'))} #{meta.get('or_tools_index')}",
                'lat': float(p.get('lat')),
                'lon': float(p.get('lng')),
                'peso': _safe_int(meta.get('peso'), default=0),
                'cidade': meta.get('cidade_original') or meta.get('cidade') or 'DESCONHECIDA',
                'cidade_limpa': meta.get('cidade') or '',
            }
        )

    # Converte OSRM -> int
    cost_matrix = _normalize_square_matrix_to_int(matriz_distancias)

    # Monta pedidos para o solver (ids = node_index como string)
    orders: List[Order] = []
    for meta in pedidos_metadata:
        node_index = int(meta['or_tools_index'])
        point = lista_osrm[node_index]
        orders.append(
            Order(
                id=str(node_index),
                city=str(meta.get('cidade') or ''),
                weight_kg=_safe_int(meta.get('peso'), default=0),
                x=float(point.get('lon')),
                y=float(point.get('lat')),
            )
        )

    # Base por cidade: caminhões que "aceitam" a cidade.
    # Regra prática para compatibilidade: caminhão sem cidades configuradas vira "coringa" (aceita qualquer cidade).
    all_truck_ids = [t.id for t in trucks]
    city_to_base: Dict[str, List[str]] = {}
    for o in orders:
        if o.city in city_to_base:
            continue
        base = [t.id for t in trucks if (not t.cities) or (o.city in t.cities)]
        city_to_base[o.city] = base if base else all_truck_ids[:]

    # Travar pre-alocações da etapa 3 (baldes): índice do nó -> caminhão
    pre_aloc = logica_negocio.get('pre_alocacoes') or {}

    # ---------- ETAPA 1: identifica excedentes por capacidade ----------
    allowed_1: Dict[str, List[str]] = {}
    drop_1: Dict[str, int] = {}
    base_trucks_per_order: Dict[str, Set[str]] = {}

    for o in orders:
        fixed_truck = pre_aloc.get(int(o.id))
        if fixed_truck is not None:
            allowed = [str(fixed_truck)]
        else:
            allowed = city_to_base.get(o.city, all_truck_ids)

        allowed_1[o.id] = allowed
        base_trucks_per_order[o.id] = set(allowed)
        drop_1[o.id] = int(PENALIDADE_DE_DROP)

    res1 = solve_vrp(
        orders=orders,
        trucks=trucks,
        depot_xy=(0.0, 0.0),
        cost_matrix=cost_matrix,
        allowed_trucks_per_order=allowed_1,
        drop_penalty_per_order=drop_1,
        vehicle_fixed_costs=None,
        base_trucks_per_order=None,
        move_penalty_if_outside_base=0,
        time_limit_seconds=int(TEMPO_LIMITE_SOLVER_SEGUNDOS),
    )

    excess_set = set(res1.dropped_orders)

    # ---------- ETAPA 2: só excedente pode circular ----------
    allowed_2: Dict[str, List[str]] = {}
    drop_2: Dict[str, int] = {}

    for o in orders:
        base = list(base_trucks_per_order[o.id])
        if o.id in excess_set:
            allowed_2[o.id] = all_truck_ids[:]  # excedente pode ir para qualquer caminhão
            drop_2[o.id] = int(PENALIDADE_DE_DROP)
        else:
            allowed_2[o.id] = base
            drop_2[o.id] = int(PENALIDADE_DE_DROP)

    res2 = solve_vrp(
        orders=orders,
        trucks=trucks,
        depot_xy=(0.0, 0.0),
        cost_matrix=cost_matrix,
        allowed_trucks_per_order=allowed_2,
        drop_penalty_per_order=drop_2,
        vehicle_fixed_costs=None,
        base_trucks_per_order=base_trucks_per_order,
        move_penalty_if_outside_base=200_000,
        time_limit_seconds=int(TEMPO_LIMITE_SOLVER_SEGUNDOS),
    )

    # ---------- Formata saída compatível ----------
    def estimate_minutes_from_meters(m: int) -> int:
        # Aproximação simples: 60 km/h ~ 1000 m/min.
        return int(round(m / 1000))

    rotas_out: List[Dict[str, Any]] = []
    for truck in trucks:
        truck_id = truck.id
        seq_orders = res2.routes.get(truck_id) or []
        if not seq_orders:
            continue

        nodes = [0] + [int(oid) for oid in seq_orders]
        distancia = 0
        carga = 0
        tempo = 0
        paradas: List[Dict[str, Any]] = [
            {
                'node_index': 0,
                'cliente': clientes[0],
                'carga_acumulada': 0,
                'distancia_acumulada': 0,
                'tempo_acumulado': 0,
            }
        ]

        prev = 0
        for node in nodes[1:]:
            step = int(cost_matrix[prev][node])
            distancia += step
            tempo += estimate_minutes_from_meters(step)
            carga += _safe_int(clientes[node].get('peso'), default=0)
            paradas.append(
                {
                    'node_index': node,
                    'cliente': clientes[node],
                    'carga_acumulada': carga,
                    'distancia_acumulada': distancia,
                    'tempo_acumulado': tempo,
                }
            )
            prev = node

        distancia_total = distancia + int(cost_matrix[prev][0])
        rotas_out.append(
            {
                'veiculo_id': int(truck_id) + 1 if str(truck_id).isdigit() else truck_id,
                'paradas': paradas,
                'distancia_total': distancia_total,
                'carga_total': carga,
            }
        )

    excedentes_out = [clientes[int(oid)] for oid in res2.dropped_orders]

    return {
        'rotas': rotas_out,
        'excedentes': excedentes_out,
        'objective': res2.objective,
        'excedentes_etapa1': [clientes[int(oid)] for oid in res1.dropped_orders],
    }


# =========================
# 5) REALOCAÇÃO DE EXCEDENTES (ETAPA 2 INTELIGENTE)
# =========================

def _calcular_distancia_rota_osrm_coords(coords_list: List[Tuple[float, float]], osrm_url: str) -> float:
    """
    Calcula distância total de uma sequência de coordenadas via OSRM.
    coords_list: Lista de tuplas (lat, lng)
    Retorna distância em km, ou -1 se falhar.
    """
    import requests
    
    if len(coords_list) < 2:
        return 0.0
    
    # Formatar para OSRM: lng,lat
    coords_str = ";".join([f"{lng},{lat}" for lat, lng in coords_list])
    url = f"{osrm_url}/route/v1/driving/{coords_str}?overview=false"
    
    try:
        r = requests.get(url, timeout=15)
        if r.status_code == 200:
            data = r.json()
            if data.get('code') == 'Ok' and data.get('routes'):
                distance_m = data['routes'][0].get('distance', 0)
                return distance_m / 1000.0  # metros -> km
    except Exception as e:
        print(f"[WARN] Erro ao consultar OSRM: {e}", flush=True)
    
    return -1.0


def _calcular_distancia_euclidiana(coord1: Tuple[float, float], coord2: Tuple[float, float]) -> float:
    """Calcula distância euclidiana aproximada em km entre duas coordenadas (lat, lng)."""
    # Aproximação simples: 1 grau ≈ 111km
    lat1, lng1 = coord1
    lat2, lng2 = coord2
    dlat = (lat2 - lat1) * 111.0
    dlng = (lng2 - lng1) * 111.0 * math.cos(math.radians((lat1 + lat2) / 2))
    return math.sqrt(dlat**2 + dlng**2)


def realocar_excedentes_inteligente(
    res_etapa1: SolveResult,
    orders: List[Order],
    trucks: List[Truck],
    depot_coords: Tuple[float, float],  # (lat, lng)
    order_coords: Dict[str, Tuple[float, float]],  # order_id -> (lat, lng)
    max_km_adicional: float = 50.0,
    prioridade_menor_km: bool = True,
    prioridade_mais_perto: bool = False,
    osrm_url: str = "http://router.project-osrm.org",
) -> Tuple[SolveResult, List[PedidoRealocado]]:
    """
    Analisa os excedentes da etapa 1 e tenta realocá-los em caminhões com espaço disponível,
    desde que o aumento de km seja aceitável.
    
    Retorna:
        - SolveResult atualizado com as rotas reotimizadas
        - Lista de PedidoRealocado com info dos pedidos movidos
    """
    from config.settings import CAPACIDADE_VEICULO_KG
    
    excedentes = res_etapa1.dropped_orders[:]
    if not excedentes:
        print("[REALOCAÇÃO] Nenhum excedente para realocar.", flush=True)
        return res_etapa1, []
    
    print(f"\n{'='*50}", flush=True)
    print(f"🔄 ETAPA 2: REALOCAÇÃO INTELIGENTE DE EXCEDENTES", flush=True)
    print(f"{'='*50}", flush=True)
    print(f"📦 Excedentes a analisar: {len(excedentes)}", flush=True)
    print(f"⚙️  Max KM adicional permitido: {max_km_adicional}km", flush=True)
    
    # Mapear orders por id
    orders_by_id = {o.id: o for o in orders}
    trucks_by_id = {t.id: t for t in trucks}
    
    # Copiar rotas e cargas para modificação
    novas_rotas = {tid: list(orders) for tid, orders in res_etapa1.routes.items()}
    novas_cargas = dict(res_etapa1.loads_by_truck)
    novo_served = dict(res_etapa1.served_by_truck)
    
    pedidos_realocados: List[PedidoRealocado] = []
    excedentes_finais: List[str] = []
    
    # Identificar o caminhão original de cada excedente (qual rota era a "dona" da cidade)
    city_to_trucks = city_to_base_trucks(trucks)
    
    for exc_id in excedentes:
        order = orders_by_id.get(exc_id)
        if not order:
            excedentes_finais.append(exc_id)
            continue
        
        exc_coords = order_coords.get(exc_id, (order.y, order.x))  # (lat, lng)
        caminhao_original = city_to_trucks.get(order.city, ["?"])[0]  # Primeiro caminhão da rota base
        
        print(f"\n📍 Analisando excedente: {exc_id} (cidade: {order.city}, peso: {order.weight_kg}kg)", flush=True)
        print(f"   Rota original: Caminhão {caminhao_original}", flush=True)
        
        # Avaliar todos os caminhões que têm espaço
        candidatos = []
        
        for truck in trucks:
            # Pular o caminhão da própria rota base (já sabemos que não coube)
            if truck.id in city_to_trucks.get(order.city, []):
                continue
            
            # Verificar se tem capacidade
            carga_atual = novas_cargas.get(truck.id, 0)
            capacidade = truck.capacity_kg or CAPACIDADE_VEICULO_KG
            espaco_disponivel = capacidade - carga_atual
            
            if espaco_disponivel < order.weight_kg:
                print(f"   ❌ Caminhão {truck.id}: sem espaço ({espaco_disponivel}kg disponível, precisa {order.weight_kg}kg)", flush=True)
                continue
            
            # Calcular distância atual da rota do caminhão
            rota_atual = novas_rotas.get(truck.id, [])
            
            # Montar sequência de coordenadas: depósito -> paradas -> depósito
            coords_atual = [depot_coords]
            for oid in rota_atual:
                if oid in order_coords:
                    coords_atual.append(order_coords[oid])
            coords_atual.append(depot_coords)
            
            dist_atual_km = _calcular_distancia_rota_osrm_coords(coords_atual, osrm_url)
            if dist_atual_km < 0:
                # Fallback para euclidiana se OSRM falhar
                dist_atual_km = sum(_calcular_distancia_euclidiana(coords_atual[i], coords_atual[i+1]) 
                                   for i in range(len(coords_atual)-1))
            
            # Simular adição do excedente no final da rota (depois será reotimizado)
            coords_com_exc = [depot_coords]
            for oid in rota_atual:
                if oid in order_coords:
                    coords_com_exc.append(order_coords[oid])
            coords_com_exc.append(exc_coords)
            coords_com_exc.append(depot_coords)
            
            dist_com_exc_km = _calcular_distancia_rota_osrm_coords(coords_com_exc, osrm_url)
            if dist_com_exc_km < 0:
                dist_com_exc_km = sum(_calcular_distancia_euclidiana(coords_com_exc[i], coords_com_exc[i+1]) 
                                     for i in range(len(coords_com_exc)-1))
            
            km_adicional = dist_com_exc_km - dist_atual_km
            
            # Calcular distância direta do depósito/rota ao excedente (para critério "mais perto")
            dist_direta = _calcular_distancia_euclidiana(depot_coords, exc_coords)
            if rota_atual:
                # Distância do último ponto da rota ao excedente
                ultimo_ponto = order_coords.get(rota_atual[-1], depot_coords)
                dist_direta = min(dist_direta, _calcular_distancia_euclidiana(ultimo_ponto, exc_coords))
            
            print(f"   🚛 Caminhão {truck.id}: rota atual={dist_atual_km:.1f}km, com exc={dist_com_exc_km:.1f}km, adicional={km_adicional:.1f}km", flush=True)
            
            if km_adicional <= max_km_adicional:
                candidatos.append({
                    'truck_id': truck.id,
                    'km_adicional': km_adicional,
                    'dist_direta': dist_direta,
                    'dist_com_exc': dist_com_exc_km,
                })
                print(f"      ✅ VIÁVEL! (adiciona {km_adicional:.1f}km <= {max_km_adicional}km)", flush=True)
            else:
                print(f"      ❌ Inviável (adiciona {km_adicional:.1f}km > {max_km_adicional}km)", flush=True)
        
        # Escolher o melhor candidato
        if candidatos:
            if prioridade_menor_km:
                candidatos.sort(key=lambda c: c['km_adicional'])
            elif prioridade_mais_perto:
                candidatos.sort(key=lambda c: c['dist_direta'])
            
            melhor = candidatos[0]
            truck_escolhido = melhor['truck_id']
            
            # Realocar o pedido
            novas_rotas[truck_escolhido].append(exc_id)
            novas_cargas[truck_escolhido] = novas_cargas.get(truck_escolhido, 0) + order.weight_kg
            novo_served[exc_id] = truck_escolhido
            
            pedidos_realocados.append(PedidoRealocado(
                id=exc_id,
                cidade=order.city,
                caminhao_original=caminhao_original,
                caminhao_realocado=truck_escolhido,
                distancia_adicional_km=round(melhor['km_adicional'], 2),
                peso_kg=order.weight_kg,
                motivo="excedente_capacidade"
            ))
            
            print(f"   🎯 REALOCADO para Caminhão {truck_escolhido} (+{melhor['km_adicional']:.1f}km)", flush=True)
        else:
            excedentes_finais.append(exc_id)
            print(f"   ⚠️ Mantido como EXCEDENTE (nenhum caminhão viável)", flush=True)
    
    print(f"\n{'='*50}", flush=True)
    print(f"📊 RESULTADO DA REALOCAÇÃO:", flush=True)
    print(f"   ✅ Realocados: {len(pedidos_realocados)}", flush=True)
    print(f"   ⚠️ Excedentes finais: {len(excedentes_finais)}", flush=True)
    print(f"{'='*50}\n", flush=True)
    
    # Criar novo SolveResult
    novo_result = SolveResult(
        objective=res_etapa1.objective,  # Mantém objetivo original (poderia recalcular)
        dropped_orders=excedentes_finais,
        served_by_truck=novo_served,
        routes=novas_rotas,
        loads_by_truck=novas_cargas,
    )
    
    return novo_result, pedidos_realocados


# =========================
# 6) SEU FLUXO (2 ETAPAS)
# =========================

def build_vehicle_fixed_costs_for_priority(trucks: List[Truck], open_cost_step: int) -> Dict[str, int]:
    """
    Para cada grupo de rota (mesma lista de cidades), define:
      1º caminhão do grupo: custo 0
      2º caminhão do grupo: open_cost_step
      3º caminhão do grupo: 2*open_cost_step
    Isso força: "use o 1º antes de abrir o 2º".
    """
    groups = group_trucks_by_route(trucks)
    fixed: Dict[str, int] = {}
    for _, truck_ids in groups.items():
        for j, tid in enumerate(truck_ids):
            fixed[tid] = j * open_cost_step
    return fixed


def plan_day_two_stage(
    orders: List[Order],
    trucks: List[Truck],
    depot_xy: Tuple[float, float],
    # Ajustes principais:
    open_cost_step: int = 1_000_000,        # quão caro é "abrir" o 2º caminhão da mesma rota
    stage1_drop_penalty: int = 50_000_000,  # altíssimo: só vira excedente se NÃO couber mesmo
    stage2_drop_penalty: int = 2_000_000,   # custo de deixar como excedente final
    stage2_move_penalty: int = 200_000,     # custo operacional por remanejamento fora da base
    time_limit_seconds: int = 10,
    # Novos parâmetros para realocação inteligente
    max_km_adicional_realocacao: float = 50.0,
    prioridade_menor_km: bool = True,
    prioridade_mais_perto: bool = False,
    osrm_url: str = "http://router.project-osrm.org",
    order_coords: Optional[Dict[str, Tuple[float, float]]] = None,  # order_id -> (lat, lng)
    depot_coords: Optional[Tuple[float, float]] = None,  # (lat, lng)
) -> Tuple[SolveResult, SolveResult, List[PedidoRealocado]]:
    """
    Etapa 1: respeita base (listas de cidades) e prioriza 1º caminhão do grupo.
             Se estourar 19t por caminhão, dropa alguns pedidos => EXCEDENTE.

    Etapa 2: pedidos não excedentes ficam travados na base.
             apenas pedidos excedentes podem ir para alt_truck_ids (se compensar).
    """
    city_base = city_to_base_trucks(trucks)

    # base trucks por pedido
    base_trucks_per_order: Dict[str, Set[str]] = {}
    for o in orders:
        if o.city not in city_base:
            raise ValueError(f"Pedido {o.id} tem cidade '{o.city}' que não existe em nenhuma rota do dia.")
        base_trucks_per_order[o.id] = set(city_base[o.city])

    # prioridade: custo fixo por caminhão dentro da mesma rota
    vehicle_fixed = build_vehicle_fixed_costs_for_priority(trucks, open_cost_step)

    # -------------------
    # ETAPA 1
    # -------------------
    allowed_1 = {o.id: list(base_trucks_per_order[o.id]) for o in orders}
    drop_1 = {o.id: stage1_drop_penalty + o.priority_penalty for o in orders}

    res1 = solve_vrp(
        orders=orders,
        trucks=trucks,
        depot_xy=depot_xy,
        cost_matrix=None,
        allowed_trucks_per_order=allowed_1,
        drop_penalty_per_order=drop_1,
        vehicle_fixed_costs=vehicle_fixed,
        base_trucks_per_order=None,
        move_penalty_if_outside_base=0,
        time_limit_seconds=time_limit_seconds,
    )

    excess_set = set(res1.dropped_orders)

    # -------------------
    # ETAPA 2
    # -------------------
    truck_ids_all = {t.id for t in trucks}
    allowed_2: Dict[str, List[str]] = {}
    drop_2: Dict[str, int] = {}

    for o in orders:
        base = list(base_trucks_per_order[o.id])

        if o.id in excess_set:
            # só excedente pode circular
            alts = [tid for tid in o.alt_truck_ids if tid in truck_ids_all]
            allowed_2[o.id] = list(dict.fromkeys(base + alts))
            drop_2[o.id] = stage2_drop_penalty + o.priority_penalty
        else:
            # não excedente fica travado na base e não pode virar excedente agora
            allowed_2[o.id] = base
            drop_2[o.id] = stage1_drop_penalty + o.priority_penalty  # altíssimo

    res2 = solve_vrp(
        orders=orders,
        trucks=trucks,
        depot_xy=depot_xy,
        cost_matrix=None,
        allowed_trucks_per_order=allowed_2,
        drop_penalty_per_order=drop_2,
        vehicle_fixed_costs=vehicle_fixed,
        base_trucks_per_order=base_trucks_per_order,
        move_penalty_if_outside_base=stage2_move_penalty,
        time_limit_seconds=time_limit_seconds,
    )

    # -------------------
    # ETAPA 3: REALOCAÇÃO INTELIGENTE DE EXCEDENTES
    # -------------------
    pedidos_realocados: List[PedidoRealocado] = []
    
    if res2.dropped_orders and order_coords and depot_coords:
        print(f"\n[ETAPA 3] Iniciando realocação inteligente de {len(res2.dropped_orders)} excedentes...", flush=True)
        
        res2, pedidos_realocados = realocar_excedentes_inteligente(
            res_etapa1=res2,
            orders=orders,
            trucks=trucks,
            depot_coords=depot_coords,
            order_coords=order_coords,
            max_km_adicional=max_km_adicional_realocacao,
            prioridade_menor_km=prioridade_menor_km,
            prioridade_mais_perto=prioridade_mais_perto,
            osrm_url=osrm_url,
        )
    elif res2.dropped_orders:
        print(f"\n[ETAPA 3] Realocação desabilitada (sem coordenadas). Excedentes mantidos: {len(res2.dropped_orders)}", flush=True)

    return res1, res2, pedidos_realocados


# =========================
# 6) EXEMPLO (RODAR E VER)
# =========================

if __name__ == "__main__":
    depot = (0.0, 0.0)

    # Exemplo de 1 dia:
    # - Rota AB tem 2 caminhões (T1 é priorizado, depois T2)
    # - Rota CD tem 1 caminhão (T3)
    trucks = [
        Truck(id="T1", cities=("A", "B")),
        Truck(id="T2", cities=("A", "B")),  # mesma lista => mesma rota => 2 caminhões
        Truck(id="T3", cities=("C", "D")),
    ]

    # Pedidos (aqui eu forço excesso na rota AB se você aumentar um pouco os pesos)
    orders = [
        Order(id="P01", city="A", weight_kg=10_000, x=2, y=1, alt_truck_ids=["T3"]),
        Order(id="P02", city="A", weight_kg=10_000, x=3, y=1, alt_truck_ids=["T3"]),
        Order(id="P03", city="B", weight_kg=10_000, x=4, y=1, alt_truck_ids=["T3"]),
        # Soma AB = 30t. Com 2 caminhões dá 38t, então cabe. Se subir pra 40t, vira excedente.
        Order(id="P04", city="C", weight_kg=6_000, x=1, y=5),
        Order(id="P05", city="D", weight_kg=6_000, x=1, y=6),
    ]

    res1, res2, pedidos_realocados = plan_day_two_stage(
        orders=orders,
        trucks=trucks,
        depot_xy=depot,
        open_cost_step=1_000_000,
        stage1_drop_penalty=50_000_000,
        stage2_drop_penalty=2_000_000,
        stage2_move_penalty=200_000,
        time_limit_seconds=5,
    )

    print("=== ETAPA 1 (base fixa; identifica excedente) ===")
    print("Objective:", res1.objective)
    print("Excedente (dropped):", res1.dropped_orders)
    for t in trucks:
        print(f"{t.id}: rota={res1.routes[t.id]} | carga={res1.loads_by_truck[t.id]} kg")

    print("\n=== ETAPA 2 (só excedente circula se compensar) ===")
    print("Objective:", res2.objective)
    print("Excedente final (dropped):", res2.dropped_orders)
    for t in trucks:
        print(f"{t.id}: rota={res2.routes[t.id]} | carga={res2.loads_by_truck[t.id]} kg")

    # Reporta remanejados (excedente atendido fora da base)
    base_map = city_to_base_trucks(trucks)
    for o in orders:
        if o.id in res2.served_by_truck:
            assigned = res2.served_by_truck[o.id]
            base_set = set(base_map[o.city])
            if assigned not in base_set:
                print(f"REMANEJADO: {o.id} (cidade {o.city}) -> {assigned} | base={sorted(base_set)}")

    # Mostra pedidos realocados pela etapa 3
    if pedidos_realocados:
        print("\n=== ETAPA 3 (realocação inteligente) ===")
        for pr in pedidos_realocados:
            print(f"REALOCADO: {pr.id} ({pr.cidade}) | {pr.caminhao_original} -> {pr.caminhao_realocado} | +{pr.distancia_adicional_km}km")
