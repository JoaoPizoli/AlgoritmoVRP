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

    # (E) Busca (estratégias configuráveis via settings.py)
    from config.settings import FIRST_SOLUTION_STRATEGY, METODO_BUSCA_LOCAL

    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = getattr(
        routing_enums_pb2.FirstSolutionStrategy,
        FIRST_SOLUTION_STRATEGY,
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC,
    )
    params.local_search_metaheuristic = getattr(
        routing_enums_pb2.LocalSearchMetaheuristic,
        METODO_BUSCA_LOCAL,
        routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH,
    )
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


def _normalize_square_matrix_to_int(matrix: List[List[Any]]) -> List[List[int]]:
    """Converte matriz para inteiros. Valores None recebem fallback baseado no max real da matriz."""
    if not matrix:
        return []
    n = len(matrix)

    # Primeiro passo: encontrar o maior valor real da matriz para calcular fallback
    max_real = 0
    for row in matrix:
        for v in row:
            if v is not None:
                try:
                    val = int(round(float(v)))
                    if val > max_real:
                        max_real = val
                except Exception:
                    pass
    fallback = max(max_real * 3, 1_000_000)  # 3x o máximo real, mínimo 1M

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


# =========================
# 5) CONSTRUÇÃO DA MATRIZ DE CUSTO VIA OSRM
# =========================

def _build_osrm_cost_matrix(
    orders: List[Order],
    depot_coords: Tuple[float, float],
    order_coords: Dict[str, Tuple[float, float]],
) -> Optional[List[List[int]]]:
    """
    Constrói matriz de custo NxN via OSRM (índice 0 = depósito, 1..N = pedidos).
    Usa distâncias reais de estrada em metros.
    Retorna None se a consulta OSRM falhar.
    """
    from services.osrm_service import criar_matriz_distancias

    pontos = [{'lat': depot_coords[0], 'lon': depot_coords[1]}]
    for o in orders:
        coord = order_coords.get(o.id, (o.y, o.x))
        pontos.append({'lat': coord[0], 'lon': coord[1]})

    return criar_matriz_distancias(pontos)


# =========================
# 6) REALOCAÇÃO DE EXCEDENTES (ETAPA 2 INTELIGENTE)
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


def reordenar_rota_tsp(
    order_ids: List[str],
    order_coords: Dict[str, Tuple[float, float]],
    depot_coords: Tuple[float, float],
    osrm_url: str = "http://router.project-osrm.org",
) -> List[str]:
    """
    Reordena pedidos de uma rota usando TSP via OR-Tools.
    Retorna a sequência otimizada de IDs.
    
    Args:
        order_ids: Lista de IDs dos pedidos na rota
        order_coords: Dict order_id -> (lat, lng)
        depot_coords: (lat, lng) do depósito
        osrm_url: URL do servidor OSRM
    
    Returns:
        Lista de order_ids na sequência otimizada
    """
    import requests
    
    if len(order_ids) <= 1:
        return order_ids
    
    # Montar lista de coordenadas: [depot, order1, order2, ...]
    coords_list = [depot_coords]
    valid_ids = []
    for oid in order_ids:
        if oid in order_coords:
            coords_list.append(order_coords[oid])
            valid_ids.append(oid)
    
    if len(valid_ids) <= 1:
        return order_ids
    
    n = len(coords_list)  # depot + pedidos
    
    # Tentar obter matriz de distância via OSRM
    try:
        coords_str = ";".join([f"{lng},{lat}" for lat, lng in coords_list])
        url = f"{osrm_url}/table/v1/driving/{coords_str}?annotations=distance"
        r = requests.get(url, timeout=30)
        
        if r.status_code == 200:
            data = r.json()
            if data.get('code') == 'Ok' and data.get('distances'):
                cost_matrix = []
                for row in data['distances']:
                    cost_matrix.append([int(d) if d is not None else int(_calcular_distancia_euclidiana(coords_list[0], coords_list[0]) * 1000) for d in row])
                # Corrigir Nones com euclidiana real por par
                for ri, row in enumerate(data['distances']):
                    for ci, d in enumerate(row):
                        if d is None:
                            cost_matrix[ri][ci] = int(_calcular_distancia_euclidiana(coords_list[ri], coords_list[ci]) * 1000 * 1.4)
            else:
                # Fallback para distância euclidiana
                cost_matrix = _build_euclidean_matrix(coords_list)
        else:
            cost_matrix = _build_euclidean_matrix(coords_list)
    except Exception as e:
        print(f"[WARN] OSRM falhou para TSP, usando euclidiana: {e}", flush=True)
        cost_matrix = _build_euclidean_matrix(coords_list)
    
    # Resolver TSP com OR-Tools
    manager = pywrapcp.RoutingIndexManager(n, 1, 0)  # 1 veículo, depot no index 0
    routing = pywrapcp.RoutingModel(manager)
    
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return cost_matrix[from_node][to_node]
    
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    
    # Parâmetros de busca
    search_params = pywrapcp.DefaultRoutingSearchParameters()
    from config.settings import TSP_TIME_LIMIT_SECONDS
    search_params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_params.time_limit.FromSeconds(TSP_TIME_LIMIT_SECONDS)
    
    solution = routing.SolveWithParameters(search_params)
    
    if solution:
        # Extrair sequência otimizada
        index = routing.Start(0)
        ordered_ids = []
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            if node > 0:  # Pular depot (node 0)
                ordered_ids.append(valid_ids[node - 1])
            index = solution.Value(routing.NextVar(index))
        return ordered_ids
    else:
        # Se não conseguiu resolver, retorna ordem original
        return order_ids


def _build_euclidean_matrix(coords_list: List[Tuple[float, float]]) -> List[List[int]]:
    """Constrói matriz de distância euclidiana em metros."""
    n = len(coords_list)
    matrix = []
    for i in range(n):
        row = []
        for j in range(n):
            if i == j:
                row.append(0)
            else:
                dist = _calcular_distancia_euclidiana(coords_list[i], coords_list[j])
                row.append(int(dist * 1000))  # km -> metros
        matrix.append(row)
    return matrix


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
    Analisa os excedentes e tenta realocá-los em caminhões com espaço disponível,
    desde que o aumento de km seja aceitável.

    Avalia TODOS os pares (excedente, caminhão) de uma vez e aloca sempre o melhor
    par disponível (menor km adicional ou mais próximo), evitando a­simple greedy
    sequencial que pode desperdiçar espaço.

    Retorna:
        - SolveResult atualizado com as rotas reotimizadas
        - Lista de PedidoRealocado com info dos pedidos movidos
    """
    from config.settings import CAPACIDADE_VEICULO_KG

    excedentes = res_etapa1.dropped_orders[:]
    if not excedentes:
        print("[REALOCACAO] Nenhum excedente para realocar.", flush=True)
        return res_etapa1, []

    print(f"\n{'='*50}", flush=True)
    print(f"[REALOCACAO] ETAPA: REALOCACAO INTELIGENTE DE EXCEDENTES", flush=True)
    print(f"{'='*50}", flush=True)
    print(f"Excedentes a analisar: {len(excedentes)}", flush=True)
    print(f"Max KM adicional permitido: {max_km_adicional}km", flush=True)

    orders_by_id = {o.id: o for o in orders}
    city_to_trucks = city_to_base_trucks(trucks)

    # Copiar rotas e cargas para modificação
    novas_rotas = {tid: list(oids) for tid, oids in res_etapa1.routes.items()}
    novas_cargas = dict(res_etapa1.loads_by_truck)
    novo_served = dict(res_etapa1.served_by_truck)

    # ---------------------------------------------------------------
    # Pré-calcular distância atual de cada caminhão (só uma vez)
    # ---------------------------------------------------------------
    dist_atual_cache: Dict[str, float] = {}
    for truck in trucks:
        rota = novas_rotas.get(truck.id, [])
        if not rota:
            dist_atual_cache[truck.id] = 0.0
            continue
        coords = [depot_coords]
        for oid in rota:
            if oid in order_coords:
                coords.append(order_coords[oid])
        coords.append(depot_coords)
        d = _calcular_distancia_rota_osrm_coords(coords, osrm_url)
        if d < 0:
            d = sum(_calcular_distancia_euclidiana(coords[i], coords[i + 1])
                    for i in range(len(coords) - 1))
        dist_atual_cache[truck.id] = d

    # ---------------------------------------------------------------
    # Avaliar TODOS os pares (excedente, caminhão) de uma vez
    # ---------------------------------------------------------------
    excedentes_restantes = set(excedentes)
    pedidos_realocados: List[PedidoRealocado] = []

    while excedentes_restantes:
        melhor_par = None  # (exc_id, truck_id, km_adicional, dist_direta)

        for exc_id in list(excedentes_restantes):
            order = orders_by_id.get(exc_id)
            if not order:
                excedentes_restantes.discard(exc_id)
                continue

            exc_coords = order_coords.get(exc_id, (order.y, order.x))

            for truck in trucks:
                # Pular rota base do excedente (já sabemos que não coube)
                if truck.id in city_to_trucks.get(order.city, []):
                    continue

                # Verificar capacidade
                carga_atual = novas_cargas.get(truck.id, 0)
                capacidade = truck.capacity_kg or CAPACIDADE_VEICULO_KG
                if (capacidade - carga_atual) < order.weight_kg:
                    continue

                # Calcular distância com o excedente adicionado
                rota_atual = novas_rotas.get(truck.id, [])
                coords_com_exc = [depot_coords]
                for oid in rota_atual:
                    if oid in order_coords:
                        coords_com_exc.append(order_coords[oid])
                coords_com_exc.append(exc_coords)
                coords_com_exc.append(depot_coords)

                dist_com_exc = _calcular_distancia_rota_osrm_coords(coords_com_exc, osrm_url)
                if dist_com_exc < 0:
                    dist_com_exc = sum(
                        _calcular_distancia_euclidiana(coords_com_exc[i], coords_com_exc[i + 1])
                        for i in range(len(coords_com_exc) - 1)
                    )

                km_adicional = dist_com_exc - dist_atual_cache.get(truck.id, 0.0)
                if km_adicional > max_km_adicional:
                    continue

                # Distância direta (para critério "mais perto")
                dist_direta = _calcular_distancia_euclidiana(depot_coords, exc_coords)
                if rota_atual:
                    ultimo = order_coords.get(rota_atual[-1], depot_coords)
                    dist_direta = min(dist_direta, _calcular_distancia_euclidiana(ultimo, exc_coords))

                # Comparar com o melhor par atual
                if melhor_par is None:
                    melhor_par = (exc_id, truck.id, km_adicional, dist_direta)
                else:
                    if prioridade_menor_km:
                        if km_adicional < melhor_par[2]:
                            melhor_par = (exc_id, truck.id, km_adicional, dist_direta)
                    elif prioridade_mais_perto:
                        if dist_direta < melhor_par[3]:
                            melhor_par = (exc_id, truck.id, km_adicional, dist_direta)

        # Se não encontrou nenhum par viável, sai do loop
        if melhor_par is None:
            break

        # Efetuar a realocação do melhor par
        exc_id, truck_escolhido, km_add, _ = melhor_par
        order = orders_by_id[exc_id]
        caminhao_original = city_to_trucks.get(order.city, ["?"])[0]

        novas_rotas[truck_escolhido].append(exc_id)
        novas_cargas[truck_escolhido] = novas_cargas.get(truck_escolhido, 0) + order.weight_kg
        novo_served[exc_id] = truck_escolhido
        excedentes_restantes.discard(exc_id)

        # Atualizar cache de distância do caminhão que recebeu o pedido
        rota_atualizada = novas_rotas[truck_escolhido]
        coords_upd = [depot_coords]
        for oid in rota_atualizada:
            if oid in order_coords:
                coords_upd.append(order_coords[oid])
        coords_upd.append(depot_coords)
        d = _calcular_distancia_rota_osrm_coords(coords_upd, osrm_url)
        if d < 0:
            d = sum(_calcular_distancia_euclidiana(coords_upd[i], coords_upd[i + 1])
                    for i in range(len(coords_upd) - 1))
        dist_atual_cache[truck_escolhido] = d

        pedidos_realocados.append(PedidoRealocado(
            id=exc_id,
            cidade=order.city,
            caminhao_original=caminhao_original,
            caminhao_realocado=truck_escolhido,
            distancia_adicional_km=round(km_add, 2),
            peso_kg=order.weight_kg,
            motivo="excedente_capacidade",
        ))

        print(f"   REALOCADO: {exc_id} ({order.city}) -> Caminhao {truck_escolhido} (+{km_add:.1f}km)", flush=True)

    excedentes_finais = list(excedentes_restantes)

    print(f"\n{'='*50}", flush=True)
    print(f"RESULTADO DA REALOCACAO:", flush=True)
    print(f"   Realocados: {len(pedidos_realocados)}", flush=True)
    print(f"   Excedentes finais: {len(excedentes_finais)}", flush=True)
    print(f"{'='*50}\n", flush=True)

    # ===========================================
    # REORDENAR ROTAS QUE RECEBERAM REALOCAÇÕES (TSP)
    # ===========================================
    if pedidos_realocados:
        caminhoes_afetados = {pr.caminhao_realocado for pr in pedidos_realocados}

        print(f"\n{'='*50}", flush=True)
        print(f"[TSP] REORDENANDO ROTAS AFETADAS", flush=True)
        print(f"{'='*50}", flush=True)
        print(f"Rotas a reotimizar: {len(caminhoes_afetados)} ({', '.join(sorted(caminhoes_afetados))})", flush=True)

        for truck_id in caminhoes_afetados:
            rota_atual = novas_rotas.get(truck_id, [])
            if len(rota_atual) >= 2:
                print(f"   Caminhao {truck_id}: reordenando {len(rota_atual)} paradas...", flush=True)
                rota_otimizada = reordenar_rota_tsp(
                    order_ids=rota_atual,
                    order_coords=order_coords,
                    depot_coords=depot_coords,
                    osrm_url=osrm_url,
                )
                novas_rotas[truck_id] = rota_otimizada
                print(f"      Ordem otimizada: {rota_otimizada}", flush=True)

        print(f"{'='*50}\n", flush=True)

    # Criar novo SolveResult
    novo_result = SolveResult(
        objective=res_etapa1.objective,
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
    depot_coords: Tuple[float, float],
    order_coords: Dict[str, Tuple[float, float]],
    open_cost_step: Optional[int] = None,
    stage1_drop_penalty: Optional[int] = None,
    stage2_drop_penalty: Optional[int] = None,
    stage2_move_penalty: Optional[int] = None,
    time_limit_seconds: Optional[int] = None,
    max_km_adicional_realocacao: Optional[float] = None,
    prioridade_menor_km: Optional[bool] = None,
    prioridade_mais_perto: Optional[bool] = None,
    osrm_url: Optional[str] = None,
) -> Tuple[SolveResult, SolveResult, List[PedidoRealocado]]:
    """
    Etapa 1: respeita base (listas de cidades) e prioriza 1º caminhão do grupo.
             Se estourar 19t por caminhão, dropa alguns pedidos => EXCEDENTE.

    Etapa 2: pedidos não excedentes ficam travados na base.
             apenas pedidos excedentes podem ir para alt_truck_ids (se compensar).

    Etapa 3: realocação inteligente de excedentes em caminhões com espaço.

    Todos os defaults vêm do config/settings.py e podem ser sobrescritos via params.
    """
    from config.settings import (
        OPEN_COST_STEP, STAGE1_DROP_PENALTY, STAGE2_DROP_PENALTY,
        STAGE2_MOVE_PENALTY, TEMPO_LIMITE_SOLVER_SEGUNDOS,
        MAX_KM_ADICIONAL_REALOCACAO, PRIORIDADE_MENOR_KM_ADICIONAL,
        PRIORIDADE_MAIS_PERTO, OSRM_URL,
    )

    # Aplica defaults do settings.py quando não fornecido via params
    open_cost_step = open_cost_step if open_cost_step is not None else OPEN_COST_STEP
    stage1_drop_penalty = stage1_drop_penalty if stage1_drop_penalty is not None else STAGE1_DROP_PENALTY
    stage2_drop_penalty = stage2_drop_penalty if stage2_drop_penalty is not None else STAGE2_DROP_PENALTY
    stage2_move_penalty = stage2_move_penalty if stage2_move_penalty is not None else STAGE2_MOVE_PENALTY
    time_limit_seconds = time_limit_seconds if time_limit_seconds is not None else TEMPO_LIMITE_SOLVER_SEGUNDOS
    max_km_adicional_realocacao = max_km_adicional_realocacao if max_km_adicional_realocacao is not None else MAX_KM_ADICIONAL_REALOCACAO
    prioridade_menor_km = prioridade_menor_km if prioridade_menor_km is not None else PRIORIDADE_MENOR_KM_ADICIONAL
    prioridade_mais_perto = prioridade_mais_perto if prioridade_mais_perto is not None else PRIORIDADE_MAIS_PERTO
    osrm_url = osrm_url if osrm_url is not None else OSRM_URL

    # Construir matriz de custo via OSRM (distâncias reais de estrada)
    cost_matrix = _build_osrm_cost_matrix(orders, depot_coords, order_coords)
    if cost_matrix is not None:
        print(f"[OK] Matriz OSRM calculada ({len(cost_matrix)}x{len(cost_matrix)})", flush=True)
    else:
        print("[WARN] OSRM falhou, usando distância euclidiana como fallback.", flush=True)

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
        cost_matrix=cost_matrix,
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
        cost_matrix=cost_matrix,
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
    
    if res2.dropped_orders:
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
