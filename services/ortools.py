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
    city_switch_penalty: int = 0,
    time_limit_seconds: int = 10,
) -> SolveResult:
    """
    Resolve VRP com:
      - capacidade por caminhão (19t)
      - allowed vehicles por pedido (SetAllowedVehiclesForIndex)
      - pedidos opcionais com penalidade (AddDisjunction) => EXCEDENTE
      - custo fixo por veículo (SetFixedCostOfVehicle) => prioriza abrir o 1º antes do 2º
      - penalidade de remanejamento (move_penalty) se pedido for atendido fora do conjunto base
      - penalidade de troca de cidade (city_switch_penalty) para agrupar entregas da mesma cidade
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

    # (C.2) Pré-computa cidade de cada nó para penalidade de troca de cidade
    # node_city[0] = None (depósito), node_city[i] = orders[i-1].city
    node_city: List[Optional[str]] = [None] + [o.city for o in orders]

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

                # Penalidade de troca de cidade: se saímos de um pedido em cidade A
                # para um pedido em cidade B (ambas != depósito e cidades diferentes),
                # soma penalidade para incentivar agrupamento por cidade.
                if city_switch_penalty > 0 and from_node != 0 and to_node != 0:
                    if node_city[from_node] != node_city[to_node]:
                        base_cost += city_switch_penalty

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
    order_cities: Optional[Dict[str, str]] = None,
    city_switch_penalty: int = 0,
) -> List[str]:
    """
    Reordena pedidos de uma rota usando TSP via OR-Tools.
    Retorna a sequência otimizada de IDs.
    
    Args:
        order_ids: Lista de IDs dos pedidos na rota
        order_coords: Dict order_id -> (lat, lng)
        depot_coords: (lat, lng) do depósito
        osrm_url: URL do servidor OSRM
        order_cities: Dict order_id -> cidade (para penalidade de troca)
        city_switch_penalty: Penalidade em metros ao trocar de cidade
    
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
    
    # Pré-computa cidade de cada nó TSP para penalidade de troca
    # tsp_city[0] = None (depot), tsp_city[i] = cidade do valid_ids[i-1]
    tsp_city: List[Optional[str]] = [None]
    if order_cities and city_switch_penalty > 0:
        for oid in valid_ids:
            tsp_city.append(order_cities.get(oid))

    # Resolver TSP com OR-Tools
    manager = pywrapcp.RoutingIndexManager(n, 1, 0)  # 1 veículo, depot no index 0
    routing = pywrapcp.RoutingModel(manager)
    
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        base = cost_matrix[from_node][to_node]
        # Penalidade de troca de cidade no TSP
        if city_switch_penalty > 0 and len(tsp_city) > max(from_node, to_node):
            if from_node != 0 and to_node != 0:
                if tsp_city[from_node] != tsp_city[to_node]:
                    base += city_switch_penalty
        return base
    
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
    cost_matrix: List[List[int]],  # matriz de custo (metros) — nó 0 = depósito
    node_of: Dict[str, int],  # order_id -> índice na cost_matrix
    max_km_adicional: float = 50.0,
    prioridade_menor_km: bool = True,
    prioridade_mais_perto: bool = False,
    osrm_url: str = "http://router.project-osrm.org",
) -> Tuple[SolveResult, List[PedidoRealocado]]:
    """
    Analisa os excedentes e tenta realocá-los em caminhões com espaço disponível,
    usando a cost_matrix já calculada (zero chamadas HTTP adicionais).

    Para cada par (excedente, caminhão), avalia TODAS as posições de inserção
    na rota e escolhe a que causa menor km adicional (cheapest insertion).
    Avalia todos os pares de uma vez e aloca sempre o melhor par disponível.

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

    # Converter max_km para metros (unidade da cost_matrix OSRM)
    max_metros_adicional = max_km_adicional * 1000.0

    # ---------------------------------------------------------------
    # Avaliar TODOS os pares (excedente, caminhão) via cost_matrix
    # Testa TODAS as posições de inserção (cheapest insertion)
    # Zero chamadas HTTP — tudo via lookup na matriz em memória
    # ---------------------------------------------------------------
    excedentes_restantes = set(excedentes)
    pedidos_realocados: List[PedidoRealocado] = []

    while excedentes_restantes:
        # (exc_id, truck_id, delta_metros, insert_pos, dist_direta_metros)
        melhor_par = None

        for exc_id in list(excedentes_restantes):
            order = orders_by_id.get(exc_id)
            if not order:
                excedentes_restantes.discard(exc_id)
                continue

            exc_node = node_of.get(exc_id)
            if exc_node is None:
                excedentes_restantes.discard(exc_id)
                continue

            for truck in trucks:
                # Pular rota base do excedente (já sabemos que não coube)
                if truck.id in city_to_trucks.get(order.city, []):
                    continue

                # Verificar capacidade
                carga_atual = novas_cargas.get(truck.id, 0)
                capacidade = truck.capacity_kg or CAPACIDADE_VEICULO_KG
                if (capacidade - carga_atual) < order.weight_kg:
                    continue

                # ---- CHEAPEST INSERTION: avaliar TODAS as posições ----
                rota_atual = novas_rotas.get(truck.id, [])
                # Nós da rota: [depósito, p1, p2, ..., pK, depósito]
                route_nodes = [0] + [node_of[oid] for oid in rota_atual if oid in node_of] + [0]

                best_delta = float('inf')
                best_pos = 0

                for k in range(len(route_nodes) - 1):
                    nk = route_nodes[k]
                    nk1 = route_nodes[k + 1]
                    # Δ = custo(k→exc) + custo(exc→k+1) - custo(k→k+1)
                    delta = (cost_matrix[nk][exc_node]
                             + cost_matrix[exc_node][nk1]
                             - cost_matrix[nk][nk1])
                    if delta < best_delta:
                        best_delta = delta
                        best_pos = k  # inserir na posição k da lista de order_ids

                if best_delta > max_metros_adicional:
                    continue

                # Distância direta (para critério "mais perto") via matrix
                dist_direta = cost_matrix[0][exc_node]  # do depósito
                for oid in rota_atual:
                    n = node_of.get(oid)
                    if n is not None:
                        dist_direta = min(dist_direta, cost_matrix[n][exc_node])

                # Comparar com o melhor par global
                if melhor_par is None:
                    melhor_par = (exc_id, truck.id, best_delta, best_pos, dist_direta)
                else:
                    if prioridade_menor_km:
                        if best_delta < melhor_par[2]:
                            melhor_par = (exc_id, truck.id, best_delta, best_pos, dist_direta)
                    elif prioridade_mais_perto:
                        if dist_direta < melhor_par[4]:
                            melhor_par = (exc_id, truck.id, best_delta, best_pos, dist_direta)

        # Se não encontrou nenhum par viável, sai do loop
        if melhor_par is None:
            break

        # Efetuar a realocação do melhor par na posição ótima
        exc_id, truck_escolhido, delta_metros, insert_pos, _ = melhor_par
        order = orders_by_id[exc_id]
        caminhao_original = city_to_trucks.get(order.city, ["?"])[0]

        # Inserir na posição ótima (cheapest insertion, não no final!)
        novas_rotas[truck_escolhido].insert(insert_pos, exc_id)
        novas_cargas[truck_escolhido] = novas_cargas.get(truck_escolhido, 0) + order.weight_kg
        novo_served[exc_id] = truck_escolhido
        excedentes_restantes.discard(exc_id)

        km_add = delta_metros / 1000.0
        pedidos_realocados.append(PedidoRealocado(
            id=exc_id,
            cidade=order.city,
            caminhao_original=caminhao_original,
            caminhao_realocado=truck_escolhido,
            distancia_adicional_km=round(km_add, 2),
            peso_kg=order.weight_kg,
            motivo="excedente_capacidade",
        ))

        print(f"   REALOCADO: {exc_id} ({order.city}) -> Caminhao {truck_escolhido} (+{km_add:.1f}km, pos {insert_pos})", flush=True)

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

        # Montar dicionário de cidades para o TSP respeitar agrupamento
        order_cities_dict = {o.id: o.city for o in orders}
        from config.settings import PENALIDADE_TROCA_CIDADE

        for truck_id in caminhoes_afetados:
            rota_atual = novas_rotas.get(truck_id, [])
            if len(rota_atual) >= 2:
                print(f"   Caminhao {truck_id}: reordenando {len(rota_atual)} paradas...", flush=True)
                rota_otimizada = reordenar_rota_tsp(
                    order_ids=rota_atual,
                    order_coords=order_coords,
                    depot_coords=depot_coords,
                    osrm_url=osrm_url,
                    order_cities=order_cities_dict,
                    city_switch_penalty=PENALIDADE_TROCA_CIDADE,
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


def _reordenar_cidade_pesada_primeiro(
    route_order_ids: List[str],
    orders_by_id: Dict[str, Order],
    min_pedidos: int = 15,
) -> List[str]:
    """
    Pós-processamento: se a última cidade visitada na rota tem >= min_pedidos,
    move TODOS os pedidos dessa cidade para o início da rota.
    Mantém a ordem relativa dos demais pedidos.

    Lógica:
      1. Identifica a cidade do último pedido da rota.
      2. Conta quantos pedidos dessa cidade existem na rota inteira.
      3. Se >= min_pedidos, separa em [pedidos_cidade_pesada] + [restante].
    """
    if not route_order_ids or min_pedidos <= 0:
        return route_order_ids

    # Cidade do último pedido
    last_order = orders_by_id.get(route_order_ids[-1])
    if not last_order:
        return route_order_ids
    last_city = last_order.city

    # Conta total de pedidos dessa cidade na rota
    city_ids = [oid for oid in route_order_ids
                if orders_by_id.get(oid) and orders_by_id[oid].city == last_city]

    if len(city_ids) < min_pedidos:
        return route_order_ids

    # Move todos os pedidos da cidade pesada para o início
    city_set = set(city_ids)
    other_ids = [oid for oid in route_order_ids if oid not in city_set]

    print(f"      [CIDADE-PESADA] '{last_city}' tem {len(city_ids)} pedidos (>= {min_pedidos}) "
          f"e era a última -> movida para o INÍCIO da rota", flush=True)

    return city_ids + other_ids


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
        PRIORIDADE_MAIS_PERTO, OSRM_URL, PENALIDADE_TROCA_CIDADE,
        MIN_PEDIDOS_CIDADE_PRIORIDADE,
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
        print("[WARN] OSRM falhou, construindo matriz euclidiana como fallback.", flush=True)
        coords_list = [depot_coords] + [order_coords.get(o.id, (o.y, o.x)) for o in orders]
        cost_matrix = _build_euclidean_matrix(coords_list)

    # Mapeamento order_id -> índice na cost_matrix (nó 0 = depósito)
    node_of = {orders[i].id: i + 1 for i in range(len(orders))}

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
        city_switch_penalty=PENALIDADE_TROCA_CIDADE,
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
        city_switch_penalty=PENALIDADE_TROCA_CIDADE,
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
            cost_matrix=cost_matrix,
            node_of=node_of,
            max_km_adicional=max_km_adicional_realocacao,
            prioridade_menor_km=prioridade_menor_km,
            prioridade_mais_perto=prioridade_mais_perto,
            osrm_url=osrm_url,
        )

    # -------------------
    # ETAPA 4: PÓS-PROCESSAMENTO — Cidade pesada primeiro
    # -------------------
    if MIN_PEDIDOS_CIDADE_PRIORIDADE > 0:
        orders_by_id = {o.id: o for o in orders}
        for truck_id, rota in res2.routes.items():
            if rota:
                res2.routes[truck_id] = _reordenar_cidade_pesada_primeiro(
                    rota, orders_by_id, MIN_PEDIDOS_CIDADE_PRIORIDADE,
                )

    return res1, res2, pedidos_realocados
