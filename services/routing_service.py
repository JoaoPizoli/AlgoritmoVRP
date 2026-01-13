"""Serviço de roteirização usado pelo modo CLI (main.py).

Mantém a estrutura de saída esperada por:
- utils/formatters.py
- services/map_service.py

Observação: este módulo usa o solver em services/ortools.py.
"""

from __future__ import annotations

from typing import Any, Dict, List

from config.settings import CAPACIDADE_VEICULO_KG, PENALIDADE_DE_DROP, TEMPO_LIMITE_SOLVER_SEGUNDOS, NUM_CAMINHOES_PADRAO
from services.ortools import Truck, Order, solve_vrp, _normalize_square_matrix_to_int


def resolver_roteamento(clientes: List[Dict[str, Any]], matriz_distancias: List[List[Any]]):
    """Resolve um CVRP simples (sem preferências por cidade) para o modo local."""

    if not clientes or len(clientes) < 2:
        return {"rotas": [], "excedentes": []}

    num_caminhoes = int(NUM_CAMINHOES_PADRAO)
    if num_caminhoes <= 0:
        raise ValueError("NUM_CAMINHOES_PADRAO precisa ser > 0")

    cost_matrix = _normalize_square_matrix_to_int(matriz_distancias)

    trucks = [Truck(id=str(i), cities=tuple(), capacity_kg=int(CAPACIDADE_VEICULO_KG)) for i in range(num_caminhoes)]

    orders: List[Order] = []
    for node_index in range(1, len(clientes)):
        cli = clientes[node_index]
        orders.append(
            Order(
                id=str(node_index),
                city=str(cli.get("cidade") or ""),
                weight_kg=int(round(float(cli.get("peso") or 0))),
                x=float(cli.get("lon") or 0.0),
                y=float(cli.get("lat") or 0.0),
            )
        )

    all_truck_ids = [t.id for t in trucks]
    allowed = {o.id: all_truck_ids[:] for o in orders}
    drop = {o.id: int(PENALIDADE_DE_DROP) for o in orders}

    res = solve_vrp(
        orders=orders,
        trucks=trucks,
        depot_xy=(0.0, 0.0),
        cost_matrix=cost_matrix,
        allowed_trucks_per_order=allowed,
        drop_penalty_per_order=drop,
        vehicle_fixed_costs=None,
        base_trucks_per_order=None,
        move_penalty_if_outside_base=0,
        time_limit_seconds=int(TEMPO_LIMITE_SOLVER_SEGUNDOS),
    )

    def estimate_minutes_from_meters(m: int) -> int:
        return int(round(m / 1000))

    rotas_out: List[Dict[str, Any]] = []
    for truck in trucks:
        seq_orders = res.routes.get(truck.id) or []
        if not seq_orders:
            continue

        nodes = [0] + [int(oid) for oid in seq_orders]
        distancia = 0
        carga = 0
        tempo = 0

        paradas: List[Dict[str, Any]] = [
            {
                "node_index": 0,
                "cliente": clientes[0],
                "carga_acumulada": 0,
                "distancia_acumulada": 0,
                "tempo_acumulado": 0,
            }
        ]

        prev = 0
        for node in nodes[1:]:
            step = int(cost_matrix[prev][node])
            distancia += step
            tempo += estimate_minutes_from_meters(step)
            carga += int(round(float(clientes[node].get("peso") or 0)))
            paradas.append(
                {
                    "node_index": node,
                    "cliente": clientes[node],
                    "carga_acumulada": carga,
                    "distancia_acumulada": distancia,
                    "tempo_acumulado": tempo,
                }
            )
            prev = node

        distancia_total = distancia + int(cost_matrix[prev][0])
        rotas_out.append(
            {
                "veiculo_id": int(truck.id) + 1,
                "paradas": paradas,
                "distancia_total": distancia_total,
                "carga_total": carga,
            }
        )

    excedentes_out = [clientes[int(oid)] for oid in res.dropped_orders]
    return {"rotas": rotas_out, "excedentes": excedentes_out, "objective": res.objective}
