from flask import Flask, request, jsonify
from services.geoloc_service import get_coordinates
from services.processamento_dados import limpar_string_blindada
from services.ortools import Truck, Order, plan_day_two_stage, PedidoRealocado
import math
import requests

app = Flask(__name__)


@app.route('/health', methods=['GET'])
def health():
    return jsonify({"status": "ok"}), 200


def _as_str(v):
    if v is None:
        return ""
    return str(v)


def _normalize_city(city_name):
    """
    Normaliza nome de cidade para comparação consistente.
    Usa a mesma função da v1 para garantir consistência.
    """
    return limpar_string_blindada(city_name) if city_name else ""


def _as_int(v, default=0):
    try:
        return int(round(float(v)))
    except Exception:
        return default


def _as_float(v, default=0.0):
    try:
        return float(v)
    except Exception:
        return default


def _validate_coord(lat, lng, label=""):
    """Valida que lat está entre -90/90 e lng entre -180/180."""
    if not (-90 <= lat <= 90):
        raise ValueError(f"Latitude inválida ({lat}) {label}. Deve estar entre -90 e 90.")
    if not (-180 <= lng <= 180):
        raise ValueError(f"Longitude inválida ({lng}) {label}. Deve estar entre -180 e 180.")
    if lat == 0.0 and lng == 0.0:
        raise ValueError(f"Coordenadas zeradas (0,0) {label}. Verifique lat/lng.")


def _parse_trucks(payload):
    trucks_in = payload.get('trucks')
    if not isinstance(trucks_in, list) or not trucks_in:
        raise ValueError("Campo 'trucks' deve ser uma lista não vazia")

    trucks = []
    for t in trucks_in:
        if not isinstance(t, dict):
            raise ValueError("Cada item de 'trucks' deve ser um objeto")

        tid = _as_str(t.get('id'))
        cities = t.get('cities')
        if not tid:
            raise ValueError("Truck sem 'id'")
        if not isinstance(cities, list):
            raise ValueError(f"Truck '{tid}' com 'cities' inválido (esperado lista)")

        cap = _as_int(t.get('capacity_kg'), default=19000)
        # Normaliza as cidades para garantir comparação consistente
        normalized_cities = tuple(_normalize_city(c) for c in cities if _as_str(c))
        trucks.append(Truck(id=tid, cities=normalized_cities, capacity_kg=cap))

    return trucks


def _parse_orders(payload):
    orders_in = payload.get('orders')
    if not isinstance(orders_in, list) or not orders_in:
        raise ValueError("Campo 'orders' deve ser uma lista não vazia")

    orders = []
    for o in orders_in:
        if not isinstance(o, dict):
            raise ValueError("Cada item de 'orders' deve ser um objeto")

        oid = _as_str(o.get('id'))
        city = _normalize_city(o.get('city'))  # Normaliza para comparação consistente
        if not oid:
            raise ValueError("Order sem 'id'")

        ox = _as_float(o.get('x'), default=0.0)
        oy = _as_float(o.get('y'), default=0.0)
        _validate_coord(oy, ox, label=f"no pedido '{oid}'")

        orders.append(
            Order(
                id=oid,
                city=city,
                weight_kg=_as_int(o.get('weight_kg'), default=0),
                x=ox,
                y=oy,
                alt_truck_ids=[_as_str(tid) for tid in (o.get('alt_truck_ids') or []) if _as_str(tid)],
                priority_penalty=_as_int(o.get('priority_penalty'), default=0),
            )
        )

    return orders


def _param_int(params, key):
    """Retorna valor do param como int, ou None se não fornecido."""
    val = params.get(key)
    if val is None:
        return None
    try:
        return int(round(float(val)))
    except Exception:
        return None


def _param_float(params, key):
    """Retorna valor do param como float, ou None se não fornecido."""
    val = params.get(key)
    if val is None:
        return None
    try:
        return float(val)
    except Exception:
        return None


def _calcular_distancia_rota_osrm(order_ids, order_coords, depot_coords):
    """Calcula distância total da rota via OSRM (CD -> paradas -> CD) em km."""
    if not order_ids:
        return 0.0

    from config.settings import OSRM_URL

    coords = [depot_coords]
    for oid in order_ids:
        if oid in order_coords:
            coords.append(order_coords[oid])
    coords.append(depot_coords)

    if len(coords) < 3:
        return 0.0

    coords_str = ";".join([f"{lng},{lat}" for lat, lng in coords])

    try:
        url = f"{OSRM_URL}/route/v1/driving/{coords_str}?overview=false"
        r = requests.get(url, timeout=15)
        if r.status_code == 200:
            data = r.json()
            if data.get('code') == 'Ok' and data.get('routes'):
                return data['routes'][0].get('distance', 0) / 1000.0
    except Exception as e:
        print(f"[WARN] Erro ao consultar OSRM para distância: {e}", flush=True)

    return 0.0


@app.route('/geoloc', methods=['POST'])
def get_geolocation():
    data = request.get_json()
    if not data:
        return jsonify({"erro": "Body JSON é obrigatório"}), 400
    
    rua = data.get('rua')
    cidade = data.get('cidade')
    
    if not rua or not cidade:
        return jsonify({"erro": "Campos 'rua' e 'cidade' são obrigatórios"}), 400
    
    try:
        resultado = get_coordinates(rua, cidade)
        if resultado:
            return jsonify(resultado)
        else:
            return jsonify({"erro": "Endereço não encontrado"}), 404
    except Exception as e:
        return jsonify({"erro": str(e)}), 500


@app.route('/rotas/redespacho', methods=['POST'])
def ordenar_redespacho():
    """
    Endpoint para ordenação de entregas por proximidade.
    Recebe uma lista de pontos com código, latitude e longitude.
    Retorna os códigos ordenados por proximidade, partindo de Mococa.
    
    Body esperado:
    {
        "pontos": [
            {"codigo": "ID1", "latitude": -22.123, "longitude": -47.456},
            {"codigo": "ID2", "latitude": -22.234, "longitude": -47.567},
            ...
        ]
    }
    """
    data = request.get_json(silent=True)
    
    if not data:
        return jsonify({"erro": "Body JSON é obrigatório"}), 400
    
    pontos = data.get('pontos')
    if not isinstance(pontos, list) or not pontos:
        return jsonify({"erro": "Campo 'pontos' deve ser uma lista não vazia"}), 400
    
    # Validar estrutura dos pontos
    for i, ponto in enumerate(pontos):
        if not isinstance(ponto, dict):
            return jsonify({"erro": f"Ponto {i} deve ser um objeto"}), 400
        if 'codigo' not in ponto:
            return jsonify({"erro": f"Ponto {i} sem campo 'codigo'"}), 400
        if 'latitude' not in ponto or 'longitude' not in ponto:
            return jsonify({"erro": f"Ponto {i} (código: {ponto.get('codigo')}) sem 'latitude' ou 'longitude'"}), 400
        
        try:
            float(ponto['latitude'])
            float(ponto['longitude'])
        except (ValueError, TypeError):
            return jsonify({"erro": f"Coordenadas inválidas no ponto {ponto.get('codigo')}"}), 400
    
    try:
        # Coordenadas de Mococa (ponto de partida/depósito)
        from config.settings import DEPOSITO
        mococa_lat = DEPOSITO['lat']
        mococa_lng = DEPOSITO['lon']
        
        def calcular_distancia(lat1, lng1, lat2, lng2):
            """Calcula distância euclidiana aproximada em km entre duas coordenadas."""
            # Aproximação simples: 1 grau ≈ 111km
            dlat = (lat2 - lat1) * 111.0
            dlng = (lng2 - lng1) * 111.0 * math.cos(math.radians((lat1 + lat2) / 2))
            return math.sqrt(dlat**2 + dlng**2)
        
        # Algoritmo Nearest Neighbor para ordenação
        pontos_restantes = pontos.copy()
        ordem_entrega = []
        distancias_acumuladas = []
        
        # Posição atual começa em Mococa
        lat_atual = mococa_lat
        lng_atual = mococa_lng
        distancia_total = 0.0
        
        print(f"\n{'='*60}", flush=True)
        print(f"🚚 ORDENAÇÃO DE REDESPACHO - {len(pontos)} pontos", flush=True)
        print(f"📍 Partindo de: Mococa (lat={mococa_lat}, lng={mococa_lng})", flush=True)
        print(f"{'='*60}\n", flush=True)
        
        # Enquanto houver pontos para visitar
        while pontos_restantes:
            # Encontrar o ponto mais próximo da posição atual
            ponto_mais_proximo = None
            menor_distancia = float('inf')
            
            for ponto in pontos_restantes:
                lat = float(ponto['latitude'])
                lng = float(ponto['longitude'])
                distancia = calcular_distancia(lat_atual, lng_atual, lat, lng)
                
                if distancia < menor_distancia:
                    menor_distancia = distancia
                    ponto_mais_proximo = ponto
            
            # Adicionar o ponto mais próximo à ordem de entrega
            ordem_entrega.append(ponto_mais_proximo['codigo'])
            distancia_total += menor_distancia
            distancias_acumuladas.append(round(distancia_total, 2))
            
            print(f"   {len(ordem_entrega)}. {ponto_mais_proximo['codigo']} (+{menor_distancia:.2f}km) - Total: {distancia_total:.2f}km", flush=True)
            
            # Atualizar posição atual
            lat_atual = float(ponto_mais_proximo['latitude'])
            lng_atual = float(ponto_mais_proximo['longitude'])
            
            # Remover ponto visitado
            pontos_restantes.remove(ponto_mais_proximo)
        
        # Calcular distância de volta para Mococa (opcional, para informação)
        distancia_retorno = calcular_distancia(lat_atual, lng_atual, mococa_lat, mococa_lng)
        distancia_total_com_retorno = distancia_total + distancia_retorno
        
        print(f"\n{'='*60}", flush=True)
        print(f"✅ Ordenação concluída!", flush=True)
        print(f"📏 Distância total (sem retorno): {distancia_total:.2f}km", flush=True)
        print(f"🔄 Distância de retorno a Mococa: {distancia_retorno:.2f}km", flush=True)
        print(f"📐 Distância total (com retorno): {distancia_total_com_retorno:.2f}km", flush=True)
        print(f"{'='*60}\n", flush=True)
        
        return jsonify({
            "sucesso": True,
            "ordem_entrega": ordem_entrega
        }), 200
        
    except Exception as e:
        print(f"[ERRO] Erro ao processar redespacho: {e}", flush=True)
        return jsonify({"erro": "Erro interno ao processar ordenação", "detalhes": str(e)}), 500


@app.route('/rotas', methods=['POST'])
def calcular_rotas():
    """
    Endpoint principal de roteirização.

    Input:
    {
        "orders": [{"id", "city", "weight_kg", "x", "y", "alt_truck_ids?", "priority_penalty?"}],
        "trucks": [{"id", "cities": [], "capacity_kg?"}],
        "depot": {"x", "y"},
        "params": { overrides opcionais do settings.py }
    }
    """
    data = request.get_json(silent=True)

    if not data:
        return jsonify({"erro": "Body JSON é obrigatório"}), 400

    if not isinstance(data.get('orders'), list):
        return jsonify({"erro": "Campo 'orders' é obrigatório (lista)."}), 400
    if not isinstance(data.get('trucks'), list):
        return jsonify({"erro": "Campo 'trucks' é obrigatório (lista)."}), 400
    if not isinstance(data.get('depot'), dict):
        return jsonify({"erro": "Campo 'depot' {x,y} é obrigatório."}), 400

    try:
        depot = data['depot']
        depot_xy = (_as_float(depot.get('x')), _as_float(depot.get('y')))
        depot_coords = (_as_float(depot.get('y')), _as_float(depot.get('x')))  # (lat, lng)
        _validate_coord(depot_coords[0], depot_coords[1], label="no depot")

        trucks = _parse_trucks(data)
        orders = _parse_orders(data)
        order_coords = {o.id: (o.y, o.x) for o in orders}  # (lat, lng)

        params = data.get('params') if isinstance(data.get('params'), dict) else {}

        print("[VRP] Resolvendo roteirização...", flush=True)
        res1, res2, pedidos_realocados = plan_day_two_stage(
            orders=orders,
            trucks=trucks,
            depot_xy=depot_xy,
            depot_coords=depot_coords,
            order_coords=order_coords,
            open_cost_step=_param_int(params, 'open_cost_step'),
            stage1_drop_penalty=_param_int(params, 'stage1_drop_penalty'),
            stage2_drop_penalty=_param_int(params, 'stage2_drop_penalty'),
            stage2_move_penalty=_param_int(params, 'stage2_move_penalty'),
            time_limit_seconds=_param_int(params, 'time_limit_seconds'),
            max_km_adicional_realocacao=_param_float(params, 'max_km_adicional_realocacao'),
        )

        # Monta resultado no formato de saída
        def pack_result(res):
            distances_by_truck = {}
            for truck_id, oids in res.routes.items():
                distances_by_truck[truck_id] = (
                    _calcular_distancia_rota_osrm(oids, order_coords, depot_coords) if oids else 0.0
                )
            return {
                'objective': int(res.objective),
                'dropped_orders': list(res.dropped_orders),
                'served_by_truck': dict(res.served_by_truck),
                'routes': dict(res.routes),
                'loads_by_truck': dict(res.loads_by_truck),
                'distances_by_truck_km': distances_by_truck,
            }

        # Log de caminhões
        trucks_usados = [tid for tid, oids in res2.routes.items() if oids]
        trucks_nao_usados = [tid for tid, oids in res2.routes.items() if not oids]

        print(f"\n{'='*50}", flush=True)
        print(f"RESUMO DE CAMINHOES", flush=True)
        print(f"{'='*50}", flush=True)
        print(f"Utilizados ({len(trucks_usados)}): {', '.join(sorted(trucks_usados, key=lambda x: int(x) if x.isdigit() else x))}", flush=True)
        if trucks_nao_usados:
            print(f"Nao utilizados ({len(trucks_nao_usados)}): {', '.join(sorted(trucks_nao_usados, key=lambda x: int(x) if x.isdigit() else x))}", flush=True)
        print(f"{'='*50}\n", flush=True)

        # Formatar realocações
        pedidos_realocados_json = [
            {
                'id': pr.id,
                'cidade': pr.cidade,
                'caminhao_original': pr.caminhao_original,
                'caminhao_realocado': pr.caminhao_realocado,
                'distancia_adicional_km': pr.distancia_adicional_km,
                'peso_kg': pr.peso_kg,
                'motivo': pr.motivo,
            }
            for pr in pedidos_realocados
        ]

        if pedidos_realocados:
            print(f"\n{'='*50}", flush=True)
            print(f"PEDIDOS REALOCADOS: {len(pedidos_realocados)}", flush=True)
            for pr in pedidos_realocados:
                print(f"   {pr.id} ({pr.cidade}): Caminhao {pr.caminhao_original} -> {pr.caminhao_realocado} (+{pr.distancia_adicional_km}km)", flush=True)
            print(f"{'='*50}\n", flush=True)

        return jsonify({
            'stage1': pack_result(res1),
            'stage2': pack_result(res2),
            'pedidos_realocados': pedidos_realocados_json,
        }), 200

    except ValueError as e:
        print(f"[VALIDACAO] {e}")
        return jsonify({"erro": "Dados inválidos para roteirização.", "detalhes": str(e)}), 422
    except Exception as e:
        print(f"[ERRO] Erro interno: {e}")
        return jsonify({"erro": "Erro interno no servidor ao processar rotas.", "detalhes": str(e)}), 500


if __name__ == '__main__':
    app.run(debug=True, port=4003, host='0.0.0.0')