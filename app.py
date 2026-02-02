from flask import Flask, request, jsonify
from services.geoloc_service import get_coordinates

from services.processamento_dados import normalizar_dados_entrada, aplicar_regras_alocacao
from services.osrm_service import criar_matriz_distancias
from services.ortools import resolver_vrp_ortools, Truck, Order, plan_day_two_stage, PedidoRealocado
import math

app = Flask(__name__)


@app.route('/health', methods=['GET'])
def health():
    return jsonify({"status": "ok"}), 200


def _as_str(v):
    if v is None:
        return ""
    return str(v)


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
        trucks.append(Truck(id=tid, cities=tuple(_as_str(c) for c in cities if _as_str(c)), capacity_kg=cap))

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
        city = _as_str(o.get('city'))
        if not oid:
            raise ValueError("Order sem 'id'")

        orders.append(
            Order(
                id=oid,
                city=city,
                weight_kg=_as_int(o.get('weight_kg'), default=0),
                x=_as_float(o.get('x'), default=0.0),
                y=_as_float(o.get('y'), default=0.0),
                alt_truck_ids=[_as_str(tid) for tid in (o.get('alt_truck_ids') or []) if _as_str(tid)],
                priority_penalty=_as_int(o.get('priority_penalty'), default=0),
            )
        )

    return orders


def _solve_payload_v2(payload):
    from config.settings import (
        OSRM_URL, 
        MAX_KM_ADICIONAL_REALOCACAO, 
        PRIORIDADE_MENOR_KM_ADICIONAL, 
        PRIORIDADE_MAIS_PERTO
    )
    
    depot = payload.get('depot')
    if not isinstance(depot, dict):
        raise ValueError("Campo 'depot' {x,y} é obrigatório")

    depot_xy = (_as_float(depot.get('x')), _as_float(depot.get('y')))
    trucks = _parse_trucks(payload)
    orders = _parse_orders(payload)

    # Criar mapa de order_id -> coordenadas para cálculo de distância
    order_coords = {o.id: (o.y, o.x) for o in orders}  # (lat, lng)
    depot_coords = (_as_float(depot.get('y')), _as_float(depot.get('x')))  # (lat, lng)

    params = payload.get('params') if isinstance(payload.get('params'), dict) else {}
    res1, res2, pedidos_realocados = plan_day_two_stage(
        orders=orders,
        trucks=trucks,
        depot_xy=depot_xy,
        open_cost_step=_as_int(params.get('open_cost_step'), default=1_000_000),
        stage1_drop_penalty=_as_int(params.get('stage1_drop_penalty'), default=50_000_000),
        stage2_drop_penalty=_as_int(params.get('stage2_drop_penalty'), default=2_000_000),
        stage2_move_penalty=_as_int(params.get('stage2_move_penalty'), default=200_000),
        time_limit_seconds=_as_int(params.get('time_limit_seconds'), default=10),
        # Novos parâmetros para realocação inteligente
        max_km_adicional_realocacao=_as_float(params.get('max_km_adicional_realocacao'), default=MAX_KM_ADICIONAL_REALOCACAO),
        prioridade_menor_km=PRIORIDADE_MENOR_KM_ADICIONAL,
        prioridade_mais_perto=PRIORIDADE_MAIS_PERTO,
        osrm_url=OSRM_URL,
        order_coords=order_coords,
        depot_coords=depot_coords,
    )

    def _calcular_distancia_rota_osrm(order_ids):
        """Calcula distância total da rota via OSRM (CD -> paradas -> CD) em km"""
        if not order_ids:
            return 0.0
        
        # Montar sequência: depósito -> paradas -> depósito
        coords = [depot_coords]
        for oid in order_ids:
            if oid in order_coords:
                coords.append(order_coords[oid])
        coords.append(depot_coords)  # Volta ao depósito
        
        if len(coords) < 2:
            return 0.0
        
        # Formatar para OSRM: lng,lat
        coords_str = ";".join([f"{lng},{lat}" for lat, lng in coords])
        url = f"http://router.project-osrm.org/route/v1/driving/{coords_str}?overview=false"
        
        try:
            import requests
            from config.settings import OSRM_URL
            # Tentar OSRM local primeiro (configuração do settings.py)
            local_url = f"{OSRM_URL}/route/v1/driving/{coords_str}?overview=false"
            try:
                r = requests.get(local_url, timeout=10)
                if r.status_code == 200:
                    data = r.json()
                    if data.get('code') == 'Ok' and data.get('routes'):
                        distance_m = data['routes'][0].get('distance', 0)
                        return distance_m / 1000.0  # metros -> km
            except Exception:
                pass  # Fallback to public OSRM
            
            # Fallback: OSRM público
            r = requests.get(url, timeout=15)
            if r.status_code == 200:
                data = r.json()
                if data.get('code') == 'Ok' and data.get('routes'):
                    distance_m = data['routes'][0].get('distance', 0)
                    return distance_m / 1000.0  # metros -> km
        except Exception as e:
            print(f"[WARN] Erro ao consultar OSRM para distância: {e}", flush=True)
        
        return 0.0

    def pack_result(res):
        # Calcular distância de cada rota
        distances_by_truck = {}
        for truck_id, order_ids in res.routes.items():
            if order_ids:
                distances_by_truck[truck_id] = _calcular_distancia_rota_osrm(order_ids)
            else:
                distances_by_truck[truck_id] = 0.0
        
        return {
            'objective': int(res.objective),
            'dropped_orders': list(res.dropped_orders),
            'served_by_truck': dict(res.served_by_truck),
            'routes': dict(res.routes),
            'loads_by_truck': dict(res.loads_by_truck),
            'distances_by_truck_km': distances_by_truck,  # NOVO: distâncias em km
        }

    # Log de caminhões utilizados e não utilizados
    trucks_usados = [tid for tid, orders in res2.routes.items() if orders]
    trucks_nao_usados = [tid for tid, orders in res2.routes.items() if not orders]
    
    print(f"\n{'='*50}", flush=True)
    print(f"📊 RESUMO DE CAMINHÕES", flush=True)
    print(f"{'='*50}", flush=True)
    print(f"✅ Utilizados ({len(trucks_usados)}): {', '.join(sorted(trucks_usados, key=lambda x: int(x) if x.isdigit() else x))}", flush=True)
    if trucks_nao_usados:
        print(f"⏸️  Não utilizados ({len(trucks_nao_usados)}): {', '.join(sorted(trucks_nao_usados, key=lambda x: int(x) if x.isdigit() else x))}", flush=True)
    print(f"{'='*50}\n", flush=True)

    # Formatar pedidos_realocados para o retorno JSON
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
        print(f"🔄 PEDIDOS REALOCADOS: {len(pedidos_realocados)}", flush=True)
        for pr in pedidos_realocados:
            print(f"   📦 {pr.id} ({pr.cidade}): Caminhão {pr.caminhao_original} → {pr.caminhao_realocado} (+{pr.distancia_adicional_km}km)", flush=True)
        print(f"{'='*50}\n", flush=True)

    return {
        'stage1': pack_result(res1), 
        'stage2': pack_result(res2),
        'pedidos_realocados': pedidos_realocados_json,
    }

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
    Endpoint principal que orquestra:
    1. Normalização dos dados
    2. Cálculo de Matriz (OSRM)
    3. Lógica de Negócio (Baldes/Caminhões)
    4. Otimização Matemática (OR-Tools)
    """
    data = request.get_json(silent=True)
    
    if not data:
        return jsonify({"erro": "Body JSON é obrigatório"}), 400

    # Suporta 2 formatos:
    # (v1 legado) { pedidos:[{lat,lng,peso,cidade}], deposito:{lat,lng}, num_caminhoes:int, preferencias_caminhoes?:{} }
    # (v2)        { orders:[...], trucks:[...], depot:{x,y}, params?:{} }
    is_v2 = isinstance(data.get('orders'), list) and isinstance(data.get('trucks'), list) and isinstance(data.get('depot'), dict)

    if not is_v2:
        # Verifica se os campos cruciais existem (v1)
        if 'pedidos' not in data or not isinstance(data['pedidos'], list):
            return jsonify({"erro": "Campo 'pedidos' é obrigatório."}), 400

        # IMPORTANTE: O algoritmo precisa saber onde começa (Depósito).
        if 'deposito' not in data:
            return jsonify({"erro": "Campo 'deposito' {lat, lng} é obrigatório para calcular a rota."}), 400

        if 'num_caminhoes' not in data:
            return jsonify({"erro": "Campo 'num_caminhoes' é obrigatório."}), 400


    try:
        if is_v2:
            print("[VRP v2] Resolvendo com payload orders/trucks...", flush=True)
            resultado_final = _solve_payload_v2(data)
            return jsonify(resultado_final), 200

        # --- FLUXO LEGADO (v1) ---
        # --- ETAPA 1: PREPARAÇÃO DOS DADOS ---
        print("[1/4] Normalizando dados...", flush=True)
        dados_norm = normalizar_dados_entrada(data)

        # --- ETAPA 2: MATRIZ DE DISTÂNCIAS (OSRM) ---
        print("[2/4] Consultando OSRM...", flush=True)
        matriz_distancias = criar_matriz_distancias(dados_norm['lista_osrm'])

        if not matriz_distancias:
            return jsonify({"erro": "Falha de comunicação com o serviço de rotas (OSRM)."}), 503

        # --- ETAPA 3: LÓGICA DE NEGÓCIO (OS BALDES) ---
        print("[3/4] Aplicando regras de alocacao e prioridade...", flush=True)
        logica_negocio = aplicar_regras_alocacao(dados_norm)

        print(f"   -> Fixos: {len(logica_negocio['pre_alocacoes'])} | Excedentes: {len(logica_negocio['excedentes'])}", flush=True)

        # --- ETAPA 4: SOLVER (OR-TOOLS) ---
        print("[4/4] Otimizando rotas com OR-Tools...", flush=True)
        resultado_final = resolver_vrp_ortools(dados_norm, logica_negocio, matriz_distancias)

        if not resultado_final:
            return jsonify({"atencao": "O algoritmo rodou mas não encontrou uma solução viável (verifique restrições)."}), 422

        return jsonify(resultado_final), 200

    except ValueError as e:
        # Erros de validação/regra de negócio (ex.: cidade fora das rotas do dia)
        print(f"[VALIDACAO] {e}")
        return jsonify({"erro": "Dados inválidos para roteirização.", "detalhes": str(e)}), 422
    except Exception as e:
        print(f"[ERRO] Erro interno: {e}")
        return jsonify({"erro": "Erro interno no servidor ao processar rotas.", "detalhes": str(e)}), 500


if __name__ == '__main__':
    app.run(debug=True, port=4003, host='0.0.0.0')