import requests
from config.settings import ( OSRM_URL, OSRM_BATCH_SIZE )



# Limite de pontos por requisição (para evitar URL muito longa)
MAX_PONTOS_POR_REQUISICAO = OSRM_BATCH_SIZE


def _consultar_osrm_tabela(lista_clientes, sources=None, destinations=None):
    """
    Faz uma consulta ao OSRM table API.
    
    Args:
        lista_clientes: Lista de dicts com 'lat' e 'lon'
        sources: Índices dos pontos de origem (opcional)
        destinations: Índices dos pontos de destino (opcional)
    
    Returns:
        Dados da resposta ou None se falhar
    """
    coordenadas_formatadas = [f"{c['lon']},{c['lat']}" for c in lista_clientes]
    string_coords = ";".join(coordenadas_formatadas)
    
    url = f"{OSRM_URL}/table/v1/driving/{string_coords}?annotations=distance"
    
    # Adiciona sources e destinations se especificados
    if sources is not None:
        url += f"&sources={';'.join(map(str, sources))}"
    if destinations is not None:
        url += f"&destinations={';'.join(map(str, destinations))}"
    
    try:
        resposta = requests.get(url, timeout=120)
        
        if resposta.status_code == 200:
            dados = resposta.json()
            if dados.get('code') == 'Ok':
                return dados
            else:
                print(f"[ERRO] OSRM: {dados.get('code')} - {dados.get('message', '')}")
                return None
        else:
            print(f"[ERRO] OSRM status: {resposta.status_code}")
            print(f"   Resposta: {resposta.text[:500]}")
            return None
            
    except requests.exceptions.Timeout:
        print("[ERRO] Timeout na conexao com OSRM")
        return None
    except Exception as e:
        print(f"[ERRO] Conexao com OSRM: {e}")
        return None


def _calcular_fallback_distancia(lista_clientes, i, j):
    """
    Calcula distância euclidiana aproximada (em metros) entre dois pontos
    para usar como fallback quando o OSRM retorna None.
    """
    import math
    lat1, lon1 = float(lista_clientes[i]['lat']), float(lista_clientes[i]['lon'])
    lat2, lon2 = float(lista_clientes[j]['lat']), float(lista_clientes[j]['lon'])
    dlat = (lat2 - lat1) * 111_320
    dlng = (lon2 - lon1) * 111_320 * math.cos(math.radians((lat1 + lat2) / 2))
    return int(math.sqrt(dlat**2 + dlng**2) * 1.4)  # Fator 1.4 para compensar ruas não retas


def criar_matriz_distancias(lista_clientes):
    """
    Consulta o OSRM para calcular a matriz de distâncias entre todos os pontos.
    Se houver muitos pontos, divide em lotes e monta a matriz completa.
    
    Args:
        lista_clientes: Lista de dicts com 'lat' e 'lon'
        
    Returns:
        Matriz NxN de distâncias em metros, ou None se falhar
    """
    n = len(lista_clientes)
    print(f"[OSRM] Consultando servidor local... ({n} pontos)")
    
    # Se poucos pontos, faz consulta direta
    if n <= MAX_PONTOS_POR_REQUISICAO:
        dados = _consultar_osrm_tabela(lista_clientes)
        if dados is None:
            return None
        
        matriz_final = []
        for i, linha in enumerate(dados['distances']):
            linha_int = []
            for j, val in enumerate(linha):
                if val is not None:
                    linha_int.append(int(val))
                else:
                    linha_int.append(_calcular_fallback_distancia(lista_clientes, i, j))
            matriz_final.append(linha_int)
        
        print(f"[OK] Matriz calculada via OSRM ({n}x{n})")
        return matriz_final
    
    # Se muitos pontos, divide em lotes
    print(f"[OSRM] Dividindo em lotes (limite: {MAX_PONTOS_POR_REQUISICAO} pontos por requisicao)...")
    
    # Inicializa matriz NxN vazia (será preenchida com dados reais)
    matriz_final = [[0 for _ in range(n)] for _ in range(n)]
    
    # Calcula quantos lotes precisamos
    num_lotes = (n + MAX_PONTOS_POR_REQUISICAO - 1) // MAX_PONTOS_POR_REQUISICAO
    total_requisicoes = num_lotes * num_lotes
    requisicao_atual = 0
    
    # Itera sobre blocos de origem e destino
    for i_lote in range(num_lotes):
        inicio_i = i_lote * MAX_PONTOS_POR_REQUISICAO
        fim_i = min((i_lote + 1) * MAX_PONTOS_POR_REQUISICAO, n)
        indices_origem = list(range(inicio_i, fim_i))
        
        for j_lote in range(num_lotes):
            inicio_j = j_lote * MAX_PONTOS_POR_REQUISICAO
            fim_j = min((j_lote + 1) * MAX_PONTOS_POR_REQUISICAO, n)
            indices_destino = list(range(inicio_j, fim_j))
            
            requisicao_atual += 1
            print(f"   [OSRM] Lote {requisicao_atual}/{total_requisicoes}: origens [{inicio_i}-{fim_i-1}] x destinos [{inicio_j}-{fim_j-1}]")
            
            # Faz consulta com sources e destinations
            dados = _consultar_osrm_tabela(
                lista_clientes,
                sources=indices_origem,
                destinations=indices_destino
            )
            
            if dados is None:
                print(f"[ERRO] Falha no lote {requisicao_atual}")
                return None
            
            # Preenche a parte correspondente da matriz
            for idx_i, i in enumerate(indices_origem):
                for idx_j, j in enumerate(indices_destino):
                    val = dados['distances'][idx_i][idx_j]
                    if val is not None:
                        matriz_final[i][j] = int(val)
                    else:
                        matriz_final[i][j] = _calcular_fallback_distancia(lista_clientes, i, j)
    
    print(f"[OK] Matriz completa via OSRM ({n}x{n})")
    return matriz_final


def obter_geometria_rota(lista_coordenadas):
    """
    Obtém a geometria detalhada de uma rota via OSRM.
    
    Args:
        lista_coordenadas: Lista de [lat, lon] na ordem da rota
        
    Returns:
        Lista de [lat, lon] com o traçado detalhado nas ruas
    """
    coords_str = ";".join([f"{lon},{lat}" for lat, lon in lista_coordenadas])
    url = f"{OSRM_URL}/route/v1/driving/{coords_str}?overview=full&geometries=geojson"
    
    try:
        r = requests.get(url, timeout=30)
        
        if r.status_code != 200:
            print(f"[ERRO] OSRM status {r.status_code}")
            return lista_coordenadas
            
        dados = r.json()
        
        if 'routes' not in dados or not dados['routes']:
            print("[ERRO] OSRM retornou vazio (nao achou caminho entre os pontos).")
            return lista_coordenadas

        geometria_osrm = dados['routes'][0]['geometry']['coordinates']
        caminho_detalhado = [[lat, lon] for lon, lat in geometria_osrm]
        
        return caminho_detalhado

    except Exception as e:
        print(f"[ERRO] Conexao: {e}")
        return lista_coordenadas
