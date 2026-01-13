import sys
from unidecode import unidecode
from config.settings import CAPACIDADE_VEICULO_KG

def limpar_string_blindada(texto):
    """
    Remove acentos, espaços extras e coloca em maiúsculo.
    """
    if not isinstance(texto, str): return str(texto)
    
    texto_sem_acento = unidecode(texto)
    
    texto_limpo = " ".join(texto_sem_acento.upper().split())
    
    return texto_limpo

def normalizar_dados_entrada(json_input):
    deposito = json_input['deposito']
    pedidos = json_input['pedidos']

    lista_para_osrm = [{'lat': deposito['lat'], 'lon': deposito['lng']}]
    demandas = [0] 
    pedidos_metadata = []

    print(f"\n[DEBUG] Analisando {len(pedidos)} pedidos...", flush=True)

    for i, p in enumerate(pedidos):
        try: peso_float = float(p['peso'])
        except: peso_float = 0.0

        cidade_crua = p.get('cidade', 'DESCONHECIDA')
        cidade_limpa = limpar_string_blindada(cidade_crua)

        lista_para_osrm.append({'lat': p['lat'], 'lon': p['lng']})
        demandas.append(peso_float)
        
        pedidos_metadata.append({
            'or_tools_index': i + 1,
            'cidade': cidade_limpa,
            'cidade_original': cidade_crua,
            'peso': peso_float,
            'original_data': p,
            'alocado': False
        })
    
    # Limpa configurações dos caminhões também
    prefs_limpas = {}
    raw_prefs = json_input.get('preferencias_caminhoes', {})
    
    for k, v in raw_prefs.items():
        cidades_sujas = v.get('cidades', [])
        cidades_limpas = [limpar_string_blindada(c) for c in cidades_sujas]
        
        prefs_limpas[str(k)] = {
            'cidades': cidades_limpas,
            'prioridade': v.get('prioridade', 1)
        }

    return {
        "lista_osrm": lista_para_osrm,
        "demandas": demandas,
        "pedidos_metadata": pedidos_metadata,
        "num_caminhoes": json_input.get('num_caminhoes', 0),
        "prefs": prefs_limpas
    }

def aplicar_regras_alocacao(dados_normalizados):
    pedidos = dados_normalizados['pedidos_metadata']
    num_caminhoes = dados_normalizados['num_caminhoes']
    preferencias = dados_normalizados['prefs']
    
    # Agrupa por cidade LIMPA
    mapa_cidades = {}
    for p in pedidos:
        c = p['cidade']
        if c not in mapa_cidades: mapa_cidades[c] = []
        mapa_cidades[c].append(p)

    pre_alocacoes = {} 
    peso_caminhoes = {i: 0 for i in range(num_caminhoes)}

    print("\n" + "="*50, flush=True)
    print("[DIAGNOSTICO] Detalhado (AGORA VAI!)", flush=True)
    print("="*50, flush=True)

    total_pedidos_fixos = 0

    for caminhao_id in range(num_caminhoes):
        str_id = str(caminhao_id)
        if str_id not in preferencias: continue

        config = preferencias[str_id]
        cidades_alvo = config.get('cidades', [])
        
        print(f"\n[CAMINHAO] {caminhao_id} configurado para: {cidades_alvo}", flush=True)

        for cidade in cidades_alvo:
            if cidade not in mapa_cidades:
                print(f"   [ERRO] Não encontrou pedidos para '{cidade}'", flush=True)
                
                # Tenta ajudar a achar o erro
                parecidas = [k for k in mapa_cidades.keys() if cidade[:3] in k]
                if parecidas:
                    print(f"      (Dica: Existem pedidos para {parecidas[0:3]}...)", flush=True)
                continue
            
            disponiveis = [p for p in mapa_cidades[cidade] if not p['alocado']]
            
            if not disponiveis:
                print(f"   [INFO] Cidade '{cidade}' existe mas já foi esvaziada.", flush=True)
                continue

            count = 0
            for pedido in disponiveis:
                if peso_caminhoes[caminhao_id] + pedido['peso'] <= CAPACIDADE_VEICULO_KG:
                    peso_caminhoes[caminhao_id] += pedido['peso']
                    pedido['alocado'] = True
                    pre_alocacoes[pedido['or_tools_index']] = caminhao_id
                    count += 1
            
            if count > 0:
                print(f"   [OK] Alocou {count} pedidos de '{cidade}'. Carga: {peso_caminhoes[caminhao_id]:.0f}kg", flush=True)
                total_pedidos_fixos += count
            else:
                print(f"   [ATENCAO] Encontrou pedidos em '{cidade}' mas CAMINHAO LOTOU!", flush=True)

    print("\n" + "="*50, flush=True)
    
    indices_excedentes = [p['or_tools_index'] for p in pedidos if not p['alocado']]

    return {
        "pre_alocacoes": pre_alocacoes,
        "excedentes": indices_excedentes,
        "pesos_finais": peso_caminhoes
    }