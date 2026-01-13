"""
Serviço de geração de mapas com Folium.
Responsável por criar visualizações das rotas.
"""

import math
import folium
from folium.plugins import AntPath

from config.settings import CORES_VEICULOS, OFFSET_ROTAS_METROS, DEPOSITO
from services.osrm_service import obter_geometria_rota


def gerar_mapa_html(solucao, clientes, arquivo_saida="mapa_osrm.html"):
    """
    Gera um mapa HTML com as rotas dos veículos.
    """

    print("\n--- DESENHANDO ROTAS NAS RUAS (OSRM) ---")
    
    # Criar mapa centrado no depósito
    mapa = folium.Map(
        location=[DEPOSITO['lat'], DEPOSITO['lon']], 
        zoom_start=9
    )
    num_veiculos = len(solucao['rotas'])
    
    for rota in solucao['rotas']:
        vehicle_id = rota['veiculo_id'] - 1
        cor = CORES_VEICULOS[vehicle_id % len(CORES_VEICULOS)]
        
        print(f"\n[VEICULO] {rota['veiculo_id']} (Cor: {cor}):")
        
        # Lista de coordenadas para traçar a rota
        paradas_coords = []
        sequencia = 0
        
        for parada in rota['paradas']:
            cli = parada['cliente']
            paradas_coords.append([cli['lat'], cli['lon']])
            
            # Adicionar marcador
            node_index = parada['node_index']
            texto_tooltip = "DEPÓSITO" if node_index == 0 else f"{sequencia}ª: {cli['cidade']}"
            icone = 'home' if node_index == 0 else 'info-sign'
            
            folium.Marker(
                [cli['lat'], cli['lon']],
                popup=f"<b>{cli['id']}</b><br>Peso: {cli['peso']}kg",
                tooltip=texto_tooltip,
                icon=folium.Icon(color=cor, icon=icone)
            ).add_to(mapa)
            
            if node_index != 0:
                print(f"   {sequencia}. {cli['cidade']}")
                sequencia += 1
        
        # Adicionar retorno ao depósito
        paradas_coords.append([DEPOSITO['lat'], DEPOSITO['lon']])
        
        # Obter geometria detalhada
        print("   [OSRM] Baixando tracado das ruas...")
        trajeto = obter_geometria_rota(paradas_coords)
        
        # Aplicar offset para evitar sobreposição
        trajeto_com_offset = _aplicar_offset_lateral(
            trajeto, OFFSET_ROTAS_METROS, vehicle_id, num_veiculos
        )
        
        print(f"   [OK] Geometria OSRM baixada ({len(trajeto_com_offset)} pontos)")
        
        # Desenhar rota animada
        AntPath(
            locations=trajeto_com_offset,
            color=cor,
            weight=5,
            delay=800,
            dash_array=[10, 20],
            pulse_color='#FFFFFF',
            opacity=0.8,
            tooltip=f"Rota Veículo {rota['veiculo_id']}"
        ).add_to(mapa)
        
        print(f"   [OK] Rota desenhada (Carga: {rota['carga_total']}kg)")
    
    for cli in solucao['excedentes']:
        folium.Marker(
            [cli['lat'], cli['lon']],
            popup=f"<b>❌ NÃO ATENDIDO</b><br>{cli['id']}<br>Peso: {cli['peso']}kg",
            tooltip=f"EXCEDENTE: {cli['cidade']}",
            icon=folium.Icon(color='gray', icon='remove')
        ).add_to(mapa)
    
    mapa.save(arquivo_saida)
    print(f"\n[OK] Mapa gerado: '{arquivo_saida}'")


def _aplicar_offset_lateral(coordenadas, offset_metros, vehicle_index, num_vehicles):
    """
    Aplica deslocamento lateral perpendicular à direção da rota.
    Evita que rotas sobrepostas fiquem uma em cima da outra.
    """
    if len(coordenadas) < 2:
        return coordenadas
    
    offset_normalizado = (vehicle_index - (num_vehicles - 1) / 2) * offset_metros
    METROS_POR_GRAU_LAT = 111320
    
    coordenadas_deslocadas = []
    
    for i, (lat, lon) in enumerate(coordenadas):
        # Calcular direção do segmento
        if i == 0:
            lat_next, lon_next = coordenadas[i + 1]
            dlat = lat_next - lat
            dlon = lon_next - lon
        elif i == len(coordenadas) - 1:
            lat_prev, lon_prev = coordenadas[i - 1]
            dlat = lat - lat_prev
            dlon = lon - lon_prev
        else:
            lat_prev, lon_prev = coordenadas[i - 1]
            lat_next, lon_next = coordenadas[i + 1]
            dlat = (lat_next - lat_prev) / 2
            dlon = (lon_next - lon_prev) / 2
        
        comprimento = math.sqrt(dlat**2 + dlon**2)
        
        if comprimento < 1e-10:
            coordenadas_deslocadas.append([lat, lon])
            continue
        
        # Vetor perpendicular
        dlat_norm = dlat / comprimento
        dlon_norm = dlon / comprimento
        perp_lat = -dlon_norm
        perp_lon = dlat_norm
        
        # Converter offset para graus
        offset_lat = (offset_normalizado * perp_lat) / METROS_POR_GRAU_LAT
        metros_por_grau_lon = METROS_POR_GRAU_LAT * math.cos(math.radians(lat))
        offset_lon = (offset_normalizado * perp_lon) / metros_por_grau_lon if metros_por_grau_lon > 0 else 0
        
        coordenadas_deslocadas.append([lat + offset_lat, lon + offset_lon])
    
    return coordenadas_deslocadas
