# =============================================================================
# 1. CONFIGURAÇÕES FÍSICAS (Limites Rígidos)
# =============================================================================

# Capacidade máxima de carga (usado no processamento_dados.py e solver_ortools.py)
CAPACIDADE_VEICULO_KG = 19000 

# =============================================================================
# 2. TUNING DO OR-TOOLS (Ajuste Fino do Solver)
# =============================================================================

# Penalidade para "Dropped Nodes" (Pedidos não entregues).
# Deve ser um número MUITO alto (maior que qualquer distância em metros possível).
PENALIDADE_DE_DROP = 10_000_000

# Tempo máximo do solver principal (em segundos)
TEMPO_LIMITE_SOLVER_SEGUNDOS = 10

# Custo para "abrir" caminhão adicional na mesma rota (prioriza preencher o 1º)
OPEN_COST_STEP = 1_000_000

# Penalidades por estágio de resolução
STAGE1_DROP_PENALTY = 50_000_000   # Altíssimo: só vira excedente se NÃO couber
STAGE2_DROP_PENALTY = 2_000_000    # Custo de manter como excedente final
STAGE2_MOVE_PENALTY = 200_000      # Custo operacional por remanejamento fora da base

# Tempo limite do TSP para reordenação de rotas após realocação (segundos)
TSP_TIME_LIMIT_SECONDS = 5

# Estratégia de solução inicial do OR-Tools
# Opções: PATH_CHEAPEST_ARC, PATH_MOST_CONSTRAINED_ARC, SAVINGS, CHRISTOFIDES
FIRST_SOLUTION_STRATEGY = 'PATH_CHEAPEST_ARC'

# Metaheurística de busca local do OR-Tools
# Opções: GUIDED_LOCAL_SEARCH, SIMULATED_ANNEALING, TABU_SEARCH
METODO_BUSCA_LOCAL = 'GUIDED_LOCAL_SEARCH'

# =============================================================================
# 3. REALOCAÇÃO DE EXCEDENTES
# =============================================================================

# Máximo de KM adicionais permitidos ao realocar um excedente para outro caminhão
MAX_KM_ADICIONAL_REALOCACAO = 150

# Critério de prioridade para escolher qual caminhão recebe o excedente
# (apenas um deve ser True por vez)
PRIORIDADE_MENOR_KM_ADICIONAL = True   # Escolhe o caminhão que resulta em menor km adicional
PRIORIDADE_MAIS_PERTO = False          # Escolhe o caminhão mais próximo do pedido

# =============================================================================
# 4. CONFIGURAÇÕES DO OSRM (Serviço de Mapas)
# =============================================================================

OSRM_URL = "http://192.168.20.17:4000"

# Timeout para a requisição HTTP (evita que o sistema trave se o OSRM cair)
OSRM_TIMEOUT_SECONDS = 240

# Tamanho do lote para cálculo de matriz (para não estourar a URL GET)
# Se tiver 100 pedidos, ele quebra em pedaços de 50x50.
OSRM_BATCH_SIZE = 50

# =============================================================================
# 5. MODO CLI (main.py) + MAPA
# =============================================================================

# Depósito padrão usado no modo CLI e no mapa.
# Ajuste para a sua base real.
DEPOSITO = {
	"id": "DEPÓSITO",
	"lat": -22.9050824,
	"lon": -47.0613327,
	"peso": 0,
	"cidade": "DEPÓSITO",
}

# Quantidade de veículos padrão do modo CLI (main.py)
NUM_CAMINHOES_PADRAO = 6

# Cores aceitas pelo folium.Icon(color=...)
CORES_VEICULOS = [
	"red",
	"blue",
	"green",
	"purple",
	"orange",
	"darkred",
	"darkblue",
	"darkgreen",
]

# Offset lateral (em metros) para desenhar rotas sobrepostas no mapa
OFFSET_ROTAS_METROS = 50

