"""
Funções de formatação e exibição de resultados.
"""

def exibir_resultado(solucao):
    """Exibe o resultado da roteirização no console."""
    
    # Exibir excedentes
    if solucao['excedentes']:
        peso_excedente = sum(c['peso'] for c in solucao['excedentes'])
        print(f"\n[ATENCAO] PEDIDOS EXCEDENTES (nao atendidos por falta de capacidade):")
        print(f"{'='*60}")
        print(f"Total: {len(solucao['excedentes'])} pedidos | {peso_excedente} kg")
        print(f"{'='*60}")
        for cli in solucao['excedentes']:
            print(f"   [X] {cli['id']} | {cli['cidade']} | {cli['peso']} kg")
        print(f"{'='*60}")
    else:
        print(f"\n[OK] Todos os pedidos foram atendidos!")
    
    # Exibir rotas
    for rota in solucao['rotas']:
        print(f"\n🚛 Veículo {rota['veiculo_id']}:")
        
        for parada in rota['paradas']:
            node = parada['node_index']
            print(
                f" - Cliente {node} | "
                f"Carga até aqui: {parada['carga_acumulada']} kg | "
                f"Dist: {parada['distancia_acumulada']} m | "
                f"Tempo: {parada['tempo_acumulado']} min"
            )
        
        print(f"   >> Distância total estimada: {rota['distancia_total']} m")
        print(f"   >> Carga total: {rota['carga_total']} kg")


def formatar_distancia(metros):
    """Formata distância em metros para exibição."""
    if metros >= 1000:
        return f"{metros / 1000:.1f} km"
    return f"{metros} m"


def formatar_tempo(minutos):
    """Formata tempo em minutos para exibição."""
    if minutos >= 60:
        horas = minutos // 60
        mins = minutos % 60
        return f"{horas}h {mins}min"
    return f"{minutos} min"
