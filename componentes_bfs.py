"""Detecção de focos de praga por BFS (componentes conexos).

Modelagem: a lavoura é um grafo em grade. Cada planta (célula) é um vértice,
duas plantas vizinhas (cima, baixo, esquerda, direita) são ligadas por uma
aresta, e o peso do vértice é a infestação medida. Um foco de praga é um
componente conexo do subgrafo formado pelos vértices com infestação >= limiar.

Este módulo também define a lavoura de exemplo 30x30 (a mesma exportada para o
Gephi) e é importado pelos demais scripts (rota_drone.py, experimentos.py etc.).

Uso:  python3 componentes_bfs.py
"""
import numpy as np
from collections import deque

# --- lavoura simulada do exemplo 30x30 (a mesma usada no Gephi) ---
N = 30                                   # a lavoura tem N x N plantas
rng = np.random.default_rng(42)          # semente fixa: resultado reprodutível
xs, ys = np.meshgrid(np.arange(N), np.arange(N))   # coordenadas de cada célula
inf = np.zeros((N, N))                   # matriz de infestação (0 = sadia, 1 = máxima)
# Cada tupla (cx, cy, r) é um foco real: gaussiana centrada em (cx, cy) com raio r.
for cx, cy, r in [(8, 8, 4), (22, 10, 3), (15, 24, 5)]:
    inf += np.exp(-((xs - cx)**2 + (ys - cy)**2) / (2 * r**2))
# Soma o ruído do sensor (desvio-padrão 0,03) e limita os valores ao intervalo [0, 1].
inf = np.clip(inf + rng.normal(0, 0.03, inf.shape), 0, 1)


def achar_focos(inf, limiar):
    """Acha os focos de praga: componentes conexos (BFS) das células com
    infestação >= limiar, com vizinhança 4-conexa (cima, baixo, esquerda, direita).

    Aceita grades retangulares (linhas x colunas).
    Devolve uma lista de focos; cada foco é uma lista de células (i, j).
    Complexidade: O(V + E).
    """
    linhas, colunas = inf.shape
    visitado = np.zeros((linhas, colunas), dtype=bool)   # marca células já exploradas
    focos = []
    # Varre a grade; cada célula infestada ainda não visitada inicia um novo foco.
    for i in range(linhas):
        for j in range(colunas):
            if inf[i, j] >= limiar and not visitado[i, j]:
                fila = deque([(i, j)])           # fila da BFS
                visitado[i, j] = True
                foco = []
                while fila:
                    a, b = fila.popleft()
                    foco.append((a, b))
                    # Visita os 4 vizinhos (arestas do grafo em grade).
                    for da, db in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                        c, d = a + da, b + db
                        # Só entra na fila se estiver dentro da grade, for
                        # infestado e ainda não tiver sido visitado.
                        if (0 <= c < linhas and 0 <= d < colunas
                                and inf[c, d] >= limiar and not visitado[c, d]):
                            visitado[c, d] = True
                            fila.append((c, d))
                focos.append(foco)
    return focos


if __name__ == "__main__":
    # Tabela: para cada limiar, quantos focos a BFS acha e quanto da lavoura
    # precisaria ser pulverizado.
    print(f"{'Limiar':>6} | {'Focos':>5} | {'Células a pulverizar':>20} | {'% da lavoura':>12}")
    print("-" * 55)
    for limiar in [0.3, 0.4, 0.5, 0.6, 0.7]:
        focos = achar_focos(inf, limiar)
        total = sum(len(f) for f in focos)
        print(f"{limiar:>6.1f} | {len(focos):>5} | {total:>20} | {100 * total / (N * N):>11.1f}%")
