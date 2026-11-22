"""Exporta a lavoura de exemplo 30x30 para o Gephi em CSV (nodes.csv e edges.csv).

Cada planta vira um nó (com a infestação medida) e cada par de plantas vizinhas
vira uma aresta não direcionada. A lavoura é a mesma de componentes_bfs.py.

Uso:  python3 gerar_lavoura.py
No Gephi: Arquivo > Abrir (ou Laboratório de dados > Importar planilha) e escolha
nodes.csv e edges.csv.
"""
import numpy as np, csv

N = 30                                   # lavoura N x N
rng = np.random.default_rng(42)          # mesma semente de componentes_bfs.py
xs, ys = np.meshgrid(np.arange(N), np.arange(N))

# Três focos gaussianos (cx, cy, r) + ruído do sensor, limitado a [0, 1].
inf = np.zeros((N, N))
for cx, cy, r in [(8, 8, 4), (22, 10, 3), (15, 24, 5)]:
    inf += np.exp(-((xs - cx)**2 + (ys - cy)**2) / (2 * r**2))
inf = np.clip(inf + rng.normal(0, 0.03, inf.shape), 0, 1)

idx = lambda i, j: i * N + j             # identificador do nó da célula (i, j)

# Tabela de nós: Id, rótulo, posição (para o layout em grade) e infestação.
with open("nodes.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["Id", "Label", "latitude", "longitude", "infestacao"])
    for i in range(N):
        for j in range(N):
            w.writerow([idx(i, j), idx(i, j), i, j, round(inf[i, j], 3)])

# Tabela de arestas: só os vizinhos da direita e de baixo, para não duplicar
# nenhuma aresta (o grafo é não direcionado).
with open("edges.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["Source", "Target", "Type", "Weight"])
    for i in range(N):
        for j in range(N):
            for di, dj in [(0, 1), (1, 0)]:
                a, b = i + di, j + dj
                if a < N and b < N:
                    w.writerow([idx(i, j), idx(a, b), "Undirected", 1.0])
