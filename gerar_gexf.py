"""Exporta a lavoura de exemplo 30x30 para o Gephi em GEXF (lavoura.gexf).

Igual a gerar_lavoura.py, mas em um único arquivo e já com a posição de cada nó
(layout em grade), usando o NetworkX. No grafo, cada planta é um nó com o
atributo `infestacao` e cada par de plantas vizinhas é uma aresta.

Uso:  python3 gerar_gexf.py
"""
import numpy as np
import networkx as nx

N = 30                                   # lavoura N x N
rng = np.random.default_rng(42)          # mesma semente de componentes_bfs.py
xs, ys = np.meshgrid(np.arange(N), np.arange(N))

# Três focos gaussianos (cx, cy, r) + ruído do sensor, limitado a [0, 1].
inf = np.zeros((N, N))
for cx, cy, r in [(8, 8, 4), (22, 10, 3), (15, 24, 5)]:
    inf += np.exp(-((xs - cx)**2 + (ys - cy)**2) / (2 * r**2))
inf = np.clip(inf + rng.normal(0, 0.03, inf.shape), 0, 1)

# Nós: um por planta, com rótulo, infestação e posição (x = coluna, y = linha).
G = nx.Graph()
for i in range(N):
    for j in range(N):
        G.add_node(
            i * N + j,
            label=str(i * N + j),
            infestacao=float(round(inf[i, j], 3)),
            viz={"position": {"x": float(j * 10), "y": float(i * 10), "z": 0.0}},
        )

# Arestas: só os vizinhos da direita e de baixo (grafo não direcionado).
for i in range(N):
    for j in range(N):
        for di, dj in [(0, 1), (1, 0)]:
            a, b = i + di, j + dj
            if a < N and b < N:
                G.add_edge(i * N + j, a * N + b, weight=1.0)

nx.write_gexf(G, "lavoura.gexf")
print("Pronto! lavoura.gexf criado.")
