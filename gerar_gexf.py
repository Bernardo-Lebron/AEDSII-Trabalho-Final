import numpy as np
import networkx as nx

N = 30
rng = np.random.default_rng(42)
xs, ys = np.meshgrid(np.arange(N), np.arange(N))

inf = np.zeros((N, N))
for cx, cy, r in [(8, 8, 4), (22, 10, 3), (15, 24, 5)]:
    inf += np.exp(-((xs - cx)**2 + (ys - cy)**2) / (2 * r**2))
inf = np.clip(inf + rng.normal(0, 0.03, inf.shape), 0, 1)

G = nx.Graph()
for i in range(N):
    for j in range(N):
        G.add_node(
            i * N + j,
            label=str(i * N + j),
            infestacao=float(round(inf[i, j], 3)),
            viz={"position": {"x": float(j * 10), "y": float(i * 10), "z": 0.0}},
        )

for i in range(N):
    for j in range(N):
        for di, dj in [(0, 1), (1, 0)]:
            a, b = i + di, j + dj
            if a < N and b < N:
                G.add_edge(i * N + j, a * N + b, weight=1.0)

nx.write_gexf(G, "lavoura.gexf")
print("Pronto! lavoura.gexf criado.")