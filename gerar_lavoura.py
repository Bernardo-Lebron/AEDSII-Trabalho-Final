import numpy as np, csv

N = 30
rng = np.random.default_rng(42)
xs, ys = np.meshgrid(np.arange(N), np.arange(N))

inf = np.zeros((N, N))
for cx, cy, r in [(8, 8, 4), (22, 10, 3), (15, 24, 5)]:
    inf += np.exp(-((xs - cx)**2 + (ys - cy)**2) / (2 * r**2))
inf = np.clip(inf + rng.normal(0, 0.03, inf.shape), 0, 1)

idx = lambda i, j: i * N + j

with open("nodes.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["Id", "Label", "latitude", "longitude", "infestacao"])
    for i in range(N):
        for j in range(N):
            w.writerow([idx(i, j), idx(i, j), i, j, round(inf[i, j], 3)])

with open("edges.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["Source", "Target", "Type", "Weight"])
    for i in range(N):
        for j in range(N):
            for di, dj in [(0, 1), (1, 0)]:
                a, b = i + di, j + dj
                if a < N and b < N:
                    w.writerow([idx(i, j), idx(a, b), "Undirected", 1.0])