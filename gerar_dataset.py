import csv
import os
import numpy as np

# Mesmos parâmetros do experimentos.py
N = 60
K_FOCOS = 6
RUIDO = 0.08
SEMENTES = range(30)
PASTA = os.path.join("dataset", "lavouras_60x60")


def gerar(seed):
    """Devolve (verdade, observado): infestação real e a medida com ruído."""
    rng = np.random.default_rng(seed)
    xs, ys = np.meshgrid(np.arange(N), np.arange(N))
    verdade = np.zeros((N, N))
    for _ in range(K_FOCOS):
        cx, cy = rng.uniform(3, N - 3, 2)
        r = rng.uniform(2.5, 5)
        verdade += np.exp(-((xs - cx) ** 2 + (ys - cy) ** 2) / (2 * r ** 2))
    verdade = np.clip(verdade, 0, 1)
    observado = np.clip(verdade + rng.normal(0, RUIDO, verdade.shape), 0, 1)
    return verdade, observado


os.makedirs(PASTA, exist_ok=True)
for s in SEMENTES:
    verdade, obs = gerar(s)
    caminho = os.path.join(PASTA, f"lavoura_seed{s:02d}.csv")
    with open(caminho, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["linha", "coluna", "infestacao_verdadeira", "infestacao_observada"])
        for i in range(N):
            for j in range(N):
                w.writerow([i, j, round(float(verdade[i, j]), 4), round(float(obs[i, j]), 4)])

print(f"Pronto! {len(SEMENTES)} arquivos criados em {PASTA}")