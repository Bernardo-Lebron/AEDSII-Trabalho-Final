"""Gera a base de dados do trabalho: 30 lavouras sintéticas de 60x60 plantas.

Cada arquivo dataset/lavouras_60x60/lavoura_seedXX.csv tem uma linha por planta:
    linha, coluna, infestacao_verdadeira, infestacao_observada
A "verdadeira" é a infestação real (soma de focos gaussianos); a "observada" é o
que o sensor mede (verdade + ruído) e é a única que o algoritmo enxerga.

Os parâmetros são os mesmos de experimentos.py, então os resultados do artigo
podem ser reproduzidos a partir destes arquivos.

Uso:  python3 gerar_dataset.py
"""
import csv
import os
import numpy as np

# Mesmos parâmetros do experimentos.py
N = 60                   # lavoura N x N
K_FOCOS = 6              # número de focos reais por lavoura
RUIDO = 0.08             # desvio-padrão do ruído do sensor
SEMENTES = range(30)     # uma lavoura por semente
PASTA = os.path.join("dataset", "lavouras_60x60")


def gerar(seed):
    """Devolve (verdade, observado): infestação real e a medida com ruído."""
    rng = np.random.default_rng(seed)
    xs, ys = np.meshgrid(np.arange(N), np.arange(N))
    verdade = np.zeros((N, N))
    for _ in range(K_FOCOS):
        cx, cy = rng.uniform(3, N - 3, 2)    # centro aleatório do foco
        r = rng.uniform(2.5, 5)              # raio aleatório do foco
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
