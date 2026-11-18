import time
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from componentes_bfs import achar_focos
from rota_drone import percurso_no_foco, custo_rota, vizinho_mais_proximo

# ----------------- parâmetros -----------------
TAMANHOS = [50, 100, 200, 400, 800]   # lavoura N x N
LIMIAR = 0.5
RUIDO = 0.08
MIN_CELULAS = 5
REPETICOES = 3                         # repete e pega a mediana do tempo


def gerar(N, seed=0):
    """Lavoura N x N com densidade fixa de focos (1 foco a cada ~600 células)."""
    K = max(3, N * N // 600)
    rng = np.random.default_rng(seed)
    verdade = np.zeros((N, N))
    for _ in range(K):
        cx, cy = rng.uniform(0, N, 2)
        r = rng.uniform(2.5, 5)
        m = int(3 * r) + 1
        i0, i1 = max(int(cy) - m, 0), min(int(cy) + m + 1, N)
        j0, j1 = max(int(cx) - m, 0), min(int(cx) + m + 1, N)
        yy, xx = np.mgrid[i0:i1, j0:j1]
        verdade[i0:i1, j0:j1] += np.exp(-((xx - cx) ** 2 + (yy - cy) ** 2) / (2 * r ** 2))
    return np.clip(verdade + rng.normal(0, RUIDO, (N, N)), 0, 1)


def mediana_tempo(funcao):
    tempos = []
    for _ in range(REPETICOES):
        t0 = time.perf_counter()
        resultado = funcao()
        tempos.append(time.perf_counter() - t0)
    return float(np.median(tempos)), resultado


def pipeline(obs):
    focos = [f for f in achar_focos(obs, LIMIAR) if len(f) >= MIN_CELULAS]
    percursos = [percurso_no_foco(f) for f in focos]
    custo_rota(vizinho_mais_proximo(percursos), percursos)
    return len(focos)


print(f"{'N':>5} | {'Nós (V)':>9} | {'Arestas (E)':>11} | {'Focos':>5} | {'BFS (s)':>8} | {'Total (s)':>9} | {'BFS x anterior':>14}")
print("-" * 78)

Vs, t_bfs, t_tot = [], [], []
for N in TAMANHOS:
    obs = gerar(N)
    V, E = N * N, 2 * N * (N - 1)
    tb, _ = mediana_tempo(lambda: achar_focos(obs, LIMIAR))
    tt, n_focos = mediana_tempo(lambda: pipeline(obs))
    razao = f"{tb / t_bfs[-1]:>13.1f}x" if t_bfs else f"{'—':>14}"
    print(f"{N:>5} | {V:>9} | {E:>11} | {n_focos:>5} | {tb:>8.4f} | {tt:>9.4f} | {razao}")
    Vs.append(V); t_bfs.append(tb); t_tot.append(tt)

plt.figure(figsize=(6, 4))
plt.loglog(Vs, t_bfs, "o-", label="BFS (achar focos)")
plt.loglog(Vs, t_tot, "s-", label="Pipeline completo")
plt.xlabel("Número de nós (V)")
plt.ylabel("Tempo de execução (s)")
plt.grid(True, which="both", alpha=0.3)
plt.legend()
plt.tight_layout()
plt.savefig("escalabilidade.png", dpi=200)
print("\nGráfico salvo em escalabilidade.png")