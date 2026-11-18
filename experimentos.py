import numpy as np
from componentes_bfs import achar_focos
from rota_drone import (percurso_no_foco, custo_rota, vizinho_mais_proximo,
                        comprimento, dist, BASE)

# ----------------- parâmetros do experimento -----------------
N = 60                  # lavoura N x N
K_FOCOS = 6             # número de focos reais
RUIDO = 0.08            # erro do "sensor" (desvio padrão)
LIMIAR_REAL = 0.3       # a partir de quanto a célula é considerada praga REAL
MIN_CELULAS = 5         # focos menores que isso são descartados como ruído
SEMENTES = range(30)    # 30 lavouras diferentes
LIMIARES = [0.3, 0.4, 0.5, 0.6, 0.7]


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


def distancia_varredura_total():
    v = [(i, j) for i in range(N)
         for j in (range(N) if i % 2 == 0 else range(N - 1, -1, -1))]
    return dist(BASE, v[0]) + comprimento(v) + dist(v[-1], BASE)


def fmt(valores):
    return f"{np.mean(valores):5.1f}±{np.std(valores):4.1f}"


D_TOTAL = distancia_varredura_total()

print(f"{N}x{N} | {K_FOCOS} focos | ruído {RUIDO} | {len(SEMENTES)} lavouras por limiar")
print(f"{'Limiar':>6} | {'Focos':>5} | {'Área %':>11} | {'Cobertura %':>12} | {'Falsos pos. %':>13} | {'Economia %':>11}")
print("-" * 76)

for limiar in LIMIARES:
    n_focos, area, cob, fp, econ = [], [], [], [], []
    for s in SEMENTES:
        verdade, obs = gerar(s)
        real = verdade >= LIMIAR_REAL

        focos = [f for f in achar_focos(obs, limiar) if len(f) >= MIN_CELULAS]
        detectado = np.zeros((N, N), dtype=bool)
        for f in focos:
            for i, j in f:
                detectado[i, j] = True

        n_focos.append(len(focos))
        area.append(100 * detectado.sum() / (N * N))
        cob.append(100 * (detectado & real).sum() / max(real.sum(), 1))
        fp.append(100 * (detectado & ~real).sum() / max(detectado.sum(), 1))

        if focos:
            percursos = [percurso_no_foco(f) for f in focos]
            d = custo_rota(vizinho_mais_proximo(percursos), percursos)
            econ.append(100 * (D_TOTAL - d) / D_TOTAL)
        else:
            econ.append(100.0)

    print(f"{limiar:>6.1f} | {np.mean(n_focos):>5.1f} | {fmt(area)} | {fmt(cob):>12} | {fmt(fp):>13} | {fmt(econ):>11}")