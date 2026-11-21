import numpy as np
from collections import deque

# --- lavoura simulada do exemplo 30x30 (a mesma usada no Gephi) ---
N = 30
rng = np.random.default_rng(42)
xs, ys = np.meshgrid(np.arange(N), np.arange(N))
inf = np.zeros((N, N))
for cx, cy, r in [(8, 8, 4), (22, 10, 3), (15, 24, 5)]:
    inf += np.exp(-((xs - cx)**2 + (ys - cy)**2) / (2 * r**2))
inf = np.clip(inf + rng.normal(0, 0.03, inf.shape), 0, 1)


def achar_focos(inf, limiar):
    """Acha os focos de praga: componentes conexos (BFS) das células com
    infestação >= limiar, com vizinhança 4-conexa (cima, baixo, esquerda, direita).

    Aceita grades retangulares (linhas x colunas).
    Devolve uma lista de focos; cada foco é uma lista de células (i, j).
    Complexidade: O(V + E).
    """
    linhas, colunas = inf.shape
    visitado = np.zeros((linhas, colunas), dtype=bool)
    focos = []
    for i in range(linhas):
        for j in range(colunas):
            if inf[i, j] >= limiar and not visitado[i, j]:
                fila = deque([(i, j)])
                visitado[i, j] = True
                foco = []
                while fila:
                    a, b = fila.popleft()
                    foco.append((a, b))
                    for da, db in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                        c, d = a + da, b + db
                        if (0 <= c < linhas and 0 <= d < colunas
                                and inf[c, d] >= limiar and not visitado[c, d]):
                            visitado[c, d] = True
                            fila.append((c, d))
                focos.append(foco)
    return focos


if __name__ == "__main__":
    print(f"{'Limiar':>6} | {'Focos':>5} | {'Células a pulverizar':>20} | {'% da lavoura':>12}")
    print("-" * 55)
    for limiar in [0.3, 0.4, 0.5, 0.6, 0.7]:
        focos = achar_focos(inf, limiar)
        total = sum(len(f) for f in focos)
        print(f"{limiar:>6.1f} | {len(focos):>5} | {total:>20} | {100 * total / (N * N):>11.1f}%")