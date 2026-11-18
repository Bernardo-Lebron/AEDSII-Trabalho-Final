import itertools
import numpy as np
from componentes_bfs import inf, achar_focos, N

LIMIAR = 0.5
BASE = (0, 0)   # canto da lavoura onde o drone decola e pousa


def dist(p, q):
    return float(np.hypot(p[0] - q[0], p[1] - q[1]))


def comprimento(caminho):
    return sum(dist(caminho[k], caminho[k + 1]) for k in range(len(caminho) - 1))


def percurso_no_foco(foco):
    """Zigue-zague linha a linha dentro do foco."""
    linhas = {}
    for i, j in foco:
        linhas.setdefault(i, []).append(j)
    caminho = []
    for k, i in enumerate(sorted(linhas)):
        cols = sorted(linhas[i], reverse=(k % 2 == 1))
        caminho += [(i, j) for j in cols]
    return caminho


def custo_rota(ordem, percursos):
    total, atual = 0.0, BASE
    for k in ordem:
        p = percursos[k]
        total += dist(atual, p[0]) + comprimento(p)
        atual = p[-1]
    return total + dist(atual, BASE)


def vizinho_mais_proximo(percursos):
    restantes, atual, ordem = set(range(len(percursos))), BASE, []
    while restantes:
        prox = min(restantes, key=lambda k: dist(atual, percursos[k][0]))
        ordem.append(prox)
        atual = percursos[prox][-1]
        restantes.remove(prox)
    return ordem


if __name__ == "__main__":
    # Baseline: varredura em zigue-zague da lavoura inteira
    varredura = [(i, j) for i in range(N)
                 for j in (range(N) if i % 2 == 0 else range(N - 1, -1, -1))]
    d_total = dist(BASE, varredura[0]) + comprimento(varredura) + dist(varredura[-1], BASE)

    # Proposta: só os focos
    focos = achar_focos(inf, LIMIAR)
    percursos = [percurso_no_foco(f) for f in focos]

    d_nn = custo_rota(vizinho_mais_proximo(percursos), percursos)
    melhor = min(itertools.permutations(range(len(percursos))),
                 key=lambda o: custo_rota(o, percursos))
    d_otima = custo_rota(melhor, percursos)

    print(f"Limiar: {LIMIAR} | Focos: {len(focos)}")
    print(f"{'Método':<28} | {'Distância':>10} | {'Economia':>8}")
    print("-" * 54)
    for nome, d in [("Varredura total", d_total),
                    ("Vizinho mais próximo", d_nn),
                    ("Melhor ordem (exata)", d_otima)]:
        print(f"{nome:<28} | {d:>10.1f} | {100 * (d_total - d) / d_total:>7.1f}%")