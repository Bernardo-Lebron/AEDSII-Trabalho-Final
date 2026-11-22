"""Planejamento da rota do drone sobre os focos de praga.

Dado o conjunto de focos achados pela BFS (componentes_bfs.py), este módulo:
  1. gera o percurso de pulverização dentro de cada foco (zigue-zague);
  2. decide a ordem de visita dos focos, que é um problema do caixeiro-viajante
     (TSP): solução exata por permutações (viável para poucos focos) ou a
     heurística do vizinho mais próximo (para muitos focos);
  3. compara a distância voada com a varredura total da lavoura.

Todas as distâncias estão em unidades de célula (1 unidade = 1 espaçamento
entre plantas). O simulador (simulador_drone.py) converte para metros.

Uso:  python3 rota_drone.py
"""
import itertools
import numpy as np
from componentes_bfs import inf, achar_focos, N

LIMIAR = 0.5       # infestação a partir da qual uma célula é considerada foco
BASE = (0, 0)   # canto da lavoura onde o drone decola e pousa (em unidades de célula)


def dist(p, q):
    """Distância euclidiana entre dois pontos (i, j)."""
    return float(np.hypot(p[0] - q[0], p[1] - q[1]))


def comprimento(caminho):
    """Comprimento total de um caminho, dado como lista de pontos consecutivos."""
    return sum(dist(caminho[k], caminho[k + 1]) for k in range(len(caminho) - 1))


def percurso_no_foco(foco):
    """Zigue-zague linha a linha dentro do foco (lista de células)."""
    # Agrupa as colunas de cada linha do foco.
    linhas = {}
    for i, j in foco:
        linhas.setdefault(i, []).append(j)
    caminho = []
    for k, i in enumerate(sorted(linhas)):
        # Linhas pares são percorridas da esquerda para a direita e as ímpares
        # no sentido contrário (padrão boustrofédico).
        cols = sorted(linhas[i], reverse=(k % 2 == 1))
        caminho += [(i, j) for j in cols]
    return caminho


def faixas_no_foco(foco, passo_linhas=1):
    """Faixas de pulverização (p_in, p_out) em zigue-zague para varrer um conjunto
    de células (um foco, ou a lavoura inteira).

    Usa 1 linha a cada `passo_linhas` (a largura da faixa do drone cobre as linhas
    do meio) e divide cada linha em trechos contínuos, para não cruzar áreas sem praga.
    """
    # Agrupa as colunas de cada linha.
    por_linha = {}
    for i, j in foco:
        por_linha.setdefault(int(i), []).append(int(j))
    linhas = sorted(por_linha)
    # Mantém só 1 linha a cada `passo_linhas`, a partir da primeira.
    escolhidas = [i for i in linhas if (i - linhas[0]) % passo_linhas == 0]

    faixas, sentido = [], 1      # sentido: +1 = esquerda->direita, -1 = direita->esquerda
    for i in escolhidas:
        cols = sorted(por_linha[i])
        # Divide a linha em trechos de colunas consecutivas: um "buraco" (coluna
        # ausente) encerra o trecho, para o drone não voar sobre área sem praga.
        trechos, ini = [], cols[0]
        for a, b in zip(cols, cols[1:]):
            if b != a + 1:
                trechos.append((ini, a))
                ini = b
        trechos.append((ini, cols[-1]))
        if sentido == -1:
            trechos = trechos[::-1]          # percorre os trechos da direita para a esquerda
        for j0, j1 in trechos:
            faixas.append(((i, j0), (i, j1)) if sentido == 1 else ((i, j1), (i, j0)))
        sentido = -sentido                   # inverte o sentido na linha seguinte
    return faixas


def custo_rota(ordem, percursos, base=BASE):
    """Distância de: base -> focos (na ordem dada) -> base."""
    total, atual = 0.0, base
    for k in ordem:
        p = percursos[k]
        # voo até o início do foco + percurso de pulverização dentro dele
        total += dist(atual, p[0]) + comprimento(p)
        atual = p[-1]                        # o drone sai pelo fim do percurso
    return total + dist(atual, base)         # volta para a base


def vizinho_mais_proximo(percursos, base=BASE):
    """Ordem gulosa: sempre vai ao foco cujo ponto de entrada está mais perto."""
    restantes, atual, ordem = set(range(len(percursos))), base, []
    while restantes:
        prox = min(restantes, key=lambda k: dist(atual, percursos[k][0]))
        ordem.append(prox)
        atual = percursos[prox][-1]
        restantes.remove(prox)
    return ordem


if __name__ == "__main__":
    # Linha de base: varredura total da lavoura em zigue-zague, célula a célula.
    varredura = [(i, j) for i in range(N)
                 for j in (range(N) if i % 2 == 0 else range(N - 1, -1, -1))]
    d_total = dist(BASE, varredura[0]) + comprimento(varredura) + dist(varredura[-1], BASE)

    # Roteamento por grafo: BFS acha os focos e o drone só percorre eles.
    focos = achar_focos(inf, LIMIAR)
    percursos = [percurso_no_foco(f) for f in focos]

    # Ordem de visita: heurística gulosa x solução exata (testa todas as permutações).
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
