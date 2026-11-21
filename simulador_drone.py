"""
Simulador de pulverização por drone, integrado ao algoritmo do trabalho.

Mesmo pipeline do artigo:
  1. cada planta é um vértice de um grafo em grade (vizinhança 4-conexa) e o peso
     do vértice é a infestação MEDIDA pelo sensor (verdade + ruído);
  2. os focos de praga são os componentes conexos (BFS) dos vértices com
     infestação >= LIMIAR_DETECCAO, descartando focos com menos de MIN_CELULAS;
  3. a rota visita só os focos, na melhor ordem (TSP exato para poucos focos,
     vizinho mais próximo para muitos), respeitando a autonomia da bateria.

A varredura tradicional usa a MESMA função de faixas, só que sobre todos os
vértices da lavoura, e a mesma largura de faixa (comparação justa).

Uso:
  python simulador_drone.py                  # animação lado a lado
  python simulador_drone.py --salvar         # só relatório + simulador_final.png
  python simulador_drone.py --semente 7      # outra lavoura (padrão: 42)
"""
import itertools
import sys

import numpy as np
import matplotlib

if "--salvar" in sys.argv:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.path as mpath
from scipy.interpolate import splprep, splev

from componentes_bfs import achar_focos
from rota_drone import faixas_no_foco, vizinho_mais_proximo, custo_rota, dist

# ==============================================================================
# PARÂMETROS
# ==============================================================================
SEMENTE = 42
if "--semente" in sys.argv:
    SEMENTE = int(sys.argv[sys.argv.index("--semente") + 1])

AUTONOMIA_TOTAL_METROS = 550.0
MARGEM_SEGURANCA_RTH = 0.20
AUTONOMIA_UTIL = AUTONOMIA_TOTAL_METROS * (1.0 - MARGEM_SEGURANCA_RTH)
VELOCIDADE_MS = 20.0
LARGURA_FAIXA_SPRAY = 2.4          # igual para as duas estratégias
ESPACAMENTO_PLANTAS = 1.2
PASSO_LINHAS = int(round(LARGURA_FAIXA_SPRAY / ESPACAMENTO_PLANTAS))

NUM_FOCOS = 3
DIST_MIN_ENTRE_FOCOS = 18.0        # evita que dois focos se fundam num só
RUIDO_SENSOR = 0.08                # erro de medida da infestação
LIMIAR_REAL = 0.3                  # infestação verdadeira a partir da qual há praga
LIMIAR_DETECCAO = 0.3              # limiar usado pelo algoritmo (sobre a medida)
MIN_CELULAS = 5                    # focos menores que isso são tratados como ruído
PASSO_ANIMACAO = 2                 # frames pulados por quadro (acelera a animação)

rng = np.random.default_rng(SEMENTE)

# ==============================================================================
# 1. GEOMETRIAS ORGÂNICAS (terreno e focos reais)
# ==============================================================================
def gerar_terreno_organico(centro=(45.0, 45.0), raio_medio=35.0, num_pontos_base=8):
    angulos = np.linspace(0, 2 * np.pi, num_pontos_base, endpoint=False)
    raios = raio_medio * rng.uniform(0.70, 1.20, size=num_pontos_base)
    x = np.append(centro[0] + raios * np.cos(angulos), centro[0] + raios[0])
    y = np.append(centro[1] + raios * np.sin(angulos), centro[1])
    x[-1], y[-1] = x[0], y[0]
    tck, _ = splprep([x, y], s=0, per=True, k=3)
    xs_, ys_ = splev(np.linspace(0, 1, 200), tck)
    return np.column_stack([xs_, ys_])


def gerar_foco_organico(centro, raio_base=7.0, num_vertices=10):
    angulos = np.linspace(0, 2 * np.pi, num_vertices, endpoint=False)
    raios = raio_base * rng.uniform(0.65, 1.30, size=num_vertices)
    x = centro[0] + raios * np.cos(angulos)
    y = centro[1] + raios * np.sin(angulos)
    x, y = np.append(x, x[0]), np.append(y, y[0])
    tck, _ = splprep([x, y], s=0, per=True, k=3)
    xs_, ys_ = splev(np.linspace(0, 1, 100), tck)
    return np.column_stack([xs_, ys_])


VERTICES_TERRENO = gerar_terreno_organico()
poligono_path = mpath.Path(VERTICES_TERRENO)
min_x, min_y = VERTICES_TERRENO.min(axis=0)
max_x, max_y = VERTICES_TERRENO.max(axis=0)

# ==============================================================================
# 2. GRAFO EM GRADE: cada planta é um vértice
# ==============================================================================
xs_grade = np.arange(min_x, max_x, ESPACAMENTO_PLANTAS)
ys_grade = np.arange(min_y, max_y, ESPACAMENTO_PLANTAS)
XX, YY = np.meshgrid(xs_grade, ys_grade)                      # forma (linhas, colunas)
dentro = poligono_path.contains_points(
    np.column_stack([XX.ravel(), YY.ravel()])).reshape(XX.shape)
N_LIN, N_COL = XX.shape


def celula_para_metros(i, j):
    return (float(xs_grade[j]), float(ys_grade[i]))


# Focos reais (verdade): infestação 1 no centro, decaindo até LIMIAR_REAL na borda
focos_poligonos, centros = [], []
while len(focos_poligonos) < NUM_FOCOS:
    c = rng.uniform([min_x + 12, min_y + 12], [max_x - 12, max_y - 12])
    if not poligono_path.contains_point(c):
        continue
    if any(np.hypot(*(c - cc)) < DIST_MIN_ENTRE_FOCOS for cc in centros):
        continue
    focos_poligonos.append(gerar_foco_organico(c, raio_base=rng.uniform(5.5, 8.5)))
    centros.append(c)

verdade = np.zeros(XX.shape)
for poly, c in zip(focos_poligonos, centros):
    ang = np.arctan2(poly[:, 1] - c[1], poly[:, 0] - c[0])
    rad = np.hypot(poly[:, 0] - c[0], poly[:, 1] - c[1])
    o = np.argsort(ang)
    theta = np.arctan2(YY - c[1], XX - c[0])
    r_borda = np.interp(theta, ang[o], rad[o], period=2 * np.pi)
    d = np.hypot(XX - c[0], YY - c[1])
    verdade += LIMIAR_REAL ** ((d / r_borda) ** 2)
verdade = np.clip(verdade, 0, 1) * dentro

observado = np.clip(verdade + rng.normal(0, RUIDO_SENSOR, verdade.shape), 0, 1) * dentro

# ==============================================================================
# 3. ALGORITMO DO TRABALHO: BFS -> focos -> rota
# ==============================================================================
focos_detectados = [f for f in achar_focos(observado, LIMIAR_DETECCAO) if len(f) >= MIN_CELULAS]

todas_plantas = np.column_stack([XX[dentro], YY[dentro]])
real_mask = (verdade >= LIMIAR_REAL) & dentro
detectado_mask = np.zeros(XX.shape, dtype=bool)
for f in focos_detectados:
    for i, j in f:
        detectado_mask[i, j] = True
plantas_detectadas = np.column_stack([XX[detectado_mask], YY[detectado_mask]])
plantas_alvo = np.column_stack([XX[real_mask], YY[real_mask]])

# Base: ponto do contorno (parte de baixo) mais perto das plantas detectadas
indices_amostra = np.linspace(0, len(VERTICES_TERRENO) - 1, 35, dtype=int)
candidatos = VERTICES_TERRENO[indices_amostra]
inferiores = candidatos[candidatos[:, 1] <= min_y + (max_y - min_y) * 0.35]
if len(inferiores) == 0:
    inferiores = candidatos
if len(plantas_detectadas):
    BASE = tuple(min(inferiores, key=lambda b: np.sum(np.linalg.norm(plantas_detectadas - b, axis=1))))
else:
    BASE = tuple(inferiores[0])


def faixas_em_metros(celulas):
    return [(celula_para_metros(*a), celula_para_metros(*b))
            for a, b in faixas_no_foco(celulas, PASSO_LINHAS)]


# Rota tradicional: varre TODOS os vértices da lavoura
faixas_trad = faixas_em_metros(np.argwhere(dentro))

# Rota otimizada: varre só os vértices dos focos, na melhor ordem de visita
faixas_por_foco = [faixas_em_metros(f) for f in focos_detectados]
percursos = [[p for faixa in fx for p in faixa] for fx in faixas_por_foco]
if len(percursos) <= 7:
    ordem = min(itertools.permutations(range(len(percursos))),
                key=lambda o: custo_rota(o, percursos, BASE)) if percursos else []
    METODO_ORDEM = "TSP exato"
else:
    ordem = vizinho_mais_proximo(percursos, BASE)
    METODO_ORDEM = "vizinho mais próximo"
faixas_otim = [fx for k in ordem for fx in faixas_por_foco[k]]


def montar_rota(faixas):
    """Encadeia as faixas e volta à base quando a bateria não chega."""
    rota, flags, eh_base = [BASE], [False], [True]
    recargas, dist_ciclo, pos = 0, 0.0, BASE
    for p_in, p_out in faixas:
        d_in, d_sp, d_ret = dist(pos, p_in), dist(p_in, p_out), dist(p_out, BASE)
        if dist_ciclo + d_in + d_sp + d_ret > AUTONOMIA_UTIL:
            rota.append(BASE); flags.append(False); eh_base.append(True)
            recargas += 1
            dist_ciclo, pos = 0.0, BASE
            d_in = dist(BASE, p_in)
        rota += [p_in, p_out]
        flags += [False, True]
        eh_base += [False, False]
        dist_ciclo += d_in + d_sp
        pos = p_out
    rota.append(BASE); flags.append(False); eh_base.append(True)
    return rota, flags, eh_base, recargas


def interpolar(rota, flags, eh_base, passo_m=0.99):
    """Discretiza a rota em pontos; devolve posições, flag de spray e a distância
    voada na saída (bateria) corrente."""
    pts, ativo, ciclo = [], [], []
    acum = 0.0
    for i in range(len(rota) - 1):
        p1, p2 = np.array(rota[i]), np.array(rota[i + 1])
        d = float(np.linalg.norm(p2 - p1))
        if eh_base[i]:
            acum = 0.0
        n = max(int(d / passo_m), 1)
        for t in np.linspace(0, 1, n, endpoint=False):
            pts.append(p1 + t * (p2 - p1))
            ativo.append(flags[i + 1])
            ciclo.append(acum + t * d)
        acum += d
    pts.append(np.array(rota[-1])); ativo.append(False); ciclo.append(acum)
    pts = np.array(pts)
    spray = np.array(ativo) & poligono_path.contains_points(pts)
    passos = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    dist_acum = np.concatenate([[0.0], np.cumsum(passos)])
    return pts, spray, np.array(ciclo), dist_acum


def dist_ate_faixas(rota, flags):
    """Distância de cada planta à faixa pulverizada mais próxima (ponto-segmento)."""
    menor = np.full(len(todas_plantas), np.inf)
    for i in range(len(rota) - 1):
        if not flags[i + 1]:
            continue
        A, B = np.array(rota[i]), np.array(rota[i + 1])
        AB = B - A
        t = np.clip(((todas_plantas - A) @ AB) / max(AB @ AB, 1e-12), 0.0, 1.0)
        proj = A + t[:, None] * AB
        menor = np.minimum(menor, np.linalg.norm(todas_plantas - proj, axis=1))
    return menor


def metricas(rota, flags, dist_acum, recargas):
    cobertas = dist_ate_faixas(rota, flags) <= LARGURA_FAIXA_SPRAY / 2 + 1e-6
    real = real_mask[dentro]
    insumo = sum(dist(rota[i], rota[i + 1]) for i in range(len(rota) - 1) if flags[i + 1])
    return {
        "dist": dist_acum[-1],
        "tempo": dist_acum[-1] / VELOCIDADE_MS,
        "recargas": recargas,
        "baterias": dist_acum[-1] / AUTONOMIA_UTIL,
        "insumo": insumo,
        "atendidas": 100 * (cobertas & real).sum() / max(real.sum(), 1),
        "saudaveis": 100 * (cobertas & ~real).sum() / max((~real).sum(), 1),
    }


rota_t, flags_t, base_t, recargas_t = montar_rota(faixas_trad)
rota_o, flags_o, base_o, recargas_o = montar_rota(faixas_otim)
caminho_t, spray_t, ciclo_t, dacum_t = interpolar(rota_t, flags_t, base_t)
caminho_o, spray_o, ciclo_o, dacum_o = interpolar(rota_o, flags_o, base_o)
m_t = metricas(rota_t, flags_t, dacum_t, recargas_t)
m_o = metricas(rota_o, flags_o, dacum_o, recargas_o)
total_frames = max(len(caminho_t), len(caminho_o))

# ==============================================================================
# 4. RELATÓRIO NO TERMINAL
# ==============================================================================
def economia(a, b):
    return 100 * (a - b) / a if a > 0 else 0.0


print("\n" + "=" * 74)
print(f"{'RELATÓRIO COMPARATIVO: VARREDURA TOTAL x ROTEAMENTO POR GRAFO':^74}")
print("=" * 74)
print(f"Semente: {SEMENTE} | Plantas (vértices): {len(todas_plantas)} | "
      f"Focos reais: {NUM_FOCOS} | Focos detectados: {len(focos_detectados)}")
print(f"Limiar de detecção: {LIMIAR_DETECCAO} | Ruído do sensor: {RUIDO_SENSOR} | "
      f"Ordem dos focos: {METODO_ORDEM}")
print("-" * 74)
print(f"{'Métrica':<34} | {'Tradicional':>16} | {'Otimizado':>16}")
print("-" * 74)
print(f"{'Distância percorrida (m)':<34} | {m_t['dist']:>16.1f} | {m_o['dist']:>16.1f}")
print(f"{'Tempo de operação (min)':<34} | {m_t['tempo'] / 60:>16.2f} | {m_o['tempo'] / 60:>16.2f}")
print(f"{'Trocas de bateria (RTH)':<34} | {m_t['recargas']:>16} | {m_o['recargas']:>16}")
print(f"{'Consumo de bateria (un)':<34} | {m_t['baterias']:>16.2f} | {m_o['baterias']:>16.2f}")
print(f"{'Insumo (m pulverizados)':<34} | {m_t['insumo']:>16.1f} | {m_o['insumo']:>16.1f}")
print(f"{'Plantas infectadas atendidas (%)':<34} | {m_t['atendidas']:>16.1f} | {m_o['atendidas']:>16.1f}")
print(f"{'Plantas saudáveis pulverizadas (%)':<34} | {m_t['saudaveis']:>16.1f} | {m_o['saudaveis']:>16.1f}")
print("-" * 74)
print(f" • Economia de distância: {economia(m_t['dist'], m_o['dist']):.1f}%   "
      f"• Economia de insumo: {economia(m_t['insumo'], m_o['insumo']):.1f}%   "
      f"• Recargas evitadas: {m_t['recargas'] - m_o['recargas']}")
print("=" * 74 + "\n")

# ==============================================================================
# 5. INTERFACE GRÁFICA
# ==============================================================================
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
fig.suptitle("Simulador de Pulverização: Varredura Total x Roteamento por Grafo (BFS + TSP)",
             fontweight="bold")

cores_focos = ["#e17055", "#6c5ce7", "#fdcb6e", "#00b894", "#e84393", "#0984e3"]
for ax in (ax1, ax2):
    ax.set_xlim(min_x - 5, max_x + 5)
    ax.set_ylim(min_y - 5, max_y + 5)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.add_patch(plt.Polygon(VERTICES_TERRENO, closed=True, fill=None,
                             edgecolor="#2d3436", linewidth=2))
    ax.scatter(todas_plantas[:, 0], todas_plantas[:, 1], color="#55efc4", s=1.2,
               alpha=0.6, label="Plantio saudável")
    for poly in focos_poligonos:
        ax.add_patch(plt.Polygon(poly, closed=True, facecolor=(1.0, 0.46, 0.46, 0.30),
                                 edgecolor="#d63031", linewidth=1.2))
    ax.scatter(plantas_alvo[:, 0], plantas_alvo[:, 1], color="#d63031", s=2.5,
               label="Plantas infectadas (real)")
    ax.scatter(BASE[0], BASE[1], color="#0984e3", marker="s", s=130, zorder=6, label="Base")

# No painel do grafo, mostra os focos detectados pela BFS (um por cor)
for k, f in enumerate(focos_detectados):
    pts_f = np.array([celula_para_metros(i, j) for i, j in f])
    ax2.scatter(pts_f[:, 0], pts_f[:, 1], s=5, color=cores_focos[k % len(cores_focos)],
                zorder=4, label="Focos detectados (BFS)" if k == 0 else None)

ax1.set_title(f"Varredura Total ({m_t['recargas']} recargas)")
linha_t, = ax1.plot([], [], "r--", alpha=0.20)
drone_t, = ax1.plot([], [], "ro", markersize=9, zorder=10)
spray_layer_t = ax1.scatter([], [], color="#00cec9", alpha=0.28, s=260, zorder=5)
txt_t = ax1.text(0.03, 0.62, "", transform=ax1.transAxes, fontsize=9, zorder=20,
                 bbox=dict(boxstyle="round", facecolor="white", alpha=0.92))
ax1.legend(loc="upper right", fontsize=8)

ax2.set_title(f"Roteamento por Grafo ({m_o['recargas']} recargas)")
linha_o, = ax2.plot([], [], "g-", linewidth=1.2, alpha=0.25)
drone_o, = ax2.plot([], [], "go", markersize=9, zorder=10)
spray_layer_o = ax2.scatter([], [], color="#00b894", alpha=0.32, s=260, zorder=5)
txt_o = ax2.text(0.03, 0.56, "", transform=ax2.transAxes, fontsize=9, zorder=20,
                 bbox=dict(boxstyle="round", facecolor="white", alpha=0.92))
ax2.legend(loc="upper right", fontsize=8)


def desenhar(caminho, spray, ciclo, dacum, f, drone, linha, camada, txt, extra=""):
    it = min(f, len(caminho) - 1)
    drone.set_data([caminho[it, 0]], [caminho[it, 1]])
    linha.set_data(caminho[:it + 1, 0], caminho[:it + 1, 1])
    idx = np.flatnonzero(spray[:it + 1])
    camada.set_offsets(caminho[idx] if len(idx) else np.empty((0, 2)))
    bateria = 100.0 - ciclo[it] / AUTONOMIA_TOTAL_METROS * 100.0
    txt.set_text(f"Distância: {dacum[it]:.1f} m\nTempo: {dacum[it] / VELOCIDADE_MS:.1f} s\n"
                 f"Bateria: {bateria:.1f}%\nPulverizando: {'SIM' if spray[it] else 'NÃO'}{extra}")


def update(f):
    f = f * PASSO_ANIMACAO
    desenhar(caminho_t, spray_t, ciclo_t, dacum_t, f, drone_t, linha_t, spray_layer_t, txt_t)
    it = min(f, len(caminho_t) - 1)
    io = min(f, len(caminho_o) - 1)
    eco = economia(dacum_t[it], dacum_o[io]) if dacum_t[it] > 0 else 0.0
    desenhar(caminho_o, spray_o, ciclo_o, dacum_o, f, drone_o, linha_o, spray_layer_o, txt_o,
             extra=f"\nEconomia de distância: {eco:.1f}%")
    return drone_t, linha_t, spray_layer_t, txt_t, drone_o, linha_o, spray_layer_o, txt_o


n_quadros = int(np.ceil(total_frames / PASSO_ANIMACAO))
ani = animation.FuncAnimation(fig, update, frames=n_quadros, interval=15, blit=True, repeat=False)
plt.tight_layout()

if "--salvar" in sys.argv:
    update(n_quadros - 1)
    fig.savefig("simulador_final.png", dpi=200, bbox_inches="tight")
    print("Figura salva em simulador_final.png")
else:
    plt.show()