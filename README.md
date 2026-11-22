# Roteamento de drone agrícola por grafos

Modelagem de uma lavoura como grafo em grade (cada planta é um vértice, as arestas
ligam plantas vizinhas e o peso do vértice é a infestação medida). Os focos de praga
são achados por BFS (componentes conexos) e o drone pulveriza só esses focos, em vez
de varrer a lavoura inteira.

Trabalho Final de Estruturas em Árvores e Teoria dos Grafos — UFCI-MG.

- Código: https://github.com/Bernardo-Lebron/AEDSII-Trabalho-Final
- Dataset: https://zenodo.org/records/24342470

## Organização

| Arquivo | O que faz |
|---|---|
| `componentes_bfs.py` | Grafo em grade e BFS: componentes conexos = focos de praga |
| `rota_drone.py` | Faixas de pulverização, ordem de visita dos focos (TSP exato ou vizinho mais próximo) |
| `simulador_drone.py` | Simulador visual (usa os dois módulos acima): varredura total x roteamento por grafo |
| `experimentos.py` | Cobertura, falsos positivos e economia por limiar (30 lavouras) |
| `escalabilidade.py` | Tempo de execução x tamanho da lavoura; gera `escalabilidade.png` |
| `gerar_dataset.py` | Gera a pasta `dataset/` (30 lavouras 60x60) |
| `gerar_lavoura.py`, `gerar_gexf.py` | Exemplo 30x30 para o Gephi (CSV e GEXF) |

## Como executar (Ubuntu 24.04)

```bash
sudo apt update
sudo apt install -y python3 python3-venv python3-pip python3-tk

git clone https://github.com/Bernardo-Lebron/AEDSII-Trabalho-Final.git
cd AEDSII-Trabalho-Final
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

python3 componentes_bfs.py   # focos por limiar (BFS)
python3 rota_drone.py        # rota do drone vs varredura total
python3 experimentos.py      # cobertura, falsos positivos e economia
python3 escalabilidade.py    # tempo x tamanho; gera escalabilidade.png
python3 gerar_dataset.py     # gera a pasta dataset/
python3 gerar_lavoura.py     # exemplo 30x30: nodes.csv e edges.csv (Gephi)
python3 gerar_gexf.py        # exemplo 30x30: lavoura.gexf (Gephi, com posições)
```

### Simulador

```bash
python3 simulador_drone.py                # animação lado a lado (precisa de janela gráfica)
python3 simulador_drone.py --salvar       # sem janela: imprime o relatório e salva simulador_final.png
python3 simulador_drone.py --semente 7    # outra lavoura (padrão: 42)
```

## Resultados de referência

`dataset/resultados.txt` guarda a saída de `experimentos.py` e `escalabilidade.py`
usada no artigo. Os tempos de `escalabilidade.py` dependem da máquina (os do artigo
foram medidos em Ubuntu 24.04, 2 núcleos de CPU, Python 3.13); as demais saídas são
determinísticas (sementes fixas) e devem ser idênticas em qualquer máquina.
