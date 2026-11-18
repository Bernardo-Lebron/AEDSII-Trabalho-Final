# Roteamento de drone agrícola por grafos

Modelagem de uma lavoura como grafo em grade (cada célula é um nó, arestas
ligam vizinhas, o peso do nó é a infestação). Os focos de praga são achados por
BFS (componentes conexos) e o drone pulveriza só esses focos.

Trabalho Final de Estruturas em Árvores e Teoria dos Grafos — CEFET-MG.
Dataset: a implementar

## Como executar (Ubuntu 24.04)

```bash
sudo apt update
sudo apt install -y python3 python3-venv python3-pip

git clone <LINK_DO_REPOSITORIO>
cd <NOME_DA_PASTA>
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

python3 componentes_bfs.py   # focos por limiar (BFS)
python3 rota_drone.py        # rota do drone vs varredura total
python3 experimentos.py      # cobertura, falsos positivos e economia
python3 escalabilidade.py    # tempo x tamanho; gera escalabilidade.png
python3 gerar_dataset.py     # gera a pasta dataset/
```