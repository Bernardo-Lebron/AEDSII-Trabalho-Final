# Roteamento de Drone Agrícola por Grafos

Aplicação em Python que modela uma lavoura como um grafo em grade, acha os focos de praga com busca em largura (BFS) e planeja a rota de um drone que pulveriza só esses focos, em vez de varrer a lavoura inteira. Trabalho final da disciplina de Estruturas em Árvores e Teoria dos Grafos — UFCI-MG.

## 📋 Sumário
- [O que o projeto faz](#-o-que-o-projeto-faz)
- [Modelagem](#-modelagem)
- [Estrutura do Projeto](#-estrutura-do-projeto)
- [Como Executar](#-como-executar)
- [Resultados](#-resultados)
- [Limitações](#-limitações)
- [Links](#-links)
- [Autor](#-autor)


## 🚜 O que o projeto faz

Um drone que varre a lavoura inteira gasta bateria, insumo e tempo tratando plantas que nunca foram atacadas. Como as pragas aparecem em manchas (focos), o projeto propõe duas etapas: mapear a lavoura completa e depois voar só até as áreas infestadas.


## 🧩 Modelagem

- **Vértice:** cada planta da lavoura.
- **Aresta:** liga duas plantas vizinhas (cima, baixo, esquerda e direita).
- **Peso do vértice:** infestação medida pelo sensor (entre 0 e 1).
- **Foco de praga:** componente conexo dos vértices com infestação maior ou igual a um limiar, encontrado por BFS em O(V + E).
- **Rota:** faixas de pulverização em zigue-zague dentro de cada foco. A ordem de visita dos focos é um problema do caixeiro-viajante: solução exata (todas as permutações) para até 7 focos e vizinho mais próximo para mais focos.
- **Bateria:** o drone volta à base para trocar a bateria sempre que não consegue terminar a próxima faixa e retornar.


## 📂 Estrutura do Projeto

```bash
.
├── componentes_bfs.py     # Grafo em grade e BFS (achar focos de praga)
├── rota_drone.py          # Faixas de pulverização e ordem de visita (TSP)
├── simulador_drone.py     # Simulador visual: varredura total x roteamento por grafo
├── experimentos.py        # Cobertura, falsos positivos e economia por limiar
├── escalabilidade.py      # Tempo de execução x tamanho da lavoura
├── gerar_dataset.py       # Gera a pasta dataset/ (30 lavouras 60x60)
├── gerar_lavoura.py       # Exemplo 30x30 para o Gephi (nodes.csv e edges.csv)
├── gerar_gexf.py          # Exemplo 30x30 para o Gephi (lavoura.gexf)
├── requirements.txt       # Dependências Python
└── dataset/
    ├── lavouras_60x60/    # 30 CSVs: linha, coluna, infestação verdadeira e observada
    ├── exemplo_30x30/     # Lavoura de exemplo em formato de grafo (Gephi)
    └── resultados.txt     # Saída de referência dos experimentos
```

Os scripts geram as lavouras com sementes fixas, então os resultados são reproduzíveis. Os CSVs de `dataset/` são uma cópia publicada dessas mesmas lavouras.


## 🔧 Como Executar

⚠️ **Requisito:** Python 3 (testado no Ubuntu 24.04).

```bash
# Dependências do sistema (Ubuntu)
$ sudo apt update
$ sudo apt install -y python3 python3-venv python3-pip python3-tk

# Clone o repositório e instale as dependências Python
$ git clone https://github.com/Bernardo-Lebron/AEDSII-Trabalho-Final.git
$ cd AEDSII-Trabalho-Final
$ python3 -m venv .venv
$ source .venv/bin/activate
$ pip install -r requirements.txt

# Execute os scripts
$ python3 componentes_bfs.py   # focos por limiar (BFS)
$ python3 rota_drone.py        # rota do drone x varredura total
$ python3 experimentos.py      # cobertura, falsos positivos e economia
$ python3 escalabilidade.py    # tempo x tamanho; gera escalabilidade.png
$ python3 gerar_dataset.py     # gera a pasta dataset/
```

### Simulador

```bash
$ python3 simulador_drone.py              # animação (precisa de janela gráfica)
$ python3 simulador_drone.py --salvar     # sem janela: imprime o relatório e salva simulador_final.png
$ python3 simulador_drone.py --semente 7  # outra lavoura (padrão: 42)
```


## 🧪 Resultados

### 1. Economia x cobertura (30 lavouras de 60x60, ruído 0,08)
*   Limiar 0,3: economia de distância de **76,6%**, cobertura de **93,5%** das plantas infestadas e 7,2% de falsos positivos.
*   Limiar 0,7: economia de 88,5%, mas a cobertura cai para 37,4%.
*   **Conclusão:** limiares altos economizam mais, porém deixam plantas infestadas sem tratamento. O limiar precisa equilibrar economia e cobertura.

### 2. Simulação com bateria (lavoura irregular de 2.847 plantas, semente 42)
*   Distância: 1.958,1 m (varredura total) contra 330,2 m (roteamento), economia de **83,1%**.
*   Trocas de bateria: 4 contra **0**.
*   Insumo pulverizado: redução de **90,9%**.

### 3. Escalabilidade
*   O tempo da BFS cresce de forma aproximadamente linear com o número de vértices.
*   Em 640.000 vértices (800x800 plantas), a BFS levou cerca de 0,2 s e o pipeline completo cerca de 0,7 s.
*   Os tempos dependem da máquina; os valores de referência estão em `dataset/resultados.txt`.


## ⛔ Limitações

*   **Dados sintéticos:** os focos têm formato suave e o ruído do sensor é gaussiano. Imagens reais de lavouras têm outros tipos de ruído.
*   **Drone simplificado:** velocidade constante, bateria medida só pela distância, sem vento, curvas ou capacidade do tanque.
*   **TSP exato limitado:** o método por permutações só é viável para poucos focos (até 7); acima disso, usa-se uma heurística sem garantia de ótimo.


## 🔗 Links

*   Código: https://github.com/Bernardo-Lebron/AEDSII-Trabalho-Final
*   Dataset: https://zenodo.org/records/24342470


## 👤 Autor

**Projeto desenvolvido por Bernardo Lebron**
Estudante de Engenharia de Computação – UFCI-MG
Orientação: Prof. Michel Pires da Silva