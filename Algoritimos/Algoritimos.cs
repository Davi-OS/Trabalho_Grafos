using System.Text;
using System.Collections.Generic;
using Trabalho_Grafos.Models;

namespace Trabalho_Grafos.Algoritimos
{
    abstract class Algoritimos
    {
        public static void Executar(Grafo g)
        {
            switch (g.Algoritimo)
            {
                case 1:
                    Console.WriteLine("\nAlgoritimo de Busca em Profundidade");
                    BuscaEmProfundidade(g);
                    break;
                case 2:
                    Console.WriteLine("\nAlgoritimo de Busca em Largura");
                    BuscaEmLargura(g);
                    break;
                case 3:
                    Console.WriteLine("\nAlgoritimo de Dijkstra");
                    Dijkstra(g);
                    break;
                case 4:
                    Console.WriteLine("\nAlgoritimo de Prim");
                    AlgoritmoJarnikPrim(g);
                    break;
                case 5:
                    Console.WriteLine("\nOrdenação Topológica");
                    OrdenacaoTopologica(g);
                    break;
                case 6:
                    Console.WriteLine("\n Algoritimo de  Kruskal");
                    Kruskal(g);
                    break;
                case 7:
                    Console.WriteLine("\nAlgoritmo de Flury");
                    Flury(g);
                    break;
                case 8:
                    Console.WriteLine("\nAlgoritimo de Konig Egervary");
                    KonigEgervary(g);
                    break;
                case 9:
                    Console.WriteLine("\nAlgoritimo de ColoracaoGulosa");
                    ColoracaoGulosa(g);
                    break;
                case 10:
                    Console.WriteLine("\nAlgoritimo de WelshPowell");
                    WelshPowell(g);
                    break;
                case 11:
                    Console.WriteLine("\nAlgoritimo de Brélaz");
                    Brelaz(g);
                    break;
                case 12:
                    Console.WriteLine("\nAlgoritmo de Kosaraju");
                    Kosaraju(g);
                    break;
                case 13:
                    Console.WriteLine("\nAlgoritmo de Kahn");
                    Kahn(g);
                    break;
                case 14:
                    Console.WriteLine("\nAlgoritmo de BellmanFord");
                    BellmanFord(g);
                    break;
                case 15:
                    Console.WriteLine("\nAlgoritmo de FordFulkerson");
                    FordFulkerson(g);
                    break;
                default:
                    Console.WriteLine("Não foi possivel identificar o algoritimo selecionado no Grafo");
                    break;

            }
        }


        //FordFulkerson
        public static bool BFS_FordFulkersen(int[,] matrizResidual, int source, int sink, int[] parent)
        {
            int V = matrizResidual.GetLength(0);
            bool[] visited = new bool[V];
            Queue<int> queue = new Queue<int>();

            queue.Enqueue(source);
            visited[source] = true;
            parent[source] = -1;

            while (queue.Count > 0)
            {
                int u = queue.Dequeue();

                for (int v = 0; v < V; v++)
                {
                    if (visited[v] == false && matrizResidual[u, v] > 0)
                    {
                        if (v == sink)
                        {
                            parent[v] = u;
                            return true;
                        }
                        queue.Enqueue(v);
                        parent[v] = u;
                        visited[v] = true;
                    }
                }
            }
            return false;
        }
        static int FordFulkerson(Grafo g)
        {
            int fonte = 0;
            int destino = g.nVertices - 1;
            int u, v;

            // Criar uma matriz de pesos residual e inicializar como a cópia da matriz de pesos original
            int[,] matrizResidual = new int[g.MatrizPesos.GetLength(0), g.MatrizPesos.GetLength(1)];
            for (u = 0; u < g.MatrizPesos.GetLength(0); u++)
            {
                for (v = 0; v < g.MatrizPesos.GetLength(1); v++)
                {
                    matrizResidual[u, v] = g.MatrizPesos[u, v];
                }
            }

            int[] antecessor = new int[g.MatrizPesos.GetLength(0)];
            int fluxoMaximo = 0;

            // Aumentar o fluxo enquanto existe um caminho do fonte ao destino
            while (BFS_FordFulkersen(matrizResidual, fonte, destino, antecessor))
            {
                // Encontrar a capacidade residual mínima ao longo do caminho preenchido pelo BFS_FordFulkersen
                int caminhoFluxo = int.MaxValue;
                for (v = destino; v != fonte; v = antecessor[v])
                {
                    u = antecessor[v];
                    caminhoFluxo = Math.Min(caminhoFluxo, matrizResidual[u, v]);
                }

                // Atualizar as capacidades residuais dos arcos e arcos reversos ao longo do caminho
                for (v = destino; v != fonte; v = antecessor[v])
                {
                    u = antecessor[v];
                    matrizResidual[u, v] -= caminhoFluxo;
                    matrizResidual[v, u] += caminhoFluxo;
                }

                // Adicionar o fluxo do caminho ao fluxo total
                fluxoMaximo += caminhoFluxo;
            }
            Console.WriteLine("O Fluxo maximo é {0}", fluxoMaximo);
            return fluxoMaximo;
        }

        //Busca em profundidade
        private static void BuscaEmProfundidade(Grafo g)
        {
            bool[] visitado = new bool[g.nVertices];
            StringBuilder caminho = new StringBuilder();

            for (int i = 0; i < g.nVertices; i++)
            {
                if (!visitado[i])
                {
                    DFSUtil(g, i, visitado, caminho);
                }
            }
            caminho.Append(" fim");
            Console.WriteLine("Caminho percorrido:");
            Console.WriteLine(caminho.ToString().TrimEnd(' '));
        }
        private static void DFSUtil(Grafo g, int v, bool[] visitado, StringBuilder caminho)
        {
            // Marca o vértice atual como visitado e adiciona ao caminho
            visitado[v] = true;
            caminho.Append(v).Append(" >> ");

            // Recorre para todos os vértices adjacentes ao vértice v
            foreach (int adj in g.Vertices[v].adjacentes)
            {
                if (!visitado[adj])
                {
                    DFSUtil(g, adj, visitado, caminho);
                }
            }
        }

        //Busca em largura 
        public static void BuscaEmLargura(Grafo g)
        {
            // Marca todos os vértices como não visitados
            bool[] visitado = new bool[g.nVertices];

            // Cria uma fila para a BFS_FordFulkersen
            Queue<int> fila = new Queue<int>();

            // Marca o vértice inicial como visitado e o adiciona à fila
            visitado[0] = true;
            fila.Enqueue(0);

            while (fila.Count != 0)
            {
                // Remove um vértice da fila e o imprime
                int verticeInicial = fila.Dequeue();
                Console.Write(verticeInicial + " >> ");

                // Obtém todos os vértices adjacentes ao vérticeDequeue. Se um vértice adjacente
                // não foi visitado, marca-o como visitado e o adiciona à fila
                foreach (var vizinho in g.Vertices[verticeInicial].adjacentes)
                {
                    if (!visitado[vizinho])
                    {
                        visitado[vizinho] = true;
                        fila.Enqueue(vizinho);
                    }
                }
            }
            Console.Write("fim");

        }

        //ColoracaoGulosa
        private static void ColoracaoGulosa(Grafo g)
        {
            // Array para armazenar as cores atribuídas a cada vértice
            int[] cores = new int[g.nVertices];

            // Inicializa todos os vértices como não coloridos (-1)
            for (int i = 0; i < g.nVertices; i++)
                cores[i] = -1;

            // Atribui a primeira cor ao primeiro vértice
            cores[0] = 0;

            // Disponibiliza cores para os vértices restantes
            for (int u = 1; u < g.nVertices; u++)
            {
                // Array para marcar cores usadas pelos vizinhos de u
                bool[] coresVizinhas = new bool[g.nVertices];
                for (int i = 0; i < g.nVertices; i++)
                    coresVizinhas[i] = false;

                // Verifica as cores usadas pelos vizinhos de u e marca como true
                foreach (var v in g.Vertices[u].adjacentes)
                {
                    if (cores[v] != -1)
                        coresVizinhas[cores[v]] = true;
                }

                // Encontra a menor cor não usada pelos vizinhos de u
                int cor;
                for (cor = 0; cor < g.nVertices; cor++)
                    if (!coresVizinhas[cor])
                        break;

                // Atribui a cor encontrada ao vértice u
                cores[u] = cor;
            }

            // Exibe a coloração dos vértices
            Console.WriteLine("Coloração Gulosa:");
            for (int i = 0; i < g.nVertices; i++)
                Console.WriteLine($"Vértice {i}: Cor {cores[i]}");
        }

        //WelshPowell
        private static void WelshPowell(Grafo g)
        {
            // Ordena os vértices em ordem decrescente de grau
            List<int> verticesOrdenados = Enumerable.Range(0, g.nVertices).OrderByDescending(v => g.Vertices[v].Grau).ToList();

            // Array para armazenar as cores atribuídas a cada vértice
            int[] cores = new int[g.nVertices];

            // Inicializa todos os vértices como não coloridos (-1)
            for (int i = 0; i < g.nVertices; i++)
                cores[i] = -1;

            // Atribui cores aos vértices na ordem decrescente de grau
            foreach (int u in verticesOrdenados)
            {
                // Array para marcar cores usadas pelos vizinhos de u
                bool[] coresVizinhas = new bool[g.nVertices];
                for (int i = 0; i < g.nVertices; i++)
                    coresVizinhas[i] = false;

                // Verifica as cores usadas pelos vizinhos de u e marca como true
                foreach (var v in g.Vertices[u].adjacentes)
                {
                    if (cores[v] != -1)
                        coresVizinhas[cores[v]] = true;
                }

                // Encontra a menor cor não usada pelos vizinhos de u
                int cor;
                for (cor = 0; cor < g.nVertices; cor++)
                    if (!coresVizinhas[cor])
                        break;

                // Atribui a cor encontrada ao vértice u
                cores[u] = cor;
            }

            // Exibe a coloração dos vértices
            Console.WriteLine("Coloração de Welsh-Powell:");
            for (int i = 0; i < g.nVertices; i++)
                Console.WriteLine($"Vértice {i}: Cor {cores[i]}");
        }

        //Brelaz
        private static void Brelaz(Grafo g)
        {
            // Array para armazenar as cores atribuídas a cada vértice
            int[] cores = new int[g.nVertices];

            // Inicializa todos os vértices como não coloridos (-1)
            for (int i = 0; i < g.nVertices; i++)
                cores[i] = -1;

            // Atribui cores aos vértices
            for (int u = 0; u < g.nVertices; u++)
            {
                // Lista para armazenar os vizinhos coloridos de u
                List<int> vizinhosColoridos = new List<int>();

                // Verifica os vizinhos de u e adiciona suas cores à lista
                foreach (var v in g.Vertices[u].adjacentes)
                {
                    if (cores[v] != -1 && !vizinhosColoridos.Contains(cores[v]))
                        vizinhosColoridos.Add(cores[v]);
                }

                // Encontra a menor cor disponível para u
                int cor;
                for (cor = 0; cor < g.nVertices; cor++)
                    if (!vizinhosColoridos.Contains(cor))
                        break;

                // Atribui a cor encontrada ao vértice u
                cores[u] = cor;
            }

            // Exibe a coloração dos vértices
            Console.WriteLine("Coloração de Brélaz:");
            for (int i = 0; i < g.nVertices; i++)
                Console.WriteLine($"Vértice {i}: Cor {cores[i]}");
        }

        //KonigEgervary
        private static void KonigEgervary(Grafo g)
        {
            // Lista para armazenar os pares de vértices do emparelhamento máximo
            List<Tuple<int, int>> emparelhamentoMaximo = new List<Tuple<int, int>>();

            // Marca todos os vértices como não visitados
            bool[] visitado = new bool[g.nVertices];
            for (int u = 0; u < g.nVertices; u++)
                visitado[u] = false;

            // Para cada vértice na partição esquerda do grafo bipartido
            for (int u = 0; u < g.nVertices; u++)
            {
                // Se o vértice ainda não foi emparelhado, procura um caminho aumentador começando de u
                if (!visitado[u])
                {
                    BuscaCaminhoAumentador(u, visitado, emparelhamentoMaximo, g);
                }
            }

            // Exibe o emparelhamento máximo
            Console.WriteLine("Emparelhamento Máximo:");
            foreach (var par in emparelhamentoMaximo)
            {
                Console.WriteLine($"({par.Item1} == {par.Item2})");

            }
        }
        private static bool BuscaCaminhoAumentador(int u, bool[] visitado, List<Tuple<int, int>> emparelhamentoMaximo, Grafo g)
        {
            for (int v = 0; v < g.nVertices; v++)
            {
                // Se existe uma aresta entre u e v e v não foi visitado
                if (g.Vertices[v].adjacentes.Contains(u) && !visitado[v])
                {
                    // Marca v como visitado
                    visitado[v] = true;

                    //Se v  está emparelhado
                    foreach (var par in emparelhamentoMaximo)
                    {
                        if (par.Item1 == v || par.Item2 == v)
                        {
                            return false;
                        }
                    }
                    // Se v não está emparelhado ou se encontramos um caminho aumentador
                    if (emparelhamentoMaximo.All(t => t.Item2 != v) || BuscaCaminhoAumentador(emparelhamentoMaximo.First(t => t.Item2 == v).Item1, visitado, emparelhamentoMaximo, g))
                    {

                        // Adiciona (u, v) ao emparelhamento máximo
                        emparelhamentoMaximo.Add(new Tuple<int, int>(u, v));
                        return true;
                    }
                }
            }
            return false;
        }
       
        //Prim
        public static void AlgoritmoJarnikPrim(Grafo g)
        {
            bool[] inMST = new bool[g.nVertices];  // Array para verificar se um vértice está na MST
            int[] chave = new int[g.nVertices];    // Array para armazenar os pesos mínimos
            int[] parent = new int[g.nVertices];   // Array para armazenar o pai de cada vértice na MST

            // Inicializa todas as chaves com o valor máximo e os pais com -1
            for (int i = 0; i < g.nVertices; i++)
            {
                chave[i] = int.MaxValue;
                parent[i] = -1;
            }

            chave[0] = 0;  // Começa com o primeiro vértice

            // Loop para encontrar a MST
            for (int count = 0; count < g.nVertices - 1; count++)
            {
                int u = MinKey(chave, inMST,g);  // Encontra o vértice com a chave mínima que ainda não está na MST
                inMST[u] = true;  // Adiciona o vértice u à MST
      
                // Atualiza as chaves dos vértices adjacentes de u
                for (int v = 0; v < g.nVertices; v++)
                {
                    if (g.Vertices[u].adjacentes.Contains(v) && !inMST[v] && g.MatrizPesos[u, v] < chave[v])
                    {
                        parent[v] = u;  // Atualiza o pai de v
                        chave[v] = g.MatrizPesos[u, v];  // Atualiza a chave de v
                    }
                }
            }

            PrintMST(parent,g);  // Imprime a MST
        }
        public static int MinKey(int[] chave, bool[] inMST,Grafo g)
        {
            int min = int.MaxValue, minIndex = -1;

            for (int v = 0; v < g.nVertices; v++)
            {
                if (!inMST[v] && chave[v] < min)
                {
                    min = chave[v];
                    minIndex = v;
                }
            }

            return minIndex;
        }
        public static void PrintMST(int[] parent, Grafo g)
        {
            Console.WriteLine("Árvore Geradora Mínima (MST):");
            Console.WriteLine("----------------------------");
            Console.WriteLine("Aresta \t\tPeso");
            Console.WriteLine("----------------------------");
            for (int i = 1; i < g.nVertices; i++)
            {
                Console.WriteLine($"{parent[i]} - {i} \t\t{g.MatrizPesos[i, parent[i]]}");
            }
            Console.WriteLine("----------------------------");

        }

        //Dijkstra
        public static void Dijkstra(Grafo g)
        {

            var distancias = new int[g.nVertices];
            var caminhoMaisCurtoSet = new bool[g.nVertices];

            for (int i = 0; i < g.nVertices; i++)
            {
                distancias[i] = int.MaxValue;
                caminhoMaisCurtoSet[i] = false;
            }

            distancias[0] = 0;

            for (int count = 0; count < g.nVertices - 1; count++)
            {
                int u = DistanciaMinima(distancias, caminhoMaisCurtoSet);
                caminhoMaisCurtoSet[u] = true;

                foreach (var vizinho in g.Vertices[u].adjacentes)
                {
                    int v = vizinho;
                    int peso = g.Vertices[u].distancias[vizinho];

                    if (!caminhoMaisCurtoSet[v] && distancias[u] != int.MaxValue && distancias[u] + peso < distancias[v])
                        distancias[v] = distancias[u] + peso;
                }
            }

            ImprimirDijkstra(distancias);
        }
        public static int DistanciaMinima(int[] distancias, bool[] caminhoMaisCurtoSet)
        {
            int min = int.MaxValue;
            int minIndex = -1;

            for (int v = 0; v < distancias.Length; v++)
            {
                if (!caminhoMaisCurtoSet[v] && distancias[v] <= min)
                {
                    min = distancias[v];
                    minIndex = v;
                }
            }

            return minIndex;
        }
        public static void ImprimirDijkstra(int[] distancias)
        {
            Console.WriteLine("Vértice    Distância do V0");

            for (int i = 0; i < distancias.Length; i++)
                Console.WriteLine(i + " \t\t " + distancias[i]);
        }


        //OrdenaçãoTopologica
        private static void OrdenacaoTopologica(Grafo g)
        {
            int[] inDegree = new int[g.nVertices];

            // Calcula o grau de entrada (in-degree) de cada vértice
            for (int i = 0; i < g.nVertices; i++)
            {
                foreach (int v in g.Vertices[i].adjacentes)
                {
                    inDegree[v]++;
                }
            }

            Queue<int> queue = new Queue<int>();

            // Enfila todos os vértices com grau de entrada zero
            for (int i = 0; i < g.nVertices; i++)
            {
                if (inDegree[i] == 0)
                {
                    queue.Enqueue(i);
                }
            }

            int count = 0;
            List<int> order = new List<int>();

            // Processa todos os vértices na fila
            while (queue.Count > 0)
            {
                int u = queue.Dequeue();
                order.Add(u);

                // Reduz o grau de entrada dos vértices adjacentes e enfila se o grau de entrada se tornar zero
                foreach (int v in g.Vertices[u].adjacentes)
                {
                    if (--inDegree[v] == 0)
                    {
                        queue.Enqueue(v);
                    }
                }

                count++;
            }

            // Verifica se houve um ciclo
            if (count != g.nVertices)
            {
                Console.WriteLine("O grafo contém um ciclo!");
                return;
            }

            // Imprime a ordem topológica
            Console.WriteLine("Ordenação Topológica:");
            foreach (int v in order)
            {
                Console.Write(v + " ");
            }
            Console.WriteLine();
        }


        //Flury
        private static void Flury(Grafo g)
        {
            if (!IsEulerian(g))
            {
                Console.WriteLine("O grafo não possui um circuito euleriano.");
                return;
            }

            var circuito = new List<int>();
            var stack = new Stack<int>();
            var adj = new Dictionary<int, List<int>>();

            for (int i = 0; i < g.nVertices; i++)
            {
                adj[i] = new List<int>(g.Vertices[i].adjacentes);
            }

            int atual = 0;
            stack.Push(atual);

            while (stack.Count > 0)
            {
                if (adj[atual].Count > 0)
                {
                    stack.Push(atual);
                    int prox = adj[atual][0];
                    adj[atual].Remove(prox);
                    adj[prox].Remove(atual);
                    atual = prox;
                }
                else
                {
                    circuito.Add(atual);
                    atual = stack.Pop();
                }
            }

            Console.WriteLine("Circuito Euleriano:");
            Console.WriteLine(string.Join(" >> ", circuito));
        }
        private static bool IsEulerian(Grafo g)
        {
            for (int i = 0; i < g.nVertices; i++)
            {
                if (g.Vertices[i].adjacentes.Count % 2 != 0)
                {
                    return false;
                }
            }
            return true;
        }

        //Kruskal
        private static void Kruskal(Grafo g)
        {
            // Lista para armazenar as arestas da árvore geradora mínima
            List<Tuple<int, int, int>> resultado = new List<Tuple<int, int, int>>();

            // Arrays para representar o conjunto disjunto (Union-Find)
            int[] parent = new int[g.nVertices];
            int[] rank = new int[g.nVertices];

            // Inicializa os arrays antecessor e rank
            for (int v = 0; v < g.nVertices; ++v)
            {
                parent[v] = v;
                rank[v] = 0;
            }

            // Ordena as arestas pelo peso
            g.Arestas.Sort((a, b) => a.Item3.CompareTo(b.Item3));

            // Percorre cada aresta ordenada
            foreach (var aresta in g.Arestas)
            {
                // Encontra os conjuntos dos vértices de origem e destino
                int origemRoot = Find(parent, aresta.Item1);
                int destinoRoot = Find(parent, aresta.Item2);

                // Se não formam um ciclo, adiciona a aresta ao resultado e faz a união dos conjuntos
                if (origemRoot != destinoRoot)
                {
                    resultado.Add(aresta);
                    Union(parent, rank, origemRoot, destinoRoot);
                }
            }

            // Imprime a árvore geradora mínima
            Console.WriteLine("Árvore Geradora Mínima (Kruskal):");
            foreach (var aresta in resultado)
            {
                Console.WriteLine($"{aresta.Item1} -- {aresta.Item2} == {aresta.Item3}");
            }
        }
        // Função para encontrar o conjunto de um elemento usando compressão de caminho
        private static int Find(int[] parent, int i)
        {
            if (parent[i] != i)
            {
                parent[i] = Find(parent, parent[i]);
            }
            return parent[i];
        }
        // Função para unir dois subconjuntos usando união por rank
        private static void Union(int[] parent, int[] rank, int x, int y)
        {
            int xRoot = Find(parent, x);
            int yRoot = Find(parent, y);

            if (rank[xRoot] < rank[yRoot])
            {
                parent[xRoot] = yRoot;
            }
            else if (rank[xRoot] > rank[yRoot])
            {
                parent[yRoot] = xRoot;
            }
            else
            {
                parent[yRoot] = xRoot;
                rank[xRoot]++;
            }
        }


        //Kossaraju
        private static void Kosaraju(Grafo g)
        {
            var pilha = new Stack<int>();

            bool[] visitados = new bool[g.nVertices];

            for(int i = 0; i < g.nVertices; i++)
            {
                visitados[i] = false;
            }

            for(int i = 0; i < g.nVertices; i++)
            {
                if(visitados[i] == false)
                    preenche(i, visitados, pilha, g);
            }

            Grafo gt = g.GetGrafoTransposto();

            for(int i = 0; i < g.nVertices; i++)
            {
			    visitados[i] = false;
            }

            while (pilha.Count > 0)
            {
                // Obtém o elemento do topo
                int v = pilha.Pop();

                // Imprime cada componente fortemente conexa
                if (!visitados[v])
                {
                    DFS_Kosaraju(v, visitados, gt);
                    Console.WriteLine();
                }
            }
        }
        public static void preenche(int v, bool[] visitados, Stack<int> pilha, Grafo g)
        {
            // Marca o vértice atual como visitado
            visitados[v] = true;

            // Percorre os vértices adjacentes de v na matriz de adjacência
            for (int i = 0; i < g.nVertices; i++)
            {
                if (g.MatrizADJ[v, i] != 0 && !visitados[i])
                {
                    preenche(i, visitados, pilha, g);
                }
            }

            // Empilha o vértice v
            pilha.Push(v);
        }
        public static void DFS_Kosaraju(int v, bool[] visitados, Grafo gt)
        {
            // Marca o vértice atual como visitado
            visitados[v] = true;

            // Imprime o vértice
            Console.Write(v + " ");

            // Percorre os vértices adjacentes de v na matriz de adjacência
            for (int i = 0; i < gt.nVertices; i++)
            {
                if (gt.MatrizADJ[v, i] != 0 && !visitados[i])
                {
                    DFS_Kosaraju(i, visitados, gt);
                }
            }
        }

        //Kahn
        public static void Kahn(Grafo g)
        {
            int[] inDegree = new int[g.nVertices];

            // Inicializar todos os graus de entrada como 0
            for (int i = 0; i < g.nVertices; i++)
            {
                for (int j = 0; j < g.nVertices; j++)
                {
                    if (g.MatrizADJ[i, j] != 0)
                    {
                        inDegree[j]++;
                    }
                }
            }

            Queue<int> queue = new Queue<int>();

            // Enfileira todos os vértices com grau de entrada 0
            for (int i = 0; i < g.nVertices; i++)
            {
                if (inDegree[i] == 0)
                {
                    queue.Enqueue(i);
                }
            }

            int count = 0;
            List<int> topOrder = new List<int>();

            // Processar até que a fila esteja vazia
            while (queue.Count > 0)
            {
                int u = queue.Dequeue();
                topOrder.Add(u);

                // Percorre todos os vértices adjacentes a este vértice e reduz seus graus de entrada em 1
                for (int i = 0; i < g.nVertices; i++)
                {
                    if (g.MatrizADJ[u, i] != 0)
                    {
                        if (--inDegree[i] == 0)
                        {
                            queue.Enqueue(i);
                        }
                    }
                }
                count++;
            }

            // Verifica se houve um ciclo
            if (count != g.nVertices)
            {
                Console.WriteLine("O grafo contém um ciclo");
                return;
            }

            // Imprimir a ordem topológica
            foreach (var vertex in topOrder)
            {
                Console.Write(vertex + " ");
            }
            Console.WriteLine();
        }

        //BellmanFord
        public static void BellmanFord(Grafo grafo)
        {
            int V = grafo.nVertices;
            int E = grafo.Arestas.Count;
            int source = 0; // Assume que o vértice de origem é o 0
            int[] dist = new int[V];

            // Passo 1: Inicializar as distâncias de todos os vértices como infinito, exceto o vértice de origem
            for (int i = 0; i < V; i++)
                dist[i] = int.MaxValue;
            dist[source] = 0;

            // Passo 2: Relaxar todas as arestas |V| - 1 vezes
            for (int i = 1; i < V; ++i)
            {
                for (int j = 0; j < E; ++j)
                {
                    int u = grafo.Arestas[j].Item1;
                    int v = grafo.Arestas[j].Item2;
                    int weight = grafo.Arestas[j].Item3;
                    if (dist[u] != int.MaxValue && dist[u] + weight < dist[v])
                        dist[v] = dist[u] + weight;
                }
            }

            // Passo 3: Verificar a existência de ciclos negativos
            for (int j = 0; j < E; ++j)
            {
                int u = grafo.Arestas[j].Item1;
                int v = grafo.Arestas[j].Item2;
                int weight = grafo.Arestas[j].Item3;
                if (dist[u] != int.MaxValue && dist[u] + weight < dist[v])
                {
                    Console.WriteLine("O grafo contém um ciclo negativo");
                    return;
                }
            }

            // Imprimir todas as distâncias
            Console.WriteLine("Vértice   Distância da Origem");
            for (int i = 0; i < V; ++i)
                Console.WriteLine("{0}\t  {1}", i, dist[i] == int.MaxValue ? "INF" : dist[i].ToString());
        }
    }
}
