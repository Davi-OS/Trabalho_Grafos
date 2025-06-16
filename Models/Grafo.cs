using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;

namespace Trabalho_Grafos.Models
{
    public class Grafo
    {
        public int Algoritimo;
        public int nVertices;
        public int nArestas;
        public Vertice[] Vertices;
        public List<Tuple<int, int, int>> Arestas;
        public int[,] MatrizADJ;
        public int[,] MatrizPesos;
        public Grafo(int alg, int nV, int nA, int[,] matriz, int[,] matrizPesos, List<Tuple<int, int, int>> ares, string? transposto = null)
        {
            Algoritimo = alg;
            MatrizADJ = matriz;
            nVertices = nV;
            nArestas = nA;
            Vertices = new Vertice[nVertices];
            Arestas = ares;
            MatrizPesos = matrizPesos;
            // preenchendo vertices
            for (int i = 0; i < nVertices; i++)
            {
                List<int> l = new List<int>();
                List<int> p = new List<int>();
                for (int j = 0; j < nVertices; j++)
                {
                    if (matriz[i, j] != 0 && matrizPesos[i, j] != 0)
                    {
                        l.Add(j);
                        p.Add(matrizPesos[i, j]);
                    }
                }
                Vertices[i] = new Vertice(l, p);
            }

            if(transposto == null){
                ExibeListaADJ();
                ExibeMatrizAdj();
                ExibePesos();
                ExibeGrauDeV();
            }
        }

        public Grafo GetGrafoTransposto()
        {
            int[,] matrizTransposta = new int[nVertices, nVertices];
            int[,] matrizPesosTransposta = new int[nVertices, nVertices];

            // Preenchendo a matriz transposta e a matriz de pesos transposta
            for (int i = 0; i < nVertices; i++)
            {
                for (int j = 0; j < nVertices; j++)
                {
                    matrizTransposta[j, i] = MatrizADJ[i, j];
                    matrizPesosTransposta[j, i] = MatrizPesos[i, j];
                }
            }

            // Construindo a lista de arestas para o grafo transposto
            List<Tuple<int, int, int>> aresTransposto = new List<Tuple<int, int, int>>();
            for (int i = 0; i < nVertices; i++)
            {
                for (int j = 0; j < nVertices; j++)
                {
                    if (matrizTransposta[i, j] != 0 && matrizPesosTransposta[i, j] != 0)
                    {
                        aresTransposto.Add(new Tuple<int, int, int>(i, j, matrizPesosTransposta[i, j]));
                    }
                }
            }

            // Criando e retornando o grafo transposto
            return new Grafo(Algoritimo, nVertices, nArestas, matrizTransposta, matrizPesosTransposta, aresTransposto, "transposto");
        }


        public void ExibeListaADJ()
        {
            Console.WriteLine("\nExibindo lista de adjacencia:");
            for (int i = 0; i < nVertices; i++)
            {
                Console.Write(i + ": ");
                foreach (var item in Vertices[i].adjacentes)
                {
                    Console.Write(item + " ");
                }
                Console.WriteLine();
            }
        }
        public void ExibeMatrizAdj()
        {
            Console.WriteLine("\nExibindo Matriz de adjacencia:\n");
            for (int i = 0; i < nVertices; i++)
            {
                Console.Write(i + ": ");
                for (int j = 0; j < nVertices; j++)
                {
                    Console.Write(MatrizADJ[i, j] + " ");
                }
                Console.WriteLine();
            }
        }
        public void ExibePesos()
        {
            Console.WriteLine("\nExibindo Matriz de Pesos:\n");
            for (int i = 0; i < nVertices; i++)
            {
                Console.Write(i + ": ");
                for (int j = 0; j < nVertices; j++)
                {
                    Console.Write(MatrizPesos[i, j] + " ");
                }
                Console.WriteLine();
            }
        }
        public void ExibeGrauDeV()
        {
            Console.WriteLine("\nExibindo o grau de cada Verticie:");
            for (int i = 0; i < nVertices; i++)
            {
                Console.WriteLine(i + ": " + Vertices[i].Grau);
            }
        }
    }
}
