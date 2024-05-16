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
        public Vertice[] Vertices;
        public int[,] MatrizADJ;
        public Grafo(int alg, int nV, int[,] matriz)
        {
            Algoritimo = alg;
            MatrizADJ = matriz;
            nVertices = nV;
            Vertices = new Vertice[nVertices];

            for (int i = 0; i < nVertices; i++)
            {
                List<int> l = new List<int>();
                for (int j = 0; j < nVertices; j++)
                {
                    if (matriz[i, j] != 0)
                    {
                        l.Add(j);
                    }
                }
                Vertices[i] = new Vertice(l);
            }
            ExibeListaADJ();
            ExibeMatriz();
            ExibeGrauDeV();
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
        public void ExibeMatriz()
        {
            Console.WriteLine("\nExibindo Matriz de adjacencia:");
            for (int i = 0; i < nVertices; i++)
            {
                for (int j = 0; j < nVertices; j++)
                {
                    Console.Write(MatrizADJ[i, j]+" ");
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
