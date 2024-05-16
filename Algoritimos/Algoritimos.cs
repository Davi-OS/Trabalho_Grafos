using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
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
                default:
                    Console.WriteLine("Não foi possivel identificar o algoritimo selecionado no Grafo");
                    break;
            }
        }


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
            Console.WriteLine( caminho.ToString().TrimEnd(' '));
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

        //TODO: Implementar os algoritimos aqui
    }
}
