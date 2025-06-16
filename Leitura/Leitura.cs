using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.IO;
using Trabalho_Grafos.Models;

namespace Trabalho_Grafos.IO
{
    abstract class Leitura
    {

        public static Grafo LerGrafo()
        {

            string filepath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Leitura", "input.txt");
            var lines = File.ReadAllLines($@"{filepath}");
            StreamReader sr = new StreamReader(filepath);
            
            int algoritimo = int.Parse(sr.ReadLine());
            string[] ln = sr.ReadLine().Split(" ");
            int numeroVertice = int.Parse(ln[0]);
            int numeroAresta = int.Parse(ln[1]);
            int[,] matriz = new int[numeroVertice, numeroVertice];
            int[,] matrizPesos = new int[numeroVertice, numeroVertice];
            List<Tuple<int, int, int>> Arestas = new List<Tuple<int, int, int>>();

            for (int i = 0; i < numeroAresta; i++)
            {
                ln = sr.ReadLine().Split(" ");
                matriz[int.Parse(ln[0]),int.Parse(ln[1])] = 1;
                matrizPesos[int.Parse(ln[0]), int.Parse(ln[1])]= int.Parse(ln[2]);
                Arestas.Add(new Tuple<int, int, int>(int.Parse(ln[0]), int.Parse(ln[1]), int.Parse(ln[2])));
            }

            sr.Close();
            Grafo g = new Grafo(algoritimo, numeroVertice,numeroAresta, matriz, matrizPesos,Arestas);

            return g;

        }
    }
}
