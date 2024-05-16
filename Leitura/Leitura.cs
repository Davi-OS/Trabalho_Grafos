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

        public static void LerGrafo()
        {

            string filepath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Leitura", "input.txt");
            var lines = File.ReadAllLines($@"{filepath}");
            StreamReader sr = new StreamReader(filepath);
            
            int algoritimo = int.Parse(sr.ReadLine());
            int numeroVertice = int.Parse(sr.ReadLine());
            int[,] matriz = new int[numeroVertice, numeroVertice];
            for (int i = 0; i < numeroVertice; i++)
            {
                string[] linha = sr.ReadLine().Split(" ");
                for (int j = 0; j < numeroVertice; j++)
                {
                    matriz[i, j] = int.Parse(linha[j]);
                }
            }
            sr.Close();
            Grafo g = new Grafo(algoritimo, numeroVertice, matriz);
            /*
                int algoritimo = int.Parse(Console.ReadLine());
                int numeroVertice = int.Parse(Console.ReadLine());
                int[,] matriz = new int[numeroVertice, numeroVertice];
                for (int i = 0; i < numeroVertice; i++)
                {
                    string[] linha = Console.ReadLine().Split(" ");
                    for (int j = 0; j < numeroVertice; j++)
                    {
                        matriz[i, j] = int.Parse(linha[j]);
                    }
                }
                Grafo g = new Grafo(algoritimo, numeroVertice, matriz);
            */
        }
    }
}
