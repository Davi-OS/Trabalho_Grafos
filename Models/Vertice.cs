using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Trabalho_Grafos.Models
{
    public class Vertice
    {
        public int Grau;

        public List<int> adjacentes;

        public Dictionary<int, int> distancias;

        public bool isColored = false;

        public int Cor;
        public Vertice(List<int> adj,List<int> pes)
        {
            Cor = 0;
            adjacentes = adj;
            Grau = adjacentes.Count;

            distancias = new Dictionary<int, int>();

            for (int i = 0; i < adj.Count; i++)
            {
                distancias[adj[i]] = pes[i];
            }
        }
    }
}
