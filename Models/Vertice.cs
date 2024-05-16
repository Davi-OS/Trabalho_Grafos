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

        public bool isColored = false;

        public int Cor;
        public Vertice(List<int> adj)
        {
            Cor = 0;
            adjacentes = adj;
            Grau = adjacentes.Count;
        }
    }
}
