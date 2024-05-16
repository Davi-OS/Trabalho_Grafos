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
                    break;
                default:
                    Console.WriteLine("Não foi possivel identificar o algoritimo selecionado no Grafo");
                    break;
            }
        }

        //TODO: Implementar os algoritimos aqui
    }
}
