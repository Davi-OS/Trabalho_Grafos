using Trabalho_Grafos.IO;
using Trabalho_Grafos.Models;

namespace Trabalho_Grafos
{
    public class Program
    {
        static void Main(string[] args)
        {
            // TODO: inicia a leitura
            Grafo g = Leitura.LerGrafo();
            Algoritimos.Algoritimos.Executar(g);
        }
    }
}
