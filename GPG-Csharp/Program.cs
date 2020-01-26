using System;
using System.Linq;

namespace GPG_Csharp
{
    class Program
    {
        [STAThread]
        static void Main(string[] args)
        {
            var w = new MainWindow();
            w.ShowDialog();
        }
    }
}