using HelixToolkit.Wpf;
using System.Windows;

namespace GPG_Csharp
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
            _mainFrame.Navigate(new CapturePage());
        }
    }
}
