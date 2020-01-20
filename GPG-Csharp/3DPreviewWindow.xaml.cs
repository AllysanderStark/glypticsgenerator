using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using Intel.RealSense;
using HelixToolkit;
using HelixToolkit.Wpf;
using System.Windows.Media.Media3D;

namespace GPG_Csharp
{
    /// <summary>
    /// Interaction logic for _3DPreviewWindow.xaml
    /// </summary>
    public partial class _3DPreviewWindow : Window
    {
        public Model3D model { get; set; }

        public _3DPreviewWindow()
        {
            InitializeComponent();

            ModelImporter importer = new ModelImporter();

            Material material = new DiffuseMaterial(new SolidColorBrush(Colors.Beige));
            importer.DefaultMaterial = material;

            //model = importer.Load(@"/raw.ply");
            //viewport.Children.Add(model.);
        }
    }
}
