using System;
using System.Windows;
using HelixToolkit.Wpf;
using System.Windows.Media.Media3D;

namespace GPG_Csharp
{
    /// <summary>
    /// Interaction logic for PreviewWindow.xaml
    /// </summary>
    public partial class PreviewWindow : Window
    {
        public Model3D model { get; set; }
        private PointsVisual3D points;
        private Transform3D originalTransform;

        public PreviewWindow(PointsVisual3D points)
        {
            InitializeComponent();
            this.points = points;

            double factor = 2.0;
            var transformGroup = new Transform3DGroup();
            transformGroup.Children.Add(new ScaleTransform3D(factor, factor, factor/2));
            var rotationX = new AxisAngleRotation3D(new Vector3D(1, 0, 0), -90);
            var rotationY = new AxisAngleRotation3D(new Vector3D(0, 1, 0), 0);
            var rotationZ = new AxisAngleRotation3D(new Vector3D(0, 0, 1), 90);
            transformGroup.Children.Add(new RotateTransform3D(rotationX));
            transformGroup.Children.Add(new RotateTransform3D(rotationY));
            transformGroup.Children.Add(new RotateTransform3D(rotationZ));
            this.points.Transform = transformGroup;
            originalTransform = this.points.Transform;

            viewport.Children.Add(this.points);

            //viewport.Camera.LookAt(new Point3D(-1.3, 0, 0), 2);
        }

        private void rotatePointCloud(object sender, RoutedEventArgs e)
        {
            //var transformGroup = new Transform3DGroup();
            //transformGroup.Children.Add(originalTransform); //keeping old transforms
            //var rotationX = new AxisAngleRotation3D(new Vector3D(1, 0, 0), double.Parse(rotX.Text));
            //var rotationY = new AxisAngleRotation3D(new Vector3D(0, 1, 0), double.Parse(rotY.Text));
            //var rotationZ = new AxisAngleRotation3D(new Vector3D(0, 0, 1), double.Parse(rotZ.Text));
            //transformGroup.Children.Add(new RotateTransform3D(rotationX));
            //transformGroup.Children.Add(new RotateTransform3D(rotationY));
            //transformGroup.Children.Add(new RotateTransform3D(rotationZ));
            ////transformGroup.Children.Add(new ScaleTransform3D(double.Parse(rotX.Text), double.Parse(rotY.Text), double.Parse(rotZ.Text)));
            //points.Transform = transformGroup;
            Console.WriteLine(viewport.Camera.LookDirection);
        }
    }
}
