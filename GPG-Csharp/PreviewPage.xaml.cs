using System;
using System.Windows;
using HelixToolkit.Wpf;
using System.Windows.Media.Media3D;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Controls;

namespace GPG_Csharp
{
    /// <summary>
    /// Interaction logic for PreviewPage.xaml
    /// </summary>
    public partial class PreviewPage : Page
    {
        public Model3D model { get; set; }
        private PointsVisual3D points;
        private Transform3D originalTransform;

        private double mousePosX = 0;

        public PreviewPage(PointsVisual3D points)
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
        }

        private void retryCapture(object sender, RoutedEventArgs e)
        {
            NavigationService.GoBack();
        }

        protected override void OnPreviewMouseMove(MouseEventArgs e)
        {
            if (e.RightButton == MouseButtonState.Pressed)
            {
                double deltaX = mousePosX - e.GetPosition(viewport).X;
                var transformGroup = new TransformGroup();
                transformGroup.Children.Add(bgImage.RelativeTransform);
                transformGroup.Children.Add(new TranslateTransform(deltaX * 0.0035, 0));
                bgImage.RelativeTransform = transformGroup;
            }
            mousePosX = e.GetPosition(viewport).X;
            base.OnPreviewMouseMove(e);
        }
    }
}
