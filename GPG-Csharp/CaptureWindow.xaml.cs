using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Threading;
using Intel.RealSense;
using HelixToolkit.Wpf;
using System.Windows.Media.Media3D;

namespace GPG_Csharp
{
    /// <summary>
    /// Interaction logic for Window.xaml
    /// </summary>
    public partial class CaptureWindow : Window
    {
        private Pipeline pipeline = new Pipeline();
        private Colorizer colorizer = new Colorizer();
        private Align align = new Align(Stream.Color);
        private CustomProcessingBlock block;
        private CancellationTokenSource tokenSource = new CancellationTokenSource();
        private PointCloud pc = new PointCloud();

        // Filters
        private DecimationFilter decFilter = new DecimationFilter();
        private SpatialFilter spatialFilter = new SpatialFilter();
        private TemporalFilter temporalFilter = new TemporalFilter();
        private HoleFillingFilter holeFillingFilter = new HoleFillingFilter();

        static Action<VideoFrame> UpdateImage(Image img)
        {
            var wbmp = img.Source as WriteableBitmap;
            return new Action<VideoFrame>(frame =>
            {
                var rect = new Int32Rect(0, 0, frame.Width, frame.Height);
                wbmp.WritePixels(rect, frame.Data, frame.Stride * frame.Height, frame.Stride);
            });
        }

        public CaptureWindow()
        {
            InitializeComponent();

            //spatialFilter.Options[Option.FilterMagnitude].Value = 2.0f;

            try
            {
                //Action<VideoFrame> updateDepth;
                //Action<VideoFrame> updateColor;

                /*
                var pp = pipe.Start();
                FrameSet frames = pipe.WaitForFrames();
                SetupWindow(pp, out updateDepth);
                using (var df = frames.FirstOrDefault(Stream.Depth))
                {
                    var colorizedDepth = colorizer.Process<VideoFrame>(df);
                    Dispatcher.Invoke(DispatcherPriority.Render, updateDepth, colorizedDepth);
                }
                */

                var cfg = new Config();
                cfg.EnableStream(Stream.Depth, 640, 480);
                cfg.EnableStream(Stream.Color, Format.Rgb8);

                var pp = pipeline.Start(cfg);

                // Get the recommended processing blocks for the depth sensor
                var sensor = pp.Device.QuerySensors<Sensor>().First(s => s.Is(Extension.DepthSensor));
                var blocks = sensor.ProcessingBlocks.ToList();

                //sensor.Options[Option.HolesFill].Value = 1;

                // Allocate bitmaps for rendring.
                // Since the sample aligns the depth frames to the color frames, both of the images will have the color resolution
                using (var p = pp.GetStream(Stream.Color).As<VideoStreamProfile>())
                {
                    imgColor.Source = new WriteableBitmap(p.Width, p.Height, 96d, 96d, PixelFormats.Rgb24, null);
                    imgDepth.Source = new WriteableBitmap(p.Width, p.Height, 96d, 96d, PixelFormats.Rgb24, null);
                }
                var updateColor = UpdateImage(imgColor);
                var updateDepth = UpdateImage(imgDepth);

                // Create custom processing block
                // For demonstration purposes it will:
                // a. Get a frameset
                // b. Run post-processing on the depth frame
                // c. Combine the result back into a frameset
                // Processing blocks are inherently thread-safe and play well with
                // other API primitives such as frame-queues, 
                // and can be used to encapsulate advanced operations.
                // All invocations are, however, synchronious so the high-level threading model
                // is up to the developer

                block = new CustomProcessingBlock((f, src) =>
                {
                    // We create a FrameReleaser object that would track
                    // all newly allocated .NET frames, and ensure deterministic finalization
                    // at the end of scope. 
                    using (var releaser = new FramesReleaser())
                    {
                        foreach (ProcessingBlock p in blocks)
                            f = p.Process(f).DisposeWith(releaser);

                        f = f.ApplyFilter(align).DisposeWith(releaser);
                        f = f.ApplyFilter(colorizer).DisposeWith(releaser);

                        var frames = f.As<FrameSet>().DisposeWith(releaser);

                        var colorFrame = frames[Stream.Color, Format.Rgb8].DisposeWith(releaser);
                        var colorizedDepth = frames[Stream.Depth, Format.Rgb8].DisposeWith(releaser);

                        // Combine the frames into a single result
                        var res = src.AllocateCompositeFrame(colorizedDepth, colorFrame).DisposeWith(releaser);
                        // Send it to the next processing stage
                        src.FrameReady(res);
                    }
                });

                // Register to results of processing via a callback:
                block.Start(f =>
                {
                    using (var frames = f.As<FrameSet>())
                    {
                        var colorFrame = frames.ColorFrame.DisposeWith(frames);
                        var colorizedDepth = frames.First<VideoFrame>(Stream.Depth, Format.Rgb8).DisposeWith(frames);

                        Dispatcher.Invoke(DispatcherPriority.Render, updateDepth, colorizedDepth);
                        Dispatcher.Invoke(DispatcherPriority.Render, updateColor, colorFrame);
                    }
                });

                var token = tokenSource.Token;

                var t = Task.Factory.StartNew(() =>
                {
                    while (!token.IsCancellationRequested)
                    {
                        using (var frames = pipeline.WaitForFrames())
                        {
                            // Invoke custom processing block
                            block.Process(frames);
                        }
                    }
                }, token);

                /*
                SetupWindow(pp, out updateDepth, out updateColor);

                Task.Factory.StartNew(() =>
                {
                    while (!tokenSource.Token.IsCancellationRequested)
                    {
                        // We wait for the next available FrameSet and using it as a releaser object that would track
                        // all newly allocated .NET frames, and ensure deterministic finalization
                        // at the end of scope. 
                        using (var frames = pipeline.WaitForFrames())
                        {
                            var colorFrame = frames.ColorFrame.DisposeWith(frames);
                            var depthFrame = frames.DepthFrame.DisposeWith(frames);

                            // We colorize the depth frame for visualization purposes
                            //var colorizedDepth = colorizer.Process<VideoFrame>(depthFrame).DisposeWith(frames);
                            var filteredDepth = decFilter.Process<VideoFrame>(depthFrame);
                            //filteredDepth = spatialFilter.Process<VideoFrame>(filteredDepth);
                            //pc.MapTexture(colorFrame);
                            //var test = pc.Process(depthFrame).As<Points>();

                            // Render the frames.
                            //Dispatcher.Invoke(DispatcherPriority.Render, updateDepth, colorizedDepth);
                            Dispatcher.Invoke(DispatcherPriority.Render, updateDepth, filteredDepth);
                            Dispatcher.Invoke(DispatcherPriority.Render, updateColor, colorFrame);
                        }
                    }
                }, tokenSource.Token);
                */
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
                Application.Current.Shutdown();
            }
        }

        private void control_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            tokenSource.Cancel();
        }

        private void SetupWindow(PipelineProfile pipelineProfile, out Action<VideoFrame> depth, out Action<VideoFrame> color)
        {
            using (var p = pipelineProfile.GetStream(Stream.Depth).As<VideoStreamProfile>())
                imgDepth.Source = new WriteableBitmap(p.Width, p.Height, 96d, 96d, PixelFormats.Rgb24, null);
            depth = UpdateImage(imgDepth);
            
            using (var p = pipelineProfile.GetStream(Stream.Color).As<VideoStreamProfile>())
                imgColor.Source = new WriteableBitmap(p.Width, p.Height, 96d, 96d, PixelFormats.Rgb24, null);
            color = UpdateImage(imgColor);
        }

        private void CaptureFrame(object sender, RoutedEventArgs e)
        {
            tokenSource.Cancel();
            var frames = pipeline.WaitForFrames();
            var df = frames.DepthFrame;
            //var filteredDepth = df.As<Frame>();

            //filteredDepth = decFilter.Process(filteredDepth);
            //filteredDepth = spatialFilter.Process(filteredDepth);
            //filteredDepth = temporalFilter.Process(filteredDepth);
            //filteredDepth = holeFillingFilter.Process(filteredDepth);

            var cf = frames.ColorFrame;
            pc.MapTexture(cf);

            //var filteredPoints = pc.Process(filteredDepth).As<Points>();
            //filteredPoints.ExportToPLY("filtered.ply", colorizer.Process<VideoFrame>(filteredDepth));

            var points = pc.Process(df).As<Points>();

            var vertices = new Intel.RealSense.Math.Vertex[points.Count];
            points.CopyVertices(vertices);

            //var xs = vertices.Select(x => x.x);
            //var xmin = xs.Min(); var xmax = xs.Max();
            //var ys = vertices.Select(x => x.y);
            //var ymin = ys.Min(); var ymax = ys.Max();
            //var zs = vertices.Select(x => x.z);
            //var zmin = zs.Min(); var zmax = zs.Max();
            //Console.WriteLine("X: " + xmin + " - " + xmax + ", Y: " + ymin + " - " + ymax + ", Z: " + zmin + " - " + zmax);

            points.ExportToPLY("raw.ply", colorizer.Process<VideoFrame>(df));

            //var newVertices = new List<Math.Vertex>(); //[points.Count]
            //var pcl = new PointCloudOfXYZ();
            var helixPoints = new PointsVisual3D();
            helixPoints.Color = Colors.White;
            var newPoints = vertices.Where(x => (x.z < 0.5) && (x.z > 0.4)).Select(p => new Point3D(p.x, p.y, p.z));
            helixPoints.Points = new Point3DCollection(newPoints);

            //var newPc = new PointCloud();

            //var downsampledPC = new PointCloudOfXYZ();
            //pcl.Downsample(4, downsampledPC);

            //var filter = new PclSharp.Filters.StatisticalOutlierRemovalOfXYZ();
            //var filteredPC = new PointCloudOfXYZ();
            //filter.SetInputCloud(downsampledPC);
            //filter.filter(filteredPC);

            //var helixPoints = new PointsVisual3D();
            //foreach (var p in filteredPC.Points)
            //{
            //    helixPoints.Points.Add(new System.Windows.Media.Media3D.Point3D(p.X, p.Y, p.Z));
            //}

            var previewWindow = new PreviewWindow(helixPoints);
            previewWindow.Show();
        }
    }
}
