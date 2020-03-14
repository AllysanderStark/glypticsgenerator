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
using PclSharp;
using PclSharp.Struct;
using PclSharp.Filters;
using System.Numerics;

namespace GPG_Csharp
{
    /// <summary>
    /// Interaction logic for CapturePage.xaml
    /// </summary>
    public partial class CapturePage : Page
    {
        private Pipeline pipeline = new Pipeline();
        private Colorizer colorizer = new Colorizer();
        private Align align = new Align(Stream.Color);
        private CustomProcessingBlock block;
        private CancellationTokenSource tokenSource = new CancellationTokenSource();
        private PointCloud pc = new PointCloud();

        // Filters
        private DecimationFilter decFilter = new DecimationFilter();
        private ThresholdFilter thresholdFilter = new ThresholdFilter();
        private HoleFillingFilter holeFillingFilter = new HoleFillingFilter();
        private SpatialFilter spatialFilter = new SpatialFilter();
        private TemporalFilter temporalFilter = new TemporalFilter();

        static Action<VideoFrame> UpdateImage(Image img)
        {
            var wbmp = img.Source as WriteableBitmap;
            return new Action<VideoFrame>(frame =>
            {
                var rect = new Int32Rect(0, 0, frame.Width, frame.Height);
                wbmp.WritePixels(rect, frame.Data, frame.Stride * frame.Height, frame.Stride);
            });
        }

        public CapturePage()
        {
            InitializeComponent();

            //imgColor.LayoutTransform = new RotateTransform(180);
            //imgDepth.LayoutTransform = new RotateTransform(180);

            spatialFilter.Options[Option.FilterMagnitude].Value = 5.0f;
            //spatialFilter.Options[Option.HolesFill].Value = 1.0f;
            colorizer.Options[Option.HistogramEqualizationEnabled].Value = 1f;
            colorizer.Options[Option.MinDistance].Value = 0.4f;
            colorizer.Options[Option.MaxDistance].Value = 0.5f;
            thresholdFilter.Options[Option.MinDistance].Value = 0.4f;
            thresholdFilter.Options[Option.MaxDistance].Value = 0.5f;
            try
            {
                //Action<VideoFrame> updateDepth;
                //Action<VideoFrame> updateColor;


                //var pp = pipeline.Start();
                //FrameSet frames = pipeline.WaitForFrames();
                //SetupWindow(pp, out updateDepth);
                //using (var df = frames.FirstOrDefault(Stream.Depth))
                //{
                //    var colorizedDepth = colorizer.Process<VideoFrame>(df);
                //    Dispatcher.Invoke(DispatcherPriority.Render, updateDepth, colorizedDepth);
                //}


                var cfg = new Config();
                cfg.EnableStream(Stream.Depth, 640, 480);
                cfg.EnableStream(Stream.Color, Format.Rgb8);

                var pp = pipeline.Start(cfg);

                // Get the recommended processing blocks for the depth sensor
                var sensor = pp.Device.QuerySensors<Sensor>().First(s => s.Is(Extension.DepthSensor));
                var blocks = sensor.ProcessingBlocks.ToList();

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
                        f = f.ApplyFilter(thresholdFilter).DisposeWith(releaser);
                        f = f.ApplyFilter(temporalFilter).DisposeWith(releaser);
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

                //Register to results of processing via a callback:
                block.Start(f =>
                {
                    using (var frames = f.As<FrameSet>())
                    {
                        var colorFrame = frames.ColorFrame.DisposeWith(frames);
                        var colorizedDepth = frames.First<VideoFrame>(Stream.Depth).DisposeWith(frames);

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


                //SetupWindow(pp, out updateDepth, out updateColor);

                //Task.Factory.StartNew(() =>
                //{
                //    while (!tokenSource.Token.IsCancellationRequested)
                //    {
                //        // We wait for the next available FrameSet and using it as a releaser object that would track
                //        // all newly allocated .NET frames, and ensure deterministic finalization
                //        // at the end of scope. 
                //        using (var frames = pipeline.WaitForFrames())
                //        {
                //            var colorFrame = frames.ColorFrame.DisposeWith(frames);
                //            var depthFrame = frames.DepthFrame.DisposeWith(frames);

                //            // We colorize the depth frame for visualization purposes
                //            var processedDepth = align.Process<VideoFrame>(depthFrame).DisposeWith(frames);
                //            processedDepth = colorizer.Process<VideoFrame>(processedDepth).DisposeWith(frames);
                //            //processedDepth = decFilter.Process<VideoFrame>(processedDepth).DisposeWith(frames);
                //            //processedDepth = spatialFilter.Process<VideoFrame>(processedDepth).DisposeWith(frames);
                //            processedDepth = thresholdFilter.Process<VideoFrame>(processedDepth).DisposeWith(frames);
                //            //pc.MapTexture(colorFrame);
                //            //var test = pc.Process(depthFrame).As<Points>();

                //            // Render the frames.
                //            Dispatcher.Invoke(DispatcherPriority.Render, updateDepth, processedDepth);
                //            Dispatcher.Invoke(DispatcherPriority.Render, updateColor, colorFrame);
                //        }
                //    }
                //}, tokenSource.Token);
                
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

        //private void SetupWindow(PipelineProfile pipelineProfile, out Action<VideoFrame> depth, out Action<VideoFrame> color)
        //{
        //    using (var p = pipelineProfile.GetStream(Stream.Depth).As<VideoStreamProfile>())
        //        imgDepth.Source = new WriteableBitmap(p.Width, p.Height, 96d, 96d, PixelFormats.Rgb24, null);
        //    depth = UpdateImage(imgDepth);

        //    using (var p = pipelineProfile.GetStream(Stream.Color).As<VideoStreamProfile>())
        //        imgColor.Source = new WriteableBitmap(p.Width, p.Height, 96d, 96d, PixelFormats.Rgb24, null);
        //    color = UpdateImage(imgColor);
        //}

        private void CaptureFrame(object sender, RoutedEventArgs e)
        {
            //tokenSource.Cancel();
            var frames = pipeline.WaitForFrames();
            var df = frames.DepthFrame;
            var filteredDepth = df.As<Intel.RealSense.Frame>();

            filteredDepth = decFilter.Process(filteredDepth);
            //filteredDepth = spatialFilter.Process(filteredDepth);
            //filteredDepth = holeFillingFilter.Process(filteredDepth);
            filteredDepth = temporalFilter.Process(filteredDepth);

            var cf = frames.ColorFrame;
            pc.MapTexture(cf);

            //var filteredPoints = pc.Process(filteredDepth).As<Points>();
            //filteredPoints.ExportToPLY("filtered.ply", colorizer.Process<VideoFrame>(filteredDepth));

            //var points = pc.Process(df).As<Points>();
            var points = pc.Process(filteredDepth).As<Points>();

            var vertices = new Intel.RealSense.Math.Vertex[points.Count];
            points.CopyVertices(vertices);

            points.ExportToPLY("raw.ply", colorizer.Process<VideoFrame>(df));

            var newPoints = vertices.
                Where(p => (p.z < 0.5) && (p.z > 0.4)).
                Select(p => new Point3D(p.x, p.y, p.z));
            var xMed = newPoints.Average(p => p.X);
            var yMed = newPoints.Average(p => p.Y);
            newPoints = newPoints.Select(p => new Point3D(p.X - xMed, p.Y - yMed, p.Z)); // Can apply ScaleZ here
            newPoints = newPoints.Where(p => (p.X * p.X + p.Y * p.Y <= 0.025)); // Circular shape

            // Creating a PCL point cloud
            var pcl = new PointCloudOfXYZ();
            foreach (var point in newPoints)
            {
                pcl.Add(new Vector3((float) point.X, (float) point.Y, (float) point.Z));
            }

            var downsampledPC = new PointCloudOfXYZ();

            var downsampleFilter = new VoxelGridOfXYZ();
            downsampleFilter.SetInputCloud(pcl);
            downsampleFilter.LeafSize = new Vector3(0.01f, 0.01f, 0.01f);
            downsampleFilter.filter(downsampledPC);

            //var filter = new StatisticalOutlierRemovalOfXYZ();
            //filter.SetInputCloud(pcl);
            //filter.filter(filteredPC);

            var helixPoints = new PointsVisual3D();
            foreach (var p in downsampledPC.Points)
            {
                helixPoints.Points.Add(new Point3D(p.X, p.Y, p.Z));
            }

            helixPoints.Color = Colors.White;
            NavigationService.Navigate(new PreviewPage(helixPoints));
        }
    }
}
