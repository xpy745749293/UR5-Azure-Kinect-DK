//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.AzureKinectBasics
{
    using System;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Threading.Tasks;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Azure.Kinect.Sensor;
    using URSDK.RobController.Communication;
    using URSDK.RobController.Datatype;
    using URSDK.RobController.MathLib;
    using MathNet.Numerics.LinearAlgebra.Double;
    using UnityEngine;
    using System.Collections.Generic;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        /// <summary>
        /// robotic arm global var
        /// </summary>
        DashBoard aDashBoard;
        RTClient aRTClient;
        String returnString;
        PrimaryInterface primaryInt;
        Vector6<double> tcp;
        robotState state;
        rot ROT;
        MathNet.Numerics.LinearAlgebra.Double.Matrix matrix_R = new DenseMatrix(3);
        MathNet.Numerics.LinearAlgebra.Double.Matrix matrix_T_tcp2base = new DenseMatrix(4);
        MathNet.Numerics.LinearAlgebra.Double.Matrix matrix_T_tool2tcp = new DenseMatrix(4);
        MathNet.Numerics.LinearAlgebra.Double.Matrix matrix_T = new DenseMatrix(4);
        int position = 0;               //判断机械臂位置
        bool flag = false;
        int times = 0;
        //these vectors are used to save the pointcloud data at different position of the arm
        //0 means all is in base coordinates
        Vector3[] vertices_0_b1 = null;
        Vector3[] vertices_0_b2 = null;
        Vector3[] vertices_0_b3 = null;
        Vector3[] vertices_0_b4 = null;
        Vector3[] vertices_0_b5 = null;
        //Vector3[] vertices_0_all = null;
        Color32[] colors_0_b1 = null;
        Color32[] colors_0_b2 = null;
        Color32[] colors_0_b3 = null;
        Color32[] colors_0_b4 = null;
        Color32[] colors_0_b5 = null;
        //Color32[] colors_0_all = null;

        /// <summary>
        /// Azure Kinect sensor
        /// </summary>
        private readonly Device kinect = null;

        /// <summary>
        /// Azure Kinect transformation engine
        /// </summary>
        private readonly Transformation transform = null;

        /// <summary>
        /// Bitmap to display
        /// </summary>
        private readonly WriteableBitmap bitmap = null;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        /// <summary>
        /// The width in pixels of the color image from the Azure Kinect DK
        /// </summary>
        private readonly int colorWidth = 0;

        /// <summary>
        /// The height in pixels of the color image from the Azure Kinect DK
        /// </summary>
        private readonly int colorHeight = 0;

        /// <summary>
        /// The height in pixels of the color image from the Azure Kinect DK
        /// </summary>
        private readonly int depthWidth = 0;

        /// <summary>
        /// The height in pixels of the color image from the Azure Kinect DK
        /// </summary>
        private readonly int depthHeight = 0;

        /// <summary>
        /// Status of the application
        /// </summary>
        private bool running = true;

        /// <summary>
        /// 点の集合を描画するために使用(表示する図形の詳細を管理するオブジェクト)
        /// </summary>
        int num;

        /// <summary>
        ///PointCloudの各点に対応する色の配列
        /// </summary>
        //System.Numerics.Vector3[] vertices;

        /// <summary>
        /// vertices中の何番目の点を描画するかのリスト(全部描画するけど手続き上必要)
        /// </summary>
        //Color[] colors;

        /// <summary>
        /// 座標変換(Color⇔Depth対応やDepth→xyzなど)をするためのクラス
        /// </summary>
        //int[] indices;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            //set the robotic arm
            aDashBoard = new DashBoard("169.254.213.1");
            aRTClient = new RTClient("169.254.213.1");
            primaryInt = new PrimaryInterface("169.254.213.1", 30001);    //30001
            aRTClient.startRTClient();          //169.254.213.1     //192.168.1.128
            primaryInt.startPrimary();

            // Open the default device
            this.kinect = Device.Open();

            // Configure camera modes
            this.kinect.StartCameras(new DeviceConfiguration
            {
                ColorFormat = ImageFormat.ColorBGRA32,
                ColorResolution = ColorResolution.R1080p,
                DepthMode = DepthMode.NFOV_2x2Binned,
                SynchronizedImagesOnly = true
            });
            //NFOV
            matrix_T_tool2tcp[0, 0] = -0.707;
            matrix_T_tool2tcp[0, 1] = -0.0739;
            matrix_T_tool2tcp[0, 2] = 0.7031;
            matrix_T_tool2tcp[0, 3] = 0.0566;
            matrix_T_tool2tcp[1, 0] = 0.7070;
            matrix_T_tool2tcp[1, 1] = -0.0739;
            matrix_T_tool2tcp[1, 2] = 0.7031;
            matrix_T_tool2tcp[1, 3] = 0.0566;
            matrix_T_tool2tcp[2, 0] = 0;
            matrix_T_tool2tcp[2, 1] = 0.9945;
            matrix_T_tool2tcp[2, 2] = 0.1045;
            matrix_T_tool2tcp[2, 3] = 0.1002;
            matrix_T_tool2tcp[3, 0] = 0;
            matrix_T_tool2tcp[3, 1] = 0;
            matrix_T_tool2tcp[3, 2] = 0;
            matrix_T_tool2tcp[3, 3] = 1;

            this.transform = this.kinect.GetCalibration().CreateTransformation();
            this.colorWidth = this.kinect.GetCalibration().ColorCameraCalibration.ResolutionWidth;
            this.colorHeight = this.kinect.GetCalibration().ColorCameraCalibration.ResolutionHeight;
            this.depthWidth = this.kinect.GetCalibration().DepthCameraCalibration.ResolutionWidth;
            this.depthHeight = this.kinect.GetCalibration().DepthCameraCalibration.ResolutionHeight;
            this.bitmap = new WriteableBitmap(colorWidth, colorHeight, 96.0, 96.0, PixelFormats.Bgra32, null);
            this.num = depthHeight * depthWidth;

            vertices_0_b1 = new Vector3[num];
            vertices_0_b2 = new Vector3[num];
            vertices_0_b3 = new Vector3[num];
            vertices_0_b4 = new Vector3[num];
            vertices_0_b5 = new Vector3[num];

            colors_0_b1 = new Color32[num];
            colors_0_b2 = new Color32[num];
            colors_0_b3 = new Color32[num];
            colors_0_b4 = new Color32[num];
            colors_0_b5 = new Color32[num];


            this.DataContext = this;

            this.InitializeComponent();
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.bitmap;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            running = false;

            if (this.kinect != null)
            {
                this.kinect.Dispose();
            }
        }

        /// <summary>
        /// Handles the user clicking on the screenshot button
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void ScreenshotButton_Click(object sender, RoutedEventArgs e)
        {
            //code is needed here: get the point cloud from the global var
            flag = true;
            // Create a render target to which we'll render our composite image
            RenderTargetBitmap renderBitmap = new RenderTargetBitmap((int)CompositeImage.ActualWidth, (int)CompositeImage.ActualHeight, 96.0, 96.0, PixelFormats.Pbgra32);

            DrawingVisual dv = new DrawingVisual();
            using (DrawingContext dc = dv.RenderOpen())
            {
                VisualBrush brush = new VisualBrush(CompositeImage);
                dc.DrawRectangle(brush, null, new System.Windows.Rect(new Point(), new Size(CompositeImage.ActualWidth, CompositeImage.ActualHeight)));
            }

            renderBitmap.Render(dv);

            BitmapEncoder encoder = new PngBitmapEncoder();
            encoder.Frames.Add(BitmapFrame.Create(renderBitmap));

            string time = System.DateTime.Now.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);

            string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures);

            string path = Path.Combine(myPhotos, "KinectScreenshot-" + time + ".png");

            // Write the new file to disk
            try
            {
                using (FileStream fs = new FileStream(path, FileMode.Create))
                {
                    encoder.Save(fs);
                }

                this.StatusText = string.Format(Properties.Resources.SavedScreenshotStatusTextFormat, path);
            }
            catch (IOException)
            {
                this.StatusText = string.Format(Properties.Resources.FailedScreenshotStatusTextFormat, path);
            }
        }

        private void MaskColor(Image color, Image transformedDepth)
        {
            this.bitmap.Lock();
            var region = new Int32Rect(0, 0, color.WidthPixels, color.HeightPixels);

            unsafe
            {
                using (var pin = color.Memory.Pin())
                {
                    this.bitmap.WritePixels(region, (IntPtr)pin.Pointer, (int)color.Size, color.StrideBytes);
                }

                uint* colorPixels = (uint*)this.bitmap.BackBuffer;
                Span<ushort> depthPixels = transformedDepth.GetPixels<ushort>().Span;

                for (int i = 0; i < this.colorHeight * this.colorWidth; i++)
                {
                    if (depthPixels[i] < 1000 &&
                        depthPixels[i] != 0)    //1000mm
                    {
                        continue;
                    }
                    colorPixels[i] = 0;
                }
            }

            this.bitmap.AddDirtyRect(region);
            this.bitmap.Unlock();
        }

        private  void GetPointcloud(Image colorImage,Image depth, Image xyzImage,robotState state)
        {
            flag = false;
            times++;  //make sure this program has been processed once
            //give out the tcp value and the tranformation matrix
            if (state.cartesianInfo != null)
            {
                tcp = new Vector6<double>();
                ROT = new rot();
                tcp = state.cartesianInfo.tcp;
                ROT.Rx = state.cartesianInfo.tcp.Rx;
                ROT.Ry = state.cartesianInfo.tcp.Ry;
                ROT.Rz = state.cartesianInfo.tcp.Rz;

                matrix_R = RobMath.RotVec2Matrix(ROT);
                matrix_T_tcp2base.SetSubMatrix(0, 3, 0, 3, matrix_R);
                matrix_T_tcp2base[3, 0] = 0;
                matrix_T_tcp2base[3, 1] = 0;
                matrix_T_tcp2base[3, 2] = 0;
                matrix_T_tcp2base[3, 3] = 1;
                matrix_T_tcp2base[0, 3] = tcp.X;
                matrix_T_tcp2base[1, 3] = tcp.Y;
                matrix_T_tcp2base[2, 3] = tcp.Z;

                matrix_T = (DenseMatrix)(matrix_T_tcp2base * matrix_T_tool2tcp);
                //info.Text = tcp.ToString();
                //info.Text = matrix_T_tcp2base.ToString();

                BGRA[] colorArray = colorImage.GetPixels<BGRA>().ToArray();
                Short3[] xyzArray = xyzImage.GetPixels<Short3>().ToArray();

                MathNet.Numerics.LinearAlgebra.Double.Matrix matrix_P1 = new DenseMatrix(4, 1);
                MathNet.Numerics.LinearAlgebra.Double.Matrix matrix_P0 = new DenseMatrix(4, 1);

                Vector3[] vertices = new Vector3[num];
                Color32[] colors = new Color32[num];
                //indices = new int[num];
                //描画する点の配列番号を記録。(全ての点を描画)
                //for (int i = 0; i < num; i++)
                //{
                //    indices[i] = i;
                //}

                for (int i = 0; i < num; i++)
                {
                    //頂点座標の代入
                    vertices[i].x = xyzArray[i].X * 0.001f;
                    vertices[i].y = xyzArray[i].Y * 0.001f;
                    vertices[i].z = xyzArray[i].Z * 0.001f;
                    //色の代入
                    colors[i].b = colorArray[i].B;
                    colors[i].g = colorArray[i].G;
                    colors[i].r = colorArray[i].R;
                    colors[i].a = 255;

                    ////for each point-1 , use the T-matrix to tansform it to  point-0

                    matrix_P1[0, 0] = vertices[i].x;
                    matrix_P1[1, 0] = vertices[i].y;
                    matrix_P1[2, 0] = vertices[i].z;
                    matrix_P1[3, 0] = 1;
                    matrix_P0 = (DenseMatrix)(matrix_T * matrix_P1);

                    //according to the position of the arm to save the pointcloud data
                    switch (position)
                    {
                        case 1:
                            {
                                vertices_0_b1[i].x = (float)matrix_P0[0, 0];
                                vertices_0_b1[i].y = (float)matrix_P0[1, 0];
                                vertices_0_b1[i].z = (float)matrix_P0[2, 0];
                                colors_0_b1[i] = colors[i];
                            }
                            break;
                        case 2:
                            {
                                vertices_0_b2[i].x = (float)matrix_P0[0, 0];
                                vertices_0_b2[i].y = (float)matrix_P0[1, 0];
                                vertices_0_b2[i].z = (float)matrix_P0[2, 0];
                                colors_0_b2[i] = colors[i];

                            }
                            break;
                        case 3:
                            {
                                vertices_0_b3[i].x = (float)matrix_P0[0, 0];
                                vertices_0_b3[i].y = (float)matrix_P0[1, 0];
                                vertices_0_b3[i].z = (float)matrix_P0[2, 0]; colors_0_b1[i] = colors[i];
                                colors_0_b3[i] = colors[i];

                            }
                            break;
                        case 4:
                            {
                                vertices_0_b4[i].x = (float)matrix_P0[0, 0];
                                vertices_0_b4[i].y = (float)matrix_P0[1, 0];
                                vertices_0_b4[i].z = (float)matrix_P0[2, 0];
                                colors_0_b4[i] = colors[i];

                            }
                            break;
                        case 5:
                            {
                                vertices_0_b5[i].x = (float)matrix_P0[0, 0];
                                vertices_0_b5[i].y = (float)matrix_P0[1, 0];
                                vertices_0_b5[i].z = (float)matrix_P0[2, 0];
                                colors_0_b5[i] = colors[i];

                            }
                            break;
                    }
                }
                info.Text = "times:" + times.ToString() + "," + "position:" + position.ToString() + "," + "scanning and processing done!";
            }
            else
            {
                info.Text = "times:"+times.ToString()+"state.cartesianInfo is null!";
            }
        }
        private async void Window_Loaded(object sender, RoutedEventArgs e)
        {
            while (running)
            {
                //code is needed here: using pointcloud to receive the data from the transform
                using (Image transformedDepth = new Image(ImageFormat.Depth16, colorWidth, colorHeight, colorWidth * sizeof(UInt16)))
                using (Capture capture = await Task.Run(() => { return this.kinect.GetCapture(); }))
                using (Image colorImage = transform.ColorImageToDepthCamera(capture))
                using (Image xyzImage = transform.DepthImageToPointCloud(capture.Depth))
                {
                    this.transform.DepthImageToColorCamera(capture, transformedDepth);
                    this.StatusText = "Received Capture: " + capture.Depth.DeviceTimestamp;
                    var color = capture.Color;
                    var depth = capture.Depth;

                    MaskColor(color, transformedDepth);     //for display on the screen

                    if (flag == true)
                    {
                        state = primaryInt.getRobotState();
                        try
                        {
                            info.Text = "please wait for a while ...";
                            GetPointcloud(colorImage, depth, xyzImage, state);
                        }
                        catch (Exception)
                        {
                            info.Text = "error existing when getting the pointcloud data!";
                        }
                    }

                    //for getting the point coordinates
                }
            }
        }

        //image RGB display program

        //private async void Window_Loaded(object sender, RoutedEventArgs e)
        //{
        //    while (running)
        //    {
        //        using (Capture capture = await Task.Run(() => { return this.kinect.GetCapture(); }))
        //        {
        //            this.StatusText = "Received Capture: " + capture.Depth.DeviceTimestamp;

        //            this.bitmap.Lock();

        //            var color = capture.Color;
        //            var region = new Int32Rect(0, 0, color.WidthPixels, color.HeightPixels);

        //            unsafe
        //            {
        //                using (var pin = color.Memory.Pin())
        //                {
        //                    this.bitmap.WritePixels(region, (IntPtr)pin.Pointer, (int)color.Size, color.StrideBytes);
        //                }
        //            }

        //            this.bitmap.AddDirtyRect(region);
        //            this.bitmap.Unlock();
        //        }
        //    }
        //}

        private void Button_Click(object sender, RoutedEventArgs e)
        {
            position = 0;
            //info.Text = "present position:" + position.ToString();
            //aRTClient.sendScript("movej([0,-1.57,0,-1.57,0,0],a=1.4, v=1.05, t=0, r=0)");
        }

        private void Button_Click_1(object sender, RoutedEventArgs e)
        {
            position = 1;
            //info.Text = "present position:" + position.ToString();
            //returnString = aDashBoard.loadProgram("point1.urp");
            //returnString = aDashBoard.play();
        }

        private void Button_Click_2(object sender, RoutedEventArgs e)
        {
            position = 2;
            //info.Text = "present position:" + position.ToString();
            //returnString = aDashBoard.loadProgram("point2.urp");
            //returnString = aDashBoard.play();
        }

        private void Button_Click_3(object sender, RoutedEventArgs e)
        {
            position = 3;
            //info.Text = "present position:" + position.ToString();
            //returnString = aDashBoard.loadProgram("point3.urp");
            //returnString = aDashBoard.play();
        }

        private void Button_Click_4(object sender, RoutedEventArgs e)
        {
            position = 4;
            //info.Text = "present position:" + position.ToString();
            //returnString = aDashBoard.loadProgram("point4.urp");
            //returnString = aDashBoard.play();
        }

        private void Button_Click_5(object sender, RoutedEventArgs e)
        {
            position = 5;//top
            //info.Text = "present position:" + position.ToString();
            //returnString = aDashBoard.loadProgram("pointtop.urp");
            //returnString = aDashBoard.play();
        }

        private void Button_Click_6(object sender, RoutedEventArgs e)
        {
            position = -1;
            //info.Text = "present position:" + position.ToString();
            //returnString = aDashBoard.loadProgram("Put_into_box_CB3_UR5_v2.urp");
            //returnString = aDashBoard.play();
        }

        private void TextBox_TextChanged(object sender, System.Windows.Controls.TextChangedEventArgs e)
        {
            //do  nothing
        }

        private void Button_Click_7(object sender, RoutedEventArgs e)
        {
            try
            {
                string time = System.DateTime.Now.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);
                string FileName = "C:\\Users\\Administrator\\Desktop\\ScanPoint.csv";
            FileStream of = File.Open(FileName, FileMode.Create);
            StreamWriter sw = new StreamWriter(of);
            sw.Write("x");
            sw.Write(",");

            sw.Write("y");
            sw.Write(",");

            sw.Write("z");
            sw.Write(",");

            sw.Write("r");
            sw.Write(",");

            sw.Write("g");
            sw.Write(",");

            sw.WriteLine("b");
            sw.WriteLine("pointcloud_b1");
            for(int i=0;i<num;i++)
            {
                sw.Write(vertices_0_b1[i].x.ToString());
                sw.Write(",");

                sw.Write(vertices_0_b1[i].y.ToString());
                sw.Write(",");

                sw.Write(vertices_0_b1[i].z.ToString());
                sw.Write(",");

                sw.Write(colors_0_b1[i].r.ToString());
                sw.Write(",");

                sw.Write(colors_0_b1[i].g.ToString());
                sw.Write(",");

                sw.WriteLine(colors_0_b1[i].b.ToString());
            }
            sw.WriteLine("pointcloud_b2");
            for (int i = 0; i < num; i++)
            {
                sw.Write(vertices_0_b2[i].x.ToString());
                sw.Write(",");

                sw.Write(vertices_0_b2[i].y.ToString());
                sw.Write(",");

                sw.Write(vertices_0_b2[i].z.ToString());
                sw.Write(",");

                sw.Write(colors_0_b2[i].r.ToString());
                sw.Write(",");

                sw.Write(colors_0_b2[i].g.ToString());
                sw.Write(",");

                sw.WriteLine(colors_0_b2[i].b.ToString());
            }
            sw.WriteLine("pointcloud_b3");
            for (int i = 0; i < num; i++)
            {
                sw.Write(vertices_0_b3[i].x.ToString());
                sw.Write(",");

                sw.Write(vertices_0_b3[i].y.ToString());
                sw.Write(",");

                sw.Write(vertices_0_b3[i].z.ToString());
                sw.Write(",");

                sw.Write(colors_0_b3[i].r.ToString());
                sw.Write(",");

                sw.Write(colors_0_b3[i].g.ToString());
                sw.Write(",");

                sw.WriteLine(colors_0_b3[i].b.ToString());
            }
            sw.WriteLine("pointcloud_b4");
            for (int i = 0; i < num; i++)
            {
                sw.Write(vertices_0_b4[i].x.ToString());
                sw.Write(",");

                sw.Write(vertices_0_b4[i].y.ToString());
                sw.Write(",");

                sw.Write(vertices_0_b4[i].z.ToString());
                sw.Write(",");

                sw.Write(colors_0_b4[i].r.ToString());
                sw.Write(",");

                sw.Write(colors_0_b4[i].g.ToString());
                sw.Write(",");

                sw.WriteLine(colors_0_b4[i].b.ToString());
            }
            sw.WriteLine("pointcloud_b5");
            for (int i = 0; i < num; i++)
            {
                sw.Write(vertices_0_b5[i].x.ToString());
                sw.Write(",");

                sw.Write(vertices_0_b5[i].y.ToString());
                sw.Write(",");

                sw.Write(vertices_0_b5[i].z.ToString());
                sw.Write(",");

                sw.Write(colors_0_b5[i].r.ToString());
                sw.Write(",");

                sw.Write(colors_0_b5[i].g.ToString());
                sw.Write(",");

                sw.WriteLine(colors_0_b5[i].b.ToString());
            }
            sw.Close();
            of.Close();
                info.Text = "saving files done!";
            }
            catch(Exception)
            {
                info.Text = "error existing when saving files!";
            }
        }
    }
}
