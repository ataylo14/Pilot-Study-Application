//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.DepthBasics
{
    using System;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;
    using System.Collections.Generic;
    using System.Windows.Shapes;
    using System.Linq;
    using System.Text;
    using System.Windows.Documents;
    using System.Windows.Input;
    using System.Windows.Data;
    using System.Windows.Controls; //Canvas object
    using System.Windows.Navigation;
    using System.Threading;

   



    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        /// <summary>
        /// Struct used to select sensor mode
        /// </summary>
        Mode _mode = Mode.Color;

        /// <summary>
        /// Body variable
        /// </summary>
        IList<Body> _bodies;

        /// <summary>
        /// Map depth range to byte range
        /// </summary>
        private const int MapDepthToByte = 8000 / 256;
        
        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// --Active Multi-Source Kinect sensor reader
        /// </summary>
        private MultiSourceFrameReader multiSourceReader = null;
        
        /// <summary>
        /// Description of the data contained in the depth frame
        /// </summary>
        private FrameDescription depthFrameDescription = null;
            
        /// <summary>
        /// Bitmap to display
        /// </summary>
        private WriteableBitmap depthBitmap = null;

        /// <summary>
        /// Bitmap to display
        /// </summary>
        private WriteableBitmap colorBitmap = null;

        /// <summary>
        /// Intermediate storage for frame data converted to color
        /// </summary>
        private byte[] depthPixels = null;

        /// <summary>
        /// Determines rather bodies are displayed
        /// </summary>
        private bool _displayBody = false;

        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;
        
        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;
        
        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        /// <summary>
        /// Number of samples captured from Kinect audio stream each millisecond.
        /// </summary>
        private const int SamplesPerMillisecond = 16;

        /// <summary>
        /// Number of bytes in each Kinect audio stream sample (32-bit IEEE float).
        /// </summary>
        private const int BytesPerSample = sizeof(float);

        /// <summary>
        /// Number of audio samples represented by each column of pixels in wave bitmap.
        /// </summary>
        private const int SamplesPerColumn = 40;

        /// <summary>
        /// Minimum energy of audio to display (a negative number in dB value, where 0 dB is full scale)
        /// </summary>
        private const int MinEnergy = -90;

        /// <summary>
        /// Width of bitmap that stores audio stream energy data ready for visualization.
        /// </summary>
        private const int EnergyBitmapWidth = 780;

        /// <summary>
        /// Height of bitmap that stores audio stream energy data ready for visualization.
        /// </summary>
        private const int EnergyBitmapHeight = 195;

        /// <summary>
        /// Bitmap that contains constructed visualization for audio stream energy, ready to
        /// be displayed. It is a 2-color bitmap with white as background color and blue as
        /// foreground color.
        /// </summary>
        private readonly WriteableBitmap energyBitmap;

        /// <summary>
        /// Rectangle representing the entire energy bitmap area. Used when drawing background
        /// for energy visualization.
        /// </summary>
        private readonly Int32Rect fullEnergyRect = new Int32Rect(0, 0, EnergyBitmapWidth, EnergyBitmapHeight);

        /// <summary>
        /// Array of background-color pixels corresponding to an area equal to the size of whole energy bitmap.
        /// </summary>
        private readonly byte[] backgroundPixels = new byte[EnergyBitmapWidth * EnergyBitmapHeight];

        /// <summary>
        /// Will be allocated a buffer to hold a single sub frame of audio data read from audio stream.
        /// </summary>
        private readonly byte[] audioBuffer = null;

        /// <summary>
        /// Buffer used to store audio stream energy data as we read audio.
        /// We store 25% more energy values than we strictly need for visualization to allow for a smoother
        /// stream animation effect, since rendering happens on a different schedule with respect to audio
        /// capture.
        /// </summary>
        private readonly float[] energy = new float[(uint)(EnergyBitmapWidth * 1.25)];

        /// <summary>
        /// Object for locking energy buffer to synchronize threads.
        /// </summary>
        private readonly object energyLock = new object();
        
        /// <summary>
        /// Reader for audio frames
        /// </summary>
        private AudioBeamFrameReader audioReader = null;

        /// <summary>
        /// Last observed audio beam angle in radians, in the range [-pi/2, +pi/2]
        /// </summary>
        private float beamAngle = 0;

        /// <summary>
        /// Last observed audio beam angle confidence, in the range [0, 1]
        /// </summary>
        private float beamAngleConfidence = 0;

        /// <summary>
        /// Array of foreground-color pixels corresponding to a line as long as the energy bitmap is tall.
        /// This gets re-used while constructing the energy visualization.
        /// </summary>
        private byte[] foregroundPixels;

        /// <summary>
        /// Sum of squares of audio samples being accumulated to compute the next energy value.
        /// </summary>
        private float accumulatedSquareSum;

        /// <summary>
        /// Number of audio samples accumulated so far to compute the next energy value.
        /// </summary>
        private int accumulatedSampleCount;

        /// <summary>
        /// Index of next element available in audio energy buffer.
        /// </summary>
        private int energyIndex;

        /// <summary>
        /// Number of newly calculated audio stream energy values that have not yet been
        /// displayed.
        /// </summary>
        private int newEnergyAvailable;

        /// <summary>
        /// Error between time slice we wanted to display and time slice that we ended up
        /// displaying, given that we have to display in integer pixels.
        /// </summary>
        private float energyError;

        /// <summary>
        /// Last time energy visualization was rendered to screen.
        /// </summary>
        private DateTime? lastEnergyRefreshTime;

        /// <summary>
        /// Index of first energy element that has never (yet) been displayed to screen.
        /// </summary>
        private int energyRefreshIndex;

        /// <summary>
        /// Store beam angle for and output to file
        /// </summary>
        private float[] beamAngleOutput = new float[(uint)(EnergyBitmapWidth * 1.25)];

        /// <summary>
        /// Index of next beam angle in beamAngleOutput
        /// </summary>
        private int beamAngleIdx = 0;

        /// <summary>
        /// Store beam angle confidence and output to file
        /// </summary>
        private float[] beamAngleConfidenceOutput = new float[(uint)(EnergyBitmapWidth * 1.25)];

        /// <summary>
        /// Index of next beam angle in beamAngleOutput
        /// </summary>
        private int beamAngleConfidenceIdx = 0;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // get the kinectSensor object
            this.kinectSensor = KinectSensor.GetDefault();

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            if (this.kinectSensor != null)
            {

                // create the colorFrameDescription from the ColorFrameSource using Bgra format
                FrameDescription colorFrameDescription = this.kinectSensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);

                // create the bitmap to display
                this.colorBitmap = new WriteableBitmap(colorFrameDescription.Width, colorFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);

                // get FrameDescription from DepthFrameSource
                this.depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

                // allocate space to put the pixels being received and converted
                this.depthPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height];

                // create the bitmap to display
                this.depthBitmap = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Gray8, null);

                // get the coordinate mapper
                this.coordinateMapper = this.kinectSensor.CoordinateMapper;

                // get the depth (display) extents
                FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

                // get size of joint space
                this.displayWidth = frameDescription.Width;
                this.displayHeight = frameDescription.Height;

                // open the sensor
                this.kinectSensor.Open();

                //--open multi-source reader
                this.multiSourceReader = this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth | FrameSourceTypes.Infrared | FrameSourceTypes.Body );

                // Get its audio source
                AudioSource audioSource = this.kinectSensor.AudioSource;

                // Allocate 1024 bytes to hold a single audio sub frame. Duration sub frame 
                // is 16 msec, the sample rate is 16khz, which means 256 samples per sub frame. 
                // With 4 bytes per sample, that gives us 1024 bytes.
                this.audioBuffer = new byte[audioSource.SubFrameLengthInBytes];

                // Open the reader for the audio frames
                this.audioReader = audioSource.OpenReader();

                // Uncomment these two lines to overwrite the automatic mode of the audio beam.
                // It will change the beam mode to manual and set the desired beam angle.
                // In this example, point it straight forward.
                // Note that setting beam mode and beam angle will only work if the
                // application window is in the foreground.
                // Furthermore, setting these values is an asynchronous operation --
                // it may take a short period of time for the beam to adjust.
                /*
                audioSource.AudioBeams[0].AudioBeamMode = AudioBeamMode.Manual;
                audioSource.AudioBeams[0].BeamAngle = 0;
                */

                // wire handler for frame arrival
                this.multiSourceReader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;

                // a bone defined as a line between two joints
                this.bones = new List<Tuple<JointType, JointType>>();

                // Torso
                this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
                this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
                this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
                this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
                this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
                this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
                this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
                this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

                // Right Arm
                this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
                this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
                this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
                this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
                this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

                // Left Arm
                this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
                this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
                this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
                this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
                this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

                // Right Leg
                this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
                this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
                this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

                // Left Leg
                this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
                this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
                this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

                // populate body colors, one for each BodyIndex
                this.bodyColors = new List<Pen>();

                this.bodyColors.Add(new Pen(Brushes.Red, 6));
                this.bodyColors.Add(new Pen(Brushes.Orange, 6));
                this.bodyColors.Add(new Pen(Brushes.Green, 6));
                this.bodyColors.Add(new Pen(Brushes.Blue, 6));
                this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
                this.bodyColors.Add(new Pen(Brushes.Violet, 6));

                // set the status text
                this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                                : Properties.Resources.NoSensorStatusText;

                // Create the drawing group we'll use for drawing
                this.drawingGroup = new DrawingGroup();

                // Create an image source that we can use in our image control
                this.imageSource = new DrawingImage(this.drawingGroup);

                // use the window object as the view model in this simple example
                this.DataContext = this;

                // initialize the components (controls) of the window
                this.InitializeComponent();
                
            }
            else
            {
                // On failure, set the status text
                //this.statusBarText.Text = Properties.Resources.NoSensorStatusText;
                return;
            }
            this.energyBitmap = new WriteableBitmap(EnergyBitmapWidth, EnergyBitmapHeight, 96, 96, PixelFormats.Indexed1, new BitmapPalette(new List<Color> { Colors.White, (Color)this.Resources["KinectPurpleColor"] }));
            
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

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

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            // Initialize foreground pixels
            this.foregroundPixels = new byte[EnergyBitmapHeight];
            for (int i = 0; i < this.foregroundPixels.Length; ++i)
            {
                this.foregroundPixels[i] = 0xff;
            }

            this.waveDisplay.Source = this.energyBitmap;

            CompositionTarget.Rendering += this.UpdateEnergy;

            if (this.audioReader != null)
            {
                // Subscribe to new audio frame arrived events
                this.audioReader.FrameArrived += this.AudioReader_FrameArrived;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            SaveAudioData();
            CompositionTarget.Rendering -= this.UpdateEnergy;

            if (this.multiSourceReader != null)
            {
                // DepthFrameReader is IDisposable
                this.multiSourceReader.Dispose();
                this.multiSourceReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        ///
        /// <summary>
        /// Handles multiple streams of frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            // Get a reference to multi-frame
            var reference = e.FrameReference.AcquireFrame();
            bool dataReceived = false;
            //Open color frame
            using (var colorFrame = reference.ColorFrameReference.AcquireFrame())
            {
                if(colorFrame != null )//&& _mode == Mode.Color)
                {
                    camera.Source = ToBitmap(colorFrame);
                }
            }

            //Open depth frame
            using (var depthFrame = reference.DepthFrameReference.AcquireFrame())
            {
                if (depthFrame != null )//&& _mode == Mode.Depth)
                {
                    camera1.Source = ToBitmap(depthFrame);
                }
            }

            //Open infrared frame
            using (var infraredFrame = reference.InfraredFrameReference.AcquireFrame())
            {
                if (infraredFrame != null)// && _mode == Mode.Infrared)
                {
                    camera2.Source = ToBitmap(infraredFrame);
                }
            }

            //Open body frame 
            using (var bodyFrame = reference.BodyFrameReference.AcquireFrame())
            {
                if (bodyFrame != null)// && _mode == Mode.Infrared)
                {
                    BodyToBitmap(bodyFrame);
                }
            }
        }

        /// <summary>
        /// Handles the audio frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void AudioReader_FrameArrived(object sender, AudioBeamFrameArrivedEventArgs e)
        {
            AudioBeamFrameReference frameReference = e.FrameReference;
            AudioBeamFrameList frameList = frameReference.AcquireBeamFrames();

            if (frameList != null)
            {
                // AudioBeamFrameList is IDisposable
                using (frameList)
                {
                    // Only one audio beam is supported. Get the sub frame list for this beam
                    IReadOnlyList<AudioBeamSubFrame> subFrameList = frameList[0].SubFrames;

                    // Loop over all sub frames, extract audio buffer and beam information
                    foreach (AudioBeamSubFrame subFrame in subFrameList)
                    {
                        // Check if beam angle and/or confidence have changed
                        bool updateBeam = false;

                        if (subFrame.BeamAngle != this.beamAngle)
                        {
                            this.beamAngle = subFrame.BeamAngle;
                            this.beamAngleOutput[this.beamAngleIdx] = this.beamAngle;
                            if(this.beamAngleIdx == EnergyBitmapHeight)
                            {
                                SaveAudioData();
                                this.beamAngleIdx=0;
                                this.beamAngleConfidenceIdx = 0;
                            }
                            else
                            {
                                this.beamAngleIdx++;
                            }
                            updateBeam = true;
                        }

                        if (subFrame.BeamAngleConfidence != this.beamAngleConfidence)
                        {
                            this.beamAngleConfidence = subFrame.BeamAngleConfidence;
                            this.beamAngleConfidenceOutput[this.beamAngleConfidenceIdx] = this.beamAngleConfidence;
                            this.beamAngleConfidenceIdx++;
                            updateBeam = true;
                        }

                        if (updateBeam)
                        {
                            // Refresh display of audio beam
                            this.AudioBeamChanged();
                        }

                        // Process audio buffer
                        subFrame.CopyFrameDataToArray(this.audioBuffer);

                        for (int i = 0; i < this.audioBuffer.Length; i += BytesPerSample)
                        {
                            // Extract the 32-bit IEEE float sample from the byte array
                            float audioSample = BitConverter.ToSingle(this.audioBuffer, i);

                            this.accumulatedSquareSum += audioSample * audioSample;
                            ++this.accumulatedSampleCount;

                            if (this.accumulatedSampleCount < SamplesPerColumn)
                            {
                                continue;
                            }

                            float meanSquare = this.accumulatedSquareSum / SamplesPerColumn;

                            if (meanSquare > 1.0f)
                            {
                                // A loud audio source right next to the sensor may result in mean square values
                                // greater than 1.0. Cap it at 1.0f for display purposes.
                                meanSquare = 1.0f;
                            }

                            // Calculate energy in dB, in the range [MinEnergy, 0], where MinEnergy < 0
                            float energy = MinEnergy;

                            if (meanSquare > 0)
                            {
                                energy = (float)(10.0 * Math.Log10(meanSquare));
                            }

                            lock (this.energyLock)
                            {
                                // Normalize values to the range [0, 1] for display
                                this.energy[this.energyIndex] = (MinEnergy - energy) / MinEnergy;
                                this.energyIndex = (this.energyIndex + 1) % this.energy.Length;
                                ++this.newEnergyAvailable;
                            }

                            this.accumulatedSquareSum = 0;
                            this.accumulatedSampleCount = 0;
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Store Body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void BodyToBitmap(BodyFrame bodyFrame)
        {
            bool dataReceived = false;

            if (bodyFrame != null)
            {
                if (this.bodies == null)
                {
                    this.bodies = new Body[bodyFrame.BodyCount];
                }

                // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                // As long as those body objects are not disposed and not set to null in the array,
                // those body objects will be re-used.
                bodyFrame.GetAndRefreshBodyData(this.bodies);
                dataReceived = true;
            }
            
            if (dataReceived)
            {
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                    int penIndex = 0;
                    foreach (Body body in this.bodies)
                    {
                        Pen drawPen = this.bodyColors[penIndex++];

                        if (body.IsTracked)
                        {
                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }

                            this.DrawBody(joints, jointPoints, dc, drawPen);

                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                        }
                    }

                    // prevent drawing outside of our render area
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }
            }
        }

        /// <summary>
        /// Store Color frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private ImageSource ToBitmap(ColorFrame frame)
        {
            if (frame != null)
            {
                FrameDescription colorFrameDescription = frame.FrameDescription;
                int width = colorFrameDescription.Width;
                int height = colorFrameDescription.Height;
                byte[] pixels = new byte[width * height * ((PixelFormats.Bgr32.BitsPerPixel + 7) / 8)];

                if(frame.RawColorImageFormat == ColorImageFormat.Bgra)
                {
                    frame.CopyRawFrameDataToArray(pixels);
                }
                else
                {
                    frame.CopyConvertedFrameDataToArray(pixels, ColorImageFormat.Bgra);
                }

                int stride = width * PixelFormats.Bgr32.BitsPerPixel / 8;
                //return BitmapSource.Create(width, height, 96, 96, PixelFormats.Bgr32, null, pixels, stride);
                BitmapSource bs = BitmapSource.Create(width, height, 96, 96, PixelFormats.Bgr32, null, pixels, stride);
                /*colorBitmap = new WriteableBitmap(bs);
                SaveFrames(colorBitmap, 1);*/
                return bs;
            }
            else
            {
                return null;
            }
        }

        /// <summary>
        /// Store Depth frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private ImageSource ToBitmap(DepthFrame frame)
        {
            if (frame != null)
            {
                int width = frame.FrameDescription.Width;
                int height = frame.FrameDescription.Height;

                ushort minDepth = frame.DepthMinReliableDistance;
                ushort maxDepth = frame.DepthMaxReliableDistance;

                ushort[] depthData = new ushort[width * height];
                byte[] pixelData = new byte[width * height * (PixelFormats.Bgr32.BitsPerPixel + 7) / 8];

                frame.CopyFrameDataToArray(depthData);

                int colorIndex = 0;
                for (int depthIndex = 0; depthIndex < depthData.Length; ++depthIndex)
                {
                    ushort depth = depthData[depthIndex];
                    byte intensity = (byte)(depth >= minDepth && depth <= maxDepth ? depth : 0);

                    pixelData[colorIndex++] = intensity; // Blue
                    pixelData[colorIndex++] = intensity; // Green
                    pixelData[colorIndex++] = intensity; // Red

                    ++colorIndex;
                }

                int stride = width * PixelFormats.Bgr32.BitsPerPixel / 8;

                return BitmapSource.Create(width, height, 96, 96, PixelFormats.Bgr32, null, pixelData, stride);
                BitmapSource bs = BitmapSource.Create(width, height, 96, 96, PixelFormats.Bgr32, null, pixelData, stride);
                /*depthBitmap = new WriteableBitmap(bs);
                SaveFrames(depthBitmap, 2);*/
                return bs;
            }
            else
            {
                return null;
            }
        }

        /// <summary>
        /// Store Infrared frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private ImageSource ToBitmap(InfraredFrame frame)
        {
            if (frame != null)
            {
                int width = frame.FrameDescription.Width;
                int height = frame.FrameDescription.Height;

                ushort[] infraredData = new ushort[width * height];
                byte[] pixels = new byte[width * height * (PixelFormats.Bgr32.BitsPerPixel + 7) / 8];

                frame.CopyFrameDataToArray(infraredData);

                int colorIndex = 0;
                for (int infraredIndex = 0; infraredIndex < infraredData.Length; ++infraredIndex)
                {
                    ushort ir = infraredData[infraredIndex];
                    byte intensity = (byte)(ir >> 8);

                    pixels[colorIndex++] = intensity; // Blue
                    pixels[colorIndex++] = intensity; // Green   
                    pixels[colorIndex++] = intensity; // Red

                    ++colorIndex;
                }

                int stride = width * PixelFormats.Bgr32.BitsPerPixel / 8;

                //return BitmapSource.Create(width, height, 96, 96, PixelFormats.Bgr32, null, pixels, stride);
                BitmapSource bs = BitmapSource.Create(width, height, 96, 96, PixelFormats.Bgr32, null, pixels, stride);
                /*WriteableBitmap infraredBitmap = new WriteableBitmap(bs);
                SaveFrames(infraredBitmap, 3);*/
                return bs;
            }
            else
            {
                return null;
            }
        }

        /// <summary>
        /// Method called when audio beam angle and/or confidence have changed.
        /// </summary>
        private void AudioBeamChanged()
        {
            // Maximum possible confidence corresponds to this gradient width
            const float MinGradientWidth = 0.04f;

            // Set width of mark based on confidence.
            // A confidence of 0 would give us a gradient that fills whole area diffusely.
            // A confidence of 1 would give us the narrowest allowed gradient width.
            float halfWidth = Math.Max(1 - this.beamAngleConfidence, MinGradientWidth) / 2;

            // Update the gradient representing sound source position to reflect confidence
            this.beamBarGsPre.Offset = Math.Max(this.beamBarGsMain.Offset - halfWidth, 0);
            this.beamBarGsPost.Offset = Math.Min(this.beamBarGsMain.Offset + halfWidth, 1);

            // Convert from radians to degrees for display purposes
            float beamAngleInDeg = this.beamAngle * 180.0f / (float)Math.PI;

            // Rotate gradient to match angle
            beamBarRotation.Angle = -beamAngleInDeg;
            beamNeedleRotation.Angle = -beamAngleInDeg;

            // Display new numerical values
            //beamAngleText.Text = string.Format(CultureInfo.CurrentCulture, Properties.Resources.BeamAngle, beamAngleInDeg.ToString("0", CultureInfo.CurrentCulture));
            //beamConfidenceText.Text = string.Format(CultureInfo.CurrentCulture, Properties.Resources.BeamAngleConfidence, this.beamAngleConfidence.ToString("0.00", CultureInfo.CurrentCulture));
        }

        /// <summary>
        /// Handles rendering energy visualization into a bitmap.
        /// </summary>
        /// <param name="sender">object sending the event.</param>
        /// <param name="e">event arguments.</param>
        private void UpdateEnergy(object sender, EventArgs e)
        {
            lock (this.energyLock)
            {
                // Calculate how many energy samples we need to advance since the last update in order to
                // have a smooth animation effect
                DateTime now = DateTime.UtcNow;
                DateTime? previousRefreshTime = this.lastEnergyRefreshTime;
                this.lastEnergyRefreshTime = now;

                // No need to refresh if there is no new energy available to render
                if (this.newEnergyAvailable <= 0)
                {
                    return;
                }

                if (previousRefreshTime != null)
                {
                    float energyToAdvance = this.energyError + (((float)(now - previousRefreshTime.Value).TotalMilliseconds * SamplesPerMillisecond) / SamplesPerColumn);
                    int energySamplesToAdvance = Math.Min(this.newEnergyAvailable, (int)Math.Round(energyToAdvance));
                    this.energyError = energyToAdvance - energySamplesToAdvance;
                    this.energyRefreshIndex = (this.energyRefreshIndex + energySamplesToAdvance) % this.energy.Length;
                    this.newEnergyAvailable -= energySamplesToAdvance;
                }

                // clear background of energy visualization area
                this.energyBitmap.WritePixels(this.fullEnergyRect, this.backgroundPixels, EnergyBitmapWidth, 0);

                // Draw each energy sample as a centered vertical bar, where the length of each bar is
                // proportional to the amount of energy it represents.
                // Time advances from left to right, with current time represented by the rightmost bar.
                int baseIndex = (this.energyRefreshIndex + this.energy.Length - EnergyBitmapWidth) % this.energy.Length;
                for (int i = 0; i < EnergyBitmapWidth; ++i)
                {
                    const int HalfImageHeight = EnergyBitmapHeight / 2;

                    // Each bar has a minimum height of 1 (to get a steady signal down the middle) and a maximum height
                    // equal to the bitmap height.
                    int barHeight = (int)Math.Max(1.0, this.energy[(baseIndex + i) % this.energy.Length] * EnergyBitmapHeight);

                    // Center bar vertically on image
                    var barRect = new Int32Rect(i, HalfImageHeight - (barHeight / 2), 1, barHeight);

                    // Draw bar in foreground color
                    this.energyBitmap.WritePixels(barRect, this.foregroundPixels, 1, 0);
                }
            }
        }

        /// <summary>
        /// Saves audio data stream to external HD
        /// </summary>
        private void SaveAudioData()
        {
            string time = System.DateTime.Now.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);

            string[] myPhotos = Environment.GetLogicalDrives();// GetFolderPath(Environment.SpecialFolder.MyPictures);
            string path = null;

            //Check if external hard drive was found
            if (myPhotos != null)
            {
                path = System.IO.Path.Combine(myPhotos[1] + "Social_Study\\DataRepository\\Audio\\", "KinectScreenshot-Audio-" + time + ".csv");
                using(var stream = File.CreateText(path))
                {
                    for(int i = 0; i < EnergyBitmapWidth; i++)
                    {
                        string audioBeam = beamAngleOutput[i].ToString();
                        string audioBeamConfidence = beamAngleConfidenceOutput[i].ToString();
                        string csvRow = string.Format("{0},{1}", audioBeam, audioBeamConfidence);
                        stream.WriteLine(csvRow);
                    }
                }
            }
        }

        /// <summary>
        /// Saves data stream frames to external HD
        /// </summary>
        /// <param name="bitmap">bitmap fro data stream</param>
        /// <param name="streamType">1-RGB, 2-Depth, 3-Infrared, 4-Audio</param>
        private void SaveFrames(WriteableBitmap bitmap, int streamType)
        {
            if (colorBitmap != null)
            {
                // create a png bitmap encoder which knows how to save a .png file
                BitmapEncoder encoder = new PngBitmapEncoder();

                // create frame from the writable bitmap and add to encoder
                encoder.Frames.Add(BitmapFrame.Create(colorBitmap));

                string time = System.DateTime.Now.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);

                string[] myPhotos = Environment.GetLogicalDrives();// GetFolderPath(Environment.SpecialFolder.MyPictures);
                string path = null;

                //Check if external hard drive was found
                if (myPhotos != null)
                {
                    if (myPhotos[1] != null)
                    {
                        if (streamType == 1)
                        {
                            path = System.IO.Path.Combine(myPhotos[1] + "Social_Study\\DataRepository\\RGB\\", "KinectScreenshot-Color-" + time + ".png");
                        }
                        else if (streamType == 2)
                        {
                            path = System.IO.Path.Combine(myPhotos[1] + "Social_Study\\DataRepository\\Depth\\", "KinectScreenshot-Color-" + time + ".png");
                        }
                        else if (streamType == 3)
                        {
                            path = System.IO.Path.Combine(myPhotos[1] + "Social_Study\\DataRepository\\Infrared\\", "KinectScreenshot-Color-" + time + ".png");
                        }
                        else if (streamType == 4)
                        {
                            path = System.IO.Path.Combine(myPhotos[1] + "Social_Study\\DataRepository\\Audio\\", "KinectScreenshot-Color-" + time + ".png");
                        }
                    }
                }

                // write the new file to disk
                try
                {
                    // FileStream is IDisposable
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
        }

        
        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }

        /// <summary>
        /// Directly accesses the underlying image buffer of the DepthFrame to 
        /// create a displayable bitmap.
        /// This function requires the /unsafe compiler option as we make use of direct
        /// access to the native memory pointed to by the depthFrameData pointer.
        /// </summary>
        /// <param name="depthFrameData">Pointer to the DepthFrame image data</param>
        /// <param name="depthFrameDataSize">Size of the DepthFrame image data</param>
        /// <param name="minDepth">The minimum reliable depth value for the frame</param>
        /// <param name="maxDepth">The maximum reliable depth value for the frame</param>
        private unsafe void ProcessDepthFrameData(IntPtr depthFrameData, uint depthFrameDataSize, ushort minDepth, ushort maxDepth)
        {
            // depth frame data is a 16 bit value
            ushort* frameData = (ushort*)depthFrameData;

            // convert depth to a visual representation
            for (int i = 0; i < (int)(depthFrameDataSize / this.depthFrameDescription.BytesPerPixel); ++i)
            {
                // Get the depth for this pixel
                ushort depth = frameData[i];

                // To convert to a byte, we're mapping the depth value to the byte range.
                // Values outside the reliable depth range are mapped to 0 (black).
                this.depthPixels[i] = (byte)(depth >= minDepth && depth <= maxDepth ? (depth / MapDepthToByte) : 0);
            }
        }

        /// <summary>
        /// Renders color pixels into the writeableBitmap.
        /// </summary>
        private void RenderDepthPixels()
        {
            this.depthBitmap.WritePixels(
                new Int32Rect(0, 0, this.depthBitmap.PixelWidth, this.depthBitmap.PixelHeight),
                this.depthPixels,
                this.depthBitmap.PixelWidth,
                0);
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }

        private void Color_Click(object sender, RoutedEventArgs e)
        {
            _mode = Mode.Color;
        }

        private void Depth_Click(object sender, RoutedEventArgs e)
        {
            _mode = Mode.Depth;
        }

        private void Infrared_Click(object sender, RoutedEventArgs e)
        {
            _mode = Mode.Infrared;
        }

        private void Body_Click(object sender, RoutedEventArgs e)
        {
            _mode = Mode.Body;
        }

        public enum Mode
        {
            Color,
            Depth,
            Infrared,
            Audio,
            Body
        }

        private void dataGrid1_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {

        }
    }
}
