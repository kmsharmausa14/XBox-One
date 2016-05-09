using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.Drawing.Imaging;

namespace FingerTracking
{
    public partial class MainWindow : Form
    {
        KinectTracker kinectController;
       // Boolean flag = true;
        public MainWindow()
        {
            InitializeComponent();
            

            kinectController = new KinectTracker();



            kinectController.setEventDepthReady(drawDepthImage);

            if (kinectController.isConnected())
            {
                kinectController.start();

                // Show the default values of the Kinect settings
                double initialTheta = kinectController.settings.theta / (Math.PI / 180);
                
               // thetaTrackBar.Value = (int) initialTheta;

                int initialK = kinectController.settings.k;
                
               // kTrackBar.Value = initialK;

              //  float initialNearSpace = kinectController.settings.nearSpacePerc;
                
             //   NearSpaceTrackBar.Value = (int) (initialNearSpace * 100);

              //  smoothTrackBar.Value = kinectController.settings.smoothingIterations;

               
              //  boxReductionTrackBar.Value = (int)(kinectController.settings.containerBoxReduction * 100);
            }
            else
            {
                // Show an error
                Console.WriteLine("There is not any Kinect device connected.\nConnect it and restart the application.\n");
            }

        }

        // Show the color image in both image elements
        
        // Show the color and tracked images in the image elements
        private void drawDepthImage()
        {
           // colorImage.Image = kinectController.getColorImage();
            trackingImage.Image = kinectController.getDepthImage();

            /*
            if (kinectController.Length() < Constants.DISTANCE_HAND_DOT)
            {
                Graphics g1 = trackingImage.CreateGraphics();
                Color col1 = Color.FromName("SlateBlue");
                Pen p1 = new Pen(col1);
                g1.DrawEllipse(p1, 80, 80, 10, 10);

                SolidBrush redBrush = new SolidBrush(Color.Red);

                // Create rectangle for ellipse.
                //Rectangle rect = new Rectangle(x, y, width, height);
                Rectangle rect = new Rectangle(80, 80, 10, 10);
                // Fill ellipse on screen.
                g1.FillEllipse(redBrush, rect);
                
            }
            
           
            if (flag == true)
            {
                Graphics g = trackingImage.CreateGraphics();
                Color col = Color.FromName("SlateBlue");
                Pen p = new Pen(col);
                g.DrawEllipse(p, 50, 50, 10, 10);
                flag = false;
            }

            */
        }

    }
}
