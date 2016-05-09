using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Kinect;
using System.Drawing.Imaging;
using System.Drawing;
using System.Runtime.InteropServices;
using System.IO;
//using fingertrack21;


namespace FingerTracking
{
    public class KinectTracker
    {
        KinectSensor sensor;
        private bool connected = false;

        public Bitmap depthImage;

        private IntPtr depthPtr;

        Boolean flagDot1 = true;
        Boolean flagDot2 = true;
        private void skip() { }
        public delegate void afterReady();

        private afterReady afterDepthReady;
        public double lenth1;
        public double lenth2;
        public int[] myarrays1;//= new int[5];
        public int[] myarrays2;// = new int[5];
        int dotindex = 0;
        int trialindex = 0;
        public DotPuzzle1 puzzle;//=new DotPuzzle1();
        public int counter = 0;
        public int buffercount = 0;
        public int basex = 0;
        public int basey = 0;

        public int aimx = 1;
        public int aimy = 1;

        public double epsilonvar = 0;
        //////////////ponts list
        Random rndNum;

        // public Random random = new Random();

        public int pointx1 = 100;
        public int pointy1 = 150;
        public int pointx2 = -100;
        public int pointy2 = 150;

        public KinectSettings settings { get; set; }

        public List<Hand> hands { get; set; }

        public KinectTracker()
        {
            puzzle = new DotPuzzle1();
            myarrays1 = new int[5];
            myarrays2 = new int[5];
            //HashSet<int> set = new HashSet<int>(array1);
            //   random.NextBytes(arrays1);
            //  random.NextBytes(arrays2);
            //int []arrays1=new int[20];
             rndNum = new Random();
            
            for (int i = 0; i < 3; i++)
            {
                //arrays1[i] = rndNum.Next(100, 150);
                
               // arrays1[i] = new int[8] { 10, 20, 30, 40, 50, 60, 70, 75 };
                //arrays2 = new int[8] { 100, 110, 120, 130, 140, 150, 160, 170 };
                double angle = Math.PI * (i*120+60) / 180.0;
                double sinAngle = Constants.LENGTH * Math.Sin(angle);
                double cosAngle = Constants.LENGTH * Math.Cos(angle);

                //if ((sinAngle <= 200 && sinAngle >= 0) || (cosAngle <= 250 && cosAngle >= 50))
                //{
                myarrays1[i] = Convert.ToInt32(sinAngle) + pointx1;
                myarrays2[i] = Convert.ToInt32(cosAngle) + pointy1;

               // }
               // else
                //{
               //     angle = Math.PI * (temp + 180.0) / 180.0;
               //     sinAngle = 100 * Math.Sin(angle);
               //     cosAngle = 100 * Math.Cos(angle);
               //     myarrays1[i] = Convert.ToInt32(sinAngle) + 100;
                //    myarrays2[i] = Convert.ToInt32(cosAngle) + 150;
               // }

               // arrays1[i] = 100;
               // arrays2[i] = rndNum.Next(0, 360);
            }

            


            afterDepthReady = skip;

            settings = new KinectSettings();

            hands = new List<Hand>();

            // Check if there is any Kinect device connected
            if (KinectSensor.KinectSensors.Count > 0)
            {
                connected = true;
                sensor = KinectSensor.KinectSensors.ElementAt(0);



                sensor.DepthFrameReady += new EventHandler<DepthImageFrameReadyEventArgs>(depthFrameReady);


            }
            else // No device connected
            {
                connected = false;
            }

        }

        public void start()
        {
            sensor.DepthStream.Enable(settings.depthFormat);
           // sensor.ColorStream.Enable();
            sensor.Start();
        }

        public void stop()
        {
            sensor.DepthStream.Disable();
        //    sensor.ColorStream.Disable();
            sensor.Stop();
        }


        public void setEventDepthReady(afterReady del)
        {
            afterDepthReady = del;
        }

        public void clearEventDepthReady()
        {
            afterDepthReady = skip;
        }



        public void depthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            // Get the depth frame from Kinect
            DepthImageFrame frame = e.OpenDepthImageFrame();

            // Check that the frame is not null
            if (frame == null)
                return;

            // Calculate the real distance for every pixel in the depth image
            int[] distances = generateDistances(frame);

            // Return a 0 or 1 matrix, which contains wich pixels are near enough
            bool[][] near = generateValidMatrix(frame, distances);

            // Return the tracked hands based on the near pixels
            hands = localizeHands(near);

            byte[] pixels = new byte[frame.PixelDataLength * 4];

            // Free last depth Matrix
            Marshal.FreeHGlobal(depthPtr);
            depthPtr = Marshal.AllocHGlobal(pixels.Length);
            Marshal.Copy(pixels, 0, depthPtr, pixels.Length);

            // Create the bitmap
            int height = near.Length;
            int width = 0;
            if (near.Length > 0)
            {
                width = near[0].Length;
            }
            int stride = width * 4;

            depthImage = new Bitmap(
                width,
                height,
                stride,
                PixelFormat.Format32bppRgb,
                depthPtr);

            // Calculate 3D points for the hands
            for (int i = 0; i < hands.Count; ++i)
            {
                hands[i].calculate3DPoints(settings.screenWidth, settings.screenHeight, distances);
            }

            // Call the rest of the functions
            afterDepthReady();



            // Draw fingertips and palm center
            Graphics gBmp = Graphics.FromImage(depthImage);
            Brush blueBrush = new SolidBrush(Color.Blue);
            Brush redBrush = new SolidBrush(Color.Red);
            Brush greenBrush = new SolidBrush(Color.Green);

            if (hands.Count>0)
            {

                // Red point which is the center of the palm
                // gBmp.FillEllipse(redBrush, hands[i].palm.Y - 5, hands[i].palm.X - 5, 10, 10);

                // Draw inside points
                // for (int j = 0; j < hands[i].inside.Length; ++j)
                //{
                //     Point p = hands[i].inside.Get(j);
                //     depthImage.SetPixel(p.Y, p.X, Color.Blue);
                // }

                // Draw the contour of the hands

                for (int j = 0; j < hands[0].contour.Count; ++j)
                {
                    PointFT p = hands[0].contour[j];
                    depthImage.SetPixel(p.Y, p.X, Color.Red);
                }

                // End draw the contour of the hands

                // Blue points which represent the fingertips
                /* for (int j = 0; j < hands[i].fingertips.Count; ++j)
                 {
                     if (hands[i].fingertips[j].X != -1)
                     {
                         gBmp.FillEllipse(blueBrush, hands[i].fingertips[j].Y - 5 ,
                             hands[i].fingertips[j].X - 5, 10, 10);
                       //  this.puzzle.Dots.Add(new Point(hands[i].fingertips[j].Y, hands[i].fingertips[j].X));

                       //  int x = hands[i].fingertips[j].X - 5;
                       //  int y = hands[i].fingertips[j].Y - 5;


                     }
                 }*/

                if (hands[0].fingertips.Count != 0)
                {

                    //rotation Formula comes here
                    Rotation rot = new Rotation();

                    int x01 = hands[0].fingertips[0].X - 5;
                    int y01 = hands[0].fingertips[0].Y - 5;

                    /* // calculate real location of finger
                    int pixelIndex = y01+5+(x01+5)*frame.Width;
                    double depthTip = distances[pixelIndex];
                    double distanty = depthTip * Constants.HORIZONTALTANA * 2 * (y01) / frame.Width * Constants.MILLI;
                    double distantx = depthTip * Constants.VERTICALTANA * 2 * (x01) / frame.Height * Constants.MILLI;
                    double lenthhand = Math.Sqrt(distanty * distanty + distantx * distantx);
                    Console.Write(lenthhand);
                    Console.Write("##");
                    */

                    Point pt1 = new Point();

                    //if (trialindex < 91 || trialindex > 150) { this.epsilonvar = 0; }
                    pt1 = rot.Formula(x01, y01, this.epsilonvar, this.aimy, this.aimx, this.basey, this.basex);
                    //Console.Write(" blue dot " + pt1.X + " aim " + this.aimx + " base " + this.basex + " finger " + x01 + " epsilon " + epsilonvar);

                    // double ydiff = y01  + (this.epsilonvar * (y01 - this.aimy));
                    // double xdiff = x01  + (this.epsilonvar * (x01 - this.aimx));



                    //gBmp.FillEllipse(blueBrush, y01, x01, 10, 10);
                    gBmp.FillEllipse(blueBrush, pt1.Y, pt1.X, 10, 10);


                    int xdiff = pt1.X;
                    int ydiff = pt1.Y;
                    this.puzzle.Dots.Add(new Point(pt1.Y + 5, pt1.X + 5));
                    // this.puzzle.Dots.Add(new Point(y0+5, x0+5));



                    //int x0 = hands[0].fingertips[0].X - 5;
                    //int y0 = hands[0].fingertips[0].Y - 5;

                    //To print coordinates in the file text
                    //File.Create(path);


                    Point dotDiff1 = new Point(this.pointx1 - x01, this.pointy1 - y01);
                    this.lenth1 = Math.Sqrt(dotDiff1.X * dotDiff1.X + dotDiff1.Y * dotDiff1.Y);

                    Point dotDiff2 = new Point(this.pointx2 - x01, this.pointy2 - y01);
                    this.lenth2 = Math.Sqrt(dotDiff2.X * dotDiff2.X + dotDiff2.Y * dotDiff2.Y);
                    /*
                    Color col = Color.FromName("Red");
                    Pen pens = new Pen(col);
                    gBmp.DrawEllipse(pens, 50, 50, 10, 10);
                    Rectangle rect = new Rectangle(50, 50, 10, 10);
                    // Fill ellipse on screen.
                    gBmp.FillEllipse(redBrush, rect);
                    */
                    //Console.Write(this.lenth1);
                    //Console.Write(00000000000000000000000000);
                    //Console.Write(this.lenth2);

                    if (this.lenth1 < Constants.DISTANCE_HAND_DOT && flagDot2 == true)
                    {
                        buffercount++;
                        if (buffercount > 5)
                        {
                            // Random rnd1 = new Random();
                            // int xrand1 = rnd1.Next(10, 30);
                            // int yrand1 = rnd1.Next(10, 30);
                            //for(int i=0;)

                            this.pointx2 = this.myarrays1[dotindex];
                            this.pointy2 = this.myarrays2[dotindex];

                            this.basex = x01;
                            this.basey = y01;

                            this.aimx = this.pointx2;
                            this.aimy = this.pointy2;

                            this.epsilonvar = Constants.EPSILON;

                            dotindex++;
                            trialindex++;

                            Console.Write(" Trail " + trialindex);

                            if (dotindex == 3)
                                dotindex = 0;
                            flagDot2 = false;
                            flagDot1 = true;
                            this.puzzle.Dots.RemoveRange(0, this.puzzle.Dots.Count);

                            counter = 0;
                            buffercount = 0;

                        }
                        // Console.Write(this.pointx2);
                        //  Console.Write(this.pointy2);
                    }



                    if (this.lenth2 < Constants.DISTANCE_HAND_DOT && flagDot1 == true)
                    {
                        buffercount++;
                        if (buffercount > 5)
                        {
                            //  Random rnd2 = new Random();
                            // int xrand2 = rnd2.Next(120, 150);
                            //  int yrand2 = rnd2.Next(120, 150);
                            // this.pointx1 = xrand2;
                            //  this.pointy1 = yrand2;
                            // this.pointx1 = 100;
                            // this.pointy1 = 100;

                            // this.pointx1 = arrays1[dotindex];
                            // this.pointy1 = arrays1[dotindex];

                            dotindex++;
                            trialindex++;
                            Console.Write(" Trail " + trialindex);

                            //this.pointx1 = this.pointx1 + 5;
                            //this.pointy1 = this.pointy1 + 5;
                            if (dotindex == 3)
                                dotindex = 0;
                            flagDot1 = false;
                            flagDot2 = true;
                            this.puzzle.Dots.RemoveRange(0, this.puzzle.Dots.Count);
                            counter = 0;
                            buffercount = 0;

                            this.basex = x01;
                            this.basey = y01;

                            this.aimx = this.pointx1;
                            this.aimy = this.pointy1;
                            this.epsilonvar = Constants.EPSILON;
                        }

                        //  Console.Write(this.lenth2);
                        //Console.Write(this.lenth2);
                    }

                    Color col1 = Color.FromName("Red");
                    Color col2 = Color.FromName("Green");
                    Pen p1 = new Pen(col1);
                    Pen p2 = new Pen(col2);

                    // pt1.X = this.pointx1;
                    //pt1.Y = this.pointy1;
                    //Point pt2=new Point();
                    //pt2.X = this.pointx2;
                    //pt2.Y = this.pointy2; 


                    //basey = basey + 5;
                    //basex = basex + 5;

                    gBmp.DrawEllipse(p2, this.pointy1, this.pointx1, 10, 10);
                    Rectangle rect2 = new Rectangle(this.pointy1, this.pointx1, 10, 10);
                    // Fill ellipse on screen.
                    gBmp.FillEllipse(greenBrush, rect2);

                    gBmp.DrawEllipse(p1, this.pointy2, this.pointx2, 10, 10);

                    //SolidBrush redBrush = new SolidBrush(Color.Red);

                    // Create rectangle for ellipse.
                    //Rectangle rect = new Rectangle(x, y, width, height);
                    Rectangle rect3 = new Rectangle(this.pointy2, this.pointx2, 10, 10);
                    // Fill ellipse on screen.
                    gBmp.FillEllipse(redBrush, rect3);
                    gBmp.FillEllipse(blueBrush, pt1.Y, pt1.X, 10, 10);
                    if (counter < 100)
                    {
                        int y01w = y01 + 5;
                        int ydiffw = ydiff + 5;
                        int x01w = x01 + 5;
                        int xdiffw = xdiff + 5;
                        int dcount = this.puzzle.Dots.Count;

                        // double ydiff = y01w + (Constants.epsilon * (y01 - this.pointy1));
                        // double xdiff = x01w + (Constants.epsilon * (x01 - this.pointx1));

                        if (!File.Exists(Constants.path))
                        {
                            File.Create(Constants.path);
                            TextWriter tw = new StreamWriter(Constants.path, true);
                            tw.WriteLine(Convert.ToString(ydiffw + "," + xdiffw + "," + y01w + "," + x01w + "," + basey + "," + basex + "," + trialindex + "," + dcount));
                            tw.Close();
                        }

                        else if (File.Exists(Constants.path))
                        {
                            using (StreamWriter w = File.AppendText(Constants.path))
                            {
                                w.WriteLine(Convert.ToString(ydiffw + "," + xdiffw + "," + y01w + "," + x01w + "," + basey + "," + basex + "," + trialindex + "," + dcount));
                                w.Close();
                            }
                        }
                        if (this.puzzle.Dots.Count >= 2)
                        {
                            //foreach(Point pt in this.puzzle.Dots)
                            // Point pt2 = new Point();
                            // pt2 = rot.Formula(Convert.ToInt32(xdiff), Convert.ToInt32(ydiff));

                            Point point1 = new Point();
                            point1 = this.puzzle.Dots[0];

                            Point point2 = new Point();
                            foreach (Point pt in this.puzzle.Dots)
                            {
                                point2 = point1;
                                point1 = pt;
                                if (point1 != null && point2 != null)
                                {
                                    //gBmp.DrawLine(p1, this.puzzle.Dots[0], this.puzzle.Dots[1]);
                                    gBmp.DrawLine(p1, point2, point1);
                                }

                            }
                        }

                    }
                    //SolidBrush redBrush = new SolidBrush(Color.Red);

                    // Create rectangle for ellipse.
                    //Rectangle rect = new Rectangle(x, y, width, height);

                }


                blueBrush.Dispose();
                greenBrush.Dispose();
                redBrush.Dispose();
                gBmp.Dispose();
                frame.Dispose();
                counter++;
            }
        }


        private int[] generateDistances(DepthImageFrame frame)
        {
            // Raw depth data form the Kinect
            short[] depth = new short[frame.PixelDataLength];
            frame.CopyPixelDataTo(depth);

            // Calculate the real distance
            int[] distance = new int[frame.PixelDataLength];
            for (int i = 0; i < distance.Length; ++i)
            {
                //distance[i] = depth[i];// to fix range
                distance[i] = depth[i] >> DepthImageFrame.PlayerIndexBitmaskWidth;   // old program
            }

            return distance;
        }

        private bool[][] generateValidMatrix(DepthImageFrame frame, int[] distance)
        {
            // Create the matrix. The size depends on the margins
            int x1 = (int)(frame.Width * settings.marginLeftPerc / 100.0f);
            int x2 = (int)(frame.Width * (1 - (settings.marginRightPerc / 100.0f)));
            int y1 = (int)(frame.Height * settings.marginTopPerc / 100.0f);
            int y2 = (int)(frame.Height * (1 - (settings.marginBotPerc / 100.0f)));
            bool[][] near = new bool[y2 - y1][];
            for (int i = 0; i < near.Length; ++i)
            {
                near[i] = new bool[x2 - x1];
            }
            /*
            // Calculate max and min distance (useful when calculating real finger location )
            int max = int.MinValue, min = int.MaxValue;
            
            for (int k = 0; k < distance.Length; ++k)
            {
                if (distance[k] > max) max = distance[k];
                if (distance[k] < min && distance[k] != -1) min = distance[k];
            }
            */
            // Decide if it is near or not
          //  int margin = (int)(min + settings.nearSpacePerc * (max - min));
            int index = 0;
           // if (settings.absoluteSpace != -1) margin = min + settings.absoluteSpace;
            for (int i = 0; i < near.Length; ++i)
            {
                for (int j = 0; j < near[i].Length; ++j)
                {
                    index = frame.Width * (i + y1) + (j + x1);
                   if (distance[index] == 0)// old program
                  //  if (distance[index] <= 8000 && distance[index] >=7000) //fix range
                   // if (distance[index] <= margin && distance[index] != -1) //original
                    {
                        near[i][j] = true;
                    }
                    else
                    {
                        near[i][j] = false;
                    }
                }
            }

            // Dilate and erode the image to get smoother figures
            if (settings.smoothingIterations > 0)
            {
                near = dilate(near, settings.smoothingIterations);
                near = erode(near, settings.smoothingIterations);
            }

            // Mark as not valid the borders of the matrix to improve the efficiency in some methods
            int m;
            // First row
            for (int j = 0; j < near[0].Length; ++j)
                near[0][j] = false;

            // Last row
            m = near.Length - 1;
            for (int j = 0; j < near[0].Length; ++j)
                near[m][j] = false;

            // First column
            for (int i = 0; i < near.Length; ++i)
                near[i][0] = false;

            // Last column
            m = near[0].Length - 1;
            for (int i = 0; i < near.Length; ++i)
                near[i][m] = false;

            return near;
        }

        private List<Hand> localizeHands(bool[][] valid)
        {
            int i, j, k;

            List<Hand> hands = new List<Hand>();

            List<PointFT> insidePoints = new List<PointFT>();
            List<PointFT> contourPoints = new List<PointFT>();


            bool[][] contour = new bool[valid.Length][];
            for (i = 0; i < valid.Length; ++i)
            {
                contour[i] = new bool[valid[0].Length];
            }

            // Divide points in contour and inside points
            int count = 0;
            for (i = 1; i < valid.Length - 1; ++i)
            {
                for (j = 1; j < valid[i].Length - 1; ++j)
                {

                    if (valid[i][j])
                    {
                        // Count the number of valid adjacent points
                        count = this.numValidPixelAdjacent(ref i, ref j, ref valid);

                        if (count == 4) // Inside
                        {
                            insidePoints.Add(new PointFT(i, j));
                        }
                        else // Contour
                        {
                            contour[i][j] = true;
                            contourPoints.Add(new PointFT(i, j));
                        }

                    }
                }
            }

            // Create the sorted contour list, using the turtle algorithm
            for (i = 0; i < contourPoints.Count; ++i)
            {
                Hand hand = new Hand();

                // If it is a possible start point
                if (contour[contourPoints[i].X][contourPoints[i].Y])
                {

                    // Calculate the contour
                    hand.contour = CalculateFrontier(ref valid, contourPoints[i], ref contour);

                    // Check if the contour is big enough to be a hand
                    if (hand.contour.Count / (contourPoints.Count * 1.0f) > 0.20f
                        && hand.contour.Count > settings.k)
                    {
                        // Calculate the container box
                        hand.calculateContainerBox(settings.containerBoxReduction);

                        // Add the hand to the list
                        hands.Add(hand);
                    }

                    // Don't look for more hands, if we reach the limit
                    if (hands.Count >= settings.maxTrackedHands)
                    {
                        break;
                    }
                }

            }

            // Allocate the inside points to the correct hand using its container box

            //List<int> belongingHands = new List<int>();
            for (i = 0; i < insidePoints.Count; ++i)
            {
                for (j = 0; j < hands.Count; ++j)
                {
                    if (hands[j].isPointInsideContainerBox(insidePoints[i]))
                    {
                        hands[j].inside.Add(insidePoints[i]);
                        //belongingHands.Add(j);
                    }
                }

                // A point can only belong to one hand, if not we don't take that point into account
                /*if (belongingHands.Count == 1)
                {
                    hands[belongingHands.ElementAt(0)].inside.Add(insidePoints[i]);
                }
                belongingHands.Clear();*/
            }

            // Find the center of the palm
            float min, max, distance = 0;

            for (i = 0; i < hands.Count; ++i)
            {
                max = float.MinValue;
                for (j = 0; j < hands[i].inside.Count; j += settings.findCenterInsideJump)
                {
                    min = float.MaxValue;
                    for (k = 0; k < hands[i].contour.Count; k += settings.findCenterInsideJump)
                    {
                        distance = PointFT.distanceEuclidean(hands[i].inside[j], hands[i].contour[k]);
                        if (!hands[i].isCircleInsideContainerBox(hands[i].inside[j], distance)) continue;
                        if (distance < min) min = distance;
                        if (min < max) break;
                    }

                    if (max < min && min != float.MaxValue)
                    {
                        max = min;
                        hands[i].palm = hands[i].inside[j];
                    }
                }
            }

            // Find the fingertips
            PointFT p1, p2, p3, pAux, r1, r2;
            int size;
            double angle;
            int jump;

            for (i = 0; i < hands.Count; ++i)
            {
                // Check if there is a point at the beginning to avoid checking the last ones of the list
                max = hands[i].contour.Count;
                size = hands[i].contour.Count;
                jump = (int)(size * settings.fingertipFindJumpPerc);
                for (j = 0; j < settings.k; j += 1)
                {
                    if (hands[i].fingertips.Count > 0) { break; };
                    p1 = hands[i].contour[(j - settings.k + size) % size];
                    p2 = hands[i].contour[j];
                    p3 = hands[i].contour[(j + settings.k) % size];
                    r1 = p1 - p2;
                    r2 = p3 - p2;

                    angle = PointFT.angle(r1, r2);

                    if (angle > 0 && angle < settings.theta)
                    {
                        pAux = p3 + ((p1 - p3) / 2);
                        if (PointFT.distanceEuclideanSquared(pAux, hands[i].palm) >
                            PointFT.distanceEuclideanSquared(hands[i].contour[j], hands[i].palm))
                            continue;

                        hands[i].fingertips.Add(hands[i].contour[j]);
                        max = hands[i].contour.Count + j - jump;
                        max = Math.Min(max, hands[i].contour.Count);
                        j += jump;
                        break;
                    }
                }

                // Continue with the rest of the points
                if (hands[i].fingertips.Count > 0) { break; };
                /*
                for ( ; j < max; j += settings.findFingertipsJump)
                {
                    p1 = hands[i].contour[(j - settings.k + size) % size];
                    p2 = hands[i].contour[j];
                    p3 = hands[i].contour[(j + settings.k) % size];
                    r1 = p1 - p2;
                    r2 = p3 - p2;

                    angle = PointFT.angle(r1, r2);

                    if (angle > 0 && angle < settings.theta )
                    {
                        pAux = p3 + ((p1 - p3) / 2);
                        if (PointFT.distanceEuclideanSquared(pAux, hands[i].palm) >
                            PointFT.distanceEuclideanSquared(hands[i].contour[j], hands[i].palm))
                            continue;

                        hands[i].fingertips.Add(hands[i].contour[j]);
                        j += jump;
                    }
                }*/
            }

            return hands;
        }

        /*
         * This function calcute the border of a closed figure starting in one of the contour points.
         * The turtle algorithm is used.
         */
        private List<PointFT> CalculateFrontier(ref bool[][] valid, PointFT start, ref bool[][] contour)
        {
            List<PointFT> list = new List<PointFT>();
            PointFT last = new PointFT(-1, -1);
            PointFT current = new PointFT(start);
            int dir = 0;

            do
            {
                if (valid[current.X][current.Y])
                {
                    dir = (dir + 1) % 4;
                    if (current != last)
                    {
                        list.Add(new PointFT(current.X, current.Y));
                        last = new PointFT(current);
                        contour[current.X][current.Y] = false;
                    }
                }
                else
                {
                    dir = (dir + 4 - 1) % 4;
                }

                switch (dir)
                {
                    case 0: current.X += 1; break; // Down
                    case 1: current.Y += 1; break; // Right
                    case 2: current.X -= 1; break; // Up
                    case 3: current.Y -= 1; break; // Left
                }
            } while (current != start);

            return list;
        }

        private bool[][] dilate(bool[][] image, int it)
        {
            // Matrix to store the dilated image
            bool[][] dilateImage = new bool[image.Length][];
            for (int i = 0; i < image.Length; ++i)
            {
                dilateImage[i] = new bool[image[i].Length];
            }

            // Distances matrix
            int[][] distance = manhattanDistanceMatrix(image, true);

            // Dilate the image
            for (int i = 0; i < image.Length; i++)
            {
                for (int j = 0; j < image[i].Length; j++)
                {
                    dilateImage[i][j] = ((distance[i][j] <= it) ? true : false);
                }
            }

            return dilateImage;
        }

        private bool[][] erode(bool[][] image, int it)
        {
            // Matrix to store the dilated image
            bool[][] erodeImage = new bool[image.Length][];
            for (int i = 0; i < image.Length; ++i)
            {
                erodeImage[i] = new bool[image[i].Length];
            }

            // Distances matrix
            int[][] distance = manhattanDistanceMatrix(image, false);

            // Dilate the image
            for (int i = 0; i < image.Length; i++)
            {
                for (int j = 0; j < image[i].Length; j++)
                {
                    erodeImage[i][j] = ((distance[i][j] > it) ? true : false);
                }
            }

            return erodeImage;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="image"></param>
        /// <param name="zeroDistanceValue"></param>
        /// <returns></returns>
        private int[][] manhattanDistanceMatrix(bool[][] image, bool zeroDistanceValue)
        {
            int[][] distanceMatrix = new int[image.Length][];
            for (int i = 0; i < distanceMatrix.Length; ++i)
            {
                distanceMatrix[i] = new int[image[i].Length];
            }

            // traverse from top left to bottom right
            for (int i = 0; i < distanceMatrix.Length; i++)
            {
                for (int j = 0; j < distanceMatrix[i].Length; j++)
                {
                    if ((image[i][j] && zeroDistanceValue) || (!image[i][j] && !zeroDistanceValue))
                    {
                        // first pass and pixel was on, it gets a zero
                        distanceMatrix[i][j] = 0;
                    }
                    else
                    {
                        // pixel was off
                        // It is at most the sum of the lengths of the array
                        // away from a pixel that is on
                        distanceMatrix[i][j] = image.Length + image[i].Length;
                        // or one more than the pixel to the north
                        if (i > 0) distanceMatrix[i][j] = Math.Min(distanceMatrix[i][j], distanceMatrix[i - 1][j] + 1);
                        // or one more than the pixel to the west
                        if (j > 0) distanceMatrix[i][j] = Math.Min(distanceMatrix[i][j], distanceMatrix[i][j - 1] + 1);
                    }
                }
            }
            // traverse from bottom right to top left
            for (int i = distanceMatrix.Length - 1; i >= 0; i--)
            {
                for (int j = distanceMatrix[i].Length - 1; j >= 0; j--)
                {
                    // either what we had on the first pass
                    // or one more than the pixel to the south
                    if (i + 1 < distanceMatrix.Length)
                        distanceMatrix[i][j] = Math.Min(distanceMatrix[i][j], distanceMatrix[i + 1][j] + 1);
                    // or one more than the pixel to the east
                    if (j + 1 < distanceMatrix[i].Length)
                        distanceMatrix[i][j] = Math.Min(distanceMatrix[i][j], distanceMatrix[i][j + 1] + 1);
                }
            }

            return distanceMatrix;
        }

        /*
         * Counts the number of adjacent valid points without taking into account the diagonals
         */
        private int numValidPixelAdjacent(ref int i, ref int j, ref bool[][] valid)
        {
            int count = 0;

            if (valid[i + 1][j]) ++count;
            if (valid[i - 1][j]) ++count;
            if (valid[i][j + 1]) ++count;
            if (valid[i][j - 1]) ++count;
            //if (valid[i + 1][j + 1]) ++count;
            //if (valid[i + 1][j - 1]) ++count;
            //if (valid[i - 1][j + 1]) ++count;
            //if (valid[i - 1][j - 1]) ++count;

            return count;
        }

        // Generate a representable image of the valid matrix
        private byte[] generateDepthImage(bool[][] near)
        {
            // Image pixels
            byte[] pixels = new byte[near.Length * near[0].Length * 4];
            int width = near[0].Length;

            for (int i = 1; i < near.Length - 1; ++i)
            {
                for (int j = 1; j < near[i].Length - 1; ++j)
                {
                    if (near[i][j])
                    {

                        if (!near[i + 1][j] || !near[i - 1][j]
                        || !near[i][j + 1] || !near[i][j - 1]) // Is border
                        {
                            pixels[(i * width + j) * 4 + 0] = 255;
                            pixels[(i * width + j) * 4 + 1] = 0;
                            pixels[(i * width + j) * 4 + 2] = 0;
                            pixels[(i * width + j) * 4 + 3] = 0;
                        }
                        else
                        {
                            pixels[(i * width + j) * 4 + 0] = 255;
                            pixels[(i * width + j) * 4 + 1] = 255;
                            pixels[(i * width + j) * 4 + 2] = 255;
                            pixels[(i * width + j) * 4 + 3] = 0;
                        }
                    }
                }
            }

            return pixels;
        }

        public bool isConnected()
        {
            return connected;
        }

        public Bitmap getDepthImage()
        {
            return depthImage;
        }


    }

    public class DotPuzzle1
    {
        public DotPuzzle1()
        {
            this.Dots = new List<Point>();
        }

        public List<Point> Dots { get; set; }
    }
}
