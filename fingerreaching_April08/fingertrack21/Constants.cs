using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace FingerTracking
{
    class Constants
    {
        /*
        public const int pointx1 = 50;
        public const int pointy1 = 50;
        public const int pointx2 = 100;
        public const int pointy2 = 100;
        */
        public const double DISTANCE_HAND_DOT = 7;
        public const string path = @"C:\Users\Tao\Desktop\" + "coordinates.csv";

        public static readonly double HORIZONTALTANA = Math.Tan(28.5 * Math.PI / 180);
        public static readonly double VERTICALTANA = Math.Abs(Math.Tan(21.5 * Math.PI / 180));
        public const double MILLI = 0.0393700787;
       // public const double epsilon = 0.23;
        public const double EPSILON = 10;
        public const double LENGTH = 80;
    }
}
