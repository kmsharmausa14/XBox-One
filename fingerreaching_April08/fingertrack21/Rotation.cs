using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;

namespace FingerTracking
{
    public class Rotation
    {
        Point pt1 = new Point();

        public Point Formula(int x0, int y0, double epsilonvar, int aimy, int aimx, int basey, int basex)
        {
            /* quadratic noise
             * 
            double Delta = epsilonvar * Math.Sqrt((aimx - basex)*(aimx - basex));

            double xdiff0 = Delta * (-4 * x0 * x0 + 4 * (aimx + basex)*x0 - 4 * aimx * basex) / ((aimx - basex) * (aimx - basex)+0.1);

            Delta = epsilonvar * Math.Sqrt((aimy - basey) * (aimy - basey));

            double ydiff0 = Delta * (-4 * y0 * y0 + 4 * (aimy + basey)*y0 - 4 * aimy * basey) / ((aimy - basey) * (aimy - basey)+0.1);

            double xdiff = Math.Max(xdiff0 + x0, x0);
            double ydiff = Math.Max(ydiff0 + y0, y0);

           // double ydiff = y0 + (epsilonvar * (y0 - aimy));
            //double xdiff = x0 + (epsilonvar * (x0 - aimx));
             * 
            */

           // double theta = Math.Atan((x0-basex)/(y0-basey));

            x0 = x0 - basex;
            y0 = y0 - basey;

            double r = Math.Sqrt(x0 * x0 + y0 * y0);

            double n = 1;

            double theta = 0;

            if (x0 > 0)
            {
                theta = Math.Acos(y0 / (r+0.1));
            }
            else
            {
                theta = 2 * Math.PI - Math.Acos(y0 / (r + 0.1));
            }

            double ydiff = y0 - Math.Sin(theta) * epsilonvar * Math.Sin(2 * Math.PI * n / Constants.LENGTH * y0 / Math.Cos(theta)) + basey;
            double xdiff = x0 + Math.Cos(theta) * epsilonvar * Math.Sin(2 * Math.PI * n / Constants.LENGTH * y0 / Math.Cos(theta)) + basex;

            pt1.X = Convert.ToInt32(xdiff);
            pt1.Y = Convert.ToInt32(ydiff);
            return pt1;
        }
    }
}
