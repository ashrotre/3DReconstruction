/*****************************************************************************************************************************
* Copyright:
* 
* ------ COPYRIGHT NOTICE, CONDITIONS, AND DISCLAIMER START WITH THIS LINE -----------------
* 
* Copyright (c) 2010-2013 Arizona Board of Regents.  All Rights Reserved.
* 
* Contact: Lina Karam (karam@asu.edu), Aditee Shrotre (ashrotre@asu.edu), Tejas Borkar (tsborkar@asu.edu), 
*          Jinjin Li (jinjinli@asu.edu)
* 
* Image, Video, and Usabilty (IVU) Lab, http://ivulab.asu.edu , Arizona State University
* 
* This copyright statement may not be removed from any file containing it or from modifications to these files.
* This copyright notice must also be included in any file or product that is derived from the source files.
*  
* Redistribution and use of this code in source and binary forms,  with or without modification, 
* are permitted provided that the following conditions are met:
* 
* - Redistribution's of source code must retain the above copyright notice, this list of conditions
*   and the following disclaimer.
* 
* - Redistribution's in binary form must reproduce the above copyright notice, this list of conditions 
*   and the following disclaimer in the documentation and/or other materials provided with the distribution.
* 
* - The Image, Video, and Usability Laboratory (IVU Lab, http://ivulab.asu.edu) is acknowledged in any 
*   publication that reports research results using this code, copies of this code, or modifications of this code.
* 
* DISCLAIMER:
* 
* This software is provided by the copyright holders and contributors "as is" and any express or implied 
* warranties, including, but not limited to, the implied warranties of merchantability and fitness for a 
* particular purpose are disclaimed.
* 
* In no event shall the Arizona Board of Regents, Arizona State University, IVU Lab members, authors or contributors
* be liable for any direct, indirect, incidental, special, exemplary, or consequential damages (including, but not 
* limited to, procurement of substitute goods or services; loss of use, data, or profits; or business interruption) 
* however caused and on any theory of liability, whether in contract, strict liability, or tort (including negligence
* or otherwise) arising in any way out of the use of this software, even if advised of the possibility of such damage.
* 
* ------ COPYRIGHT NOTICE, CONDITIONS, AND DISCLAIMER END WITH THIS LINE -----------------

* ****************************************************************************************************************************/

#include "Common.h"

using namespace std;
using namespace cv;

extern void loadimages(vector<Mat>& imgptr);

extern vector<Mat> imgstack1;

// Calculates the reprojection error
double CalcReProjErr(vector<Mat>* P_BA, vector<Vec4d>* X_BA, vector<vector<Point2d>>* x_ori, unsigned int nviews)
{
   size_t npts = X_BA->size();

   // K Normalization matrix
   Mat K = (Mat_<double>(3,3) << 
                1020, 0, 0.5*imgstack1[0].cols,
                0, 1020, 0.5*imgstack1[0].rows,
                0, 0, 1); 

   vector<vector<Point2d>> center;
   vector<vector<Point2d>> center_reproj;

   vector<Mat> diff;
   for (size_t view = 0; view < nviews; view++)
   {
      vector<Point2d> cen;
      vector<Point2d> cen_reproj;

      Mat diff_view = (Mat_<double>(3,1) << 0.0, 0.0, 0.0);
      for (size_t pt = 0; pt < npts; pt++)
      {
         // Calculate and normalize reprojection
         Mat X = (Mat_<double>(4,1) << 
                  X_BA->at(pt)[0], X_BA->at(pt)[1], X_BA->at(pt)[2], X_BA->at(pt)[3]);
         Mat x_reProj = K * P_BA->at(view) * X;
         //cout << K << " * " << x_temp << "\n";
         x_reProj /= x_reProj.at<double>(2,0);
         cen_reproj.push_back(Point2d(x_reProj.at<double>(0,0),x_reProj.at<double>(1,0)));
         //cout << "x_reProj: " << x_reProj << "\n";

         // Normalize the original x's
         Mat x = (Mat_<double>(3,1) << 
                  x_ori->at(view)[pt].x, x_ori->at(view)[pt].y, 1);
         //cout << "x: " << x << "\n";
         Mat x_norm = K * x;
         x_norm /= x_norm.at<double>(2,0);
         cen.push_back(Point2d(x_norm.at<double>(0,0), x_norm.at<double>(1,0)));
         //cout << "x_norm: " << x_norm << "\n";

         // Difference between the normalized reprojected x and original normalized x
         Mat d = abs(x_reProj - x_norm);
         //cout << "d: " << d << "\n";
         diff_view += d;
         //cout << "diff: " << diff_view << "\n";
      }

      //store the center points
      center.push_back(cen);
      center_reproj.push_back(cen_reproj);

      //Avg the differences
      diff_view /= npts;

      // Average differences for every view
      diff.push_back(diff_view);
   }

#ifdef DISPLAY_REPROJECTED
   // Show the reprojected points on to the image 
   namedWindow("Reprojected Pts", CV_WINDOW_AUTOSIZE);
   vector<Mat> imgBGR(nviews);
   for (unsigned int view = 0; view < nviews; view++)
   {
      cvtColor(imgstack1[view], imgBGR[view], CV_GRAY2BGR);
      for (unsigned int pts = 0; pts < npts; pts++)
      {
         circle(imgBGR[view], center[view][pts], 5, Scalar(0,0,255), 1, 8, 0); //red-original
         circle(imgBGR[view], center_reproj[view][pts], 5, Scalar(255,0,0), 1, 8, 0); //blue-reprojected
      }

      imshow("Reprojected Pts", imgBGR[view]);
      waitKey();
   }
#endif

   // Average over all views
   Mat accSum = (Mat_<double>(3,1) << 0.0, 0.0, 0.0);
   for (size_t view = 0; view < diff.size(); view++)
   {
      accSum += diff[view];
   }
   accSum /= (double)diff.size();

   double avgDiff = (accSum.at<double>(0,0) + accSum.at<double>(1,0)) / 2;

   return avgDiff;
}