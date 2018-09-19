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

#include <time.h>
#include <iomanip>
#include "Common.h"
#include "sba.h"

#define MAXITER 1000
#define CLOCKS_PER_MSEC (CLOCKS_PER_SEC/1000.0)
#define MIDDLEVIEW 3

using namespace std;
using namespace cv;

extern void loadimages(vector<Mat>& imgptr);

// Function to calculate the projection of ith point in image j, xij = aj*bi
// The equations are calculcated using MAPLE as x = PX;
// The below functions for proj and projac are taken from the Vincent's matlab toolbox implementation 
// for ProjectiveFull option
static void img_projPX(int j, int i, double *p, double *xyz, double *xij, void *adata)
{
   double x, y, z, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12;

   double t4, t5, t6, t7, t8;

   x = xyz[0];
   y = xyz[1];
   z = xyz[2];
   p1 = p[0];
   p2 = p[1];
   p3 = p[2];
   p4 = p[3];
   p5 = p[4];
   p6 = p[5];
   p7 = p[6];
   p8 = p[7];
   p9 = p[8];
   p10 = p[9];
   p11 = p[10];
   p12 = p[11];

   t4 = p9*x;
   t5 = p10*y;
   t6 = p11*z;
   t7 = p12+t4+t5+t6;
   t8 = 1/t7;
   xij[0] = t8*(p4+p1*x+p2*y+p3*z);
   xij[1] = t8*(p8+p5*x+p6*y+p7*z);
}

static void img_projPX_jac(int j, int i, double *p, double *xyz, double *Aij, double *Bij, void *adata)
{
   double x, y, z, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12;

   double t10, t11, t12, t13, t14, t15, t16, t17, t18, t19, t20, t21, t22, 
          t23, t24, t25, t26, t27, t28, t29, t30, t31, t32, t33, t34, t35, 
          t36, t37, t38, t39, t40, t41;

   x = xyz[0];
   y = xyz[1];
   z = xyz[2];
   p1 = p[0];
   p2 = p[1];
   p3 = p[2];
   p4 = p[3];
   p5 = p[4];
   p6 = p[5];
   p7 = p[6];
   p8 = p[7];
   p9 = p[8];
   p10 = p[9];
   p11 = p[10];
   p12 = p[11];

   t10 = p9*p9;
   t11 = x*x;
   t12 = t10*t11;
   t13 = p10*p10;
   t14 = y*y;
   t15 = t13*t14;
   t16 = p11*p11;
   t17 = z*z;
   t18 = t16*t17;
   t19 = p12*p12;
   t20 = p12*p9*2.0;
   t21 = p10*p9*y*2.0;
   t22 = p11*p9*z*2.0;
   t23 = t20+t21+t22;
   t24 = t23*x;
   t25 = p10*p12*y*2.0;
   t26 = p11*p12*z*2.0;
   t27 = p10*p11*y*z*2.0;
   t28 = t12+t15+t18+t19+t24+t25+t26+t27;
   t29 = 1/t28;
   t30 = p9*x;
   t31 = p10*y;
   t32 = p11*z;
   t33 = p12+t30+t31+t32;
   t34 = 1/t33;
   t35 = p2*y;
   t36 = p3*z;
   t37 = t34*x;
   t38 = t34*y;
   t39 = t34*z;
   t40 = p6*y;
   t41 = p7*z;
   Bij[0] = t29*(p1*p12-p4*p9+p1*p10*y-p2*p9*y+p1*p11*z-p3*p9*z);
   Bij[1] = -t29*(-p12*p2+p10*p4+x*(p1*p10-p2*p9)-p11*p2*z+p10*p3*z);
   Bij[2] = -t29*(-p12*p3+p11*p4+x*(p1*p11-p3*p9)+p11*p2*y-p10*p3*y);
   Bij[3] = t29*(p12*p5-p8*p9+p10*p5*y-p6*p9*y+p11*p5*z-p7*p9*z);
   Bij[4] = -t29*(-p12*p6+p10*p8+x*(p10*p5-p6*p9)-p11*p6*z+p10*p7*z);
   Bij[5] = -t29*(-p12*p7+p11*p8+x*(p11*p5-p7*p9)+p11*p6*y-p10*p7*y);
   Aij[0] = t37;
   Aij[1] = t38;
   Aij[2] = t39;
   Aij[3] = t34;
   Aij[4] = 0.0;
   Aij[5] = 0.0;
   Aij[6] = 0.0;
   Aij[7] = 0.0;
   Aij[8] = -t29*(p1*t11+x*(p4+t35+t36));
   Aij[9] = -t29*(p2*t14+p4*y+p1*x*y+p3*y*z);
   Aij[10] = -t29*(p3*t17+p4*z+p1*x*z+p2*y*z);
   Aij[11] = -t29*(p4+t35+t36+p1*x);
   Aij[12] = 0.0;
   Aij[13] = 0.0;
   Aij[14] = 0.0;
   Aij[15] = 0.0;
   Aij[16] = t37;
   Aij[17] = t38;
   Aij[18] = t39;
   Aij[19] = t34;
   Aij[20] = -t29*(p5*t11+x*(p8+t40+t41));
   Aij[21] = -t29*(p6*t14+p8*y+p5*x*y+p7*y*z);
   Aij[22] = -t29*(p7*t17+p8*z+p5*x*z+p6*y*z);
   Aij[23] = -t29*(p8+t40+t41+p5*x);
}

// This function computes the SBA using the sba library
void SBA(vector<Mat>* P, vector<Vec4d>* X, vector<vector<Point2d>>* x, 
         vector<Mat>* P_BA, vector<Vec4d>* X_BA, unsigned int nviews)
{
   size_t npts = X->size();

   // SBA parameters
   int cnp = 12;  // #parameters defining a single camera
   int pnp = 3;   // #parameters defining a single 3D point
   int mnp = 2;   // #parameters defining a single 2D point
   int verbose = 1;
   double opts[SBA_OPTSSZ], info[SBA_INFOSZ];
   opts[0]=SBA_INIT_MU; opts[1]=SBA_STOP_THRESH; opts[2]=SBA_STOP_THRESH;
   opts[3]=SBA_STOP_THRESH;
   //opts[3]=0.05*numprojs; // uncomment to force termination if the average reprojection error drops below 0.05
   opts[4]=0.0;
   //opts[4]=1E-05; // uncomment to force termination if the relative reduction in the RMS reprojection error drops below 1E-05

   clock_t start_time, end_time;    // For timing the SBA operation

   // SBA using the toolbox
   // visibility mask
   char *vmask = new char[nviews * npts]();
   fill(vmask, vmask + (nviews*npts), 1); // Initialize the visibility mask to 1, 
                                          // since all pts are visible in our case

   // allocate memory of size: #views*projection parameters + #points*point dimensions
   double *p = (double *)malloc((nviews*cnp + npts*pnp)*sizeof(double));
   // copy the data from P into this matrix - Reorder to have the middle view as first values
   //copy the Middle view matrix first
   int idx = 0;
   for (int row = 0; row < P->at(MIDDLEVIEW).rows; row++)
   {
      for (int col = 0; col < P->at(MIDDLEVIEW).cols; col++)
      {
         p[idx++] = P->at(MIDDLEVIEW).at<double>(row, col);
      }
   }
   for (unsigned int view = 0; view < nviews; view++)
   {
      if (view != MIDDLEVIEW)
      {
         //copy all the Projection matrices
         for (int row = 0; row < P->at(view).rows; row++)
         {
            for (int col = 0; col <P->at(view).cols; col++)
            {
               p[idx++] = P->at(view).at<double>(row, col);
            }
         }
      }
   }
   // Copy all the 3D points
   for (unsigned int pts = 0; pts < npts; pts++)
   {
      for (int i = 0; i < pnp; i++)
      {
         p[idx++] = X->at(pts)[i];
      }
   }

   // create x array with 2D points; points in all frames corresponding to the 3D point
   double *xMeasure = (double*)malloc(sizeof(double)*(npts*nviews*mnp));
   idx = 0;
   for (unsigned int pts = 0; pts < npts; pts++)
   {
      // Copy the middle view points first
      xMeasure[idx++] = x->at(MIDDLEVIEW)[pts].x;
      xMeasure[idx++] = x->at(MIDDLEVIEW)[pts].y;
      //cout << xMeasure[idx-1] << " ";
      //cout << "\n";

      // Copy the points from the rest of the views
      for (unsigned int view = 0; view < nviews; view++)
      {
         if (view != MIDDLEVIEW)
         {
            xMeasure[idx++] = x->at(view)[pts].x;
            xMeasure[idx++] = x->at(view)[pts].y;
            //cout << xMeasure[idx-1] << " ";
            //cout << "\n";
         }
      }
   }

   start_time = clock();
   int n = sba_motstr_levmar( npts,        // number of points
                              nviews,      // number of images
                              1,           // number of const points
                              vmask,       // visibility mask, all 1 for our case
                              p,           // parameter array
                              cnp,
                              pnp,
                              xMeasure,    // 2D image points
                              NULL,        // Covariance matrix
                              mnp,         // parameter of 2D points (mnp)
                              img_projPX,
                              img_projPX_jac,
                              NULL, 
                              MAXITER,
                              verbose,
                              opts,
                              info);
   end_time = clock();

   unsigned int numprojs = (unsigned int)x->size();
   fprintf(stdout, "SBA returned %d in %g iter, reason %g, error %g [initial %g], %d/%d func/fjac evals, %d lin. systems\n", n,
                    info[5], info[6], info[1]/numprojs, info[0]/numprojs, (int)info[7], (int)info[8], (int)info[9]);
   fprintf(stdout, "Elapsed time: %.2lf seconds, %.2lf msecs\n", ((double) (end_time - start_time)) / CLOCKS_PER_SEC,
                  ((double) (end_time - start_time)) / CLOCKS_PER_MSEC);
   fflush(stdout);

   // Copy the values out into X and P and reorder them
   for (unsigned int view = 0; view < nviews; view++)
   {
      P_BA->push_back(Mat(P->at(view).rows, P->at(view).cols, P->at(view).type()));
   }

   // Copy out the middle view projection matrix
   // Reodered P matrices to original sequence, 
   idx = 0;
   for (int row = 0; row < P->at(MIDDLEVIEW).rows; row++)
   {
      for (int col = 0; col < P->at(MIDDLEVIEW).cols; col++)
      {
         P_BA->at(MIDDLEVIEW).at<double>(row, col) = p[idx++];
      }
   }
   for (unsigned int view = 0; view < nviews; view++)
   {
      if (view != MIDDLEVIEW)
      {
         //copy all the Projection matrices
         for (int row = 0; row < P_BA->at(view).rows; row++)
         {
            for (int col = 0; col < P_BA->at(view).cols; col++)
            {
               P_BA->at(view).at<double>(row, col) = p[idx++];
            }
         }
      }
   }

   // Copy out the 3D points
   for (unsigned int i = 0; i < npts; i++)
   {
      //cout << "X[" << i << "]: " << p[idx] << " " << p[idx+1] << " " << p[idx+2] << "\n";
      X_BA->push_back(Vec4d(p[idx], p[idx+1], p[idx+2], 1));
      idx+=3;
      DEBUG_PRINT(DEBUG_LEVEL_INFO, X_BA->at(i));
   }

   //clean up all the allocated memory
   delete((void*)p);
   delete((void*)xMeasure);
}