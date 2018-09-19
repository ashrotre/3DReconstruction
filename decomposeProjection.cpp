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

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <opencv2\core\core.hpp>
#include <mkl.h>

using namespace cv;
using namespace std;

int decomposeRQ(const Mat& S, Mat& U, Mat& Q)
{
   Mat S_tmp;
   transpose(S, S_tmp);
   Mat Sreodered(S.rows, S.cols, S.type());
   for (int row = 0, rowS = S.rows-1; row < S.rows && rowS >= 0 ; row++, rowS--)
   {
      for (int col = 0, colS = S.cols-1; col < S.cols && colS >= 0 ; col++, colS--)
      {
         Sreodered.at<double>(row,col) = S_tmp.at<double>(rowS, colS);
      }
   }
   cout << Sreodered << endl;

   // decompose QR
   const int m = Sreodered.rows;
   const int n = Sreodered.cols;
   double* a = (double *)&(Sreodered.data[0]);
   const int lda = MAX(m,1);//m > 1 ? m : 1; //max(1,m)
   double* tau = (double *)malloc(MIN(m,n) * sizeof (double));
   lapack_int result = LAPACKE_dgeqrfp(LAPACK_ROW_MAJOR, m, n, a, lda, tau);
   if (result != 0 )
   {
      cerr << "LAPACKE_dgeqrf returned: " << result << endl;
      return -1;
   }

   // Extract R into U
   Mat R_tmp = Mat::zeros(S.rows, S.cols, S.type());
   for (int row = 0; row < R_tmp.rows; row++)
   {
      for (int col = 0; col < R_tmp.cols; col++)
      {
         if (col >= row)
            R_tmp.at<double>(row,col) = Sreodered.at<double>(row,col);
      }
   }
   
   // Extract Q and copy to Q
   result = LAPACKE_dorgqr(LAPACK_ROW_MAJOR, m, n, n, a, lda, tau);
   if (result != 0 )
   {
      cerr << "LAPACKE_dorgqr returned: " << result << endl;
      return -1;
   }
   Mat Q_tmp;
   Sreodered.copyTo(Q_tmp);

   // Transpose Q and U and revert to original order
   transpose(Q_tmp, Q_tmp);
   transpose(R_tmp, R_tmp); 
   Q = Mat::zeros(Q_tmp.rows, Q_tmp.cols, Q_tmp.type());
   U = Mat::zeros(Q_tmp.rows, Q_tmp.cols, Q_tmp.type());
   for (int row = 0, rowS = Q.rows-1; row < Q.rows && rowS >= 0 ; row++, rowS--)
   {
      for (int col = 0, colS = Q.cols-1; col < Q.cols && colS >= 0 ; col++, colS--)
      {
         Q.at<double>(row,col) = Q_tmp.at<double>(rowS, colS);
         U.at<double>(row,col) = R_tmp.at<double>(rowS, colS);
      }
   }

   if (determinant(Q) < 0)
   {
      for (int row = 0; row < Q.rows; row++)
      {
         // possible because of square matrix
         U.at<double>(row, 0) = -U.at<double>(row, 0);
         Q.at<double>(0, row) = -Q.at<double>(0, row);
      }
   }
   
   free(tau);

   return 1;
}

void decomposeProjection (const Mat& P, Mat& K, Mat& R, Mat& C)
{
   int n = P.rows;
   // RQ decomposition
   Mat H;
   Rect ROI((int)0, (int)0, (int)n, (int)n);
   Mat matROI (P, ROI);
   matROI.copyTo(H);
   //cout << "H:\n" << H << endl;
   int result = decomposeRQ(H, K, R);
   if (result != 1)
   {
      cerr << "decomposeRQ Failed." << endl;
   }

   // Scale the values
   K /= K.at<double>(n-1,n-1);
   if ( K.at<double>(0,0) < 0 )
   {
      Mat d1 = (Mat_<double>(1, 2) << -1, -1);
      Mat d2 = Mat::ones(1, n-2, CV_64FC1);
      hconcat(d1, d2, d1);
      Mat D = Mat::diag(d1);
      K = K * D;
      R = D * R;
   }
   //cout << "K: \n" << K << endl; 
   //cout << "R: \n" << R << endl;

   // Find t
   C = (Mat_<double>(n, 1) << P.at<double>(0,3),
                              P.at<double>(1,3),
                              P.at<double>(2,3));
   H = -H;
   double *a = (double *)&(H.data[0]);
   double *b = (double *)&(C.data[0]);
   const int nlhs = H.rows;
   const int nrhs = C.cols;
   const int lda = MAX(1, nlhs);
   const int ldb = MAX(1, nrhs);
   //const int ldx = MAX(1, nlhs);
   int *ipiv = (int *)malloc(sizeof(int) * MAX(1,nlhs));
   lapack_int lresult = LAPACKE_dgesv(LAPACK_ROW_MAJOR, nlhs, nrhs, a, lda, ipiv, b, ldb);
   if (lresult != 0)
      cerr << "LAPACKE_dgesv returned: " << lresult << endl;

   free(ipiv);
}