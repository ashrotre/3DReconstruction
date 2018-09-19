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

int computeSVD(const Mat& Q, Mat& U, Mat& D, Mat& Vt, char jobu, char jobvt)
{
   if (jobu == 'O')
   {
      cerr << "jobu 'O' not supported, enter 'A', 'S' or 'N'." << endl;
      return -1;
   }

   const int m = Q.rows;
   const int n = Q.cols;
   double *a = (double *)&(Q.data[0]);
   const int lda = n;
   const int ldu =(jobu=='A')?m:(jobu=='S')?MIN(m,n):0;
   const int ldvt = (jobvt == 'A') ? n : (jobvt == 'S') ? MIN(m,n) : 0;
   
   // allocate output matrices
   if (jobu == 'A')
      U.create(m, m, CV_64FC1);
   else if (jobu == 'S')
      U.create(m, MIN(m,n), CV_64FC1);

   if (jobvt == 'A')
      Vt.create(n, n, CV_64FC1);
   else if (jobvt == 'S')
      Vt.create(min(m,n), n, CV_64FC1);

   double *u_svd = (double *)&(U.data[0]);
   double *vt_svd = (double *)&(Vt.data[0]);

   // Diag values of the matrix Q
   D.create(1, MAX(1,MIN(m,n)), CV_64FC1);
   double *s = (double *)&(D.data[0]);//malloc(sizeof(double) * MAX(1, MIN(m,n)));
   double *superb = (double *)malloc(sizeof(double) * (MIN(m,n)-1));

   // SVD 
   lapack_int lresult = LAPACKE_dgesvd(LAPACK_ROW_MAJOR, jobu, jobvt, m, n, a, lda, s, u_svd, ldu, vt_svd, ldvt, superb);

   if (lresult != 0)
   {
      cerr << "LAPACKE_dgesvd returned: " << lresult << endl;
      return -1;
   }

   free(superb);

   return 1;
}