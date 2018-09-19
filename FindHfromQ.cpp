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

#include <iostream>
#include <opencv2\core\core.hpp>

using namespace std;
using namespace cv;

extern int computeSVD(const Mat& Q, Mat& U, Mat& D, Mat& Vt, char jobu, char jobvt);

int FindHfromQ(Mat* QFinal, Mat& H)
{
   // SVD decomposition of Q matrix
   Mat Q = Mat(4, 4, CV_64FC1, QFinal->data);
   
   Mat U, D, Vt, V;
   computeSVD(Q, U, D, Vt, 'A', 'A');
   transpose(Vt, V);
   /*cout << "SVD d:\n" << D << endl;
   cout << "SVD u:\n" << U << endl;
   cout << "SVD vt:\n" << V << endl;*/

   sqrt(D, D);
   Mat Dsqrt = Mat::diag(D);
   Dsqrt.at<double>(3,3) = 1;
   //cout << "Dsqrt:\n" << Dsqrt << endl;

   H = U * Dsqrt;

   return 1;
}