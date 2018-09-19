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
int computeSVD(const Mat& Q, Mat& U, Mat& D, Mat& Vt, char jobu, char jobvt);
/*computes a projection matrix given the fundamental matrix*/
Mat getprojmatrix(Mat& F)
{   
	Mat Ftemp;
	Mat W( 3, 1, CV_64F);
    Mat V( 3, 3, CV_64F);
    Mat U(3,3,CV_64F);
	Mat e(3,1,CV_64F);
	Mat P(3,4,CV_64F);
	Mat e1(3,3,CV_64F);
	F.copyTo(Ftemp);

	/*SVD::compute(F, W, U, V,SVD::FULL_UV);*/
	int value=0;
	/*compute SVD*/
	value=computeSVD(Ftemp,U,W,V,'S','S');
	
	e.at<double>(0)=U.at<double>(0,2);
	e.at<double>(1)=U.at<double>(1,2);
	e.at<double>(2)=U.at<double>(2,2);
	
	e1.at<double>(0,0)=0;
	e1.at<double>(0,1)=e.at<double>(2);
	e1.at<double>(0,2)=-(e.at<double>(1));
	e1.at<double>(1,0)=-(e.at<double>(2));
	e1.at<double>(1,1)=0;
	e1.at<double>(1,2)=e.at<double>(0);
	e1.at<double>(2,0)=e.at<double>(1);
	e1.at<double>(2,1)=-(e.at<double>(0));
	e1.at<double>(2,2)=0;
	
	e1=(-e1)*F;
	/*reshape and form Projection matrix*/
	P.at<double>(0,0)=e1.at<double>(0,0);
	P.at<double>(0,1)=e1.at<double>(0,1);
	P.at<double>(0,2)=e1.at<double>(0,2);
	P.at<double>(1,0)=e1.at<double>(1,0);
	P.at<double>(1,1)=e1.at<double>(1,1);
    P.at<double>(1,2)=e1.at<double>(1,2);
    P.at<double>(2,0)=e1.at<double>(2,0);
    P.at<double>(2,1)=e1.at<double>(2,1);
	P.at<double>(2,2)=e1.at<double>(2,2);
	P.at<double>(0,3)=e.at<double>(0);
	P.at<double>(1,3)=e.at<double>(1);
	P.at<double>(2,3)=e.at<double>(2);
	/*release resources*/
	Ftemp.~Mat();
	W.~Mat();
	U.~Mat();
	V.~Mat();
	e.~Mat();
	e1.~Mat();
    return P;
}