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
extern int computeSVD(const Mat& Q, Mat& U, Mat& D, Mat& Vt, char jobu, char jobvt);
/*traingulates a set of 8 2D points to a common 3D point*/
Mat triangulate(vector<Point2d> pts,vector<Mat> p,double a[])
{ 
	double h[3][3]={{2/a[0],0,-1},{0,2/a[1],-1},{0,0,1}};//preconditioning matrix
	Mat A(24,4,CV_64F);
	Mat W,V,U;
	double temp[3][3];
	Mat X(4,1,CV_64F);//store tringulated points
	int i=0,j=0;
	for ( i=0;i<8;i++)
	{
		pts.at(i).x=(h[0][0]*pts.at(i).x)-1;
		pts.at(i).y=(h[1][1]*pts.at(i).y)-1;

		// compute contraction with epsilon tensor
		temp[0][0]=0;
		temp[0][1]=1;
		temp[0][2]=-(pts.at(i).y);
		temp[1][0]=-1;
		temp[1][1]=0;
		temp[1][2]=pts.at(i).x;
		temp[2][0]=pts.at(i).y;
		temp[2][1]=-(pts.at(i).x);
		temp[2][2]=0;
		for( j=0;j<4;j++)// generate constraint matrix
		{
			A.at<double>(3*i,j)= (temp[0][0]*p.at(i).at<double>(0,j))+(temp[0][1]*p.at(i).at<double>(1,j))+(temp[0][2]*p.at(i).at<double>(2,j));
			A.at<double>(3*i+1,j)= (temp[1][0]*p.at(i).at<double>(0,j))+(temp[1][1]*p.at(i).at<double>(1,j))+(temp[1][2]*p.at(i).at<double>(2,j));
			A.at<double>(3*i+2,j)= (temp[2][0]*p.at(i).at<double>(0,j))+(temp[2][1]*p.at(i).at<double>(1,j))+(temp[2][2]*p.at(i).at<double>(2,j));
		}
	}
	/*compute SVD*/

	int value=0;
	value=computeSVD(A, U, W, V,'S','S');

	X.at<double>(0)=(double)V.at<double>(3,0);
	X.at<double>(1)=(double)V.at<double>(3,1);
	X.at<double>(2)=(double)V.at<double>(3,2);
	X.at<double>(3)=(double)V.at<double>(3,3);
	V.~Mat();
	W.~Mat();
	U.~Mat();
	/*Check orientation of points*/
	double t=0;
	for ( i=0;i<8;i++)
	{ 
		t=(p.at(i).at<double>(2,0)*X.at<double>(0))+(p.at(i).at<double>(2,1)*X.at<double>(1))
			+(p.at(i).at<double>(2,2)*X.at<double>(2))+(p.at(i).at<double>(2,3)*X.at<double>(3));
		if (t<0)// invert orientation
		{
			X.at<double>(0)=-(X.at<double>(0));
			X.at<double>(1)=-(X.at<double>(1));
			X.at<double>(2)=-(X.at<double>(2));
			X.at<double>(3)=-(X.at<double>(3));
			break;
		}
	}

	return(X);
}

/*Wrapper function that performs traingulation for all common points by calling function triangulate for each set of 8 2D points*/
void triangulatepts(vector<Vec4d>& X4D,vector<vector<Point2d>> finpts,vector<Mat> projm,double size[],int minsize)
{
	vector<Point2d> tempxy(8);
	Mat t1;
	cout<<" Traingulating points..."<<"\n";
	/*Compute 3D points for all 2D points */
	for (int j=0;j<minsize;j++)
	{   
		tempxy.at(0)=finpts.at(0).at(j); tempxy.at(1)=finpts.at(1).at(j); tempxy.at(2)=finpts.at(2).at(j);
		tempxy.at(3)=finpts.at(3).at(j); tempxy.at(4)=finpts.at(4).at(j); tempxy.at(5)=finpts.at(5).at(j);
		tempxy.at(6)=finpts.at(6).at(j); tempxy.at(7)=finpts.at(7).at(j);
       /*compute triangulation*/
		transpose(triangulate(tempxy,projm,size),t1);
		X4D.push_back(t1);
		//  cout<<"\n"<<X4D.at(j);
	}
}