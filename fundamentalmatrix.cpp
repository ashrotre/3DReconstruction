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

extern Mat fundamentalmatrix( InputArray _points1, InputArray _points2,
	                           double param1, double param2,
	                           OutputArray _mask,long int& rngstate);
extern int computeSVD(const Mat& Q, Mat& U, Mat& D, Mat& Vt, char jobu, char jobvt);

/*normalizes points before computing fundamental matrix*/
Mat normalize2dpts(vector<Point2d>& keypt)
{ 
	Point2d temp;
	temp.x=0;
	temp.y=0;
	double t;
	int size1=(int)keypt.size();
	t=(1./size1);
	for( int i=0;i<size1;i++)
	{	
		temp.x+=keypt.at(i).x;
		temp.y+=keypt.at(i).y;
	}
	temp.x*=t;
	temp.y*=t;
	double x1,y1,scale=0;
	for (int i=0;i<size1;i++)
	{
		x1=keypt.at(i).x-temp.x;
		y1=keypt.at(i).y-temp.y;
		scale+=std::sqrt(x1*x1+y1*y1);
	}
	scale*=t;
	scale=std::sqrt(2.)/scale;

	for (int i=0;i<size1;i++)
	{
		keypt.at(i).x=(keypt.at(i).x-temp.x)*scale;
		keypt.at(i).y=(keypt.at(i).y-temp.y)*scale;
	}

	Mat T(3,3,CV_64F);// transform matrix
	T.at<double>(0,0)=scale; T.at<double>(0,1)=0; T.at<double>(0,2)=-temp.x*scale; T.at<double>(1,0)=0;
	T.at<double>(1,1)=scale; T.at<double>(1,2)=-temp.y*scale; T.at<double>(2,0)=0; T.at<double>(2,1)=0; 
	T.at<double>(2,2)=1;

	return T;

}

/* finds inliers for the least squares fitting after fundamental matrix computation*/
void getInliers(Mat& indxmask,vector<Point2d>& Keypts1,vector<Point2d>& Keypts2,vector<Point2d>& outkey1,vector<Point2d>& outkey2)
{  
	int j=0;
	for(int i=0;i<indxmask.rows;i++)
	{
		if (indxmask.at<bool>(i)==1)
		{
			outkey1.push_back(Keypts1.at(i));
			outkey2.push_back(Keypts2.at(i));
			j+=1;
		}
	}
	cout<<"\n"<<"number of inliers";cout<<"\n";
	cout<<j;
	cout<<"\n";
	cout<<"percentage of inliers";
	cout<<(float(j)/float(indxmask.rows));
}

/* computes least squares fit for obtained inliers*/
Mat fundmat(vector<Point2d>& keys1,vector<Point2d>& keys2,Mat& indxmask)
{
	Mat T_1,T_2;

	//vector<Point2f> keys1,keys2;
	//getInliers(indxmask,keypts1,keypts2,keys1,keys2);

	T_1=normalize2dpts(keys1);// normalize points


	T_2=normalize2dpts(keys2);//normalize points

	int count=(int)keys1.size();
	Mat A(count,9,CV_64F);

	Mat U_1(count,count,CV_64F);
	Mat W( 9, 1, CV_64F);
	Mat V( 9, 9, CV_64F);

	Mat U, TF,W1;

	for( int i = 0; i < count; i++ )//form constrained matrix
	{
		double x1,x2,y1,y2;

		x1 = keys1.at(i).x;
		y1 = keys1.at(i).y;

		x2 = keys2.at(i).x;
		y2 = keys2.at(i).y;

		A.at<double>(i,0)=x2*x1; A.at<double>(i,1)=x2*y1; A.at<double>(i,2)=x2; A.at<double>(i,3)=y2*x1;
		A.at<double>(i,4)= y2*y1; A.at<double>(i,5)=y2; A.at<double>(i,6)=x1; A.at<double>(i,7)=y1; A.at<double>(i,8)=1;
	}

	if( count==8)
		/*SVDecomp(A, W,U_1,V,SVD::FULL_UV);*/
	{
		int value=0;
		value=computeSVD( A, U_1, W, V, 'A', 'A');
	}
	else
		/* SVDecomp(A, W,U_1,V);*/
	{
		int value=0;
		value=computeSVD( A, U_1, W, V, 'S', 'S');
	}

	Mat F0(3,3,CV_64F);
	double minval=100000;
	/*reshape last row of Vt to form F0*/
	F0.at<double>(0,0)=V.at<double>(8,0);F0.at<double>(0,1)=V.at<double>(8,1);F0.at<double>(0,2)=V.at<double>(8,2);
	F0.at<double>(1,0)=V.at<double>(8,3);F0.at<double>(1,1)=V.at<double>(8,4);F0.at<double>(1,2)=V.at<double>(8,5);
	F0.at<double>(2,0)=V.at<double>(8,6);F0.at<double>(2,1)=V.at<double>(8,7);F0.at<double>(2,2)=V.at<double>(8,8);

	//cout<<"\n"<<F0;
	/* SVDecomp( F0, W1, U, V, SVD::FULL_UV );*/
	U.~Mat();
	V.~Mat();
	W1.~Mat();
	Mat V1;
	Mat F0_1;
	F0.copyTo(F0_1);
	int value=0;
	value=computeSVD( F0_1, U, W1, V1, 'A', 'A');

	W1.at<double>(2)=0;
	// F0 <- U*diag([W(1), W(2), 0])*V'
	gemm( U, Mat::diag(W1), 1., 0, 0., TF,0 );
	gemm( TF, V1, 1., 0, 0., F0, 0 );

	// F0 <- T2'*F0*T1
	gemm( T_2, F0, 1., 0, 0., TF,GEMM_1_T );
	F0 = Mat(3, 3, CV_64F);
	gemm( TF, T_1, 1., 0, 0., F0, 0 );

	// release resources
	V1.~Mat();
	F0_1.~Mat();
	T_1.~Mat();
	T_2.~Mat();
	A.~Mat();
	TF.~Mat();
	U.~Mat();
	V.~Mat();
	W1.~Mat();
	W.~Mat();
	//return final matrix
	return(F0);

}

/*Wrapper function that computes final value of fundamental matrix */
Mat getfundmat(vector<Point2d>& keypts1,vector<Point2d>& keypts2,vector<Point2d>& new1,vector<Point2d>& new2,Mat& indxmask,long int& rngstate)
{
	vector<Point2d> temp1,temp2,temp3,temp4;
	temp1=keypts1;
	temp2=keypts2;
	Mat T1,T2,Fundm;
	/* normalize points */
	T1=normalize2dpts(keypts1);
	T2=normalize2dpts(keypts2);

	Fundm=fundamentalmatrix(keypts1,keypts2,0.001,0.99,indxmask,rngstate);//compute best fit model for Fundamental matrix

	/*Find inliers for least squares fit*/
	getInliers(indxmask,keypts1,keypts2,temp3,temp4);
	getInliers(indxmask,temp1,temp2,new1,new2);
	Fundm=fundmat(temp3,temp4,indxmask);

	//release matrices
	temp1.~vector();
	temp2.~vector();
	temp3.~vector();
	temp4.~vector();
	Mat TF(3,3,CV_64F);
	/*denormalize fundamental matrix*/
	gemm( T2, Fundm, 1., 0, 0., TF,GEMM_1_T );
	gemm( TF, T1, 1., 0, 0., Fundm, 0 );
	/*release resources*/
	T1.~Mat();
	T2.~Mat();
	return(Fundm);
}
