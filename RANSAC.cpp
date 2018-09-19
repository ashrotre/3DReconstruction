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
#include <opencv2\calib3d\calib3d.hpp>
#include <opencv/cv.h>

//#include "opencv2/core/core.hpp"  
/*class for computing fundamental matrix using ransac */
using namespace cv;
using namespace std;
extern int computeSVD(const Mat& Q, Mat& U, Mat& D, Mat& Vt, char jobu, char jobvt);
class FundamentalMatRansac
{ 
	int modelPoints;
	int maxBasicSolutions;
	bool checkPartialSubsets;
	double threshold;
	double confidence;
	int maxIters;


public:
	FundamentalMatRansac(double param1,double param2)
	{
		threshold=param1;
		confidence=param2;
		modelPoints=8;
		checkPartialSubsets=true;
		maxIters=1000;

	}

	void computeError( InputArray _m1, InputArray _m2, InputArray _model, OutputArray _err) const
	{
		Mat __m1 = _m1.getMat(), __m2 = _m2.getMat(), __model = _model.getMat();
		int i, count = __m1.checkVector(2);
		const Point2d* m1 = __m1.ptr<Point2d>();
		const Point2d* m2 = __m2.ptr<Point2d>();
		const double* F = __model.ptr<double>();
		_err.create(count, 1, CV_64F);
		double* err = _err.getMat().ptr<double>();

		for( i = 0; i < count; i++ )
		{
			double a, b, c, d, e, f, g;

			a = F[0]*m1[i].x + F[1]*m1[i].y + F[2];
			b = F[3]*m1[i].x + F[4]*m1[i].y + F[5];
			c = F[6]*m1[i].x + F[7]*m1[i].y + F[8];


			d = m2[i].x*a + m2[i].y*b + c;

			e = F[0]*m2[i].x + F[3]*m2[i].y + F[6];
			f = F[1]*m2[i].x + F[4]*m2[i].y + F[7];
			g = F[2]*m2[i].x + F[5]*m2[i].y + F[8];


			err[i] = (double)abs(((d*d)/((a*a)+(b*b)+(e*e)+(f*f))));
		}
	}
	static bool haveCollinearPoints( const Mat& m, int count )
	{
		int j, k, i = count-1;
		const Point2d* ptr = m.ptr<Point2d>();

		// check that the i-th selected point does not belong
		// to a line connecting some previously selected points
		for( j = 0; j < i; j++ )
		{
			double dx1 = ptr[j].x - ptr[i].x;
			double dy1 = ptr[j].y - ptr[i].y;
			for( k = 0; k < j; k++ )
			{
				double dx2 = ptr[k].x - ptr[i].x;
				double dy2 = ptr[k].y - ptr[i].y;
				if( fabs(dx2*dy1 - dy2*dx1) <= FLT_EPSILON*(fabs(dx1) + fabs(dy1) + fabs(dx2) + fabs(dy2)))
					return true;
			}
		}
		return false;
	}


	int findInliers( const Mat& m1, const Mat& m2, const Mat& model, Mat& err, Mat& mask, double thresh ) const
	{
		computeError( m1, m2, model, err );
		mask.create(err.size(), CV_8U);

		CV_Assert( err.isContinuous() && err.type() == CV_64F && mask.isContinuous() && mask.type() == CV_8U);
		const double* errptr = err.ptr<double>();
		uchar* maskptr = mask.ptr<uchar>();
		double t = thresh;
		int i, n = (int)err.total(), nz = 0;
		for( i = 0; i < n; i++ )
		{
			int f = errptr[i] < t;
			maskptr[i] = (uchar)f;
			nz += f;
		}
		return nz;
	}
	bool checkSubset( InputArray _ms1, InputArray _ms2, int count ) const
	{
		Mat ms1 = _ms1.getMat(), ms2 = _ms2.getMat();
		return !haveCollinearPoints(ms1, count) && !haveCollinearPoints(ms2, count);
	}
	int find8Point( const Mat& _m1, const Mat& _m2, Mat& _fmatrix )const
	{

		Mat U1(8,8,CV_64F);
		Mat W( 9, 1, CV_64F);
		Mat V( 9, 9, CV_64F);
		Mat A( 8, 9, CV_64F);
		Mat U, TF,W1;

		Point2d m1c(0,0), m2c(0,0);
		double t, scale1 = 0, scale2 = 0;

		const Point2d* m1 = _m1.ptr<Point2d>();
		const Point2d* m2 = _m2.ptr<Point2d>();
		// double* fmatrix = _fmatrix.ptr<double>();
		CV_Assert( (_m1.cols == 1 || _m1.rows == 1) && _m1.size() == _m2.size());
		int i, count = _m1.checkVector(2);

		// compute centers and average distances for each of the two point sets
		for( i = 0; i < count; i++ )
		{
			double x = m1[i].x, y = m1[i].y;
			m1c.x += x; m1c.y += y;

			x = m2[i].x, y = m2[i].y;
			m2c.x += x; m2c.y += y;
		}

		// calculate the normalizing transformations for each of the point sets:
		// after the transformation each set will have the mass center at the coordinate origin
		// and the average distance from the origin will be ~sqrt(2).
		t = 1./count;
		m1c.x *= t; m1c.y *= t;
		m2c.x *= t; m2c.y *= t;

		for( i = 0; i < count; i++ )
		{
			double x = m1[i].x - m1c.x, y = m1[i].y - m1c.y;
			scale1 += std::sqrt(x*x + y*y);

			x = m2[i].x - m2c.x, y = m2[i].y - m2c.y;
			scale2 += std::sqrt(x*x + y*y);
		}

		scale1 *= t;
		scale2 *= t;

		if( scale1 < FLT_EPSILON || scale2 < FLT_EPSILON )
			return 0;

		scale1 = std::sqrt(2.)/scale1;
		scale2 = std::sqrt(2.)/scale2;

		A.setTo(Scalar::all(0));


		// form a linear system Ax=0: for each selected pair of points m1 & m2,
		// the row of A(=a) represents the coefficients of equation: (m2, 1)'*F*(m1, 1) = 0
		// to save computation time, we compute (At*A) instead of A and then solve (At*A)x=0.

		for( i = 0; i < count; i++ )
		{
			double x1,x2,y1,y2;

			x1 = (m1[i].x - m1c.x)*scale1;
			y1 = (m1[i].y - m1c.y)*scale1;
			x2 = (m2[i].x - m2c.x)*scale2;
			y2 = (m2[i].y - m2c.y)*scale2;

			A.at<double>(i,0)=x2*x1; A.at<double>(i,1)=x2*y1; A.at<double>(i,2)=x2; A.at<double>(i,3)=y2*x1;
			A.at<double>(i,4)= y2*y1; A.at<double>(i,5)=y2; A.at<double>(i,6)=x1; A.at<double>(i,7)=y1; A.at<double>(i,8)=1;

		}


		/* SVD::compute(A, W, U1, V,SVD::FULL_UV);*/
		int value=0;
		value=computeSVD( A, U1, W, V, 'A', 'A');


		double minval=100000;
		for (int i=0;i<W.rows;i++)
		{
			if (W.at<double>(i)<=minval)
				minval=W.at<double>(i);
			else
				return(-1);
		}


		Mat F0(3,3,CV_64F);
		F0.at<double>(0,0)=V.at<double>(8,0);F0.at<double>(0,1)=V.at<double>(8,1);F0.at<double>(0,2)=V.at<double>(8,2);
		F0.at<double>(1,0)=V.at<double>(8,3);F0.at<double>(1,1)=V.at<double>(8,4);F0.at<double>(1,2)=V.at<double>(8,5);
		F0.at<double>(2,0)=V.at<double>(8,6);F0.at<double>(2,1)=V.at<double>(8,7);F0.at<double>(2,2)=V.at<double>(8,8);

		/*SVDecomp( F0, W1, U, V, SVD::FULL_UV );*/
		Mat V1;
		Mat F0_1;

		F0.copyTo(F0_1);
		value=computeSVD( F0_1, U, W1, V1, 'A', 'A');

		W1.at<double>(2)=0;

		// F0 <- U*diag([W(1), W(2), 0])*V'
		gemm( U, Mat::diag(W1), 1., 0, 0., TF,0 );
		gemm( TF, V1, 1., 0, 0., F0, 0 );

		//denormalize the model
		double tt1[] = { scale1, 0, -scale1*m1c.x, 0, scale1, -scale1*m1c.y, 0, 0, 1 };
		double tt2[] = { scale2, 0, -scale2*m2c.x, 0, scale2, -scale2*m2c.y, 0, 0, 1 };
		Mat T1(3, 3, CV_64F, tt1), T2(3, 3, CV_64F, tt2);

		// F0 <- T2'*F0*T1
		gemm( T2, F0, 1., 0, 0., TF,GEMM_1_T );
		F0 = Mat(3, 3, CV_64F);
		gemm( TF, T1, 1., 0, 0., F0, 0 );

		_fmatrix=F0;
		// make F(3,3) = 1
		//  if( fabs(F0.at<double>(2,2)) > FLT_EPSILON )
		//F0 *= 1./F0.at<double>(2,2);

		return 1;
	}



	int RANSACUpdateNumIters( double p, double ep, int modelPoints, int maxIters )const
	{
		if( modelPoints <= 0 )
			// CV_Error( Error::StsOutOfRange, "the number of model points should be positive" );

			p = MAX(p, 0.);
		p = MIN(p, 1.);
		ep = MAX(ep, 0.);
		ep = MIN(ep, 1.);

		// avoid inf's & nan's
		double num = MAX(1. - p, DBL_MIN);
		double denom = 1. - std::pow(1. - ep, modelPoints);
		if( denom < DBL_MIN )
			return 0;

		num = std::log(num);
		denom = std::log(denom);

		return denom >= 0 || -num >= maxIters*(-denom) ? maxIters : cvRound(num/denom);
	}
	bool getSubset( const Mat& m1, const Mat& m2,
		Mat& ms1, Mat& ms2, RNG& rng,
		int maxAttempts=1000 ) const
	{
		AutoBuffer<int> _idx(modelPoints);
		int* idx = _idx;
		int i = 0, j, k, iters = 0;
		int esz1 = (int)m1.elemSize(), esz2 = (int)m2.elemSize();
		int d1 = m1.channels() > 1 ? m1.channels() : m1.cols;
		int d2 = m2.channels() > 1 ? m2.channels() : m2.cols;
		int count = m1.checkVector(d1), count2 = m2.checkVector(d2);
		const int *m1ptr = (const int*)m1.data, *m2ptr = (const int*)m2.data;

		ms1.create(modelPoints, 1, CV_MAKETYPE(m1.depth(), d1));
		ms2.create(modelPoints, 1, CV_MAKETYPE(m2.depth(), d2));

		int *ms1ptr = (int*)ms1.data, *ms2ptr = (int*)ms2.data;

		CV_Assert( count >= modelPoints && count == count2 );
		CV_Assert( (esz1 % sizeof(int)) == 0 && (esz2 % sizeof(int)) == 0 );
		esz1 /= sizeof(int);
		esz2 /= sizeof(int);

		for(; iters < maxAttempts; iters++)
		{
			for( i = 0; i < modelPoints && iters < maxAttempts; )
			{
				int idx_i = 0;
				for(;;)
				{
					idx_i = idx[i] = rng.uniform(0, count);

					for( j = 0; j < i; j++ )
						if( idx_i == idx[j] )
							break;
					if( j == i )
						break;
				}
				for( k = 0; k < esz1; k++ )
					ms1ptr[i*esz1 + k] = m1ptr[idx_i*esz1 + k];
				for( k = 0; k < esz2; k++ )
					ms2ptr[i*esz2 + k] = m2ptr[idx_i*esz2 + k];
				if( checkPartialSubsets && !checkSubset( ms1, ms2, i+1 ))
				{
					iters++;
					continue;
				}
				i++;
			}
			if( !checkPartialSubsets && i == modelPoints && !checkSubset(ms1, ms2, i))
				continue;
			break;
		}

		return i == modelPoints && iters < maxAttempts;
	}
	/*run RANSAC for computing best fit model*/
	bool run(InputArray _m1, InputArray _m2, OutputArray _model, OutputArray _mask,long int& rngstate) const
	{
		bool result = false;
		Mat m1 = _m1.getMat(), m2 = _m2.getMat();
		Mat err, mask, bestModel, ms1, ms2;
		Mat model=Mat::zeros(3,3,CV_64F);
		int iter, niters = MAX(maxIters, 1);
		int d1 = m1.channels() > 1 ? m1.channels() : m1.cols;
		int d2 = m2.channels() > 1 ? m2.channels() : m2.cols;
		int count = m1.checkVector(d1), count2 = m2.checkVector(d2), maxGoodCount = 0;

		RNG rng(rngstate);


		CV_Assert( confidence > 0 && confidence < 1 );

		CV_Assert( count >= 0 && count2 == count );
		if( count < modelPoints )
			return false;

		Mat bestMask0, bestMask;

		if( _mask.needed() )
		{
			_mask.create(count, 1, CV_8U, -1, true);
			bestMask0 = bestMask = _mask.getMat();
			CV_Assert( (bestMask.cols == 1 || bestMask.rows == 1) && (int)bestMask.total() == count );
		}
		else
		{
			bestMask.create(count, 1, CV_8U);
			bestMask0 = bestMask;
		}

		if( count == modelPoints )
		{
			if( find8Point(m1,m2,bestModel) <= 0 )
				return false;

			bestModel.copyTo(_model);
			bestMask.setTo(Scalar::all(1));
			Mat H;
			double reperr=0;
			//compute homography for selected 8 points
			H=findHomography(m1,m2,0);
			double temp;
			// int countnum = srcmat.checkVector(2);
			const Point2d* srcm = m1.ptr<Point2d>();
			const Point2d* dstm = m2.ptr<Point2d>();

			/*compute homography error*/
			for (int j1=0;j1<m1.rows;j1++)
			{ double x,y,z;

			temp=srcm[j1].x;

			temp=srcm[j1].y;

			x=(H.at<double>(0,0)*(double)srcm[j1].x)+(H.at<double>(0,1)*(double)srcm[j1].y)+ H.at<double>(0,2);
			y=(H.at<double>(1,0)*(double)srcm[j1].x)+(H.at<double>(1,1)*(double)srcm[j1].y)+ H.at<double>(1,2);
			z=(H.at<double>(2,0)*(double)srcm[j1].x)+(H.at<double>(2,1)*(double)srcm[j1].y)+ H.at<double>(2,2);
			reperr=reperr+(((double)(dstm[j1].x)-(x/z))*((double)(dstm[j1].x)-(x/z)))+(((double)(dstm[j1].y)-(y/z))*((double)(dstm[j1].y)-(y/z)));

			}

			Mat errin;
			double totalerr=0.0;
			//compute sampson distance for model
			computeError(m1,m2,bestModel,errin);
			for (int j1=0;j1<(int)errin.total();j1++)
			{
				totalerr=totalerr+errin.at<double>(j1);
			}

			temp=(reperr)/(double)totalerr;
			return true;
		}

		for( iter = 0; iter < niters; iter++ )
		{    Size modelSize;
		double H_error=0;
		int  goodCount, nmodels=1;
		while( H_error<2000)//ratio of homography error to model fitting error should be less than threshold
		{
			if( count > modelPoints )
			{
				bool found = getSubset( m1, m2, ms1, ms2, rng );
				rngstate=rng.state;
				if( !found )
				{
					if( iter == 0 )
						return false;
					break;
				}
			}
			//model fitting for random set of 8 correspondences
			nmodels = find8Point( ms1, ms2, model );
			//	bool skipupdate=false;
			if( nmodels <= 0 )
				continue;
			CV_Assert( model.rows % nmodels == 0 );
			modelSize=Size(model.cols, model.rows/nmodels);

			Mat H;
			double reperr=0;
			// computing homography 
			H=findHomography(ms1,ms2,0);

			// int countnum = srcmat.checkVector(2);
			const Point2d* srcm = m1.ptr<Point2d>();
			const Point2d* dstm = m2.ptr<Point2d>();
			double temp;
			//compute homography error
			for (int j1=0;j1<ms1.rows;j1++)
			{ double x,y,z;

			temp=srcm[j1].x;

			temp=srcm[j1].y;

			x=(H.at<double>(0,0)*(double)srcm[j1].x)+(H.at<double>(0,1)*(double)srcm[j1].y)+ H.at<double>(0,2);
			y=(H.at<double>(1,0)*(double)srcm[j1].x)+(H.at<double>(1,1)*(double)srcm[j1].y)+ H.at<double>(1,2);
			z=(H.at<double>(2,0)*(double)srcm[j1].x)+(H.at<double>(2,1)*(double)srcm[j1].y)+ H.at<double>(2,2);
			reperr=reperr+(((double)(dstm[j1].x)-(x/z))*((double)(dstm[j1].x)-(x/z)))+(((double)(dstm[j1].y)-(y/z))*((double)(dstm[j1].y)-(y/z)));

			}
			double totalerr=0.0;
			Mat errin;
			totalerr=0;
			Mat model_i = model;
			//compute model fitting error
			computeError(ms1,ms2,model_i,errin);
			for (int j1=0;j1<(int)errin.total();j1++)
			{
				totalerr=totalerr+errin.at<double>(j1);
			}
			H_error=reperr/(double)totalerr;
		}
		//check for best fit model
		if( nmodels<=0)
			continue;
		else
		{
			Mat model_i = model;
			goodCount = findInliers( m1, m2, model_i, err, mask, threshold );

			if( goodCount > MAX(maxGoodCount, modelPoints-1) )
			{
				std::swap(mask, bestMask);
				model_i.copyTo(bestModel);
				maxGoodCount = goodCount;
				niters = RANSACUpdateNumIters( confidence, (double)(count - goodCount)/count, modelPoints, niters );
			}
		}
		}

		if( maxGoodCount > 0 )
		{
			if( bestMask.data != bestMask0.data )
			{
				if( bestMask.size() == bestMask0.size() )
					bestMask.copyTo(bestMask0);
				else
					transpose(bestMask, bestMask0);
			}
			bestModel.copyTo(_model);
			result = true;
		}
		else
			_model.release();

		return result;
	}

};


/* wrapper function that computes fundamental matrix 
param1 - ransac error threshold
param2 - confidence 
_mask - output mask representing inliers */
Mat fundamentalmatrix( InputArray _points1, InputArray _points2,
	double param1, double param2,
	OutputArray _mask,long int& rngstate )
{
	Mat points1 = _points1.getMat(), points2 = _points2.getMat();
	Mat m1, m2, F;
	int npoints = -1;

	for( int i = 1; i <= 2; i++ )
	{
		Mat& p = i == 1 ? points1 : points2;
		Mat& m = i == 1 ? m1 : m2;
		npoints = p.checkVector(2, -1, false);

		p.reshape(2, npoints).convertTo(m, CV_64F);
	}

	CV_Assert( m1.checkVector(2) == m2.checkVector(2) );

	int result;
	FundamentalMatRansac Fmat(param1,param2);
	result =Fmat.run(m1, m2, F, _mask,rngstate);//compute fundamental matrix using 8point solution



	if( result <= 0 )
		return Mat();

	return F;
}
