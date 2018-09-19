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
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>

using namespace std;
using namespace cv;

#define MIDDLEVIEW 3
#define NVIEWS 8

// Debug Macros
#define DEBUG_LEVEL_LOCAL (0)
#define DEBUG_LEVEL_ERROR (1)
#define DEBUG_LEVEL_WARNING (2)
#define DEBUG_LEVEL_INFO (3)

// Uncomment following line to enable debug prints.
#define DEBUG (1)

// Set the DEBUG level. By default, set to display errors only.
#define DEBUG_LEVEL_CURRENT (DEBUG_LEVEL_ERROR)

#if DEBUG
#define DEBUG_PRINT(level, x) {                 \
   if (level <= DEBUG_LEVEL_CURRENT) {          \
   std::cout << #x << ":" << (x) << std::endl;  \
   }                                            \
}
#else
#define DEBUG_PRINT(level, x)
#endif

// Enable to display internmediate images
//#define DISPLAY_REPROJECTED
//#define DISPLAY_METRICx
//#define DISPLAY_KEYPOINTS

#define ROUND(X) ( ((X) + 0.5) >= (int(X) + 1) ? (int(X)+1) :  (int(X)) )

// Globals
extern vector<Mat> imgstack1;
extern bool isGroundTruth;

// Functions
/* Computes the fundamental matrix using input keypoints from an image pair */
extern      Mat getfundmat(vector<Point2d>& keypts1,vector<Point2d>& keypts2,vector<Point2d>& new1,vector<Point2d>& new2,Mat& indxmask,long int& rngstate);
/* Finds inlier points that fit the fundamental matrix model */
extern void getInliers(Mat& indxmask,vector<Point2d>& Keypts1,vector<Point2d>& Keypts2,vector<Point2d>& outkey1,
                       vector<Point2d>& outkey2);
/* Finds the common keypoints present in all views*/
extern void findcommon(int& minsize,int minindx,vector<vector<Point2d>>& newsrc,vector<vector<Point2d>>& newtgt,
                       vector<vector<Point2d>>& finpts,vector<vector<KeyPoint>>& finalpts,double size[]);
/* Generates 3D points from corresponding 2D points using triangulation */
extern void triangulatepts(vector<Vec4d>& X4D,vector<vector<Point2d>> finpts,vector<Mat> projm,double size[],int minsize);
/* Computes the projection matrix from the provided fundamental matrix */
extern      Mat getprojmatrix(Mat& F);
/* Reads a stack of images*/
extern void loadimages(vector<Mat>& imgptr,String entirepath);
/* Computes SIFT Keypoints and descriptors for an image */
extern void getKeypoints(vector<Mat>& imgptr,vector<vector<Point2d>>& keypts,vector<Mat>& desc);
/* Finds matching Keypoints for an image pair using descriptors*/
extern void extractandmatch(vector<Mat>& imgptr,vector<Mat>& desc,vector<vector<DMatch>>& goodmatches);
/* Computes normalized Keypoints */
extern void refinepoints(vector<vector<Point2d>>& keypts,vector<vector<DMatch>>& goodmatches,
   vector<vector<Point2d>>& targetkey,vector<vector<Point2d>>& sourcekey,double size[]);
/* Displays matches in multiple views*/
extern      Mat displaymatches(vector<Mat>& imgstack1,vector<vector<KeyPoint>>& finalpts);
/* Bundle Adjustment */
extern void SBA(vector<Mat>* P, vector<Vec4d>* X, vector<vector<Point2d>>* x, 
                vector<Mat>* P_BA, vector<Vec4d>* X_BA, unsigned int nviews);
/* Reprojection error calculation */
extern double CalcReProjErr(vector<Mat>* P_BA, vector<Vec4d>* X_BA, vector<vector<Point2d>>* x, unsigned int nviews);
/* Decompose the Projection Matrix - using LAPACK instead of OpenCV */
void decomposeProjection(const Mat& P, Mat& K, Mat& R, Mat& C);
/* Metric Upgrade */
extern int MetricUpgrade(vector<Mat>* P_BA, Mat* QFinal);
/* Calculate Homogeneous matrix H from quadric Q */
extern int FindHfromQ(Mat* QFinal, Mat& H);
/* Calculates the ground truth data and then displays the results */
extern void displayDepth(vector<Mat>& X_Final, vector<vector<Vec3d>>& x_Match);
/* Reads in the file with 2D points and corresponding 3D points. */
extern void parsePoints(vector<Mat>& X_Final, vector<vector<Vec3d>>& x_Match, string& filename);