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
#include "windows.h"
#include <opencv2\features2d\features2d.hpp>
#include <opencv2\calib3d\calib3d.hpp>
//#define WRITE_IMAGE 1
using namespace std;
using namespace cv;

vector<Mat> imgstack1 (NVIEWS);
string PointsFile = "output/Points.txt";
bool isGroundTruth = true;    // is initialized to true, change otherwise
bool DisplayPtsInFile = false;

/*main function for code*/
int main(int argc, char* argv[])
{   
	String path1;
	bool useprovidedoutpath=false;
	bool usegroundtruth=false;
	if (argc<2)
	{
		cout<<"Invalid number of arguments. Atleast 1 argument expected\n";
		cout<<"Expected argument sequence:\n-i completefilename  -o outputfolderpath  -g groundtruthfilepath \n";
		cout<<"Optional arguments: -o outputfolderpath -g groundtruthfilepath  \n";
		char c=0;
		while(c!='27')
		{ 
			c=waitKey();
		}

	}
	else
	{   
		char* temp;
		char* filename;
		string filepath,outputpath,groundtruthpath;
		String entirepath;
		int i=1;
		while(i<argc)
		{
			temp=argv[i];
			if (strcmp("-i",temp)==0)
			{
				i++;
				filename=argv[i];
				char* ptrindx=strrchr(filename,'.');
				string filename1=filename;
				char* filepath1=new char[ptrindx-filename];
				int length=filename1.copy(filepath1,ptrindx-filename-1,0);
				filepath1[length]='\0';
				filepath=filepath1;
				//cout<<"\n"<<filepath;	
				entirepath=filepath;
			}
			else if (strcmp("-o",temp)==0)
			{
				i++;
				outputpath=argv[i];
				
				//cout<<"\n"<<outputpath;
				useprovidedoutpath=true;
				std::wstring stemp = std::wstring(outputpath.begin(), outputpath.end());
                 LPCWSTR sw = stemp.c_str();
				 CreateDirectory(sw,NULL);     
			}
			else if (strcmp("-g",temp)==0)
			{
				i++;
				groundtruthpath=argv[i];
				//cout<<"\n"<<groundtruthpath;
				usegroundtruth=true;
			}
			i++;
		}
   double ReProjErr = 0;
   double MaxReProjErr = 1;

   int numberofimages = NVIEWS;

   vector<vector<Point2d>> keypoints(numberofimages);//stores keypoints for each image

   vector<Mat> desc(numberofimages);
   /*loads 8 images from memory and stores in vector imgstack1*/
   loadimages(imgstack1,entirepath);
   double size[2]={imgstack1.at(0).rows,imgstack1.at(0).cols};//stores height and width of image

   DEBUG_PRINT(DEBUG_LEVEL_INFO, imgstack1[0].cols);
   DEBUG_PRINT(DEBUG_LEVEL_INFO, imgstack1[0].rows);

   /* Normalization matrix */
   Mat K = (Mat_<double>(3,3) << 
               1020, 0, 0.5*imgstack1[0].cols,
               0, 1020, 0.5*imgstack1[0].rows,
               0, 0, 1); 
   DEBUG_PRINT(DEBUG_LEVEL_INFO, K);

   /*detects keypoints and computes corresponding SIFT descriptors for all images*/
   /*Using vlfeat package found at http://www.vlfeat.org/ for sift Keypoint detection and descriptor computation*/
   getKeypoints(imgstack1,keypoints,desc);

   vector<vector<DMatch>> goodmatches(numberofimages);//stores keypoint matching indices

   /*performs keypoint matching for each possible pair of images in imgstack1*/
   extractandmatch(imgstack1,desc,goodmatches);

   vector<vector<Point2d>> sourcekey(numberofimages),targetkey(numberofimages),
      newsrc(numberofimages),newtgt(numberofimages)/*,n1(numberofimages),n2(numberofimages)*/;//stores keypoints for traingulation and plotting

   /* normalizes matched keypoints*/
   refinepoints(keypoints,goodmatches,targetkey,sourcekey,size);

   double min_max_depth = 0.0; //product of max and min depths
   vector<Mat> X_Final; // Stores the final X points whose depth is displayed in the final image
   vector<vector<Vec3d>> x_Match; // Stores the x calculated as reprojection after metric upgrade

   do 
   {
      // Clear the vectors
      X_Final.clear();
      x_Match.clear();

      // Output of Metric Upgrade
      vector<Mat> P_Metric;
      vector<Vec4d> X_Metric;

      // Ouptut from SBA
      vector<Mat> P_BA;
      vector<Vec4d> X_BA;

      bool RandStateReset = false;
      long int rngstate=((uint64)-1);
      do
      {
         vector<Mat> indxmask(numberofimages);
         /*8x3x3 vector of fundamental matrices*/
         vector<Mat>  Fundm(numberofimages);
         /*8x3x4 vector of projection matrices*/
         vector<Mat>  projm(numberofimages);
         //vector<vector<Point2f>> ntemp1(8),ntemp2(8);

         P_BA.clear();
         X_BA.clear();

         // DO NOT Reset the random state for next iteration
         RandStateReset = false;

         Mat tempmat,temp;
         int j,minindx, minsize=7000;

         for( j=0;j<numberofimages;j++)
         {
            if(j!=3)
            {  
               //long int rngstate=((uint64)-1);
               /*if (RandStateReset)
               {
               long int rngstate=((uint64)-1);
               RandStateReset = true;
               }*/
				/* computes fundamental matrix between each image pair*/
               Fundm.at(j) =getfundmat(sourcekey.at(j),targetkey.at(j),newsrc.at(j),newtgt.at(j),indxmask.at(j),rngstate);
               //cout << "F[" << j << "]:\n" << Fundm[j] << endl; 
               //cout<<"\n"<<Fundm.at(j);

               /*computes projectionmatrix for each image pair */
               projm.at(j)=getprojmatrix(Fundm.at(j));
			   /*computes minimum number of keypoints and the corresponding frame index for a stack of images*/
               if (minsize>=newsrc.at(j).size())
               {
                  minsize=newsrc.at(j).size();//number of keypoints
                  minindx=j;//frame index
               }
               
            }
            RandStateReset = true;

         }

         /*projection matrix for central frame*/
         projm.at(3)=Mat::eye(3,4,CV_64F);

         /*8xN vector that stores denormalized  2D co-ordinates of common points. Used for displaying points in image.
         Co-ordinates are not in homogeneous form*/
         vector<vector<KeyPoint>> finalpts;

         /*8xN vector that stores normalized 2D co-ordinates of common points. Used for triangulation and bundle adjustment
         co-ordinates are not in homogeneous form*/
         vector<vector<Point2d>> finpts;

         /* finds common points in all views and stores them in finpts and finalpts for further use*/
         findcommon(minsize,minindx,newsrc,newtgt,finpts,finalpts,size);

         /* Nx4 vector for storing homogeneous 3D co-ordinates obtained after triangulation*/
         vector<Vec4d> X4D;

         /*performs triangulation for all points to generate a Nx4 vector of homogeneous 3D points*/
         triangulatepts(X4D,finpts,projm,size,minsize);
         DEBUG_PRINT(DEBUG_LEVEL_INFO, minsize);
        /*Write images to file*/
#ifdef WRITE_IMAGE
		if (useprovidedoutpath==true)
			path1=outputpath;
		 else
			 path1="output/table_image_set";
		 for (int i=0; i < numberofimages; i++)
		 {  
			 Mat finalimage;
			 drawKeypoints(imgstack1.at(i),finalpts.at(i),finalimage,cv::Scalar(0,0,512));  
			 ostringstream indx; 
			 indx<<i;
			 String path=path1+"/keypoint"+indx.str()+".jpg";
			 imwrite(path,finalimage);
			 finalimage.release();
		 }
		 Mat dispmatches1;
		 dispmatches1=displaymatches(imgstack1,finalpts);
		 imwrite(path1+"/matches.jpg",dispmatches1);
		 dispmatches1.release();
#endif
   
         //displays the common points in all views
#ifdef DISPLAY_KEYPOINTS
         namedWindow("Keypoints output",0);
         for (int i=0; i < numberofimages; i++)
         {  
            Mat finalimage;
            drawKeypoints(imgstack1.at(i),finalpts.at(i),finalimage,cv::Scalar(0,0,512));
            imshow("Keypoints output",finalimage);
            waitKey();
            finalimage.release();
         }
		 Mat dispmatches;
		 dispmatches=displaymatches(imgstack1,finalpts);
		 namedWindow("Matches",0);
		 imshow("Matches",dispmatches);
         waitKey();
		 dispmatches.release();
#endif

         /* Bundle adjustment */
   //write the outputs to a files for debugging
   //ofstream InputP("dumpPIn.txt");
   //for (int view=0; view < numberofimages; view++)
   //   InputP << "P[" << view << "]:\n" << projm[view] << "\n";
   //InputP.close();

   //ofstream InputX("dumpXIn.txt");
   //for (unsigned int pts=0; pts < X4D.size(); pts++)
   //   InputX << "X[" << pts << "]:\n" << X4D[pts] << "\n";
   //InputX.close();

         /* Calculate the Adjusted projeciton matrices and 3D points. 
         Using the sba package found at http://users.ics.forth.gr/~lourakis/sba/ ver 1.5*/
         SBA(&projm, &X4D, &finpts, &P_BA, &X_BA, numberofimages);

         // Calculate the reprojection error to make sure the Projection Matrices are acceptable
         ReProjErr = CalcReProjErr(&P_BA, &X_BA, &finpts, numberofimages);
         DEBUG_PRINT(DEBUG_LEVEL_LOCAL, ReProjErr);

         // Loop here in case of errors
      } while (ReProjErr > MaxReProjErr);

      ofstream PBAfile("PBA.txt");
      if ( !PBAfile.is_open() )
      {
         cerr << "Could not open file MetricUpgrade.gms" << endl;
         return -1;
      }
      for ( int view = 0; view < NVIEWS; view++ )
      {
         PBAfile << P_BA[view] << endl;
      }
      PBAfile.close();

      /* Metric Upgrade
      Uses SymbolicC++ libraries
      Uses C++ version of the library http://www.is.titech.ac.jp/~kojima/SparsePOP/ */
      Mat Q = Mat(4, 4, CV_64FC1);
      int result = MetricUpgrade(&P_BA, &Q);
      if ( !result )
      {
         DEBUG_PRINT(DEBUG_LEVEL_ERROR, "MetricUpgrade.cpp returned error.");
      }

      cout << Q << endl;

      // Calculate H from Q*
      Mat H = Mat::zeros(4, 4, CV_64FC1);
      result = FindHfromQ(&Q, H);
      DEBUG_PRINT(DEBUG_LEVEL_INFO, H);
      Mat H_inv = H.inv();

      cout << H_inv << endl;

      /*
      * Find Metric X, P and x
      */
      // P Metric Calculation
      for (int view = 0; view < numberofimages; view++)
      {
         P_Metric.push_back(P_BA[view] * H);
      }
      // X Metric Calculation
      for (unsigned int pt = 0; pt < X_BA.size(); pt++)
      {
         Mat X = (Mat_<double>(4,1) << 
            X_BA[pt][0], X_BA[pt][1], X_BA[pt][2], X_BA[pt][3]);

         Mat X_tmp = H_inv * X;
         // Normalize
         X_tmp /= X_tmp.at<double>(3,0);
         // Store the X_Metric
         X_Metric.push_back(X_tmp);
      }
      // x Metric Calculation
      for (int view = 0; view < numberofimages; view++)
      {
         vector<Vec3d> x_view;
         for (unsigned int pt = 0; pt < X_BA.size(); pt++)
         {
            // Find the metric x
            Mat X = (Mat_<double>(4,1) << 
               X_Metric[pt][0], X_Metric[pt][1], X_Metric[pt][2], X_Metric[pt][3]);

            Mat x_tmp = P_Metric[view] * X;
            x_tmp = K * x_tmp;
            x_tmp /= x_tmp.at<double>(2,0);
            x_view.push_back(Vec3d(x_tmp.at<double>(0,0), 
               x_tmp.at<double>(1,0),
               1));
         }
         x_Match.push_back(x_view);
      }

#ifdef DISPLAY_METRICx
      /* Display the x cor-ordinates in metric form */
      namedWindow("Metric x", CV_WINDOW_AUTOSIZE);
      vector<Mat> imgBGR(numberofimages);
      for (int view = 0; view < numberofimages; view++)
      {
         cvtColor(imgstack1[view], imgBGR[view], CV_GRAY2BGR);
         for (unsigned int pts = 0; pts < X_Metric.size(); pts++)
         {
            Point2d x_tmp = Point2d(x_Match[view][pts][0], x_Match[view][pts][1]);
            circle(imgBGR[view], x_tmp, 5, Scalar(0,0,255), 1, 8, 0);
         }
         imshow("Metric x", imgBGR[view]);
         waitKey();
      }
#endif

      /* 
      * Transform the local coordinates
      */
      // find the K, R and t from the Middle view Projection matrix
      Mat K_localCord, R_localCord, C_localCord, T_localCord;

      decomposeProjection(P_Metric[MIDDLEVIEW], K_localCord, R_localCord, C_localCord);
      T_localCord = -1 * R_localCord * C_localCord;
      DEBUG_PRINT(DEBUG_LEVEL_INFO, K_localCord);
      DEBUG_PRINT(DEBUG_LEVEL_INFO, R_localCord);
      DEBUG_PRINT(DEBUG_LEVEL_INFO, T_localCord);

      // Transformation matrix 4x4 matrix
      Mat T_Middle;
      hconcat(R_localCord, T_localCord, T_Middle);
      Mat vec = (Mat_<double>(1,4) << 0, 0, 0, 1); 
      vconcat(T_Middle, vec, T_Middle);
      DEBUG_PRINT(DEBUG_LEVEL_INFO, T_Middle);

      // Transform the 3D co-ordinates to the local co-ordinates using the transformation matrix
      for (unsigned int pts = 0 ; pts < X_Metric.size(); pts++)
      {
         Mat X = (Mat_<double>(4,1) << 
            X_Metric[pts][0], X_Metric[pts][1], X_Metric[pts][2], X_Metric[pts][3]);
         DEBUG_PRINT(DEBUG_LEVEL_INFO, X);

         Mat X_f = T_Middle * X;
         DEBUG_PRINT(DEBUG_LEVEL_INFO, X_f);
         X_Final.push_back (X_f);
      }

      // Check the depth of the 3D points if the signs match
      double min_X_Final = 9999;
      double max_X_Final = -9999;
      for (unsigned int pts = 0 ; pts < X_Metric.size(); pts++)
      {
         if (X_Final[pts].at<double>(2, 0) > max_X_Final)
            max_X_Final = X_Final[pts].at<double>(2, 0);
         if (X_Final[pts].at<double>(2, 0) < min_X_Final)
            min_X_Final = X_Final[pts].at<double>(2, 0);
      }
      min_max_depth = min_X_Final * max_X_Final;

      // if the max depth is -ve then invert the sign 
      if (min_max_depth > 0 && max_X_Final < 0 )
      {
         for (unsigned int pts = 0 ; pts < X_Metric.size(); pts++)
         {
            X_Final[pts].at<double>(2, 0) = -1 * X_Final[pts].at<double>(2, 0);
         }
      }
      // Dump the values for Error plot
      ofstream XFinalFileM("dumpXFinalM.txt");
      for (int pts=0; pts < X_Final.size(); pts++)
      {
         for (int i = 0; i < 4; i++)
         {
            XFinalFileM << X_Final[pts].at<double>(i) << " ";
         }
         XFinalFileM << "\n";
      }
      XFinalFileM.close();

   // Loop here if necessary for depth value consistency 
   // i.e if the min depth and max depth do not have the same values then loop
   } while(min_max_depth < 0);

   // Dump the 2D points and 3D points for display purposes to the given file path
   ofstream XDump(PointsFile);
   XDump << "x\t" << "y\t" << "X\t" << "Y\t" << "Z" << "\n";
   for (int pts=0; pts < X_Final.size(); pts++)
   {
      XDump << x_Match[MIDDLEVIEW][pts][0] << "\t" << x_Match[MIDDLEVIEW][pts][1] << "\t"
            << X_Final[pts].at<double>(0) << "\t" << X_Final[pts].at<double>(1) << "\t" << X_Final[pts].at<double>(2) << "\n";
   }
   XDump.close();

      //   Mat DepthImg;
      //cout<<"----------------------------------------------------------------------"<<"\n"<<"\n";
      //cvtColor(imgstack1[MIDDLEVIEW], DepthImg, CV_GRAY2BGR);    
      //for (unsigned int pts = 0 ; pts < X_Final.size(); pts++) // plot every 2nd pt
      //{
      //   Point2d x_tmp = Point2d(x_Match[MIDDLEVIEW][pts][0], x_Match[MIDDLEVIEW][pts][1]);
      //   circle(DepthImg, x_tmp, 2, Scalar(0,0,255), -1, 8, 0);
      //   ostringstream depth;
      //   depth << ROUND(X_Final[pts].at<double>(2, 0)*4);

      //   Point2d pos; 
      //   pos.x = x_tmp.x + 2; pos.y = x_tmp.y;
      //   putText(DepthImg, depth.str(), pos, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,255,0), 1, 8);
      //}
   
      // /*Write images to file*/
      //#ifdef WRITE_IMAGE
      //   imwrite("output/table_image_set/depthimage.jpg",DepthImg);
      //#endif
   
      //namedWindow("Final Depths", CV_WINDOW_AUTOSIZE);
      //imshow("Final Depths", DepthImg);
      //waitKey();


   if (isGroundTruth)
   {
     /* 
      * Display the depth values on the middle view image
      */
      if (DisplayPtsInFile)
      {
         parsePoints(X_Final, x_Match, PointsFile);
      }
      displayDepth(X_Final, x_Match);
   }

   if (!isGroundTruth)
   {
      /* 
      * Scale the values 
      */
      Mat depth = Mat(1, (int)X_Final.size(), CV_64FC1);
      for (int pts = 0; pts < X_Final.size(); pts++)
      {
         depth.at<double>(0, pts) = X_Final[pts].at<double>(2,0); // get the depth
      }
      Scalar meanDepth = mean(depth);

      Mat ZeroMeanDepth = depth - meanDepth[0];
      double positiveMean = 0.0; unsigned int positiveVals = 0;
      double negativeMean = 0.0; unsigned int negativeVals = 0;
      for (int col = 0; col < ZeroMeanDepth.cols; col++)
      {
         if (ZeroMeanDepth.at<double>(0,col) > 0)
         {
            positiveMean+=ZeroMeanDepth.at<double>(0,col);
            positiveVals++;
         }
         if (ZeroMeanDepth.at<double>(0,col) < 0)
         {
            negativeMean+=ZeroMeanDepth.at<double>(0,col);
            negativeVals++;
         }
      }
      positiveMean/=positiveVals;
      negativeMean/=negativeVals;

      double scale = 200 / (positiveMean-negativeMean);
      for (int pts = 0; pts < X_Final.size(); pts++)
      {
         X_Final[pts].at<double>(2,0) = ZeroMeanDepth.at<double>(0, pts) * scale + 200; // get the depth
      }

      /* 
      * Display the depth values on the middle view image
      */
      Mat DepthImg;
      cout<<"----------------------------------------------------------------------"<<"\n"<<"\n";
      cvtColor(imgstack1[MIDDLEVIEW], DepthImg, CV_GRAY2BGR);    
      for (unsigned int pts = 0 ; pts < X_Final.size(); pts++) // plot every 2nd pt
      {
         Point2d x_tmp = Point2d(x_Match[MIDDLEVIEW][pts][0], x_Match[MIDDLEVIEW][pts][1]);
         circle(DepthImg, x_tmp, 2, Scalar(0,0,255), -1, 8, 0);
         ostringstream depth;
         depth << ROUND(X_Final[pts].at<double>(2, 0));

         Point2d pos; 
         pos.x = x_tmp.x + 2; pos.y = x_tmp.y;
         putText(DepthImg, depth.str(), pos, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,255,0), 1, 8);
      }
   
       /*Write images to file*/
      #ifdef WRITE_IMAGE
        // imwrite("output/table_image_set/depthimage.jpg",DepthImg);
	  if (useprovidedoutpath==true)
			path1=outputpath;
		 else
			 path1="output/table_image_set";
		 imwrite(path1+"/depthimage.jpg",DepthImg);
      #endif
   
      namedWindow("Final Depths", CV_WINDOW_AUTOSIZE);
      imshow("Final Depths", DepthImg);
   }

   waitKey();
   destroyAllWindows();
}
}