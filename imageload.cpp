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

#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\highgui\highgui.hpp>
#include "Common.h"

using namespace std;
using namespace cv;

/*loads a stack of 8 images, downsamples by 4 and converts to grayscale. imgptr is an output vector for storing stack*/
void loadimages(vector<Mat>& imgptr,String entirepath)
{
   /* path for input images,if path is different from the one mentioned, change entirepath to the folder containing images*/
   //String entirepath="input/table_image_set/table5_";
	
   ostringstream indx;    
   int numimg=8;
   cout<<"Reading input image files ...."<<"\n";
   for(int i=0;i<numimg;i++)
   {

      indx<<i+1;

      String indtxt=indx.str();

      String fullpath=entirepath+indtxt[i]+".jpg";

      Mat  temp;
      temp = imread(fullpath);
      if (temp.data == NULL)
      {
         cout << "Failed to read image file: " << fullpath << endl;
         system("pause");
         exit(-1);
      }
	  /*convert 3 channel to single channel*/
      cvtColor(temp,imgptr.at(i),CV_BGR2GRAY);
	  /*downsample by a factor of 4 */
      resize(imgptr.at(i), imgptr.at(i), Size( temp.cols/4, temp.rows/4), 0.25, 0.25, INTER_LINEAR);
      temp.release();
   }
}