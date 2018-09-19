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

using namespace cv;
using namespace std;
/* Displays matches in multiple views*/
Mat displaymatches(vector<Mat>& imgstack1,vector<vector<KeyPoint>>& finalpts)

{
	double size[2]={imgstack1.at(0).rows,imgstack1.at(0).cols};// stores image height and width
	Mat newimage,newimage2;
	hconcat(imgstack1.at(0),imgstack1.at(1),newimage);
	for (int num=2;num<5;num++)
		hconcat(newimage,imgstack1.at(num),newimage);
	hconcat(Mat::zeros(size[0],size[1]/2,CV_8UC1),imgstack1.at(4),newimage2);
	for (int num=5;num<=7;num++)
		hconcat(newimage2,imgstack1.at(num),newimage2);
	hconcat(newimage2,Mat::zeros(size[0],size[1]/2,CV_8UC1),newimage2);
	vconcat(newimage,newimage2,newimage2);
	vector<vector<Point2f>> displaypoints(9);
	Mat newimage1;
	cvtColor(newimage2,newimage1,CV_GRAY2RGB);
	for(int p2=0;p2<=4;p2++)
	{
		for (int p1=0;p1<20;p1++)
		{
			Point2f tempt;
			tempt.x=finalpts.at(p2).at(p1*4).pt.x+(p2*size[1]);
			tempt.y=finalpts.at(p2).at(p1*4).pt.y;
			displaypoints.at(p2).push_back(tempt);
		}
	}
	for(int p2=4;p2<=7;p2++)
	{
		for (int p1=0;p1<20;p1++)
		{
			Point2f tempt;
			tempt.x=finalpts.at(p2).at(p1*4).pt.x+((p2-4)*size[1])+(size[1]/2);
			tempt.y=finalpts.at(p2).at(p1*4).pt.y+size[0];
			displaypoints.at(p2+1).push_back(tempt);
		}
	}
	for (int p2=0;p2<=3;p2++)
	{
		for(int p1=0;p1<displaypoints.at(0).size();p1++)
		{
			line(newimage1,displaypoints.at(p2).at(p1),displaypoints.at(p2+1).at(p1),Scalar(0,0,255),1,CV_AA);
		}
	}
	for (int p2=5;p2<=7;p2++)
	{
		for(int p1=0;p1<displaypoints.at(0).size();p1++)
		{
			line(newimage1,displaypoints.at(p2).at(p1),displaypoints.at(p2+1).at(p1),Scalar(0,0,255),1,CV_AA);
		}
	}
	newimage2.~Mat();
	newimage.~Mat();
	return(newimage1);
}