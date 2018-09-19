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

/*refines detected keypoints by choosing matches lesser than an error threshold and normalizes them multiplying with Kinvs*/
void refinepoints(vector<vector<Point2d>>& keypts,vector<vector<DMatch>>& goodmatches,
    vector<vector<Point2d>>& targetkey,vector<vector<Point2d>>& sourcekey,double size[])
{    
    int numberofimages=8;
    //double k[3][3]={{0.0010,0,-0.4471},{0,0.0010,-0.3353},{0,0,1}};

    /*int height=684,width=912;*/
    double k[3][3]={{1020,0,(double)0.5*size[1]},{0,1020,(double)0.5*size[0]},{0,0,1}};
    Mat K(3,3,CV_64F,k);
   /* cout<<K.inv();*/
	cout<<"Normalizing Keypoints..."<<"\n";
    /*normalizing co-ordinates by multiplying with Kinv */
    for( int j=0;j<numberofimages;j++)
    {
        if (j!=3)
        {
            for( int i=0;i<goodmatches.at(j).size();i++)
            {	
                Point2d temp;
                Mat t(3,1,CV_64F);
                t.at<double>(0)=keypts.at(3).at(goodmatches.at(j).at(i).queryIdx).x;
                t.at<double>(1)=keypts.at(3).at(goodmatches.at(j).at(i).queryIdx).y;
                t.at<double>(2)=1;
				/*Normalizing points*/
                t=K.inv()*t;
                temp.x=t.at<double>(0);
                temp.y=t.at<double>(1);
                sourcekey.at(j).push_back(temp);
                t.at<double>(0)=keypts.at(j).at(goodmatches.at(j).at(i).trainIdx).x;
                t.at<double>(1)=keypts.at(j).at(goodmatches.at(j).at(i).trainIdx).y;
                t.at<double>(2)=1;
				/*Normalizing points*/
                t=K.inv()*t;
                temp.x=t.at<double>(0);
                temp.y=t.at<double>(1);
                targetkey.at(j).push_back(temp);
            }

        }
    }
}