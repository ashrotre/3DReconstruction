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



/* find common points among all 8 views
finpts - normalized common points. finpts is a vector of size 8xN, where N is the number of common keypoints.
Each column of finpts contains the co-ordinates of an identified common point in all 8 views.
finalpts - denormalized common points. finalpts is a vector of size 8xN, where N is the number of common keypoints.
Each column of finalpts contains the co-ordinates of an identified common point in all 8 views.*/
void findcommon(int& minsize,int minindx,vector<vector<Point2d>>& newsrc,vector<vector<Point2d>>& newtgt,
    vector<vector<Point2d>>& finpts,vector<vector<KeyPoint>>& finalpts,double size[])
{
    Mat common=Mat::ones(minsize,8,CV_32S)*(-1);
    vector<Point2d> frame;
	cout<<"\n"<<"Finding common points..."<<"\n";
    int numberofimages=8;
    int m,counter=0;
    bool flag=true;
    frame=newsrc.at(minindx);

    int i=0;
    int j=0;
    int k=0;
	/* Computing the common point indices through comparison */
    for (j=0;j<numberofimages;j++)
    {
        vector<Point2d> tlist;
        if (j!=3)
        {
            counter=0;
            for(i=0;i<minsize;i++)
            {  
                flag=true;

                for(k=0;k<newsrc.at(j).size();k++)
                {
                    if (  (frame.at(i).x==newsrc.at(j).at(k).x)&&( frame.at(i).y==newsrc.at(j).at(k).y  ))
                    {
                        tlist.push_back(frame.at(i));


                        if (counter!=i)
                        {
                            for(m=0;m<numberofimages;m++)
                            {
                                common.at<int>(counter,m)=common.at<int>(i,m);
                                common.at<int>(i,m)=-1;
                            }
                        }
                        common.at<int>(counter,j)=k;
                        flag=false;
                        counter+=1;
                        break;

                    }

                }
                if ( flag==true)
                {
                    for(m=0;m<numberofimages;m++)
                        common.at<int>(i,m)=-1;
                }

            }
            frame=tlist;

            tlist.~vector();

            minsize=(int)frame.size();
        }

    }
    vector<vector<KeyPoint>> finalpts_1(8,minsize);// store keypoints for display
    vector<vector<Point2d>> finpts_1(8,minsize);
    /*int height=684,width=912;*/
   double kmat[3][3]={{1020,0,0.5*size[1]},{0,1020,0.5*size[0]},{0,0,1}};
    Mat K(3,3,CV_64F,kmat);
  /* Storing the common points*/
    for (i=0;i<numberofimages;i++)
    {   if (i!=3)
    {  
        for(j=0;j<minsize;j++)
        {
            if (common.at<int>(j,i)!=-1)
            {
                Mat npts(3,1,CV_64F);
                npts.at<double>(0)=newtgt.at(i).at(common.at<int>(j,i)).x;
                npts.at<double>(1)=newtgt.at(i).at(common.at<int>(j,i)).y;
                npts.at<double>(2)=1;
                finpts_1.at(i).at(j).x=newtgt.at(i).at(common.at<int>(j,i)).x;
                finpts_1.at(i).at(j).y=newtgt.at(i).at(common.at<int>(j,i)).y;

                npts=K*npts;
                finalpts_1.at(i).at(j).pt.x=npts.at<double>(0);
                finalpts_1.at(i).at(j).pt.y=npts.at<double>(1);

            }

        }
    }
    else
    {
        for (j=0;j<minsize;j++)
        {
            Mat npts(3,1,CV_64F);
            npts.at<double>(0)=frame.at(j).x;
            npts.at<double>(1)=frame.at(j).y;
            npts.at<double>(2)=1;

            finpts_1.at(i).at(j).x=frame.at(j).x;
            finpts_1.at(i).at(j).y=frame.at(j).y;
            npts=K*npts;
            finalpts_1.at(i).at(j).pt.x=npts.at<double>(0);
            finalpts_1.at(i).at(j).pt.y=npts.at<double>(1);
        }
    }
    }
    finpts=finpts_1;
    finalpts=finalpts_1;


}