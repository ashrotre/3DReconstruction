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

// MetricUpgrade.cpp : Uses the sparsePOP (http://www.is.titech.ac.jp/~kojima/SparsePOP/) for metric upgrade.

#include <Windows.h>
#include <stdio.h>
#include <fstream>
#include <time.h>
#include "Common.h"

using namespace std;
using namespace cv;

extern string equation;
extern int equationReduce(vector<Mat>* P, string &equation, string& reducedEquation);

int MetricUpgrade(vector<Mat>* P_BA, Mat* QFinal)
{
   clock_t start_time, end_time;    // For timing the Metric Upgrade operation
   start_time = clock();

   // Reduce the equation
   string funcF;
   equationReduce(P_BA, equation, funcF);

   ofstream eqfile("RedEquation.txt");
   if ( !eqfile.is_open() )
   {
      cerr << "Could not open file MetricUpgrade.gms" << endl;
      return -1;
   }
   eqfile << funcF << endl;
   eqfile.close();

   // replace (2) by 2
   while (1)
   {
      size_t pos = funcF.find("(2)");
      if (pos != string::npos)
      {
         funcF.replace(pos, strlen("(2)"), "2");
         pos += strlen("(2)");
      }
      else
      {
         break;
      }
   }

   // Read the reference GMS file
   ifstream refFile ("MetricUpgradeRef.gms");
   if ( !refFile.is_open() )
   {
      cerr << "Could not the Reference gms file." << endl;
      return -1;
   }
   string fileText;
   refFile.seekg(0, ios::end);
   fileText.resize(refFile.tellg());
   refFile.seekg(0, ios::beg);
   refFile.read(&fileText[0], fileText.size());
   refFile.close();

   // Write to the GMS file and append the objective function
   ofstream gmsFile("MetricUpgrade.gms");
   if ( !gmsFile.is_open() )
   {
      cerr << "Could not open file MetricUpgrade.gms" << endl;
      return -1;
   }
   gmsFile << fileText;
   gmsFile.seekp(0, ios::end);
   gmsFile << "\n\ne1..    " << endl;
   gmsFile << funcF;
   gmsFile << "- objvar =E= 0;" << endl;
   gmsFile.close();

   //cout << "Created the GMS file.\n";

   //Create a process for sparsePOP
   STARTUPINFO si;
   PROCESS_INFORMATION pi;

   ZeroMemory( &si, sizeof(si) );
   si.cb = sizeof(si);
   ZeroMemory( &pi, sizeof(pi) );

   if( remove( "MetricUpgradeOut.txt" ) != 0 )
    perror( "Error deleting file" );

   cout << "Creating Process for sparsePOP.\n";
   if ( !CreateProcess( TEXT("sparsePOP.exe"), 
                        TEXT("sparsePOP.exe MetricUpgrade.gms"), 
                        NULL, NULL, FALSE, 0, 
                        NULL, NULL, &si, &pi)
      )
   {
      cerr << "Failed to Create a Process.\n";
      return -1;
   }

   // Wait for the process to complete and exit. 
   WaitForSingleObject( pi.hProcess, INFINITE );

   // Close the process and thread handles.
   CloseHandle (pi.hProcess);
   CloseHandle (pi.hThread);

   // Read the elements of Q from the output File
   ifstream outFile ("MetricUpgradeOut.txt");
   if ( !outFile.is_open() )
   {
      cerr << "Could not the Output Text file.\n" << endl;
      return -1;
   }
   string outText;
   outFile.seekg(0, ios::end);
   outText.resize(outFile.tellg());
   outFile.seekg(0, ios::beg);
   outFile.read(&outText[0], outText.size());
   outFile.close();

   // Extract and assign values
   size_t solPos = outText.find("# Approximate optimal solution information:", 0);
   solPos += strlen("# Approximate optimal solution information:");

   // Find q
   double q[10] = {0.0};
   for ( int i = 0; i < 10; i++ )
   {
      char pattern[10];
      sprintf(pattern, "%d: ", i+1);
      size_t valPos = outText.find(pattern, solPos);
      valPos += strlen(pattern);
      size_t endPos = outText.find(" ", valPos);
      string foundText = outText.substr(valPos, endPos - valPos);
      q[i] = (double)atof(foundText.c_str());
   }
   
   // From the Final Q Matrix to be returned
   //[q1 q2 q3 q4
   // q2 q5 q6 q7
   // q3 q6 q8 q9
   // q4 q7 q9 q10]
   for ( int row = 0, i = 0; row < 4; row++ )
   {
      for ( int col = 0; col < 4; col++ )
      {
         if ( col >= row )
         {
            QFinal->at<double>(row,col) = q[i];
            i++;
         }
         else
         {
            QFinal->at<double>(row,col) = QFinal->at<double>(col,row);
         }
      }
   }

   end_time = clock();
   cout << "Metric Upgrade Run Time: " << ((double) (end_time - start_time) / CLOCKS_PER_SEC) << endl; 

   return 1;
}

