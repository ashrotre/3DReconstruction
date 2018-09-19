{\rtf1\ansi\ansicpg1252\deff0\nouicompat{\fonttbl{\f0\fnil\fcharset0 Calibri;}{\f1\fnil Consolas;}}
{\colortbl ;\red0\green128\blue0;\red0\green0\blue255;}
{\*\generator Riched20 6.2.9200}\viewkind4\uc1 
\pard\sl240\slmult1\qc\ul\f0\fs36\lang9 README\fs22\par

\pard\sl240\slmult1\par
\fs28 Folders:\fs22\par
\par
\ulnone\b\fs24 3DReconstruction\fs22\par
\b0 MSVC2010 solution file: 3DReconstruction.sln\ul\par
\ulnone Header Files:\par
\tab Common.h\par
Source Files:\par
main.cpp - contains the main function\par
\par
CalcReProjErr.cpp - calcuates the error between the reprojection of calculated 3D points and keypoints detected\par
\par
computeSVD.cpp - Replacement for the OpenCV SVD calculation using LAPACK libraries from MKL package \par
\par
depcomposeProjection.cpp - Replacement of OpenCV decomposeProjectionMatrix() using LAPACK libraries from MKL package\par
\par
displaymatches.cpp - Produces a single image showing matching keypoints in multiple views\par
\par
findcommon.cpp - Finds the common points for triangulation from all 8 views\par
\par
FindHfromQ.cpp - Decomposes Q matrix to find H \par
\par
fundamentalmartix.cpp - Replacement of OPENCV findFundamentalMat()\par
\par
getkeypoints.cpp - Computes SIFT keypoints and corresponding descriptors for an image\par
\par
imageload.cpp - loads images, the path to the images to be loaded\par
\par
matchdesciptors.cpp - Matches Keypoints from each image pair using their descriptors\par
\par
MetricUpgrade.cpp - Metric Upgrade using the sparsePOP function. \par
\tab Requires: sparsePOP.exe, param.pop, param.sdpa, for execution (included in the folder)\par
\tab Input File: MetricUpgradeRef.gms (included)\par
\tab Output File: MetricUpgrageOut.txt\par
\tab URL: \f1\fs19 http://www.is.titech.ac.jp/~kojima/SparsePOP/\cf1\par
\cf0\f0\fs22\tab Library Depndency: SymbolicC++ (3rdParty folder)\cf1\f1\fs19\par
\cf0\f0\fs22\par
projectionmatrix.cpp - Computes projection matrix from Fundamental matrix\par
RANSAC.cpp - Implements RANSAC for use in computing fundamental matrix\par
refinepoints.cpp - Normalizes matched keypoints\par
\par
SBA.cpp - Bundle Adjustment using the 3rd party SBA libraries. \par
\tab\par
sift_new.cpp - Implements a SIFT Keypoint detector using VLFeat library package \par
\tab URL: \f1\fs19 http://www.vlfeat.org/\cf1\par
\cf0\f0\fs22\tab Library Depndency: vlfeat (3rdParty folder)\cf1\f1\fs19\par
\cf0\f0\fs22\par
\tab\par
triangulation.cpp - Computes 3D points from provided set of 2D points using triangulation\par

\pard\li720\sl240\slmult1\par

\pard\sl240\slmult1\b\fs24 3rdParty\fs22\par
\b0 SBA - includes, x64 and x86 binaries (ver 1.5) \par
\tab\tab  {\f1\fs19{\field{\*\fldinst{HYPERLINK http://users.ics.forth.gr/~lourakis/sba/ }}{\fldrslt{http://users.ics.forth.gr/~lourakis/sba/\ul0\cf0}}}}\cf1\f1\fs19  \cf0\f0\fs22\par
SymbolicC++ - includes, x64 and x86 compiled libraries \tab\par
\tab\tab {{\field{\*\fldinst{HYPERLINK http://issc.uj.ac.za/symbolic/symbolic.html }}{\fldrslt{http://issc.uj.ac.za/symbolic/symbolic.html\ul0\cf0}}}}\f0\fs22\par
vlfeat - Vision Lab Features Library (SIFT)\par
\tab\tab URL: {{\field{\*\fldinst{HYPERLINK http://www.vlfeat.org/api/index.html }}{\fldrslt{http://www.vlfeat.org/api/index.html\ul0\cf0}}}}\f0\fs22\par
\ul\par
\ulnone\b\fs24 Issues\fs22\par
\b0 1. sparsePOP is provided as an application (.exe) and hence is run on a child process. Compiling instructions require cross compilation from Linux environment.\par
2. SymbolicC++ libraries required by sparsePOP to form the optimization equation is slow and needs to be substituted with faster options.\ul\par
\ulnone\par
}
 