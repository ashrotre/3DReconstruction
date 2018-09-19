/*****************************************************************************************************************************
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

#include <opencv2\features2d\features2d.hpp>
#include "Common.h"
extern "C"{
#include "generic.h"
#include <sift.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
}
using namespace cv;
using namespace std;
/*detects keypoints and computes descriptors. 
dst1 - image, kpts1 - vector of Keypoints, framenum - number ofkeypoints, d1 - matrix of descriptors
each row of d1 matrix contains 128 elements that form the descriptor for a single keypoint.*/
/* uses the vlfeat package found at http://www.vlfeat.org/ for SIFT keypoints and descriptors*/
void sift_new(Mat dst1,vector<Point2d>& kpts1,int framenum,Mat& d1)
{
   /* algorithm parameters */
   double   edge_thresh  = 10;
   double   peak_thresh  = 0.0067 ;
   double   magnif       = 3;
   int      O = -1, S = 3, omin = 0;
   Point2d temp;

   vl_bool  err    = VL_ERR_OK ;
   char     err_msg [1024] ;
   //  int      n ;
   int      exit_code          = 0 ;
   int      verbose            = 0 ;
   vl_bool  force_output       = 0 ;
   vl_bool  force_orientations = 0 ;

   FILE            *in    = 0 ;
   vl_sift_pix     *fdata = 0 ;


   VlSiftFilt      *filt = 0 ;
   vl_size          q ;
   int              i ;
   vl_bool          first ;

   double           *ikeys = 0 ;
   int              nikeys = 0, ikeys_size = 0 ;

   uchar* data=new uchar[dst1.cols*dst1.rows];
   for(int i=0;i<dst1.rows;i++)
      for (int j=0;j<dst1.cols;j++)
         data[i*dst1.cols+j]=dst1.at<uchar>(i,j);

   fdata=new vl_sift_pix[dst1.cols*dst1.rows];





   /* convert data type */
   for (q = 0 ; q < (unsigned) (dst1.cols*dst1.rows) ; q++) {
      fdata [q] = (vl_sift_pix)data [q] ;
   }


   /* ...............................................................
   *                                                     Make filter
   * ............................................................ */

   filt = vl_sift_new (dst1.cols, dst1.rows, O, S, omin) ;
   filt->sigman=0.5;
   filt->sigma0=2.5;

   if (edge_thresh >= 0) vl_sift_set_edge_thresh (filt, edge_thresh) ;
   if (peak_thresh >= 0) vl_sift_set_peak_thresh (filt, peak_thresh) ;
   if (magnif      >= 0) vl_sift_set_magnif      (filt, magnif) ;

   if (!filt) {
      snprintf (err_msg, sizeof(err_msg),
         "Could not create SIFT filter.") ;
      err = VL_ERR_ALLOC ;
      goto done ;
   }

   if (verbose > 1) {
      printf ("sift: filter settings:\n") ;
      printf ("sift:   octaves      (O)     = %d\n",
         vl_sift_get_noctaves     (filt)) ;
      printf ("sift:   levels       (S)     = %d\n",
         vl_sift_get_nlevels      (filt)) ;
      printf ("sift:   first octave (o_min) = %d\n",
         vl_sift_get_octave_first (filt)) ;
      printf ("sift:   edge thresh           = %g\n",
         vl_sift_get_edge_thresh  (filt)) ;
      printf ("sift:   peak thresh           = %g\n",
         vl_sift_get_peak_thresh  (filt)) ;
      printf ("sift:   magnif                = %g\n",
         vl_sift_get_magnif       (filt)) ;
      printf ("sift: will source frames? %s\n",
         ikeys ? "yes" : "no") ;
      printf ("sift: will force orientations? %s\n",
         force_orientations ? "yes" : "no") ;
   }

   /* ...............................................................
   *                                             Process each octave
   * ............................................................ */
   i     = 0 ;
   first = 1 ;
   while (1) {
      VlSiftKeypoint const *keys = 0 ;
      int                   nkeys ;

      /* calculate the GSS for the next octave .................... */
      if (first) {
         first = 0 ;
         err = vl_sift_process_first_octave (filt, fdata) ;
      } else {
         err = vl_sift_process_next_octave  (filt) ;
      }

      if (err) {
         err = VL_ERR_OK ;
         break ;
      }

      if (verbose > 1) {
         printf("sift: GSS octave %d computed\n",
            vl_sift_get_octave_index (filt));
      }


      /* run detector ............................................. */
      if (ikeys == 0) {
         vl_sift_detect (filt);

         keys  = vl_sift_get_keypoints(filt);
         nkeys = vl_sift_get_nkeypoints(filt);
         i     = 0 ;

         if (verbose > 1) {
            printf ("sift: detected %d (unoriented) keypoints\n", nkeys) ;
         }
      } else {
         nkeys = nikeys ;
      }

      /* for each keypoint ........................................ */
      for (; i < nkeys ; ++i) 
      {
         double                angles [4] ;
         int                   nangles ;
         //        VlSiftKeypoint        ik ;
         VlSiftKeypoint const *k ;

         framenum=framenum+1;
         k = keys + i ;
         nangles = vl_sift_calc_keypoint_orientations(filt, angles, k) ;


         /* for each orientation ................................... */
         for (q = 0 ; q < (unsigned) nangles ; ++q) 
         {
            vl_sift_pix descr [128] ;
            Mat tempdesc(1,128,CV_8U);
            /* compute descriptor (if necessary) */

            vl_sift_calc_keypoint_descriptor(filt, descr, k, angles [q]) ;

            temp.x=k->x;
            temp.y=k->y;
            /*temp.size=k->sigma;
            temp.angle=angles[q];*/
            kpts1.push_back(temp);
            int l ;
            for (l = 0 ; l < 128 ; ++l) 
            {
               double x = 512.0 * descr[l] ;

               tempdesc.at<uchar>(0,l) =(x < 255.0) ? x : 255.0 ;
            }
            d1.push_back(tempdesc);
            tempdesc.~Mat();
         }
      }
   }

   /* ...............................................................
   *                                                       Finish up
   * ............................................................ */
done :
   /* release input keys buffer */
   if (ikeys) {
      free (ikeys) ;
      ikeys_size = nikeys = 0 ;
      ikeys = 0 ;
   }

   /* release filter */
   if (filt) {
      vl_sift_delete (filt) ;
      filt = 0 ;
   }

   /* release image data */
   if (fdata) {
      free (fdata) ;
      fdata = 0 ;
   }

   /* release image data */
   if (data) {
      free (data) ;
      data = 0 ;
   }

   /* close files */
   if (in) {
      fclose (in) ;
      in = 0 ;
   }


   /* if bad print error message */
   if (err) {
      fprintf
         (stderr,
         "sift: err: %s (%d)\n",
         err_msg,
         err) ;
      exit_code = 1 ;
   }

}
