#include "cv.h"      // include it to used Main OpenCV functions.
#include "highgui.h" //include it to use GUI functions.
 
 int main(int argc, char** argv)
 {
         IplImage* img = cvLoadImage( "fwm-small.jpg" ); //change the name (image.jpg) according to your Image filename.
         cvNamedWindow( "Example1", CV_WINDOW_AUTOSIZE );
         cvShowImage("Example1", img);
         cvWaitKey(0);
         cvReleaseImage( &img );
         cvDestroyWindow( "Example1" );
         return 0;
 }
