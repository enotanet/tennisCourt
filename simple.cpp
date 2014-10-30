#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"

using namespace cv;
using namespace std;

int main( int argc, const char** argv )
{
    cout << "Starting up!\n";
    Mat img = imread("fwm-small.jpg", CV_LOAD_IMAGE_COLOR);

    if (img.empty()) //check whether the image is loaded or not
    {
        cout << "Error : Image cannot be loaded..!!" << endl;
                  return -1;
    }

    namedWindow("MyWindow", CV_WINDOW_AUTOSIZE); //create a window with the name "MyWindow"
    imshow("MyWindow", img); //display the image which is stored in the 'img' in the "MyWindow" window
 
    waitKey(0);
    destroyWindow("MyWindow"); //destroy the window with the name, "MyWindow"

    return 0;
}
