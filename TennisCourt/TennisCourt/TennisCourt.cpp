#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ballFinder.h"
#include <iostream>
#include <cstdio>
#include "utils.h"
#include <ctime>

#include <pylon/PylonIncludes.h>

using namespace cv;
using namespace Pylon;
using namespace std;

int main(int argc, char *argv[])
{
	// The exit code of the sample application.
	int exitCode = 0;

	// Automagically call PylonInitialize and PylonTerminate to ensure 
	// the pylon runtime system is initialized during the lifetime of this object.
	Pylon::PylonAutoInitTerm autoInitTerm;


	CGrabResultPtr ptrGrabResult;
	namedWindow("CV_Image", WINDOW_AUTOSIZE);
	namedWindow("CV_Image2", WINDOW_AUTOSIZE);
	try
	{
		DeviceInfoList_t dl;
		CTlFactory::GetInstance().EnumerateDevices(dl);
		cout << dl.size() << " cameras found\n";
		CInstantCamera camera(CTlFactory::GetInstance().CreateDevice(dl[0]));
		cout << "Using device " << camera.GetDeviceInfo().GetModelName() << endl;
		camera.Open();
		CInstantCamera camera2(CTlFactory::GetInstance().CreateDevice(dl[1]));
		cout << "Using device " << camera2.GetDeviceInfo().GetModelName() << endl;
		camera2.Open();

		GenApi::CIntegerPtr width(camera.GetNodeMap().GetNode("Width"));
		GenApi::CIntegerPtr height(camera.GetNodeMap().GetNode("Height"));
		Mat cv_img(width->GetValue(), height->GetValue(), CV_8UC3);

		camera.StartGrabbing();
		camera2.StartGrabbing();
		CPylonImage image;
		CImageFormatConverter fc;
		fc.OutputPixelFormat = PixelType_RGB8packed;

		int frames = 0;
		int frames2 = 0;
		printf("GRABBING STARTED AT %d\n", time(nullptr));
		clock_t st = clock();
		vector<Mat> video;
		vector<Mat> video2;
		while(camera.IsGrabbing() || camera2.IsGrabbing()){
			camera.RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);
			if (ptrGrabResult->GrabSucceeded()){
					++frames;
					if (clock() - st > 10 * CLOCKS_PER_SEC)
						break;
					fc.Convert(image, ptrGrabResult);

					cv_img = cv::Mat(ptrGrabResult->GetHeight(),     ptrGrabResult->GetWidth(), CV_8UC3,(uint8_t*)image.GetBuffer());
					video.push_back(cv_img.clone());
					//cout << cv_img.size() << endl;
					//imshow("CV_Image",cv_img);
					//waitKey(1);
					//if(waitKey(30)==27){
						 //camera.StopGrabbing();
					//}
			}

			camera2.RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);
			if (ptrGrabResult->GrabSucceeded()){
					++frames2;
					if (clock() - st > 10 * CLOCKS_PER_SEC)
						break;
					fc.Convert(image, ptrGrabResult);

					cv_img = cv::Mat(ptrGrabResult->GetHeight(),     ptrGrabResult->GetWidth(), CV_8UC3,(uint8_t*)image.GetBuffer());
					video2.push_back(cv_img.clone());
					//cout << cv_img.size() << endl;
					//imshow("CV_Image",cv_img);
					//waitKey(1);
					//if(waitKey(30)==27){
						 //camera.StopGrabbing();
					//}
			}
		}
		printf("%d, %d frames grabed in 5 seconds\n", frames, frames2);
		printf("GRABBING FINISHED AT %d\n", time(nullptr));

		int i = 0;
		while (true)
		{
			cout << "Showing frame " << i << "\n";
			imshow("CV_Image", video[i]);
			imshow("CV_Image2", video2[i]);
			char c = waitKey(0);
			if (c == 'k')
				break;
			else if (c == 'n' && i + 1 < (int) video.size() && i + 1 < (int) video2.size())
				++i;
			else if (c == 'p' && i > 0)
				--i;
		}
	}

	catch (GenICam::GenericException &e)
	{
		// Error handling.
		cerr << "An exception occurred." << endl
		<< e.GetDescription() << endl;
		exitCode = 1;
	}

	// Comment the following two lines to disable waiting on exit
	cerr << endl << "Press Enter to exit." << endl;
	while( cin.get() != '\n');

	return exitCode;
}