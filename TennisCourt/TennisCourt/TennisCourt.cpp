#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ballFinder.h"
#include <iostream>
#include <cstdio>
#include "utils.h"
#include <thread>
#include <algorithm>
#include "video_writer.h"

// Grab_UsingActionCommand.cpp
/* 
    Note: Before getting started, Basler recommends reading the Programmer's Guide topic
    in the pylon C++ API documentation that gets installed with pylon.
    If you are upgrading to a higher major version of pylon, Basler also
    strongly recommends reading the Migration topic in the pylon C++ API documentation.

    This sample shows how to issue a GigE Vision ACTION_CMD to multiple cameras.
    By using an action command multiple cameras can be triggered at the same time
    compared to software triggering, which must be triggered individually.

    To make the configuration of multiple cameras easier this sample uses the CInstantCameraArray class.
    It also uses a CActionTriggerConfiguration to set up the basic action command features.
*/

#include <time.h>   // for time
#include <stdlib.h> // for rand & srand

// Include files to use the PYLON API.
#include <pylon/PylonIncludes.h>
#include <pylon/PylonGUI.h>
#include <pylon/gige/PylonGigEIncludes.h>
#include <pylon/gige/ActionTriggerConfiguration.h>

// Settings to use Basler GigE cameras.
using namespace Basler_GigECameraParams;

// Namespace for using pylon objects.
using namespace Pylon;

// Namespace for using cout.
using namespace std;

using namespace cv;

int c_maxCamerasToUse = 2;

void myconvandwrite(CImageFormatConverter *fc, CGrabResultPtr &ptrGrabResult, CPylonImage &img, VideoWriter &vw) {
  fc->Convert(img, ptrGrabResult);
  Mat res = cv::Mat(1024, 1280, CV_8UC1, (uint8_t*) img.GetBuffer());
  //cout << cameraIndex << endl;
  //cout << "Got IMG SIZE " << res.size() << endl;
  vw.write(res);
}

void myfun(CInstantCameraArray &cameras, CImageFormatConverter *fc, CGrabResultPtr &ptrGrabResult, CPylonImage &img, VideoWriter &vw) {
  if (!cameras.RetrieveResult(5000, ptrGrabResult, TimeoutHandling_Return)) {
    return;
  }
  fc->Convert(img, ptrGrabResult);
  Mat res = cv::Mat(1024, 1280, CV_8UC1, (uint8_t*) img.GetBuffer());
  //cout << cameraIndex << endl;
  //cout << "Got IMG SIZE " << res.size() << endl;
  vw.write(res);
}

int main(int argc, char* argv[])
{
    int exitCode = 0;

    // Automagically call PylonInitialize and PylonTerminate to ensure the pylon runtime system
    // is initialized during the lifetime of this object.
    Pylon::PylonAutoInitTerm autoInitTerm;

    try
    {
        // Get the transport layer factory.
        CTlFactory& tlFactory = CTlFactory::GetInstance();

        // Get all attached devices and exit application if no device is found.
        DeviceInfoList_t devices;
        if ( tlFactory.EnumerateDevices(devices) == 0 )
        {
            throw RUNTIME_EXCEPTION( "No camera present.");
        }

        // Create an array of instant cameras for the found devices and avoid exceeding a maximum number of devices.
        CInstantCameraArray cameras( min( devices.size(), c_maxCamerasToUse));

        // Create and attach all Pylon Devices.
        for ( size_t i = 0; i < cameras.GetSize(); ++i)
        {
            cameras[ i ].Attach( tlFactory.CreateDevice( devices[ i ]));

            // Print the model name of the camera.
            cout << "Using device " << cameras[ i ].GetDeviceInfo().GetModelName() << endl;
        }

        // Starts grabbing for all cameras starting with index 0. The grabbing
        // is started for one camera after the other. That's why the images of all
        // cameras are not taken at the same time.
        // However, a hardware trigger setup can be used to cause all cameras to grab images synchronously.
        // According to their default configuration, the cameras are
        // set up for free-running continuous acquisition.
        cameras.StartGrabbing(GrabStrategy_LatestImageOnly);


        long long width = GenApi::CIntegerPtr(cameras[0].GetNodeMap().GetNode("Width"))->GetValue();
        long long height = GenApi::CIntegerPtr(cameras[0].GetNodeMap().GetNode("Height"))->GetValue();
        cout << "W " << width << " H " << height << endl;
        cv::Mat cv_img(height, width, CV_8UC1); 
        cv::VideoWriter vw[4];
        for (int i = 0; i < 4; ++i) {
          char buf[32];
          sprintf(buf, "S:\\\\video%d.avi", i);
          cout << "EXPECTED IMG SIZE " << cv_img.size() << endl;
          vw[i].open(buf, CV_FOURCC('D','I','B',' '), 60, cv_img.size(), false);
          if (!vw[i].isOpened()) {
             std::cout << "!!! Output video could not be opened" << std::endl;
             return 0;
          }
        }

        Pylon::CImageFormatConverter fc[4];
        for (int i = 0; i < 4; ++i)
          fc[i].OutputPixelFormat = Pylon::PixelType_Mono8;
        Pylon::CPylonImage image[4];
        thread t[4];
        int frames = 0;
        long long start = clock();
        while ((clock() - start) < 10LL * CLOCKS_PER_SEC)
        {
          ++frames;

          // Now we issue the action command to all devices in the subnet.
          // The devices with a matching DeviceKey, GroupKey and valid GroupMask will grab an image.

          // This smart pointer will receive the grab result data.
          CGrabResultPtr ptrGrabResult[4];

          // Retrieve images from all cameras.
          const int DefaultTimeout_ms = 5000;
          for (size_t i = 0; i < cameras.GetSize() && cameras.IsGrabbing(); ++i)
          {
              // CInstantCameraArray::RetrieveResult will return grab results in the order they arrive.
              if (!cameras.RetrieveResult(DefaultTimeout_ms, ptrGrabResult[i], TimeoutHandling_Return)) {
                std::cout << "Grab failed! " << cameras.GetSize() << endl;
                cout << "Current frame: " << frames << " I " << i << endl;
                cerr << endl << "Press Enter to exit." << endl;
                while (cin.get() != '\n');
                return 0;
              }

              // When the cameras in the array are created the camera context value
              // is set to the index of the camera in the array.
              // The camera context is a user-settable value.
              // This value is attached to each grab result and can be used
              // to determine the camera that produced the grab result.
              intptr_t cameraIndex = ptrGrabResult[i]->GetCameraContext();
              // cout << "Grabbed image from " << cameraIndex << " I " << i << " F " << frames << endl;


              // Image grabbed successfully?
              if (ptrGrabResult[i]->GrabSucceeded())
              {
                  // Show the image acquired by each camera in the window related to the camera.
                  // DisplayImage supports up to 32 image windows.
                  // if (cameraIndex <= 31)
                      // Pylon::DisplayImage(cameraIndex, ptrGrabResult);
                  
                  if (t[cameraIndex].joinable()) {
                    t[cameraIndex].join();
                  }
                  t[cameraIndex] = std::thread(myconvandwrite, &fc[cameraIndex], ptrGrabResult[i], image[cameraIndex], vw[cameraIndex]);

                  // Print the index and the model name of the camera.
                  //cout << "Camera " <<  cameraIndex << ": " << cameras[cameraIndex].GetDeviceInfo().GetModelName() <<
                  //    " (" << cameras[cameraIndex].GetDeviceInfo().GetIpAddress() << ")" << endl;

                  // You could process the image here by accessing the image buffer.
                  //cout << "GrabSucceeded: " << ptrGrabResult->GrabSucceeded() << endl;
                  //const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();
                  //cout << "Gray value of first pixel: " << (uint32_t) pImageBuffer[0] << endl << endl;
              }
              else
              {
                  // If a buffer has been incompletely grabbed, the network bandwidth is possibly insufficient for transferring
                  // multiple images simultaneously. See note above c_maxCamerasToUse.
                  cout << "Error: " << ptrGrabResult[i]->GetErrorCode() << " " << ptrGrabResult[i]->GetErrorDescription() << endl;
              }
          }
        }
        std::cout << frames << " frames recoreded in 10 seconds\n";

        // In case you want to trigger again you should wait for the camera 
        // to become trigger-ready before issuing the next action command.
        // To avoid overtriggering you should call cameras[0].WaitForFrameTriggerReady
        // (see Grab_UsingGrabLoopThread sample for details).

        cameras.StopGrabbing();

        // Close all cameras.
        cameras.Close();
        for (int i = 0; i < 4; ++i)
        {
          if (t[i].joinable())
            t[i].join();
        }
    }
    catch (GenICam::GenericException &e)
    {
        // Error handling
        cerr << "An exception occurred." << endl
        << e.GetDescription() << endl;
        exitCode = 1;
    }

    // Comment the following two lines to disable waiting on exit.
    cerr << endl << "Press Enter to exit." << endl;
    while (cin.get() != '\n');

    return exitCode;
}
