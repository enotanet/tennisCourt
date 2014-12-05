/*#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ballFinder.h"
#include <iostream>
#include <cstdio>
#include "utils.h"
#include <thread>
#include <algorithm>
#include "video_writer.h"

#include <pylon/PylonIncludes.h>
#include <pylon/PylonGUI.h>
#include <pylon/SoftwareTriggerConfiguration.h>


#include "frame_grabber.h"
#include "file_frame_grabber.h"
#include "camera_frame_grabber.h"

using namespace cv;
using namespace std;
using namespace Pylon; */
/*
// Grab_UsingActionCommand.cpp
/
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

// Include files to use the PYLON API.
/* #include <pylon/PylonIncludes.h>
#include <pylon/PylonGUI.h>
#include <pylon/gige/PylonGigEIncludes.h>
#include <pylon/gige/ActionTriggerConfiguration.h>

// Settings to use Basler GigE cameras.
using namespace Basler_GigECameraParams;

// Namespace for using pylon objects.
using namespace Pylon;

// Namespace for using cout.
using namespace std;

// Limits the amount of cameras used for grabbing.
// It is important to manage the available bandwidth when grabbing with multiple 
// cameras. This applies, for instance, if two GigE cameras are connected to the 
// same network adapter via a switch. To manage the bandwidth, the GevSCPD 
// interpacket delay parameter and the GevSCFTD transmission delay parameter can 
// be set for each GigE camera device. The "Controlling Packet Transmission Timing 
// with the Interpacket and Frame Transmission Delays on Basler GigE Vision Cameras"
// Application Note (AW000649xx000) provides more information about this topic.
static const uint32_t c_maxCamerasToUse = 2;


int main(int argc, char* argv[])
{
    int exitCode = 0;

    // Automagically call PylonInitialize and PylonTerminate to ensure the pylon runtime system
    // is initialized during the lifetime of this object.
    Pylon::PylonAutoInitTerm autoInitTerm;

    try
    {
        // Get the GigE transport layer.
        // We'll need it later to issue the action commands.
        CTlFactory& tlFactory = CTlFactory::GetInstance();
        IGigETransportLayer *pTL = dynamic_cast<IGigETransportLayer*>(tlFactory.CreateTl(BaslerGigEDeviceClass));
        if (pTL == NULL)
        {
            throw RUNTIME_EXCEPTION("No GigE transport layer available.");
        }

        
        // In this sample we use the transport layer directly to enumerate cameras.
        // By calling EnumerateDevices on the TL we get get only GigE cameras.
        // You could also accomplish this by using a filter and
        // let the Transport Layer Factory enumerate.
        DeviceInfoList_t allDeviceInfos;
        if (pTL->EnumerateDevices(allDeviceInfos) == 0)
        {
            throw RUNTIME_EXCEPTION("No GigE cameras present.");
        }

        // Only use cameras in the same subnet as the first one.
        DeviceInfoList_t usableDeviceInfos;
        usableDeviceInfos.push_back(allDeviceInfos[0]);
        const String_t subnet(static_cast<const CBaslerGigEDeviceInfo&>(allDeviceInfos[0]).GetSubnetAddress());
        
        // Start with index 1 as we have already added the first one above.
        // We will also limit the number of cameras to c_maxCamerasToUse.
        for (size_t i = 1; i < allDeviceInfos.size() && usableDeviceInfos.size() < c_maxCamerasToUse; ++i)
        {
            const CBaslerGigEDeviceInfo& gigeinfo = static_cast<const CBaslerGigEDeviceInfo&>(allDeviceInfos[i]);
            if (subnet == gigeinfo.GetSubnetAddress())
            {
                // Add this deviceInfo to the ones we will be using.
                usableDeviceInfos.push_back(gigeinfo);
            }
            else
            {
                cerr << "Camera will not be used because it is in a different subnet "
                     << subnet << "!" << endl;
            }
        }

        // In this sample we'll use an CBaslerGigEInstantCameraArray to access multiple cameras.
        CBaslerGigEInstantCameraArray cameras(usableDeviceInfos.size());

        // Seed the random number generator and generate a random device key value.
        srand((unsigned)time(NULL));
        const uint32_t DeviceKey = rand();

        // For this sample we configure all cameras to be in the same group.
        const uint32_t GroupKey = 0x112233;

        // For the following sample we use the CActionTriggerConfiguration to configure the camera.
        // It will set the DeviceKey, GroupKey and GroupMask features. It will also
        // configure the camera FrameTrigger and set the TriggerSource to the action command.
        // You can look at the implementation of CActionTriggerConfiguration in <pylon/gige/ActionTriggerConfiguration.h>
        // to see which features are set.

        // Create all GigE cameras and attach them to the InstantCameras in the array.
        for (size_t i = 0; i < cameras.GetSize(); ++i)
        {
            cameras[i].Attach(tlFactory.CreateDevice(usableDeviceInfos[i]));
            // We'll use the CActionTriggerConfiguration, which will set up the cameras to wait for an action command.
            cameras[i].RegisterConfiguration(new CActionTriggerConfiguration(DeviceKey, GroupKey, AllGroupMask), RegistrationMode_Append, Cleanup_Delete);
            // Set the context. This will help us later to correlate the grab result to a camera in the array.
            cameras[i].SetCameraContext(i);

            const CBaslerGigEDeviceInfo& di = cameras[i].GetDeviceInfo();

            // Print the model name of the camera.
            cout << "Using camera " << i << ": " << di.GetModelName() << " (" << di.GetIpAddress() << ")" << endl;
        }

        // Open all cameras.
        // This will apply the CActionTriggerConfiguration specified above.
        cameras.Open();

        //////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////
        // Use an Action Command to Trigger Multiple Cameras at the Same Time.
        //////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////

        cout << endl << "Issuing an action command." << endl;

        // Starts grabbing for all cameras.
        // The cameras won't transmit any image data, because they are configured to wait for an action command.
        cameras.StartGrabbing();

        // Now we issue the action command to all devices in the subnet.
        // The devices with a matching DeviceKey, GroupKey and valid GroupMask will grab an image.
        pTL->IssueActionCommand(DeviceKey, GroupKey, AllGroupMask, subnet);

        // This smart pointer will receive the grab result data.
        CBaslerGigEGrabResultPtr ptrGrabResult;

        // Retrieve images from all cameras.
        const int DefaultTimeout_ms = 5000;
        for (size_t i = 0; i < cameras.GetSize() && cameras.IsGrabbing(); ++i)
        {
            // CInstantCameraArray::RetrieveResult will return grab results in the order they arrive.
            cameras.RetrieveResult(DefaultTimeout_ms, ptrGrabResult, TimeoutHandling_ThrowException);

            // When the cameras in the array are created the camera context value
            // is set to the index of the camera in the array.
            // The camera context is a user-settable value.
            // This value is attached to each grab result and can be used
            // to determine the camera that produced the grab result.
            intptr_t cameraIndex = ptrGrabResult->GetCameraContext();


            // Image grabbed successfully?
            if (ptrGrabResult->GrabSucceeded())
            {
                // Show the image acquired by each camera in the window related to the camera.
                // DisplayImage supports up to 32 image windows.
                if (cameraIndex <= 31)
                    Pylon::DisplayImage(cameraIndex, ptrGrabResult);

                // Print the index and the model name of the camera.
                cout << "Camera " <<  cameraIndex << ": " << cameras[cameraIndex].GetDeviceInfo().GetModelName() <<
                    " (" << cameras[cameraIndex].GetDeviceInfo().GetIpAddress() << ")" << endl;

                // You could process the image here by accessing the image buffer.
                cout << "GrabSucceeded: " << ptrGrabResult->GrabSucceeded() << endl;
                const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();
                cout << "Gray value of first pixel: " << (uint32_t) pImageBuffer[0] << endl << endl;
            }
            else
            {
                // If a buffer has been incompletely grabbed, the network bandwidth is possibly insufficient for transferring
                // multiple images simultaneously. See note above c_maxCamerasToUse.
                cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
            }
        }

        // In case you want to trigger again you should wait for the camera 
        // to become trigger-ready before issuing the next action command.
        // To avoid overtriggering you should call cameras[0].WaitForFrameTriggerReady
        // (see Grab_UsingGrabLoopThread sample for details).

        cameras.StopGrabbing();

        // Close all cameras.
        cameras.Close();
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

/*

int main(int argc, char* argv[])
{
    int exitCode = 0;

    // Automagically call PylonInitialize and PylonTerminate to ensure the pylon runtime system
    // is initialized during the lifetime of this object.
    Pylon::PylonAutoInitTerm autoInitTerm;

    try
    {
        // Get the GigE transport layer.
        // We'll need it later to issue the action commands.
        CTlFactory& tlFactory = CTlFactory::GetInstance();
        IGigETransportLayer *pTL = dynamic_cast<IGigETransportLayer*>(tlFactory.CreateTl(BaslerGigEDeviceClass));
        if (pTL == NULL)
        {
            throw RUNTIME_EXCEPTION("No GigE transport layer available.");
        }

        
        // In this sample we use the transport layer directly to enumerate cameras.
        // By calling EnumerateDevices on the TL we get get only GigE cameras.
        // You could also accomplish this by using a filter and
        // let the Transport Layer Factory enumerate.
        DeviceInfoList_t allDeviceInfos;
        if (pTL->EnumerateDevices(allDeviceInfos) == 0)
        {
            throw RUNTIME_EXCEPTION("No GigE cameras present.");
        }

        // Only use cameras in the same subnet as the first one.
        DeviceInfoList_t usableDeviceInfos;
        usableDeviceInfos.push_back(allDeviceInfos[0]);
        const String_t subnet(static_cast<const CBaslerGigEDeviceInfo&>(allDeviceInfos[0]).GetSubnetAddress());
        
        // Start with index 1 as we have already added the first one above.
        // We will also limit the number of cameras to c_maxCamerasToUse.
        for (size_t i = 1; i < allDeviceInfos.size() && usableDeviceInfos.size() < c_maxCamerasToUse; ++i)
        {
            const CBaslerGigEDeviceInfo& gigeinfo = static_cast<const CBaslerGigEDeviceInfo&>(allDeviceInfos[i]);
            if (subnet == gigeinfo.GetSubnetAddress())
            {
                // Add this deviceInfo to the ones we will be using.
                usableDeviceInfos.push_back(gigeinfo);
            }
            else
            {
                cerr << "Camera will not be used because it is in a different subnet "
                     << subnet << "!" << endl;
            }
        }

        // In this sample we'll use an CBaslerGigEInstantCameraArray to access multiple cameras.
        CBaslerGigEInstantCameraArray cameras(usableDeviceInfos.size());

        // Seed the random number generator and generate a random device key value.
        srand((unsigned)time(NULL));
        const uint32_t DeviceKey = rand();

        // For this sample we configure all cameras to be in the same group.
        const uint32_t GroupKey = 0x112233;

        // For the following sample we use the CActionTriggerConfiguration to configure the camera.
        // It will set the DeviceKey, GroupKey and GroupMask features. It will also
        // configure the camera FrameTrigger and set the TriggerSource to the action command.
        // You can look at the implementation of CActionTriggerConfiguration in <pylon/gige/ActionTriggerConfiguration.h>
        // to see which features are set.

        // Create all GigE cameras and attach them to the InstantCameras in the array.
        for (size_t i = 0; i < cameras.GetSize(); ++i)
        {
            cameras[i].Attach(tlFactory.CreateDevice(usableDeviceInfos[i]));
            // We'll use the CActionTriggerConfiguration, which will set up the cameras to wait for an action command.
            cameras[i].RegisterConfiguration(new CActionTriggerConfiguration(DeviceKey, GroupKey, AllGroupMask), RegistrationMode_Append, Cleanup_Delete);
            // Set the context. This will help us later to correlate the grab result to a camera in the array.
            cameras[i].SetCameraContext(i);

            const CBaslerGigEDeviceInfo& di = cameras[i].GetDeviceInfo();

            // Print the model name of the camera.
            cout << "Using camera " << i << ": " << di.GetModelName() << " (" << di.GetIpAddress() << ")" << endl;
        }

        // Open all cameras.
        // This will apply the CActionTriggerConfiguration specified above.
        cameras.Open();

        //////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////
        // Use an Action Command to Trigger Multiple Cameras at the Same Time.
        //////////////////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////

        // Starts grabbing for all cameras.
        // The cameras won't transmit any image data, because they are configured to wait for an action command.
        cameras.StartGrabbing();


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

        Pylon::CImageFormatConverter fc;
        fc.OutputPixelFormat = Pylon::PixelType_Mono8;
        Pylon::CPylonImage image;
        int frames = 0;
        long long start = clock();
        while ((clock() - start) < 5LL * CLOCKS_PER_SEC)
        {
          // Wait for cameras in case they aren't ready!
          for (size_t i = 0; i < cameras.GetSize(); ++i) {
            if (!cameras[i].WaitForFrameTriggerReady(100, TimeoutHandling_Return)) {
                std::cout << "Wait failed! " << cameras.GetSize() << endl;
                cout << "Current frame: " << frames << endl;
            }
          }
          ++frames;

          // Now we issue the action command to all devices in the subnet.
          // The devices with a matching DeviceKey, GroupKey and valid GroupMask will grab an image.
  
          cout << endl << "Issuing an action command." << endl;

          pTL->IssueActionCommand(DeviceKey, GroupKey, AllGroupMask, subnet);

          // This smart pointer will receive the grab result data.
          CBaslerGigEGrabResultPtr ptrGrabResult;

          // Retrieve images from all cameras.
          const int DefaultTimeout_ms = 5000;
          for (size_t i = 0; i < usableDeviceInfos.size() && cameras.IsGrabbing(); ++i)
          {
              // CInstantCameraArray::RetrieveResult will return grab results in the order they arrive.
              if (!cameras.RetrieveResult(DefaultTimeout_ms, ptrGrabResult, TimeoutHandling_Return)) {
                std::cout << "Grab failed! " << usableDeviceInfos.size() << endl;
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
              intptr_t cameraIndex = ptrGrabResult->GetCameraContext();
              cout << "Grabbed image from " << cameraIndex << " I " << i << " F " << frames << endl;


              // Image grabbed successfully?
              if (ptrGrabResult->GrabSucceeded())
              {
                  // Show the image acquired by each camera in the window related to the camera.
                  // DisplayImage supports up to 32 image windows.
                  // if (cameraIndex <= 31)
                      // Pylon::DisplayImage(cameraIndex, ptrGrabResult);
                  
                  fc.Convert(image, ptrGrabResult);
            			Mat res = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC1, (uint8_t*) image.GetBuffer());
                  //cout << cameraIndex << endl;
                  //cout << "Got IMG SIZE " << res.size() << endl;
                  vw[cameraIndex].write(res);

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
                  cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << endl;
              }
          }
        }
        std::cout << frames << " frames recoreded in 5 seconds\n";

        // In case you want to trigger again you should wait for the camera 
        // to become trigger-ready before issuing the next action command.
        // To avoid overtriggering you should call cameras[0].WaitForFrameTriggerReady
        // (see Grab_UsingGrabLoopThread sample for details).

        cameras.StopGrabbing();

        // Close all cameras.
        cameras.Close();
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
*/


/* int main(int argc, char *argv[]) {
  Pylon::PylonAutoInitTerm autoInitTerm;
  CameraFrameGrabber cfg0(0);
  char buf0[1024];
  sprintf(buf0, "S:\\\\video%d.avi", 0);
  CameraFrameGrabber cfg1(1);
  char buf1[1024];
  sprintf(buf1, "S:\\\\video%d.avi", 1);
  std::thread t1(writeVideoToFile, buf0, &cfg0, 10);
  std::thread t2(writeVideoToFile, buf1, &cfg1, 10);
  t1.join();
  t2.join();
  return 0;
}*/
  
/*  cv::Mat ff(ffg.getMat());
  // Setup output video
  cv::VideoWriter output_cap(argv[2], 
    //CV_FOURCC('F','F','V','1'),
    CV_FOURCC('H','F','Y','U'),
    // -1,
    30,
    ff.size());

  if (!output_cap.isOpened())
  {
     std::cout << "!!! Output video could not be opened" << std::endl;
       return 0;
  }
  
  // Loop to read from input and write to output
  cv::Mat frame;
  
  while (true)
  {       
      if (!ffg.getNextFrame(&frame))             
                break;
     std::cout << "FSZ " << frame.size() << std::endl;
                  
        output_cap.write(frame);
  }
  return 0;
  VideoCapture vc("S:\\video.avi");
  Mat frame;
  namedWindow("edges",1);
  while (vc.read(frame)) {
    imshow("edges", frame);
    waitKey();
  }
  return 0;
}
*/

/*
  CameraFrameGrabber cfg(0);

  CGrabResultPtr ptrGrabResult;
  namedWindow("CV_Image", WINDOW_AUTOSIZE);
  Mat cv_img(cfg.getMat());

	    int frames = 0;
	    int frames2 = 0;
	    printf("GRABBING STARTED AT %d\n", time(nullptr));
	    clock_t st = clock();
	    vector<Mat> video;
      while (cfg.getNextFrame(&cv_img)) {
				  ++frames;
				  if (clock() - st > 10 * CLOCKS_PER_SEC)
					  break;
  		    imshow("CV_Image", cv_img);
          video.push_back(cv_img.clone());
		  }

	    printf("%d, %d frames grabed in 5 seconds\n", frames, frames2);
	    printf("GRABBING FINISHED AT %d\n", time(nullptr));

	    int i = 0;
	    while (true)
	    {
		    cout << "Showing frame " << i << "\n";
		    imshow("CV_Image", video[i]);
		    char c = waitKey(0);
		    if (c == 'k')
			    break;
		    else if (c == 'n' && i + 1 < (int) video.size())
			    ++i;
		    else if (c == 'p' && i > 0)
			    --i;
	    }

      return 0;
}
*/
/*    // The exit code of the sample application.
    int exitCode = 0;

    // Automagically call PylonInitialize and PylonTerminate to ensure 
    // the pylon runtime system is initialized during the lifetime of this object.
    Pylon::PylonAutoInitTerm autoInitTerm;


    CGrabResultPtr ptrGrabResult;
    namedWindow("CV_Image", WINDOW_AUTOSIZE);
    try
    {
	    DeviceInfoList_t dl;
	    CTlFactory::GetInstance().EnumerateDevices(dl);
	    cout << dl.size() << " cameras found\n";
	    CInstantCamera camera(CTlFactory::GetInstance().CreateDevice(dl[0]));
      cout << "Using device " << camera.GetDeviceInfo().GetModelName() << endl;
	    camera.Open();
	    //CInstantCamera camera2(CTlFactory::GetInstance().CreateDevice(dl[1]));
	    //cout << "Using device " << camera2.GetDeviceInfo().GetModelName() << endl;
	    //camera2.Open();

	    GenApi::CIntegerPtr width(camera.GetNodeMap().GetNode("Width"));
	    GenApi::CIntegerPtr height(camera.GetNodeMap().GetNode("Height"));
	    Mat cv_img(width->GetValue(), height->GetValue(), CV_8UC3);

	    camera.StartGrabbing();
//	    camera2.StartGrabbing();
	    CPylonImage image;
	    CImageFormatConverter fc;
	    fc.OutputPixelFormat = PixelType_RGB8packed;

	    int frames = 0;
	    int frames2 = 0;
	    printf("GRABBING STARTED AT %d\n", time(nullptr));
	    clock_t st = clock();
	    vector<Mat> video;
	    while(camera.IsGrabbing()){
		    camera.RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);
		    if (ptrGrabResult->GrabSucceeded()){
				    ++frames;
				    if (clock() - st > 10 * CLOCKS_PER_SEC)
					    break;
				    fc.Convert(image, ptrGrabResult);

				    cv_img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t*)image.GetBuffer());
				    video.push_back(cv_img.clone());
		    }

	    }
	    printf("%d, %d frames grabed in 5 seconds\n", frames, frames2);
	    printf("GRABBING FINISHED AT %d\n", time(nullptr));

	    int i = 0;
	    while (true)
	    {
		    cout << "Showing frame " << i << "\n";
		    imshow("CV_Image", video[i]);
		    char c = waitKey(0);
		    if (c == 'k')
			    break;
		    else if (c == 'n' && i + 1 < (int) video.size())
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
*/
