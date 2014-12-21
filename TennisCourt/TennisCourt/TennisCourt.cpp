#ifdef DANCHO

// Grab_MultipleCameras.cpp
/*
    Note: Before getting started, Basler recommends reading the Programmer's Guide topic
    in the pylon C++ API documentation that gets installed with pylon.
    If you are upgrading to a higher major version of pylon, Basler also
    strongly recommends reading the Migration topic in the pylon C++ API documentation.

    This sample illustrates how to grab and process images from multiple cameras
    using the CInstantCameraArray class. The CInstantCameraArray class represents
    an array of instant camera objects. It provides almost the same interface
    as the instant camera for grabbing.
    The main purpose of the CInstantCameraArray is to simplify waiting for images and
    camera events of multiple cameras in one thread. This is done by providing a single
    RetrieveResult method for all cameras in the array.
    Alternatively, the grabbing can be started using the internal grab loop threads
    of all cameras in the CInstantCameraArray. The grabbed images can then be processed by one or more
    image event handlers. Please note that this is not shown in this example.
*/

// Include files to use the PYLON API.
#include <pylon/PylonIncludes.h>
#ifdef PYLON_WIN_BUILD
#    include <pylon/PylonGUI.h>
#endif

// Namespace for using pylon objects.
using namespace Pylon;

// Namespace for using cout.
using namespace std;

// Number of images to be grabbed.
static const uint32_t c_countOfImagesToGrab = 10;

// Limits the amount of cameras used for grabbing.
// It is important to manage the available bandwidth when grabbing with multiple cameras.
// This applies, for instance, if two GigE cameras are connected to the same network adapter via a switch.
// To manage the bandwidth, the GevSCPD interpacket delay parameter and the GevSCFTD transmission delay
// parameter can be set for each GigE camera device.
// The "Controlling Packet Transmission Timing with the Interpacket and Frame Transmission Delays on Basler GigE Vision Cameras"
// Application Notes (AW000649xx000)
// provide more information about this topic.
// The bandwidth used by a FireWire camera device can be limited by adjusting the packet size.
static const size_t c_maxCamerasToUse = 2;

int main(int argc, char* argv[])
{
    // The exit code of the sample application.
    int exitCode = 0;

    // Automagically call PylonInitialize and PylonTerminate to ensure the pylon runtime system.
    // is initialized during the lifetime of this object
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
        cameras.StartGrabbing();

        // This smart pointer will receive the grab result data.
        CGrabResultPtr ptrGrabResult;

        // Grab c_countOfImagesToGrab from the cameras.
        for( int i = 0; i < c_countOfImagesToGrab && cameras.IsGrabbing(); ++i)
        {
            cameras.RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);

            // When the cameras in the array are created the camera context value
            // is set to the index of the camera in the array.
            // The camera context is a user settable value.
            // This value is attached to each grab result and can be used
            // to determine the camera that produced the grab result.
            intptr_t cameraContextValue = ptrGrabResult->GetCameraContext();

//#ifdef PYLON_WIN_BUILD
            // Show the image acquired by each camera in the window related to each camera.
            Pylon::DisplayImage(cameraContextValue, ptrGrabResult);
//#endif

            // Print the index and the model name of the camera.
            cout << "Camera " <<  cameraContextValue << ": " << cameras[ cameraContextValue ].GetDeviceInfo().GetModelName() << endl;

            // Now, the image data can be processed.
            cout << "GrabSucceeded: " << ptrGrabResult->GrabSucceeded() << endl;
            cout << "SizeX: " << ptrGrabResult->GetWidth() << endl;
            cout << "SizeY: " << ptrGrabResult->GetHeight() << endl;
            const uint8_t *pImageBuffer = (uint8_t *) ptrGrabResult->GetBuffer();
            cout << "Gray value of first pixel: " << (uint32_t) pImageBuffer[0] << endl << endl;
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
    while( cin.get() != '\n');

    return exitCode;
}


#else
// We can use cassert. Asserts will disable with NDEBUG
//

#include <algorithm>
#include <iostream>
#include <cstring>
#include <cassert>
#include <vector>
#include <string>
#include "analysis_system.h"
#include "sys_camera_grabber.h"
#include "sys_file_frame_grabber.h"
#include "utils.h"
#include "ToolsForTesting/testing.h"

void help() {
  INFO("TODO: print an elaborate help message");
}

// Redundant!
//
std::vector<std::string> parseFilenames(int argc, char* argv[]) {
  std::vector<std::string> filenames;
  // Parse at most 4 videos.
  //
  for (int i = 0; i < argc && i < 4; ++i) {
    filenames.push_back(argv[i]);
    INFO("Parsed filename " << filenames.back());
  }
  return filenames;
}

// Modes: from file, from cameras(only support 4 camera mode?).
//
void execute() {
  if (g_args.count("calibrate")) {
    // This should be ran only occasionaly.
    //
    INFO("Begin calibration of the camera from video file. Display some output!");
  } else if (!g_args.count("fin")) {
    // This mode should do real-time video capture, track the
    // score and display a 2d model of the court.
    //
    INFO("Begin capturing from cameras");
    SystemCameraGrabber camera_grab;
    camera_grab.startGrabbing();
    RunOfflineSystem(&camera_grab);
//    RunOnlineSystem(&camera_grab);
  } else {
    // Main mode for use during development. Read from files. Require 4 files for start.
    // Allow for user interaction (pause, real-time replay, jump by frames etc.
    // First implement jumping by frames.
    //
    INFO("Analysis from file");
    if (g_args["fin"].size() < 4 && false) {
      INFO("File analysis requires at least 4 video files.  Got " << g_args["fin"].size());
      for (const auto &s : g_args["fin"]) {
        INFO(s);
      }
      help();
      return;
    }
    SystemFileFrameGrabber grabber(g_args["fin"]);
    INFO("Grabber initialised");
    RunOfflineSystem(&grabber);
  }
}

int main(int argc, char *argv[]) {
  printf("%d\n%d\n", CV_MAJOR_VERSION, CV_MINOR_VERSION);
  ParseArguments(argc, argv);
  if (g_args.count("test") > 0) {
    run_tests();
    return 0;
  }
  execute();
  return 0;
}

#endif
