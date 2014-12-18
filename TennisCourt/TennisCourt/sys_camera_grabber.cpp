#ifdef PYLON_INSTALLED

#include "sys_camera_grabber.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <cstdio>

#include "utils.h"

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

const int MAX_FRAMES_TO_KEEP = 5720;

bool SystemCameraGrabber::startGrabbing() {
  try {
    // Get the transport layer factory.
    CTlFactory& tlFactory = CTlFactory::GetInstance();

    // Get all attached devices and exit application if no device is found.
    DeviceInfoList_t devices;
    if (tlFactory.EnumerateDevices(devices) == 0) {
      throw RUNTIME_EXCEPTION("No camera present.");
    }

    // Create an array of instant cameras for the found devices and avoid exceeding a maximum number of devices.
    cameras.Initialize(min(devices.size(), c_maxCamerasToUse));

    // Create and attach all Pylon Devices.
    for ( size_t i = 0; i < cameras.GetSize(); ++i) {
      cameras[i].Attach(tlFactory.CreateDevice(devices[i]));

      // Print the model name of the camera.
      DEBUG("Using device " << cameras[ i ].GetDeviceInfo().GetModelName());
    }

    // Starts grabbing for all cameras starting with index 0. The grabbing
    // is started for one camera after the other. That's why the images of all
    // cameras are not taken at the same time.
    // However, a hardware trigger setup can be used to cause all cameras to grab images synchronously.
    // According to their default configuration, the cameras are
    // set up for free-running continuous acquisition.
    cameras.StartGrabbing(GrabStrategy_LatestImageOnly);

    fc.OutputPixelFormat = Pylon::PixelType_Mono8;
  } catch (GenICam::GenericException &e) {
      // Error handling
      cerr << "An exception occurred." << endl
      << e.GetDescription() << endl;
      return false;
  }
  return true;
}

std::vector<cv::Mat> SystemCameraGrabber::getContainer() {
  if (!cameras.GetSize()) {
    return std::vector<cv::Mat>();
  }
  std::vector<cv::Mat> container;
  for (size_t i = 0; i < cameras.GetSize(); ++i) {
    int width = (int) GenApi::CIntegerPtr(cameras[i].GetNodeMap().GetNode("Width"))->GetValue();
    int height = (int) GenApi::CIntegerPtr(cameras[i].GetNodeMap().GetNode("Height"))->GetValue();
    DEBUG("Camera " << i << " shoots at " << height << "x" << width);

    // Monochrome!
    container.push_back(cv::Mat(height, width, CV_8UC1));
  }
  return container;
}

bool SystemCameraGrabber::getNextFrames(std::vector<cv::Mat> *res) {
  //DEBUG("Get next frames!!");
  // This smart pointer will receive the grab result data.
  // Make sure this works with not 4 cameras!
  CGrabResultPtr ptrGrabResult[c_maxCamerasToUse];

  // Retrieve images from all cameras.
  const int DefaultTimeout_ms = 5000;
  for (size_t i = 0; i < cameras.GetSize() && cameras.IsGrabbing(); ++i) {
    // CInstantCameraArray::RetrieveResult will return grab results in the order they arrive.
    if (!cameras.RetrieveResult(DefaultTimeout_ms, ptrGrabResult[i], TimeoutHandling_Return)) {
      DEBUG("Grab failed! " << cameras.GetSize());
      return false;
    }

    // When the cameras in the array are created the camera context value
    // is set to the index of the camera in the array.
    // The camera context is a user-settable value.
    // This value is attached to each grab result and can be used
    // to determine the camera that produced the grab result.
    intptr_t cameraIndex = ptrGrabResult[i]->GetCameraContext();
    //DEBUG("Grabbed image from " << cameraIndex << " I " << i);

    // Image grabbed successfully?
    if (ptrGrabResult[i]->GrabSucceeded()) {
      // Do the conversions in parallel?
      //
      fc.Convert(img, ptrGrabResult[i]);
      // Hard coded dimensions. Don't like!
      //
      //DEBUG("Put frame at " << cameraIndex);
      (*res)[cameraIndex] = cv::Mat(1024, 1280, CV_8UC1, (uint8_t*) img.GetBuffer()).clone();
    } else {
      // If a buffer has been incompletely grabbed, the network bandwidth is possibly insufficient for transferring
      // multiple images simultaneously. See note above c_maxCamerasToUse.
      DEBUG("Error: " << ptrGrabResult[i]->GetErrorCode() << " " << ptrGrabResult[i]->GetErrorDescription());
      return false;
    }
  }
  return true;
}

SystemCameraGrabber::~SystemCameraGrabber() {
  DEBUG("Entering destructor");
  cameras.StopGrabbing();

  // Close all cameras.
  cameras.Close();
  DEBUG("Exiting destructor");
}

#endif  // PYLON_INSTALLED