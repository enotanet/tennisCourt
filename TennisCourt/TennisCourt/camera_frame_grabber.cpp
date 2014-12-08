#include "camera_frame_grabber.h"
#include <opencv2/core/core.hpp>
#include <pylon/PylonIncludes.h>
#include "utils.h"
#include <thread>
#include <mutex>
#include <iostream>

CameraFrameGrabber::CameraFrameGrabber(int camera_id) {
  // Exceptions in constructor?
  //std::lock_guard<std::mutex> lock(pylonFactoryLock);
  //Pylon::PylonInitialize();

  Pylon::DeviceInfoList_t dl;
	Pylon::CTlFactory::GetInstance().EnumerateDevices(dl);
  std::cerr << dl.size() << " cameras found while opening camera " << camera_id << "\n";
  camera = new Pylon::CInstantCamera(Pylon::CTlFactory::GetInstance().CreateDevice(dl[camera_id]));
  std::cerr << "Using device " << camera->GetDeviceInfo().GetModelName() << std::endl;
  
  camera->Open();
  camera->StartGrabbing();
  fc.OutputPixelFormat = Pylon::PixelType_RGB8packed;
  width = GenApi::CIntegerPtr(camera->GetNodeMap().GetNode("Width"));
  height = GenApi::CIntegerPtr(camera->GetNodeMap().GetNode("Height"));
  std::cerr << width->GetValue() << " " << height->GetValue() << std::endl;
}

CameraFrameGrabber::~CameraFrameGrabber() {
  delete camera;
  //Pylon::PylonTerminate();
}

// Returns false on failure.
bool CameraFrameGrabber::getNextFrame(cv::Mat *res) {
  if (!camera->IsGrabbing()) {
    return 0;
  }
  try {
    camera->RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
		if (ptrGrabResult->GrabSucceeded()) {
      fc.Convert(image, ptrGrabResult);
			*res = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t*) image.GetBuffer());
		} else {
      return false;
    }
  }
  catch (GenICam::GenericException &e) {
    std::cerr << "An exception occurred." << std::endl
	  << e.GetDescription() << std::endl;
    return 0;
  }
  return 1;
}

cv::Mat CameraFrameGrabber::getMat() {
  std::cerr << width->GetValue() << " " << height->GetValue() << std::endl;
  return cv::Mat(height->GetValue(), width->GetValue(), CV_8UC3);
}

//std::mutex CameraFrameGrabber::pylonFactoryLock;
