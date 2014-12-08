// Primary entry point for testing
//

#include "testing.h"
#include "../sys_frame_grabber.h"
#include "../sys_camera_grabber.h"
#include "../sys_file_frame_grabber.h"
#include "../sys_video_writer.h"
#include "../utils.h"

#include <ctime>
#include <cassert>
#include <memory>

void run_tests() {
  INFO("Running tests!");
  if (g_args.count("fout")) {
    INFO("Testing reading and writing of video");
    SystemFrameGrabber *grabber;
    if (g_args.count("fin")) {
      assert(g_args["fin"].size() == g_args["fout"].size());
      assert(g_args["fin"].size());
      grabber = new SystemFileFrameGrabber(g_args["fin"]);
    } else {
      DEBUG("Opening system cameras");
      SystemCameraGrabber *camgrab = new SystemCameraGrabber();
      camgrab->startGrabbing();
      grabber = camgrab;
    }
    auto container = grabber->getContainer();

    assert(container.size() == g_args["fout"].size());
    SystemVideoWriter videoWrite(g_args["fout"], container[0].size());
    
    long long start = clock();
    // 10 minutes!
    long long slice = start;
    long long frames = 0;
    while (clock() - start < 60LL* CLOCKS_PER_SEC) {
      if (!grabber->getNextFrames(&container)) {
        if (g_args.count("fin")) {
          INFO("Finished files. Reopen them");
          grabber = new SystemFileFrameGrabber(g_args["fin"]);
          grabber->getNextFrames(&container);
        }
      }

      videoWrite.WriteFrames(container);
      ++frames;
      if (clock() - slice > 10 * CLOCKS_PER_SEC) {
        INFO("Written " << frames << " for the past 10 sec @ " << (double) frames / (double) (clock() - slice) * CLOCKS_PER_SEC);
        slice = clock();
        frames = 0;
      }
    }
    delete grabber;
  }
}
