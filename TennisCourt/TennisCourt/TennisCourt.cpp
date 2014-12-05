// We can use cassert. Asserts will disable with NDEBUG
//

#include <iostream>
#include <cstring>
#include <cassert>
#include "utils.h"
#include "ToolsForTesting/testing.h"

void help() {
  INFO("TODO: print an elaborate help message");
}

// Modes: from file, from cameras(only support 4 camera mode?).
//
void execute(int argc, char *argv[]) {
  if (argc >= 1 && !strcmp(argv[0], "cameras")) {
    // This mode should do real-time video capture, track the
    // score and display a 2d model of the court.
    //
    INFO("Begin capturing from cameras");
  } else if (argc >= 1 && !strcmp(argv[0], "calibrate")) {
    // This should be ran only occasionaly.
    //
    INFO("Begin calibration of the camera from video file. Display some output!");
    if (argc <= 1) {
      help();
      return;
    }
  } else {
    // Main mode for use during development. Read from files. Require 4 files for start.
    // Allow for user interaction (pause, real-time replay, jump by frames etc.
    // First implement jumping by frames.
    //
    INFO("Analysis from file");
  }
}

int main(int argc, char *argv[]) {
  if (argc > 1 && !strcmp(argv[1], "test")) {
    run_tests(argc - 2, argv + 2);
    return 0;
  }
  printf("%d\n", argc);
  execute(argc - 1, argv + 1);
  return 0;
}