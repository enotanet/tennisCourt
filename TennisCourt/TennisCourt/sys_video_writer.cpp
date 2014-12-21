#include "sys_video_writer.h"

#include <string>
#include <vector>
#include "utils.h"

SystemVideoWriter::SystemVideoWriter(const std::vector<std::string> &p_outfiles, const cv::Size &p_frame_size) : file_id(0), frames_this_file(0), frame_size(p_frame_size), outfiles(p_outfiles) {
  GenerateWriters();
}

std::string SystemVideoWriter::GenerateFilename(size_t v_id) {
  char buf[MAX_FILELENGTH];
#ifdef WIN32
  sprintf_s(buf, "%s%d.avi", outfiles[v_id], file_id);
#else
  sprintf(buf, "%s%d.avi", outfiles[v_id].c_str(), file_id);
#endif
  return buf;
}

void SystemVideoWriter::GenerateWriters() {
  for (size_t i = 0; i < outfiles.size(); ++i) {
    std::string fname = GenerateFilename(i);
    writers.emplace_back(fname, CV_FOURCC('D', 'I', 'B', ' '), 60, frame_size, false);
  }
  ++file_id;
}

bool SystemVideoWriter::WriteFrames(const std::vector<cv::Mat> &frames) {
  for (size_t i = 0; i < frames.size(); ++i) {
    writers[i].write(frames[i]);
  }
  ++frames_this_file;
  if (frames_this_file >= MAX_FRAMES_PER_FILE) {
    frames_this_file = 0;
    writers.clear();
    GenerateWriters();
    DEBUG("Opening new files for writing!");
  }
  return true;
}
