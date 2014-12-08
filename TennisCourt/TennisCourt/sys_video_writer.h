#ifndef SYS_VIDEO_WRITER_H__
#define SYS_VIDEO_WRITER_H__

#include <vector>
#include <string>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>

// Apply no compression! Handle opening and closing of multiple files.
// Might want to introduce artificial delays so that we don't write > 60 fps
// Provide file names without .avi, on correct drive. Will append an additional
// id as we might need multiple files.
//
class SystemVideoWriter {
public:
  SystemVideoWriter(const std::vector<std::string> &p_outfiles, const cv::Size &frame_size);
  bool WriteFrames(const std::vector<cv::Mat> &frames);

private:
  std::string GenerateFilename(size_t v_id);
  void SystemVideoWriter::GenerateWriters();

  int file_id;
  int frames_this_file;
  cv::Size frame_size;
  std::vector<std::string> outfiles;
  std::vector<cv::VideoWriter> writers;

  static const int MAX_FILELENGTH = 128;
  static const int MAX_FRAMES_PER_FILE = 20 * 60;
};

#endif  // SYS_VIDEO_WRITER_H__