#include "rm_capture/image_capture.hpp"

namespace rm_capture
{
ImageCapture::ImageCapture(Path path) : path_(path) {}

cv::Mat ImageCapture::next() { return cv::imread(path_); }
}  // namespace rm_capture
