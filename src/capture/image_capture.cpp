#include "image_capture.hpp"

namespace rmcv::capture
{
ImageCapture::ImageCapture(Path path) : path_(path) {}

cv::Mat ImageCapture::next() { return cv::imread(path_); }
}  // namespace rmcv::capture
