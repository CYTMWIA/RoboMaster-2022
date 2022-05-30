#include "ImageCapture.hpp"

namespace rmcv::capture
{
    ImageCapture::ImageCapture(Path path):
        path_(path)
    {}

    cv::Mat ImageCapture::next()
    {
        return cv::imread(path_);
    }
}
