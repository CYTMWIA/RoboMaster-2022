#include "image.hpp"

namespace rmcv::capture
{
    ImageCapturer::ImageCapturer(Path path):
        path_(path)
    {}

    cv::Mat ImageCapturer::next()
    {
        return cv::imread(path_);
    }
}
