#include "Model.hpp"

namespace rmcv::detect
{
    std::vector<BoundingBox> Model::operator()(cv::Mat img)
    {
        return model_(img);
    }
}