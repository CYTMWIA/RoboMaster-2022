#ifndef __IMAGE_HPP__
#define __IMAGE_HPP__

#include "base.hpp"

namespace rmcv::capture
{
    class ImageCapturer: public BaseCapturer
    {
    private:
        Path path_;
    public:
        ImageCapturer() = delete;

        /**
         * @brief ImageCapturer 构造函数
         * 
         * @param path 路径
         */
        ImageCapturer(Path path);

        cv::Mat next();
    };    
}


#endif