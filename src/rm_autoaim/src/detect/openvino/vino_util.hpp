#ifndef __DETECT_OPENVINO_VINO_UTIL_HPP__
#define __DETECT_OPENVINO_VINO_UTIL_HPP__

#include <ie_core.hpp>
#include <opencv2/core.hpp>

namespace rm_autoaim
{
/**
 * @brief 缩放图像（保持比例，空白部分填充为cv::Scalar(114, 114, 114)）
 *
 * @param img 输入图像
 * @param width 目标宽度
 * @param height 目标高度
 * @return cv::Mat
 */
cv::Mat resize_image(const cv::Mat &img, int width, int height);

void image2blob(const cv::Mat &image, InferenceEngine::Blob::Ptr &blob);
}  // namespace rm_autoaim

#endif