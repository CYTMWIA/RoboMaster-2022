#ifndef __DETECTOR_UTILS_HPP__
#define __DETECTOR_UTILS_HPP__

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

namespace rm_autoaim
{

template <typename T, typename Pred>
void filter_emplace(std::vector<T>& vec, Pred pred_keep)
{
  vec.erase(std::remove_if(vec.begin(), vec.end(), [pred_keep](T& e) { return !pred_keep(e); }),
            vec.end());
}

template <typename T, typename Pred>
std::vector<T> filter_copy(const std::vector<T>& vec, Pred pred_keep)
{
  std::vector<T> new_vec;
  std::copy_if(vec.begin(), vec.end(), std::back_inserter(new_vec), pred_keep);
  return std::move(new_vec);
}

inline cv::Mat to_binary(const cv::Mat& src, double thresh)
{
  cv::Mat bin;
  cv::cvtColor(src, bin, cv::COLOR_BGR2GRAY);
  cv::threshold(bin, bin, thresh, 255, cv::THRESH_BINARY);
  return bin;
}

}  // namespace rm_autoaim

#endif