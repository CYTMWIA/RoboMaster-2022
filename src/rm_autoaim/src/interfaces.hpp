#ifndef __INTERFACES_HPP__
#define __INTERFACES_HPP__

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

#include "nerv/nerv.hpp"

namespace rm_autoaim
{
enum Color
{
  COLOR_BLUE,
  COLOR_RED,
  _COLOR_ENEMY,
};

struct Armor
{
  cv::Point2f pts[4];  // [pt0, pt1, pt2, pt3] 左上 左下 右下 右上
  int type;
  Color color;
  int number;
};

struct PowerRune
{
  cv::Point2f armor_pts[4];
  cv::Point2f armor_center;
  cv::Point2f logo;
  bool activating;
};

template <typename T>
struct DetectResult
{
  nerv::interfaces::Timestamp image_timestamp;
  nerv::interfaces::Rpy image_robot_rpy;
  std::vector<T> content;
};

}  // namespace rm_autoaim

#endif