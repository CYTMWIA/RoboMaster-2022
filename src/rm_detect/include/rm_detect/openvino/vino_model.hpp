#ifndef __RM_DETECT_OPENVINO_VINO_MODEL_HPP__
#define __RM_DETECT_OPENVINO_VINO_MODEL_HPP__

#include <memory>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "../bounding_box.hpp"

#define MODEL_NUM_CLASSES 18
#define MODEL_INPUT_W 416
#define MODEL_INPUT_H 416

#define MODEL_NMS_THRESH 0.3
#define MODEL_BBOX_CONF_THRESH 0.5

namespace rm_detect
{
class VinoModel
{
 private:
  class Impl;
  std::unique_ptr<Impl> pimpl_;

 public:
  VinoModel() = delete;
  ~VinoModel();
  /**
   * @brief VinoModel 构造函数
   *
   * @param xml_path xml 文件路径
   * @param bin_path bin 文件路径（已弃用）
   */
  VinoModel(std::string xml_path, std::string bin_path = "deprecated");

  /**
   * @brief 推理
   *
   * @param img 图像
   * @return std::vector<BoundingBox> 检测结果
   */
  std::vector<BoundingBox> operator()(const cv::Mat &img);
};
}  // namespace rm_detect

#endif