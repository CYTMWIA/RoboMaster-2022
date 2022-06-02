#include <fmt/core.h>

#include <filesystem>
#include <ie_core.hpp>
#include <iostream>

#include "rm_common/logging.hpp"
#include "rm_detect/openvino/vino_model.hpp"
#include "vino_util.hpp"

namespace rm_detect
{
struct GridAndStride
{
  int grid0;
  int grid1;
  int stride;
};

void generate_grids_and_stride(const int target_w, const int target_h, std::vector<int> &strides,
                               std::vector<GridAndStride> &grid_strides)
{
  for (auto stride : strides)
  {
    int num_grid_w = target_w / stride;
    int num_grid_h = target_h / stride;
    for (int g1 = 0; g1 < num_grid_h; g1++)
    {
      for (int g0 = 0; g0 < num_grid_w; g0++)
      {
        grid_strides.push_back((GridAndStride){g0, g1, stride});
      }
    }
  }
}

void generate_yolox_proposals(const std::vector<GridAndStride> &grid_strides, const float *feat_ptr,
                              float prob_threshold, std::vector<BoundingBox> &bboxes)
{
  const int num_anchors = grid_strides.size();

  for (int anchor_idx = 0; anchor_idx < num_anchors; anchor_idx++)
  {
    const int grid0 = grid_strides[anchor_idx].grid0;
    const int grid1 = grid_strides[anchor_idx].grid1;
    const int stride = grid_strides[anchor_idx].stride;

    const int basic_pos = anchor_idx * (MODEL_NUM_CLASSES + 5);

    // yolox/models/yolo_head.py decode logic
    //  outputs[..., :2] = (outputs[..., :2] + grids) * strides
    //  outputs[..., 2:4] = torch.exp(outputs[..., 2:4]) * strides
    float x_center = (feat_ptr[basic_pos + 0] + grid0) * stride;
    float y_center = (feat_ptr[basic_pos + 1] + grid1) * stride;
    float w_half = exp(feat_ptr[basic_pos + 2]) * stride / 2.0;
    float h_half = exp(feat_ptr[basic_pos + 3]) * stride / 2.0;

    float box_objectness = feat_ptr[basic_pos + 4];
    for (int class_idx = 0; class_idx < MODEL_NUM_CLASSES; class_idx++)
    {
      float box_cls_score = feat_ptr[basic_pos + 5 + class_idx];
      float box_prob = box_objectness * box_cls_score;
      if (box_prob > prob_threshold)
      {
        BoundingBox bbox;

        bbox.color_id = class_idx / 9;
        bbox.tag_id = class_idx % 9;
        bbox.confidence = box_prob;
        bbox.pts[0] = cv::Point2f(x_center - w_half, y_center - h_half);  // 在NMS中有用
        bbox.pts[2] = cv::Point2f(x_center + w_half, y_center + h_half);
        // bbox.pts[1] = cv::Point2f(x_center - w_half, y_center + h_half); //
        // 之后会被覆盖，所以这里就不填写了 bbox.pts[3] = cv::Point2f(x_center + w_half, y_center -
        // h_half); // 同上

        bboxes.push_back(bbox);
      }

    }  // class loop

  }  // point anchor loop
}

inline float intersection_area(const BoundingBox &a, const BoundingBox &b)
{
  float xx1 = (std::max)(a.pts[0].x, b.pts[0].x);
  float yy1 = (std::max)(a.pts[0].y, b.pts[0].y);
  float xx2 = (std::min)(a.pts[2].x, b.pts[2].x);
  float yy2 = (std::min)(a.pts[2].y, b.pts[2].y);
  float w = (std::max)(float(0), xx2 - xx1 + 1);
  float h = (std::max)(float(0), yy2 - yy1 + 1);
  return w * h;
}

void nms_sorted_bboxes(const std::vector<BoundingBox> &bboxes, std::vector<int> &picked,
                       float nms_threshold)
{
  picked.clear();

  const int n = bboxes.size();

  std::vector<float> areas(n);
  for (int i = 0; i < n; i++)
  {
    areas[i] =
        (bboxes[i].pts[2].x - bboxes[i].pts[0].x) * (bboxes[i].pts[2].y - bboxes[i].pts[0].y);
  }

  for (int i = 0; i < n; i++)
  {
    const BoundingBox &a = bboxes[i];

    int keep = 1;
    for (int j = 0; j < (int)picked.size(); j++)
    {
      const BoundingBox &b = bboxes[picked[j]];

      // intersection over union
      float inter_area = intersection_area(a, b);
      float union_area = areas[i] + areas[picked[j]] - inter_area;
      // float IoU = inter_area / union_area
      if (inter_area / union_area > nms_threshold) keep = 0;
    }

    if (keep) picked.push_back(i);
  }
}

void decode_outputs(const float *prob, std::vector<BoundingBox> &bboxes, float scale,
                    const int img_w, const int img_h)
{
  std::vector<BoundingBox> proposals;
  std::vector<int> strides = {8, 16, 32};
  std::vector<GridAndStride> grid_strides;

  generate_grids_and_stride(MODEL_INPUT_W, MODEL_INPUT_H, strides, grid_strides);
  generate_yolox_proposals(grid_strides, prob, MODEL_BBOX_CONF_THRESH, proposals);
  std::sort(proposals.begin(), proposals.end(),
            [](BoundingBox a, BoundingBox b) { return a.confidence > b.confidence; });

  std::vector<int> picked;
  nms_sorted_bboxes(proposals, picked, MODEL_NMS_THRESH);
  int count = picked.size();
  bboxes.resize(count);

  for (int i = 0; i < count; i++)
  {
    bboxes[i] = proposals[picked[i]];

    // adjust offset to original unpadded
    float x0 = (bboxes[i].pts[0].x) / scale;
    float y0 = (bboxes[i].pts[0].y) / scale;
    float x1 = (bboxes[i].pts[2].x) / scale;
    float y1 = (bboxes[i].pts[2].y) / scale;

    // clip
    x0 = std::max(std::min(x0, (float)(img_w - 1)), 0.f);
    y0 = std::max(std::min(y0, (float)(img_h - 1)), 0.f);
    x1 = std::max(std::min(x1, (float)(img_w - 1)), 0.f);
    y1 = std::max(std::min(y1, (float)(img_h - 1)), 0.f);

    bboxes[i].pts[0] = cv::Point2f(x0, y0);
    bboxes[i].pts[1] = cv::Point2f(x0, y1);
    bboxes[i].pts[2] = cv::Point2f(x1, y1);
    bboxes[i].pts[3] = cv::Point2f(x1, y0);
  }
}

class VinoModel::Impl
{
 private:
  InferenceEngine::Core core_;
  InferenceEngine::ExecutableNetwork network_;
  InferenceEngine::InferRequest infer_request_;
  std::string input_name_ = "data";
  std::string output_name_ = "output";

 public:
  Impl(std::string xml_path, std::string bin_path)
  {
    auto cache_dir = std::filesystem::path(xml_path).parent_path() / "openvino_cache";
    core_.SetConfig({{CONFIG_KEY(CACHE_DIR), cache_dir}});

    std::vector<std::string> devices = core_.GetAvailableDevices();
    __LOG_INFO("Available Devices: {}", fmt::join(devices, ", "));

    // 读取网络、设置输入输出
    InferenceEngine::CNNNetwork model = core_.ReadNetwork(xml_path);
    // 输入
    InferenceEngine::InputsDataMap inputs_map(model.getInputsInfo());
    input_name_ = inputs_map.begin()->first;
    // 输出
    InferenceEngine::OutputsDataMap outputs_map(model.getOutputsInfo());
    output_name_ = outputs_map.begin()->first;
    for (auto &output_info : outputs_map)
    {
      output_info.second->setPrecision(InferenceEngine::Precision::FP32);
    }

    // 加载网络
    network_ = core_.LoadNetwork(xml_path, "CPU");
    infer_request_ = network_.CreateInferRequest();
  }

  std::vector<BoundingBox> operator()(const cv::Mat &img)
  {
    // 预处理
    cv::Mat input_img = resize_image(img, MODEL_INPUT_W, MODEL_INPUT_H);

    InferenceEngine::Blob::Ptr input_blob = infer_request_.GetBlob(input_name_);
    image2blob(input_img, input_blob);

    // 推理
    infer_request_.Infer();

    // 后处理
    const InferenceEngine::Blob::Ptr output_blob = infer_request_.GetBlob(output_name_);
    InferenceEngine::MemoryBlob::CPtr moutput = as<InferenceEngine::MemoryBlob>(output_blob);
    auto moutputHolder = moutput->rmap();
    const float *net_pred = moutputHolder.as<
        const InferenceEngine::PrecisionTrait<InferenceEngine::Precision::FP32>::value_type *>();

    float scale = std::min(MODEL_INPUT_W / (img.cols * 1.0), MODEL_INPUT_H / (img.rows * 1.0));
    std::vector<BoundingBox> bboxes;
    decode_outputs(net_pred, bboxes, scale, img.cols, img.rows);

    return bboxes;
  }
};

VinoModel::VinoModel(std::string xml_path, std::string bin_path)
    : pimpl_(std::make_unique<Impl>(xml_path, bin_path))
{
}
VinoModel::~VinoModel() = default;

std::vector<BoundingBox> VinoModel::operator()(const cv::Mat &img)
{
  return pimpl_->operator()(img);
}
}  // namespace rm_detect