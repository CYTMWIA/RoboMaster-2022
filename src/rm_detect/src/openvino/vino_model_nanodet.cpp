#include <fmt/core.h>

#include <filesystem>
#include <ie_core.hpp>
#include <iostream>

#include "rm_common/logging.hpp"
#include "vino_model.hpp"
#include "vino_util.hpp"

namespace rm_detect
{
struct CenterPrior
{
  int x;
  int y;
  int stride;
};

inline float fast_exp(float x)
{
  union
  {
    uint32_t i;
    float f;
  } v{};
  v.i = (1 << 23) * (1.4426950409 * x + 126.93490512f);
  return v.f;
}

inline float sigmoid(float x) { return 1.0f / (1.0f + fast_exp(-x)); }

template <typename _Tp>
int activation_function_softmax(const _Tp *src, _Tp *dst, int length)
{
  const _Tp alpha = *std::max_element(src, src + length);
  _Tp denominator{0};

  for (int i = 0; i < length; ++i)
  {
    dst[i] = fast_exp(src[i] - alpha);
    denominator += dst[i];
  }

  for (int i = 0; i < length; ++i)
  {
    dst[i] /= denominator;
  }

  return 0;
}

void generate_grid_center_priors(const int input_height, const int input_width,
                                 std::vector<int> &strides, std::vector<CenterPrior> &center_priors)
{
  for (int i = 0; i < (int)strides.size(); i++)
  {
    int stride = strides[i];
    int feat_w = ceil((float)input_width / stride);
    int feat_h = ceil((float)input_height / stride);
    for (int y = 0; y < feat_h; y++)
    {
      for (int x = 0; x < feat_w; x++)
      {
        CenterPrior ct;
        ct.x = x;
        ct.y = y;
        ct.stride = stride;
        center_priors.push_back(ct);
      }
    }
  }
}

void nms(std::vector<BoundingBox> &input_boxes, float NMS_THRESH)
{
  std::sort(input_boxes.begin(), input_boxes.end(),
            [](BoundingBox a, BoundingBox b) { return a.confidence > b.confidence; });
  std::vector<float> vArea(input_boxes.size());
  for (int i = 0; i < int(input_boxes.size()); ++i)
  {
    vArea[i] = (input_boxes.at(i).pts[2].x - input_boxes.at(i).pts[0].x + 1) *
               (input_boxes.at(i).pts[2].y - input_boxes.at(i).pts[0].y + 1);
  }
  for (int i = 0; i < int(input_boxes.size()); ++i)
  {
    for (int j = i + 1; j < int(input_boxes.size());)
    {
      float xx1 = (std::max)(input_boxes[i].pts[0].x, input_boxes[j].pts[0].x);
      float yy1 = (std::max)(input_boxes[i].pts[0].y, input_boxes[j].pts[0].y);
      float xx2 = (std::min)(input_boxes[i].pts[2].x, input_boxes[j].pts[2].x);
      float yy2 = (std::min)(input_boxes[i].pts[2].y, input_boxes[j].pts[2].y);
      float w = (std::max)(float(0), xx2 - xx1 + 1);
      float h = (std::max)(float(0), yy2 - yy1 + 1);
      float inter = w * h;
      float ovr = inter / (vArea[i] + vArea[j] - inter);
      if (ovr >= NMS_THRESH)
      {
        input_boxes.erase(input_boxes.begin() + j);
        vArea.erase(vArea.begin() + j);
      }
      else
      {
        j++;
      }
    }
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

  // modify these parameters to the same with your config if you want to use your own model
  int input_size_[2] = {416, 416};              // input height and width
  int num_class_ = 36;                          // number of classes. 80 for COCO
  int reg_max_ = 7;                             // `reg_max` set in the training config. Default: 7.
  std::vector<int> strides_ = {8, 16, 32, 64};  // strides of the multi-level feature.

  float confidence_threshold_ = 0.5;
  float nms_threshold_ = 0.5;

  BoundingBox disPred2Bbox(const float *&dfl_det, int label, float score, int x, int y, int stride)
  {
    float ct_x = x * stride;
    float ct_y = y * stride;
    std::vector<float> dis_pred;
    dis_pred.resize(4);
    for (int i = 0; i < 4; i++)
    {
      float dis = 0;
      float *dis_after_sm = new float[reg_max_ + 1];
      activation_function_softmax(dfl_det + i * (reg_max_ + 1), dis_after_sm, reg_max_ + 1);
      for (int j = 0; j < reg_max_ + 1; j++)
      {
        dis += j * dis_after_sm[j];
      }
      dis *= stride;
      // std::cout << "dis:" << dis << std::endl;
      dis_pred[i] = dis;
      delete[] dis_after_sm;
    }
    float xmin = ct_x - dis_pred[0];
    float ymin = ct_y - dis_pred[1];
    float xmax = ct_x + dis_pred[2];
    float ymax = ct_y + dis_pred[3];

    BoundingBox bbox;
    bbox.pts[0] = cv::Point2f(xmin, ymin);
    bbox.pts[1] = cv::Point2f(xmin, ymax);
    bbox.pts[2] = cv::Point2f(xmax, ymax);
    bbox.pts[3] = cv::Point2f(xmax, ymin);
    bbox.confidence = score;
    bbox.color_id = label / 9;
    bbox.tag_id = label % 9;
    return bbox;
  }

  void decode_infer(const float *&pred, std::vector<CenterPrior> &center_priors, float threshold,
                    std::vector<std::vector<BoundingBox>> &results)
  {
    const int num_points = center_priors.size();
    const int num_channels = num_class_ + (reg_max_ + 1) * 4;
    // printf("num_points:%d\n", num_points);

    // cv::Mat debug_heatmap = cv::Mat::zeros(feature_h, feature_w, CV_8UC3);
    for (int idx = 0; idx < num_points; idx++)
    {
      const int ct_x = center_priors[idx].x;
      const int ct_y = center_priors[idx].y;
      const int stride = center_priors[idx].stride;

      float score = 0;
      int cur_label = 0;

      for (int label = 0; label < num_class_; label++)
      {
        if (pred[idx * num_channels + label] > score)
        {
          score = pred[idx * num_channels + label];
          cur_label = label;
        }
      }
      if (score > threshold)
      {
        // std::cout << row << "," << col <<" label:" << cur_label << " score:" << score <<
        // std::endl;
        const float *bbox_pred = pred + idx * num_channels + num_class_;
        auto bbox = this->disPred2Bbox(bbox_pred, cur_label, score, ct_x, ct_y, stride);

        bool nan_flag = false;
        for (int i = 0; i < 4; i++)
          if (bbox.pts[i].x != bbox.pts[i].x || bbox.pts[i].y != bbox.pts[i].y)
          {
            nan_flag = true;
            break;
          }
        if (nan_flag) continue;

        results[cur_label].push_back(std::move(bbox));
        // debug_heatmap.at<cv::Vec3b>(row, col)[0] = 255;
        // cv::imshow("debug", debug_heatmap);
      }
    }
  }

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
    cv::Mat input_img;
    cv::resize(img, input_img, cv::Size(input_size_[0], input_size_[1]));

    InferenceEngine::Blob::Ptr input_blob = infer_request_.GetBlob(input_name_);
    image2blob(input_img, input_blob);

    // 推理
    infer_request_.Infer();

    // 后处理
    std::vector<std::vector<BoundingBox>> results;
    results.resize(this->num_class_);

    {
      const InferenceEngine::Blob::Ptr pred_blob = infer_request_.GetBlob(output_name_);

      auto m_pred = InferenceEngine::as<InferenceEngine::MemoryBlob>(pred_blob);
      auto m_pred_holder = m_pred->rmap();
      const float *pred = m_pred_holder.as<const float *>();

      // generate center priors in format of (x, y, stride)
      std::vector<CenterPrior> center_priors;
      generate_grid_center_priors(this->input_size_[0], this->input_size_[1], this->strides_,
                                  center_priors);

      this->decode_infer(pred, center_priors, confidence_threshold_, results);
    }

    std::vector<BoundingBox> dets;
    for (int i = 0; i < (int)results.size(); i++)
    {
      nms(results[i], nms_threshold_);

      for (auto &box : results[i])
      {
        dets.push_back(box);
      }
    }

    // auto end = std::chrono::steady_clock::now();
    // double time = std::chrono::duration<double, std::milli>(end - start).count();
    // std::cout << "inference time:" << time << "ms" << std::endl;
    return dets;
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