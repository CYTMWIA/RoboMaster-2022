#include "vino_util.hpp"

#include <opencv2/imgproc/imgproc.hpp>

namespace rmcv::detect
{
cv::Mat resize_image(const cv::Mat &img, int width, int height)
{
  float r = std::min(width / (img.cols * 1.0), height / (img.rows * 1.0));
  int unpad_w = r * img.cols;
  int unpad_h = r * img.rows;

  cv::Mat re(unpad_h, unpad_w, CV_8UC3);
  cv::resize(img, re, re.size());

  cv::Mat out(height, width, CV_8UC3, cv::Scalar(114, 114, 114));
  re.copyTo(out(cv::Rect(0, 0, re.cols, re.rows)));

  return out;
}

void image2blob(const cv::Mat &image, InferenceEngine::Blob::Ptr &blob)
{
  int img_w = image.cols;
  int img_h = image.rows;
  int channels = 3;

  InferenceEngine::MemoryBlob::Ptr mblob = InferenceEngine::as<InferenceEngine::MemoryBlob>(blob);
  if (!mblob)
  {
    THROW_IE_EXCEPTION << "We expect blob to be inherited from MemoryBlob in matU8ToBlob, "
                       << "but by fact we were not able to cast inputBlob to MemoryBlob";
  }
  // locked memory holder should be alive all time while access to its buffer happens
  auto mblobHolder = mblob->wmap();

  float *blob_data = mblobHolder.as<float *>();

  for (size_t c = 0; c < channels; c++)
  {
    for (size_t h = 0; h < img_h; h++)
    {
      for (size_t w = 0; w < img_w; w++)
      {
        blob_data[c * img_w * img_h + h * img_w + w] = (float)image.at<cv::Vec3b>(h, w)[c];
      }
    }
  }
}
}  // namespace rmcv::detect