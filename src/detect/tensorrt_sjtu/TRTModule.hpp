//
// Created by xinyang on 2021/4/8.
//

#ifndef _ONNXTRTMODULE_HPP_
#define _ONNXTRTMODULE_HPP_

#include <opencv2/core.hpp>
#include <NvInfer.h>

#include "../BoundingBox.hpp"

using BoundingBox = rmcv::detect::BoundingBox;

/*
 * 四点模型
 */
class TRTModule {
    static constexpr int TOPK_NUM = 128;
    static constexpr float KEEP_THRES = 0.1f;

public:
    explicit TRTModule(const std::string &onnx_file);

    ~TRTModule();

    TRTModule(const TRTModule &) = delete;

    TRTModule operator=(const TRTModule &) = delete;

    std::vector<BoundingBox> operator()(const cv::Mat &src) const;

private:
    void build_engine_from_onnx(const std::string &onnx_file);

    void build_engine_from_cache(const std::string &cache_file);

    void cache_engine(const std::string &cache_file);

    nvinfer1::ICudaEngine *engine;
    nvinfer1::IExecutionContext *context;
    mutable void *device_buffer[2];
    float *output_buffer;
    cudaStream_t stream;
    int input_idx, output_idx;
    size_t input_sz, output_sz;
};


#endif /* _ONNXTRTMODULE_HPP_ */
