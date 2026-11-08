#pragma once

#include "tools/logger.hpp"

#include <memory>
#include <yaml-cpp/node/node.h>

#include "common.hpp"
#include <opencv2/opencv.hpp>
namespace dts_vision::auto_client {

class Encoder {
public:
    explicit Encoder(const YAML::Node& config);

    ~Encoder();

    void push_frame(const cv::Mat& frame);
    bool try_pop_packet(BlindSend& out);
    struct Impl;
    std::unique_ptr<Impl> _impl;
};

} // namespace dts_vision::auto_client
