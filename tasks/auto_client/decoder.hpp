#pragma once

#include "tasks/auto_client/common.hpp"
#include <cstring>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <vector>

namespace dts_vision::auto_client {

class Decoder {
public:
    Decoder();
    ~Decoder();

    void push_packet(const BlindSend& pkt);

    bool try_pop_frame(cv::Mat& out);

private:
    struct Impl;
    std::unique_ptr<Impl> _impl;
};

} // namespace dts_vision::auto_client