#pragma once

#include "tools/common/impl.hpp"
#include <opencv2/core/mat.hpp>
#include <yaml-cpp/node/node.h>

namespace dts_vision::auto_client {

class ImagePreprocessor {
public:
    explicit ImagePreprocessor(const YAML::Node& config);
    DTS_IMPL_DEFINITION(ImagePreprocessor)

    cv::Mat process(
        const cv::Mat& input,
        cv::Mat* roi_out = nullptr,
        cv::Mat* static_removed_out = nullptr
    );
};

} // namespace dts_vision::auto_client
