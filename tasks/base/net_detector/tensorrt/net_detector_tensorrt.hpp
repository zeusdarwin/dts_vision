#pragma once
#include "tools/common/impl.hpp"
#include "tasks/base/net_detector/net_detector_base.hpp"
namespace dts_vision::net_detector {
class NetDetectorTensorrt: public NetDetectorBase {
public:
    NetDetectorTensorrt(const YAML::Node& config, Config c);
    [[nodiscard]] OutPut detect(const cv::Mat& img, PixelFormat format) noexcept override;
    DTS_IMPL_DEFINITION(NetDetectorTensorrt)
};

} // namespace dts_vision::net_detector