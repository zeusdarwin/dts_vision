#pragma once
#include "tasks/auto_aim/type.hpp"
#include "tools/common/impl.hpp"
#include <optional>
#include <vector>
namespace dts_vision::auto_aim {
class ArmorDetector {
public:
    using Ptr = std::unique_ptr<ArmorDetector>;
    ArmorDetector(const YAML::Node& config);
    [[nodiscard]] std::tuple<std::vector<Light>, std::vector<Armor>> detect(
        const CommonFrame& common_frame,
        const cv::Rect& net_focus,
        const std::optional<cv::Rect>& detect_light = std::nullopt
    );
    double get_net_wh_ratio() const noexcept;
    DTS_IMPL_DEFINITION(ArmorDetector)
};
} // namespace dts_vision::auto_aim
