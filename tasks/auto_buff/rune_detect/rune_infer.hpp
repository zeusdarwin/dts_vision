#pragma once
#include "tasks/auto_buff/type.hpp"
#include "tools/common/image.hpp"
#include "tools/common/impl.hpp"
namespace dts_vision::auto_buff {
class RuneInfer {
public:
    using Ptr = std::unique_ptr<RuneInfer>;
    RuneInfer(const YAML::Node& config);
    DTS_IMPL_DEFINITION(RuneInfer)
    static Ptr create(const YAML::Node& config) {
        return std::make_unique<RuneInfer>(config);
    }

    [[nodiscard]] std::vector<RuneFanBladeWithR> process(const cv::Mat& output_buffer) const;

    int input_w() const noexcept;
    int input_h() const noexcept;
    bool use_norm() const noexcept;
    PixelFormat target_format() const noexcept;
};

} // namespace dts_vision::auto_buff
