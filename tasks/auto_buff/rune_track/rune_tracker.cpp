#include "rune_tracker.hpp"
#include "tasks/auto_buff/rune_track/rune_target.hpp"
#include "tasks/auto_buff/type.hpp"
#include "tasks/base/dta_utils.hpp"
#include "tools/logger.hpp"
#include <memory>
namespace dts_vision::auto_buff {
struct RuneTracker::Impl {
    Impl(const YAML::Node& config) {
        cfg_.load(config);
    }
    RuneTarget track(
        RuneDetection& detection,
        const CameraInfo& camera_info,
        const ISO3& T_camera_cv2world,
        int frame_id
    ) {
        const bool found = target.track_state.tracker_state == RuneTarget::TrackState::LOST
            ? init_target(detection, frame_id, camera_info, T_camera_cv2world)
            : update_target(detection, camera_info, T_camera_cv2world);
        if (target.get_target_state().pos().norm() > 15.0 && target.check()) {
            target.track_state.tracker_state = RuneTarget::TrackState::LOST;
            DTS_WARN("TOO FAR");
        }
        update_fsm(found, detection.timestamp);
        // detection.rune_rs.erase(
        //     std::remove_if(
        //         detection.rune_rs.begin(),
        //         detection.rune_rs.end(),
        //         [](const RuneR& rune_detection) { return rune_detection.laji; }
        //     ),
        //     detection.rune_rs.end()
        // );
        return target.fast_copy_without_ekf();
    }
    bool init_target(
        RuneDetection& rune_detection,
        int frame_id,
        const CameraInfo& camera_info,
        const ISO3& T_camera_cv2world
    ) noexcept {
        if (!target.reset(rune_detection, cfg_, rune_detection.timestamp, frame_id, camera_info, T_camera_cv2world)) {
            return false;
        }
        DTS_INFO("init rune target");
        target.track_state.tracker_state = RuneTarget::TrackState::DETECTING;
        return true;
    }
    bool update_target(
        RuneDetection& rune_detection,
        const CameraInfo& camera_info,
        const ISO3& T_camera_cv2world
    ) noexcept {
        if (rune_detection.fan_blades.empty() && rune_detection.rune_rs.empty() && rune_detection.fan_targets.empty()) {
            return false;
        }
        target.predict_ekf(rune_detection.timestamp);
        auto matched_fans = target.match_fan(rune_detection.fan_blades, rune_detection.timestamp, camera_info, T_camera_cv2world);
        auto match_r = target.match_r(matched_fans, rune_detection.rune_rs, rune_detection.timestamp, camera_info, T_camera_cv2world);
        auto matched_fan_targets = target.match_fan_target(
            rune_detection.fan_targets,
            match_r,
            rune_detection.timestamp,
            camera_info,
            T_camera_cv2world
        );
        const auto updated = target.update(
            matched_fans,
            matched_fan_targets,
            match_r,
            rune_detection.timestamp,
            camera_info,
            T_camera_cv2world
        );
        return std::get<0>(updated) > 0;
    }
    void update_fsm(bool found, const TimePoint& now) noexcept {
        auto& target_ = target;
        auto& state = target.track_state;
        if (found)
            ++found_count_;
        update_track_fsm(
            found,
            state,
            cfg_.tracking_thres,
            elapsed_sec(target_.last_update, now),
            cfg_.lost_time_thres
        );
    }
    RuneTrackerCfg cfg_;
    RuneTarget target;
    int found_count_ = 0;
};
RuneTracker::RuneTracker(const YAML::Node& config): _impl(std::make_unique<Impl>(config)) {}
RuneTracker::~RuneTracker() noexcept = default;
RuneTarget RuneTracker::track(
    RuneDetection& detection,
    const CameraInfo& camera_info,
    const ISO3& T_camera_cv2world,
    int frame_id
) {
    return _impl->track(detection, camera_info, T_camera_cv2world, frame_id);
}
int RuneTracker::get_count() {
    return _impl->found_count_;
}
void RuneTracker::reset_count() {
    _impl->found_count_ = 0;
}
} // namespace dts_vision::auto_buff
