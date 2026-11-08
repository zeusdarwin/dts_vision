#include "armor_tracker.hpp"
#include "tasks/auto_aim/armor_track/armor_target.hpp"
#include "tasks/auto_aim/type.hpp"
#include "tasks/base/dta_utils.hpp"
#include "tools/logger.hpp"
#include <algorithm>
#include <array>
#include <limits>
#include <utility>
#include <vector>
namespace dts_vision::auto_aim {
struct ArmorTracker::Impl {
    Impl(const YAML::Node& config) {
        cfg_.load(config);
    }
    ArmorTarget track(
        ArmorDetection& armor_detection,
        const CameraInfo& camera_info,
        const ISO3& T_camera_cv2world,
        int frame_id
    ) {
        auto& cur = target_buf_[cur_target_idx_];
        auto& pre = target_buf_[pre_target_idx_];
        auto process = [&](int idx) {
            auto& target = target_buf_[idx];
            bool found = (target.track_state.tracker_state == ArmorTarget::TrackState::LOST)
                ? init_target(target, armor_detection, frame_id, camera_info, T_camera_cv2world)
                : update_target(target, armor_detection, camera_info, T_camera_cv2world);
            update_fsm(found, idx, armor_detection.timestamp);
            return found;
        };
        // 双缓冲，方便异常丢失恢复和操作手换目标。
        process(cur_target_idx_);

        if (cur.track_state.tracker_state == ArmorTarget::TrackState::TEMP_LOST) {
            process(pre_target_idx_);

            if (pre.track_state.tracker_state == ArmorTarget::TrackState::TRACKING) {
                std::swap(cur, pre);
                pre.track_state.tracker_state = ArmorTarget::TrackState::LOST;
            }
        } else if (cur.track_state.tracker_state == ArmorTarget::TrackState::TRACKING) {
            pre.track_state.tracker_state = ArmorTarget::TrackState::LOST;
        }
        armor_detection.lights.erase(
            std::remove_if(
                armor_detection.lights.begin(),
                armor_detection.lights.end(),
                [](const Light& l) { return l.laji; }
            ),
            armor_detection.lights.end()
        );

        return target_buf_[cur_target_idx_].fast_copy_without_ekf(); //下游不让用ekf
    }
    bool init_target(
        ArmorTarget& target,
        ArmorDetection& armor_detection,
        int frame_id,
        const CameraInfo& camera_info,
        const ISO3& T_camera_cv2world
    ) noexcept {
        if (armor_detection.armors.empty()) {
            return false;
        }
        const auto valid_target = [&](Armor& armor) {
            const bool valid_color = armor.color != ArmorColor::NONE && armor.color != ArmorColor::PURPLE;
            const bool keep_outpost = target_buf_[cur_target_idx_].target_type == ArmorType::OUTPOST && armor.type != ArmorType::OUTPOST && target_buf_[cur_target_idx_].check();
            const bool pnp_ok = ArmorTarget::armor_pnp(armor, camera_info, T_camera_cv2world);
            return valid_color && !keep_outpost && pnp_ok;
        };
        Armor* selected_armor = nullptr;
        auto min_dis = std::numeric_limits<double>::max();
        for (auto& armor: armor_detection.armors) {
            if (valid_target(armor)) {
                auto bbox = armor.key_points.bounding_box();
                auto dis = tools::calculate_distance_to_img_center(
                    (bbox.tl() + bbox.br()) / 2.0,
                    camera_info.camera_matrix
                );
                if (dis < min_dis) {
                    min_dis = dis;
                    selected_armor = &armor;
                }
            }
        }
        if (!selected_armor) {
            return false;
        }

        Armor init_target = *selected_armor;
        DTS_INFO("init target: {}", string_by_armor_type(init_target.type));
        target.reset(init_target, cfg_, armor_detection.timestamp, frame_id, camera_info, T_camera_cv2world);
        target.track_state.tracker_state = ArmorTarget::TrackState::DETECTING;
        return true;
    }
    bool update_target(
        ArmorTarget& target,
        ArmorDetection& armor_detection,
        const CameraInfo& camera_info,
        const ISO3& T_camera_cv2world
    ) noexcept {
        std::vector<Armor> candidates;
        candidates.reserve(armor_detection.armors.size());
        for (const auto& armor: armor_detection.armors) {
            if (armor.type == target.target_type && armor.color != ArmorColor::NONE && armor.color != ArmorColor::PURPLE)
            {
                candidates.emplace_back(armor);
            }
            // if (armor.type == target.target_type)
            // {
            //     candidates.emplace_back(armor);
            // }
        }
        target.predict_ekf(armor_detection.timestamp);
        auto matched_armors = target.match_armor(candidates, armor_detection.timestamp, camera_info, T_camera_cv2world);
        auto matched_lights = target.match_light(
            armor_detection.lights,
            matched_armors,
            armor_detection.timestamp,
            camera_info,
            T_camera_cv2world
        );
        const int updated = target.update(
            matched_armors,
            matched_lights,
            armor_detection.timestamp,
            camera_info,
            T_camera_cv2world
        );
        return updated > 0;
    }
    void update_fsm(bool found, size_t i, const TimePoint& now) noexcept {
        auto& target = target_buf_[i];
        auto& track_state = target.track_state;
        if (found)
            ++found_count_;
        update_track_fsm(
            found,
            track_state,
            cfg_.tracking_thres,
            elapsed_sec(target.last_update, now),
            lost_time_thres(target)
        );
    }
    double lost_time_thres(const ArmorTarget& target) const noexcept {
        return (target.target_type == ArmorType::OUTPOST) ? cfg_.lost_time_thres_outpost : cfg_.lost_time_thres;
    }
    void set_sentry(bool is_sentry) {
        iam_sentry = is_sentry;
    }

    int found_count_ = 0;

    size_t cur_target_idx_ = 0;
    size_t pre_target_idx_ = 1;
    std::array<ArmorTarget, 2> target_buf_;
    ArmorTrackerCfg cfg_;
    bool iam_sentry = false;
};
ArmorTracker::ArmorTracker(const YAML::Node& config): _impl(std::make_unique<Impl>(config)) {}
ArmorTracker::~ArmorTracker() noexcept = default;

ArmorTarget ArmorTracker::track(
    ArmorDetection& armor_detection,
    const CameraInfo& camera_info,
    const ISO3& T_camera_cv2world,
    int frame_id
) {
    return _impl->track(armor_detection, camera_info, T_camera_cv2world, frame_id);
}
int ArmorTracker::get_count() {
    return _impl->found_count_;
}
void ArmorTracker::reset_count() {
    _impl->found_count_ = 0;
}
void ArmorTracker::set_sentry(bool is_sentry) {
    _impl->set_sentry(is_sentry);
}
} // namespace dts_vision::auto_aim
