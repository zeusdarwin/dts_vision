#pragma once

#include "tasks/base/web.hpp"
#include "tools/common/type_common.hpp"
#include "tools/utils.hpp"
#include <Eigen/src/Geometry/Quaternion.h>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <utility>
#include <vector>
#include "tools/utils.hpp"

namespace dts_vision {
////////////////////////////////////////////////////// receive ///////////////////////////////////////////////////////////////////////
struct ReceiveRobotData {
    static constexpr uint8_t ID = 0x01;
    uint8_t cmd_ID = ID;
    uint8_t head[3] = {'D', 'T', 'S'};
    uint32_t time_stamp_pc; //收到的上一包的PC时间戳
    uint32_t time_stamp_receive_micro; //收到的上一包时STM32的时间戳
    uint32_t time_stamp_send_micro; //发送此包时STM32的时间戳

    float yaw, pitch, roll; //坐标系定义： +x:前，+y:左，+z：上，+yaw 左，+pitch 下。+roll 右倾，旋转顺序ZYX
    // float q[4]; // wxyz
    // float vyaw, vpitch, vroll;
    // float v_x, v_y, v_z;

    float bullet_speed;
    uint8_t detect_color; //0 r 1 b
    uint32_t bullet_count; // 发出弹+1
    float operator_yaw_offset;
    float operator_pitch_offset;
    uint8_t mode; //0:自瞄 1:打符

    static std::optional<ReceiveRobotData> create(const std::vector<uint8_t>& data) {
        if (data.size() != sizeof(ReceiveRobotData) || data[0] != ID || data[1] != 'D' || data[2] != 'T' || data[3] != 'S')
            return std::nullopt;

        ReceiveRobotData out;
        std::memcpy(&out, data.data(), sizeof(out));
        return out;
    }

    void update_log(uint32_t delay) {
        Web::write_log("robo", [&](auto& j) {
            j["delay"] = Web::val(delay);
            j["timestamp_pc"] = Web::val(time_stamp_pc);
            j["timestamp_receive_micro"] = Web::val(time_stamp_receive_micro);
            j["timestamp_send_micro"] = Web::val(time_stamp_send_micro);

            // auto _q = Eigen::Quaternionf(q[0], q[1], q[2], q[3]);

            j["yaw"] = Web::val(yaw);
            j["pitch"] = Web::val(pitch);
            j["roll"] = Web::val(roll);

            // j["v_x"] = Web::val(v_x);
            // j["v_y"] = Web::val(v_y);
            // j["v_z"] = Web::val(v_z);

            j["bullet_speed"] = Web::val(bullet_speed);
            j["detect_color"] = (detect_color == 0 ? "Red" : "Blue");
            j["bullet_count"] = Web::val(bullet_count);
            j["operator_yaw_offset"] = Web::val(operator_yaw_offset);
            j["operator_pitch_offset"] = Web::val(operator_pitch_offset);
            // j["mode"] = Web::val(mode);
        });
    }
} __attribute__((packed));

struct SentryJointState {
    static constexpr uint8_t ID = 0x02;
    uint8_t cmd_ID = ID;
    uint8_t head[3] = {'D', 'T', 'S'};
    float big_yaw_in_world;
    static std::optional<SentryJointState> create(const std::vector<uint8_t>& data) {
        if (data.size() != sizeof(SentryJointState) || data[0] != ID || data[1] != 'D' || data[2] != 'T' || data[3] != 'S')
            return std::nullopt;

        SentryJointState out;
        std::memcpy(&out, data.data(), sizeof(out));
        return out;
    }
    void update_log() {
        Web::write_log("joint", [&](auto& j) {
            j["big_yaw_in_world"] = Web::val(big_yaw_in_world);
        });
    }
} __attribute__((packed));

struct HeroMode {
    static constexpr uint8_t ID = 0x03;
    uint8_t cmd_ID = ID;
    uint8_t head[3] = {'D', 'T', 'S'};
    uint8_t mode; //o:auto_aim 1:auto_client
    static std::optional<HeroMode> create(const std::vector<uint8_t>& data) {
        if (data.size() != sizeof(HeroMode) || data[0] != ID || data[1] != 'D' || data[2] != 'T' || data[3] != 'S')
            return std::nullopt; 

        HeroMode out;
        std::memcpy(&out, data.data(), sizeof(out));
        return out;
    }
};

struct SendRobotCmdData {
    static constexpr uint8_t ID = 0x01;
    uint8_t cmd_ID = ID;
    uint8_t head[3] = {'D', 'T', 'S'};
    uint32_t time_stamp;

    uint8_t appear;

    float pitch, yaw;
    float target_yaw, target_pitch;

    float enable_yaw_diff, enable_pitch_diff;
    float v_yaw, v_pitch;
    float a_yaw, a_pitch;

    uint8_t detect_color;
    auto to_tuple() const {
        return std::tie(
            cmd_ID,
            time_stamp,
            appear,
            pitch,
            yaw,
            target_yaw,
            target_pitch,
            enable_yaw_diff,
            enable_pitch_diff,
            v_yaw,
            v_pitch,
            a_yaw,
            a_pitch,
            detect_color
        );
    }
} __attribute__((packed));

struct SendNavCmdData {// 导航发给下位机
    static constexpr uint8_t ID = 0x02;
    uint8_t cmd_ID = ID;
    uint8_t head[3] = {'D', 'T', 'S'};
    float vx, vy, wz;
} __attribute__((packed));

struct SentryRefereeSend {
    static constexpr uint8_t ID = 0x03;
    uint8_t cmd_ID = ID;
    uint8_t head[3] = {'D', 'T', 'S'};
    uint8_t set_current_pose; //1 为进攻姿态，2 为防御姿态，3 为移动姿态
} __attribute__((packed));

} // namespace dts_vision
