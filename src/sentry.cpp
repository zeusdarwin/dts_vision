#include "angles.h"
#include "tasks/auto_aim/armor_control/armor_aimer.hpp"
#include "tasks/auto_aim/armor_omni.hpp"
#include "tasks/auto_buff/rune_control/rune_aimer.hpp"
#include "tasks/auto_buff/rune_detect/rune_detector.hpp"
#include "tasks/auto_buff/rune_track/rune_target.hpp"
#include "tasks/auto_buff/rune_track/rune_tracker.hpp"
#include "tasks/base/ballistic_trajectory.hpp"
#include "tasks/base/wheel_odometry.hpp"
#include "tasks/base/packet_serial.hpp"
#include "tools/common/ascii_banner.hpp"
#include "tools/io/video_save.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <utility>
#ifdef USE_NAV_ROS2
    #include "tools/navigation_bridge.hpp"
#endif
#ifdef USE_RERUN
    #include "tasks/rerun/recorder.hpp"
    #include "tasks/rerun/tf.hpp"
#endif
#ifdef USE_ROS2
    #include "tools/rcl/visual/armor.hpp"
    #include "tools/rcl/visual/armor_target.hpp"
    #include "sensor_msgs/msg/camera_info.hpp"
    #include "sensor_msgs/msg/image.hpp"
    #include <rclcpp/qos.hpp>
#endif
#include "backward-cpp/backward.hpp"
#include "tools/common/config.hpp"
#include "tools/io/camera_factory.hpp"
#include "tools/io/serial_driver.hpp"
#include "tools/param_deliver.h"
#include "tools/runtime_tf.hpp"
#include "tools/semaphore_guard.hpp"
#include "tools/signal_guard.hpp"
namespace backward {
static backward::SignalHandling sh;
}
using namespace dts_vision;
enum class Mode : int { AutoAim = 0, AutoBuff = 1 };
Mode str_to_Mode(std::string str) {
    str = tools::to_upper(str);
    if (str == "AUTO_AIM") {
        return Mode::AutoAim;
    } else if (str == "AUTO_BUFF") {
        return Mode::AutoBuff;
    } else {
        throw std::invalid_argument("Invalid mode string");
    }
}
enum class SentryFrame : int {
    WORLD,
    GIMBAL_ABS,
    GIMBAL,
    CAMERA,
    CAMERA_CV,
    SHOOT,
    BIG_YAW,
    OMNI0,
    OMNI0_CV,
    OMNI1,
    OMNI1_CV,
    N
};

using SimpleRobotTF = tools::RobotTF<SentryFrame, static_cast<size_t>(SentryFrame::N), false>;

std::string SentryFrame_to_str(int frame) {
    constexpr const char* details[] = { "world",     "gimbal_abs",  "gimbal",   "camera",
                                        "camera_cv", "shoot",       "big_yaw",  "omni_0",
                                        "omni_0_cv", "omni_1",      "omni_1_cv" };
    return std::string(details[frame]);
}
std::string SentryFrame_to_str(SentryFrame frame) {
    return SentryFrame_to_str(std::to_underlying(frame));
}
struct CameraTag {};
struct SerialTag {};
struct DetectTag {};
struct FrameTag {};

using CameraIO = tools::Scheduler::IOPair<CameraTag, ImageFrame>;
using SerialIO = tools::Scheduler::IOPair<SerialTag, std::vector<uint8_t>>;
using CommonFrameIo = tools::Scheduler::IOPair<FrameTag, CommonFrame>;
struct Detection {
    std::optional<std::vector<auto_aim::ArmorDetection>> armors;
    std::optional<std::vector<auto_buff::RuneDetection>> runes;
    std::optional<TimePoint> detect_start;
};
using DetIo = tools::Scheduler::IOPair<DetectTag, Detection>;
struct LogCtx {
    int camera_count = 0;
    int detect_count = 0;
    int track_count = 0;
    int solve_count = 0;
    int serial_count = 0;
    int found_count = 0;
    double latency_ms_total = 0.0;
    int omni_count = 0;
    double omni_latency_ms_total = 0.0;
    void reset() {
        *this = {};
    }
};
struct AutoExposureCfg {
    double target_brightness = 0.0;
    double step_gain = 0.0;
    double decay_step = 0.0;
    double tolerance = 0.0;
    double exposure_min = 0.0;
    double exposure_max = 0.0;
    double control_interval_ms = 0.0;

    explicit AutoExposureCfg(const YAML::Node& config):
        target_brightness(config["target_brightness"].as<double>()),
        step_gain(config["step_gain"].as<double>()),
        decay_step(config["decay_step"].as<double>()),
        tolerance(config["tolerance"].as<double>()),
        exposure_min(config["exposure_min"].as<double>()),
        exposure_max(config["exposure_max"].as<double>()),
        control_interval_ms(config["control_interval_ms"].as<double>()) {}
};
bool is_web_running() {
    static std::atomic<bool> cached { true };
    tools::dt_once(
        [&]() {
            const int ret = std::system("pgrep -x dts_vision_web > /dev/null 2>&1");
            cached = (ret == 0);
        },
        std::chrono::duration<double>(1.0)
    );
    bool running = cached.load();
#ifdef USE_RERUN
    running = running || rerun::Recorder::instance().enabled();
#endif
    return running;
}
static constexpr auto RECORD_FOLDER_PATH_ARR = tools::concat(ROOT_DIR, "/record/sentry");
static constexpr std::string_view RECORD_FOLDER_PATH(RECORD_FOLDER_PATH_ARR.data());

int main(int argc, char** argv) {
    auto start_tp = Clock::now();
    print_banner();
    auto& signal = tools::SignalGuard::instance();
    tools::logger::init(spdlog::level::trace);
    bool debug = false;
    std::string config_path;
    std::string robot_name;
    auto first_arg = tools::get_arg(1, argc, argv);
    if (first_arg) {
        robot_name = first_arg.value();
        config_path = get_robot_config_path(robot_name).value_or(robot_name);
    } else {
        return 1;
    }
    auto second_arg = tools::get_arg(2, argc, argv);
    if (second_arg) {
        debug = second_arg.value() == "true";
    }
    auto config = YAML::LoadFile(config_path);
    const bool navigation_enabled = config["common"]["navigation"]["enable"].as<bool>(false);
    const double navigation_timeout_ms = config["common"]["navigation"]["feedback_timeout_ms"].as<double>(1300.0);
    if (navigation_enabled && (!std::isfinite(navigation_timeout_ms) || navigation_timeout_ms <= 0)) {
        throw std::invalid_argument("navigation.feedback_timeout_ms must be positive and finite");
    }
#ifndef USE_NAV_ROS2
    if (navigation_enabled) {
        throw std::runtime_error("Navigation requires BUILD_WITH_NAV_ROS2=ON");
    }
#endif
    std::atomic<int64_t> navigation_last_feedback_ns{0};
    auto navigation_feedback_fresh = [&]() {
        const auto last = navigation_last_feedback_ns.load();
        const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        return last != 0 && (now_ns - last) / 1e6 <= navigation_timeout_ms;
    };

    tools::Scheduler::Scheduler scheduler;
    std::atomic<Mode> mode = str_to_Mode(config["common"]["mode"].as<std::string>());
    std::atomic<EnemyColor> enemy_color = enemy_color_from_string(config["common"]["enemy_color"].as<std::string>());
    std::atomic<double> bullet_speed = config["common"]["bullet_speed"].as<double>();
#ifdef USE_ROS2
    rcl::RclcppNode rcl_node("auto_aim");
    rcl::TF rcl_tf(rcl_node);
#endif

    std::shared_ptr<tools::io::SerialDriver> serial;
    if (!navigation_enabled && config["common"]["serial"]["enable"].as<bool>()) {
        serial = std::make_shared<tools::io::SerialDriver>(config["common"]["serial"], scheduler);
    }

    auto camera_config = config["common"]["camera"];
    auto camera_type = camera_config["type"].as<std::string>();
    std::unique_ptr<tools::io::Camera> camera;
    tools::SignalGuard::add_callback([&]() {
        if (camera) {
            camera->stop();
        }
    });

    camera = tools::io::create_camera(camera_config, scheduler, camera_type);
    camera->init();
    if (!camera->is_running()) {
        return 0;
    }
    std::unique_ptr<tools::io::VideoSaver> video_saver;
    if (config["common"]["record"]["enable"].as<bool>()) {
        video_saver = std::make_unique<tools::io::VideoSaver>(
            tools::io::VideoSaver::generate_record_filename(RECORD_FOLDER_PATH.data()),
            tools::io::VideoSaver::Mode::NonBlocking
        );
    }
    CameraInfo camera_info;
    camera_info.load(camera_config["camera_info"]);
    auto_aim::ArmorDetector armor_detector(config["armor_detector"]);
    auto_aim::ArmorTracker armor_tracker(config["armor_tracker"]);
    armor_tracker.set_sentry(true);
    auto_aim::AutoAimFsm auto_aim_fsm(config["auto_aim_fsm"]);
    auto_aim::ArmorOmni armor_omni(config["armor_omni"]);
    auto_aim::ArmorAimer armor_aimer(config["armor_aimer"]);
    auto_buff::RuneDetector rune_detector(config["rune_detector"]);
    auto_buff::RuneTracker rune_tracker(config["rune_tracker"]);
    auto_buff::RuneAimer rune_aimer(config["rune_aimer"]);
    tools::OrderedQueue<auto_aim::ArmorDetection> armor_detections_queue;
    tools::OrderedQueue<auto_buff::RuneDetection> rune_detections_queue;
    tools::SWMR<auto_aim::ArmorTarget> armor_target;
    tools::SWMR<auto_aim::ArmorTarget> omni_target;
    tools::SWMR<auto_buff::RuneTarget> rune_target;
    armor_omni.emplace_one(
        config["armor_omni"]["camera0"],
        std::to_underlying(SentryFrame::OMNI0),
        std::to_underlying(SentryFrame::OMNI0_CV)
    );
    armor_omni.emplace_one(
        config["armor_omni"]["camera1"],
        std::to_underlying(SentryFrame::OMNI1),
        std::to_underlying(SentryFrame::OMNI1_CV)
    );
    BulletReappear bullet_reappear(config["common"]);
    LogCtx log_ctx;
    std::optional<VisionDebugCtx> dbg;
    if (debug) {
        dbg.emplace();
        dbg->camera_info_ = camera_info;
    }
    WheelOdometry wheel_odometry(config["common"]["wheel_odometry"], Clock::now());
    auto tf = SimpleRobotTF::create();
    {
        for (auto [from, to]: std::array {
                 std::pair { SentryFrame::WORLD, SentryFrame::GIMBAL_ABS },
                 std::pair { SentryFrame::GIMBAL_ABS, SentryFrame::GIMBAL },
                 std::pair { SentryFrame::GIMBAL, SentryFrame::CAMERA },
                 std::pair { SentryFrame::GIMBAL, SentryFrame::SHOOT },
                 std::pair { SentryFrame::CAMERA, SentryFrame::CAMERA_CV },
                 std::pair { SentryFrame::GIMBAL_ABS, SentryFrame::BIG_YAW },
                 std::pair { SentryFrame::BIG_YAW, SentryFrame::OMNI0 },
                 std::pair { SentryFrame::OMNI0, SentryFrame::OMNI0_CV },
                 std::pair { SentryFrame::BIG_YAW, SentryFrame::OMNI1 },
                 std::pair { SentryFrame::OMNI1, SentryFrame::OMNI1_CV },
             })
        {
            tf->add_edge(from, to);
        }
        ISO3 T_camera_cv2camera = ISO3::Identity();
        T_camera_cv2camera.linear() = R_cv2physics;
        tf->push(SentryFrame::CAMERA, SentryFrame::CAMERA_CV, Clock::now(), T_camera_cv2camera);
        tf->push(
            SentryFrame::GIMBAL,
            SentryFrame::CAMERA,
            Clock::now(),
            tools::load_isometry3(config["common"]["tf"]["T_camera2gimbal"])
        );
        tf->push(
            SentryFrame::GIMBAL,
            SentryFrame::SHOOT,
            Clock::now(),
            tools::load_isometry3(config["common"]["tf"]["T_shoot2gimbal"])
        );
        tf->push(
            SentryFrame::BIG_YAW,
            SentryFrame::OMNI0,
            Clock::now(),
            tools::load_isometry3(config["common"]["tf"]["T_omin02big_yaw"])
        );
        tf->push(
            SentryFrame::BIG_YAW,
            SentryFrame::OMNI1,
            Clock::now(),
            tools::load_isometry3(config["common"]["tf"]["T_omin12big_yaw"])
        );
        tf->push(SentryFrame::OMNI0, SentryFrame::OMNI0_CV, Clock::now(), T_camera_cv2camera);
        tf->push(SentryFrame::OMNI1, SentryFrame::OMNI1_CV, Clock::now(), T_camera_cv2camera);
    }
    if (video_saver) {
        scheduler.register_task<CameraIO>("save_video", [&](CameraIO::second_type&& img_frame) {
            if (!img_frame.src_img.empty()) {
                video_saver->write_frame(img_frame.src_img);
            }
            return std::make_tuple(std::optional<CameraIO::second_type>(std::nullopt));
        });
    }
    auto serial_send_to_image_microseconds = config["common"]["serial"]["serial_send_to_image_microseconds"].as<int>();

    scheduler.register_task<CameraIO, CommonFrameIo>("push_common_frame", [&](CameraIO::second_type&& img_frame) {
        static int current_id = 0;
        if (img_frame.src_img.empty()) {
            return std::make_tuple(std::optional<CommonFrameIo::second_type>(std::nullopt));
        }
        log_ctx.camera_count++;

        CommonFrame common_frame {
            .img_frame = std::move(img_frame),
            .id = current_id++,
            .frame_id = std::to_underlying(SentryFrame::CAMERA_CV),

        };

        return std::make_tuple(std::optional<CommonFrameIo::second_type>(std::move(common_frame)));
    });
    auto receive_packets = [&](SerialIO::second_type&& data) {
            static std::mutex mutex;
            std::lock_guard<std::mutex> lock(mutex);
            auto now = Clock::now();

            // A serial read is a byte-stream read, not a packet read. Keep
            // incomplete bytes and split multiple packets from one read.
            static std::vector<uint8_t> receive_buffer;
            receive_buffer.insert(receive_buffer.end(), data.begin(), data.end());

            auto is_packet_header = [](const auto& buffer, size_t offset) {
                return buffer.size() >= offset + 4
                    && (buffer[offset] == ReceiveRobotData::ID || buffer[offset] == SentryJointState::ID)
                    && buffer[offset + 1] == 'D'
                    && buffer[offset + 2] == 'T'
                    && buffer[offset + 3] == 'S';
            };

            while (true) {
                if (receive_buffer.size() < 4) {
                    break;
                }

                size_t packet_offset = 0;
                while (packet_offset + 4 <= receive_buffer.size()
                       && !is_packet_header(receive_buffer, packet_offset)) {
                    packet_offset++;
                }

                if (packet_offset + 4 > receive_buffer.size()) {
                    // Preserve the suffix that may be the beginning of a header.
                    receive_buffer.erase(
                        receive_buffer.begin(),
                        receive_buffer.end() - std::min<size_t>(receive_buffer.size(), 3)
                    );
                    break;
                }

                if (packet_offset > 0) {
                    receive_buffer.erase(
                        receive_buffer.begin(),
                        receive_buffer.begin() + static_cast<std::ptrdiff_t>(packet_offset)
                    );
                }

                const size_t packet_size = receive_buffer[0] == ReceiveRobotData::ID
                    ? sizeof(ReceiveRobotData)
                    : sizeof(SentryJointState);
                if (receive_buffer.size() < packet_size) {
                    break;
                }

                std::vector<uint8_t> packet(
                    receive_buffer.begin(),
                    receive_buffer.begin() + static_cast<std::ptrdiff_t>(packet_size)
                );
                receive_buffer.erase(
                    receive_buffer.begin(),
                    receive_buffer.begin() + static_cast<std::ptrdiff_t>(packet_size)
                );

                log_ctx.serial_count++;
                if (auto robo_opt = ReceiveRobotData::create(packet); robo_opt.has_value()) {
                    auto robo = robo_opt.value();
                    static uint32_t last_pc = -1, delay = 0, last_bullet_count = 0;
                    if (robo.time_stamp_pc != last_pc) {
                        last_pc = robo.time_stamp_pc;
                        delay = (std::chrono::duration_cast<std::chrono::microseconds>(now - start_tp).count() - robo.time_stamp_pc
                                 - (robo.time_stamp_send_micro - robo.time_stamp_receive_micro))/ 2.0;

                        DTS_INFO("delay: {}", delay);
                                }
                    const auto yaw = robo.yaw;
                    // DTS_INFO("yaw: {}", yaw);
                    auto packet_time = now - std::chrono::microseconds(serial_send_to_image_microseconds);
                    ISO3 T_gimbal2gimbal_abs = ISO3::Identity();
                    T_gimbal2gimbal_abs.linear() = tools::euler2matrix(
                        Vec3(
                            angles::from_degrees(robo.yaw),
                            angles::from_degrees(robo.pitch),
                            angles::from_degrees(robo.roll)
                        ),
                        tools::EulerOrder::ZYX
                    );
                    tf->push(
                        SentryFrame::GIMBAL_ABS,
                        SentryFrame::GIMBAL,
                        packet_time,
                        T_gimbal2gimbal_abs
                    );
                    auto T_gimbal_abs2world = ISO3::Identity();
                    // T_gimbal_abs2world.translation() = Vec3(robo.v_x, robo.v_y, robo.v_z);
                    tf->push(
                        SentryFrame::WORLD,
                        SentryFrame::GIMBAL_ABS,
                        packet_time,
                        T_gimbal_abs2world
                    );
                    enemy_color = EnemyColor(robo.detect_color);
                    if (navigation_enabled) {
                        mode = robo.mode == 0 ? Mode::AutoAim : Mode::AutoBuff;
                        if (robo.bullet_speed > 0) {
                            bullet_speed = robo.bullet_speed;
                        }
                    }
                    robo.update_log(delay);
                    if (robo.bullet_count > last_bullet_count) {
                        bullet_reappear.push_back(Bullet {
                            .fire_time = Clock::now(),
                            .T_shoot2world = tf->pose_a_in_b(SentryFrame::SHOOT, SentryFrame::WORLD, Clock::now()),
                            .speed_in_world = bullet_speed,
                        });
                    }
                    last_bullet_count = robo.bullet_count;
                }
                if (auto joint_opt = SentryJointState::create(packet); joint_opt.has_value()) {
                    auto joint = joint_opt.value();
                    auto T_gimbal2gimbal_abs = tf->pose_a_in_b(SentryFrame::GIMBAL, SentryFrame::GIMBAL_ABS, Clock::now());
                    auto ypr = tools::matrix2euler(T_gimbal2gimbal_abs.linear(), tools::EulerOrder::ZYX);
                    ypr[0] = angles::from_degrees(joint.big_yaw_in_world);
                    ypr[1] = 0.0;
                    ISO3 T_big_yaw2gimbal_abs = ISO3::Identity();
                    T_big_yaw2gimbal_abs.translation() = Vec3(0, 0, 0.0);// 认为大小yaw都是建立在同一原点
                    T_big_yaw2gimbal_abs.linear() = tools::euler2matrix(ypr, tools::EulerOrder::ZYX);
                    tf->push(
                        SentryFrame::GIMBAL_ABS,
                        SentryFrame::BIG_YAW,
                        Clock::now(),
                        T_big_yaw2gimbal_abs
                    );
                    const auto big_yaw = joint.big_yaw_in_world;
                    // DTS_INFO("big_yaw: {}", big_yaw);
                }
            }
    };
    if (serial) {
        scheduler.register_task<SerialIO>("receive_serial", receive_packets);
    }
#ifdef USE_NAV_ROS2
    std::unique_ptr<tools::rcl::NavigationBridge> navigation;
    if (navigation_enabled) {
        navigation = std::make_unique<tools::rcl::NavigationBridge>(
            [&](const ReceiveRobotData& data, const SentryJointState& joint) {
                auto packets = tools::to_vector(data);
                const auto joint_packet = tools::to_vector(joint);
                packets.insert(packets.end(), joint_packet.begin(), joint_packet.end());
                receive_packets(std::move(packets));
                // Mark fresh only after both gimbal transforms have been updated.
                navigation_last_feedback_ns.store(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count());
            });
        scheduler.add_rate_source<>("navigation_receive", 1000.0, [&]() { navigation->poll(); });
    }
#endif
    if (camera) {
        scheduler.register_task<CommonFrameIo>("auto_exposure", [&](CommonFrameIo::second_type&& common_frame) {
            static std::mutex mutex;
            std::lock_guard<std::mutex> lock(mutex);
            static std::optional<AutoExposureCfg> auto_exposure_cfg = config["common"]["auto_exposure"]["enable"].as<bool>()
                ? std::optional<AutoExposureCfg>(std::in_place, config["common"]["auto_exposure"])
                : std::nullopt;
            if (auto_exposure_cfg) {
                auto& cfg = auto_exposure_cfg.value();
                tools::dt_once(
                    [&]() {
                        cv::Mat gray;
                        cv::cvtColor(common_frame.img_frame.src_img, gray, cv::COLOR_BGR2GRAY);
                        double exposure_time = camera->get_ExposureTime();
                        static double last_exposure_time = 0.0;
                        const double diff = cv::mean(gray)[0] - cfg.target_brightness;
                        if (std::fabs(diff) > cfg.tolerance && exposure_time > 0.0) {
                            exposure_time -= diff * cfg.step_gain;
                        } else {
                            exposure_time -= cfg.decay_step;
                        }
                        exposure_time =
                            std::clamp(exposure_time, cfg.exposure_min, cfg.exposure_max);
                        if (std::abs(exposure_time - last_exposure_time) > 10) {
                            camera->set_ExposureTime(exposure_time);
                            last_exposure_time = exposure_time;
                        }
                    },
                    std::chrono::milliseconds((int)cfg.control_interval_ms)
                );
            }
        });
    }

    scheduler.register_task<CommonFrameIo, DetIo>("detector", [&](CommonFrameIo::second_type&& common_frame) {
        Detection detection;
        cv::Rect net_focus = cv::Rect(0, 0, common_frame.img_frame.src_img.cols, common_frame.img_frame.src_img.rows); 
        if (mode == Mode::AutoAim) {
            static std::unique_ptr<std::counting_semaphore<>> detector_sem;
            if (!detector_sem) {
                detector_sem = std::make_unique<std::counting_semaphore<>>(
                    config["armor_detector"]["max_infer_num"].as<int>()
                );
            }
            std::optional<cv::Rect> detect_light = std::nullopt;
            auto target = armor_target.read();
            if (target.need_focus()) {
                auto T_camera_cv2world = tf->pose_a_in_b(
                    SentryFrame(common_frame.frame_id),
                    SentryFrame(target.get_target_state().frame_id),
                    common_frame.img_frame.timestamp
                );
                net_focus = target.get_net_focus_roi(
                    common_frame.img_frame.timestamp,
                    T_camera_cv2world,
                    camera_info,
                    common_frame.img_frame.src_img.size(),
                    armor_detector.get_net_wh_ratio()
                );
                if (target.need_detect_lights()) {
                    detect_light = target.expanded( // 送给传统越小越好
                        common_frame.img_frame.timestamp,
                        T_camera_cv2world,
                        camera_info,
                        common_frame.img_frame.src_img.size()
                    );
                    detect_light = tools::expand_and_clip_rect(
                        detect_light.value(),
                        1.6,
                        common_frame.img_frame.src_img.size()
                    );
                }
            }

            auto_aim::ArmorDetection armor_detection { .timestamp = common_frame.img_frame.timestamp,
                                      .id = common_frame.id,
                                      .frame_id = common_frame.frame_id };
            {
                bool got = detector_sem->try_acquire();
                tools::SemaphoreGuard guard(*detector_sem, got); //并发控制
                if (got) {
                    auto start = Clock::now();
                    detection.detect_start = start;
                    auto [lights, armors] = armor_detector.detect(common_frame, net_focus, detect_light);
                    armor_detection.armors = armors;
                    armor_detection.lights = lights;
                    log_ctx.detect_count++;
                }
            }
            armor_detections_queue.enqueue(armor_detection);
            auto batch_armors = armor_detections_queue.dequeue_batch(); // 根据id有序输出
            detection.armors = batch_armors;
        } else {
            static std::unique_ptr<std::counting_semaphore<>> detector_sem;
            if (!detector_sem) {
                detector_sem = std::make_unique<std::counting_semaphore<>>(
                    config["rune_detector"]["max_infer_num"].as<int>()
                );
            }
            auto target = rune_target.read();
            if (target.need_focus()) {
                auto T_camera_cv2world = tf->pose_a_in_b(
                    SentryFrame(common_frame.frame_id),
                    SentryFrame(target.get_target_state().frame_id),
                    common_frame.img_frame.timestamp
                );
                net_focus = target.get_net_focus_roi(
                    common_frame.img_frame.timestamp,
                    T_camera_cv2world,
                    camera_info,
                    common_frame.img_frame.src_img.size(),
                    armor_detector.get_net_wh_ratio()
                );
            }
            auto_buff::RuneDetection rune_detection;
            {
                bool got = detector_sem->try_acquire();
                tools::SemaphoreGuard guard(*detector_sem, got); //并发控制
                if (got) {
                    auto start = Clock::now();
                    detection.detect_start = start;
                    rune_detection = rune_detector.detect(common_frame, net_focus, enemy_color);
                    log_ctx.detect_count++;
                }
            }
            rune_detection.frame_id = common_frame.frame_id;
            rune_detection.id = common_frame.id;
            rune_detection.timestamp = common_frame.img_frame.timestamp;
            rune_detections_queue.enqueue(rune_detection);
            auto batch_runes = rune_detections_queue.dequeue_batch();
            detection.runes = batch_runes;
        }

        if (dbg && is_web_running()) {
            dbg->expanded.set(net_focus);
            dbg->img_frame.set(std::move(common_frame.img_frame));
        }

        return std::make_tuple(std::optional<DetIo::second_type>(std::move(detection)));
    });

    scheduler.register_task<DetIo>("tracker", [&](DetIo::second_type&& detection) {
        static std::mutex mutex;
        std::lock_guard<std::mutex> lock(mutex);
        if (detection.armors) {
            for (const auto& armors_raw: detection.armors.value()) {
                auto armor_detection = armors_raw;// ArmorDetection
                auto is_ally = [&](const auto& obj) {
                    return (enemy_color == EnemyColor::BLUE && obj.color == auto_aim::ArmorColor::RED)
                        || (enemy_color == EnemyColor::RED && obj.color == auto_aim::ArmorColor::BLUE);
                };
                armor_detection.armors.erase(
                    std::remove_if(armor_detection.armors.begin(), armor_detection.armors.end(), is_ally),
                    armor_detection.armors.end()
                );
                armor_detection.lights.erase(
                    std::remove_if(armor_detection.lights.begin(), armor_detection.lights.end(), is_ally),
                    armor_detection.lights.end()
                );
                auto T_camera_cv2world = tf->pose_a_in_b(
                    SentryFrame(armor_detection.frame_id),
                    SentryFrame::WORLD,
                    armor_detection.timestamp
                ); //转到world
                armor_detection.frame_id = std::to_underlying(SentryFrame::WORLD);
                auto target_armor = armor_tracker.track(armor_detection, camera_info, T_camera_cv2world, armor_detection.frame_id);
                auto_aim_fsm.update(
                    target_armor.get_target_state().vyaw(),
                    target_armor.jumped,
                    target_armor.get_target_state().timestamp
                );
                armor_target.write(target_armor);
                if (detection.detect_start) {
                    auto latency_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                          Clock::now() - detection.detect_start.value()).count();
                    log_ctx.latency_ms_total += latency_ms;
                }

                log_ctx.found_count += armor_tracker.get_count();
                armor_tracker.reset_count();
                if (dbg) {
                    dbg->armor_detection.set(armor_detection);
#ifdef USE_ROS2
                    rcl::pub_armor_marker(rcl_node, SentryFrame_to_str(armor_detection.frame_id), armor_detection);
                    rcl::pub_armor_target_marker(
                        rcl_node,
                        SentryFrame_to_str(target_armor.get_target_state().frame_id),
                        target_armor
                    );
#endif
                }

                log_ctx.track_count++;
            }
        } else if (detection.runes) {
            for (auto& rune_detection: detection.runes.value()) {
                auto T_camera_cv2world = tf->pose_a_in_b(
                    SentryFrame(rune_detection.frame_id),
                    SentryFrame::WORLD,
                    rune_detection.timestamp
                ); //转到world
                rune_detection.frame_id = std::to_underlying(SentryFrame::WORLD);

                auto target_rune = rune_tracker.track(rune_detection, camera_info, T_camera_cv2world, rune_detection.frame_id);

                rune_target.write(target_rune);
                if (detection.detect_start) {
                    auto latency_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                          Clock::now() - detection.detect_start.value()
                    )
                                          .count();
                    log_ctx.latency_ms_total += latency_ms;
                }
                log_ctx.found_count += rune_tracker.get_count();
                rune_tracker.reset_count();

                if (dbg) {
                    dbg->rune_detection.set(rune_detection);
                }
                log_ctx.track_count++;
            }
        }
    });
    scheduler.add_rate_source<>("solver", 230.0, [&]() {
        log_ctx.solve_count++;
        bool is_omni = false;
        auto target_armor = armor_target.read(); //需要转为相对gimbal_abs
        if (!target_armor.check()) {// 如果没有主目标，尝试使用omni目标
            target_armor = omni_target.read();
            is_omni = true;
        }
        int old_this_id = target_armor.this_id;
        auto gimbal_abs_state_in_world = wheel_odometry.state;
        gimbal_abs_state_in_world.predict(Clock::now());
        target_armor.set_target_state([&](auto& scheduler) {
            namespace idx = auto_aim::idx;
            scheduler.frame_id = std::to_underlying(SentryFrame::GIMBAL_ABS);
            const auto pos = gimbal_abs_state_in_world.pos();
            const auto vel = gimbal_abs_state_in_world.vel();
            scheduler.x[idx::CX] -= pos.x();
            scheduler.x[idx::CY] -= pos.y();
            scheduler.x[idx::CZ] -= pos.z();
            scheduler.x[idx::VCX] -= vel.x();
            scheduler.x[idx::VCY] -= vel.y();
            scheduler.x[idx::VCZ] -= vel.z();
        });
        target_armor.this_id = old_this_id;
        auto target_rune = rune_target.read();
        old_this_id = target_rune.this_id;
        target_rune.set_target_state([&](auto& scheduler) {
            namespace idx = auto_buff::motion_model::idx;
            scheduler.frame_id = std::to_underlying(SentryFrame::GIMBAL_ABS);
            const auto pos = gimbal_abs_state_in_world.pos();
            const auto vel = gimbal_abs_state_in_world.vel();
            scheduler.x[idx::CX] -= pos.x();
            scheduler.x[idx::CY] -= pos.y();
            scheduler.x[idx::CZ] -= pos.z();
        });
        target_rune.this_id = old_this_id;

        GimbalCmd cmd {
            .appear = false,
        };
        auto target_position_valid = [](const auto& target) {
            if (!target.check()) {
                return false;
            }
            const auto p = target.get_target_state().pos();
            const auto v = target.get_target_state().vel();
            // Reject impossible PnP solutions before they reach the ballistic solver.
            // The sentry engagement volume is well inside this bound; a result at
            // tens of metres is a camera/association failure, not a real target.
            return p.allFinite()
                && p.norm() < 15.0
                && p.x() > -8.0 && p.x() < 8.0
                && p.y() > -8.0 && p.y() < 8.0
                && p.z() > -3.0 && p.z() < 5.0
                && v.allFinite() && v.norm() < 10.0;
        };
        auto T_shoot2gimbal_abs =
            tf->pose_a_in_b(SentryFrame::SHOOT, SentryFrame::GIMBAL_ABS, Clock::now());
        auto T_gimbal2gimbal_abs =
            tf->pose_a_in_b(SentryFrame::GIMBAL, SentryFrame::GIMBAL_ABS, Clock::now());
        if (mode == Mode::AutoAim && target_position_valid(target_armor)) {
            cmd = armor_aimer.aim(
                target_armor,
                bullet_speed,
                auto_aim_fsm.get_state(),
                T_shoot2gimbal_abs,
                T_gimbal2gimbal_abs
            );

        } else if (mode == Mode::AutoBuff && target_rune.check()) {
            cmd = rune_aimer.aim(target_rune, bullet_speed, T_shoot2gimbal_abs, T_gimbal2gimbal_abs);
        }

        if (serial || navigation_enabled) {
            if (navigation_enabled && !navigation_feedback_fresh()) {
                cmd = GimbalCmd{};
            }
            if (!cmd.is_valid() || !std::isfinite(cmd.a_yaw) || !std::isfinite(cmd.a_pitch)) {
                DTS_ERROR(
                    "invalid gimbal command; suppressing NaN/Inf serial packet: yaw={} pitch={} target_yaw={} target_pitch={} v_yaw={} v_pitch={} a_yaw={} a_pitch={}",
                    cmd.yaw, cmd.pitch, cmd.target_yaw, cmd.target_pitch,
                    cmd.v_yaw, cmd.v_pitch, cmd.a_yaw, cmd.a_pitch
                );
                armor_aimer.reset();
                cmd = GimbalCmd {};
            }
            SendRobotCmdData send{};
            send.cmd_ID = SendRobotCmdData::ID;
            send.head[0] = 'D', send.head[1] = 'T', send.head[2] = 'S';
            send.time_stamp = std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - start_tp).count();
            send.appear = cmd.appear, send.detect_color = std::to_underlying(enemy_color.load());
            send.yaw = cmd.yaw, send.pitch = cmd.pitch, send.v_yaw = cmd.v_yaw;
            send.target_yaw = cmd.target_yaw, send.target_pitch = cmd.target_pitch;
            send.v_pitch = cmd.v_pitch, send.a_yaw = cmd.a_yaw, send.a_pitch = cmd.a_pitch;
            send.enable_yaw_diff = cmd.enable_yaw_diff;
            send.enable_pitch_diff = cmd.enable_pitch_diff;
            if (serial) {
                serial->write(tools::to_vector(send));
            }
#ifdef USE_NAV_ROS2
            if (navigation) {
                navigation->publish(send);
            }
#endif
        }
        auto T_world2camera_cv = tf->pose_a_in_b(
            SentryFrame(cmd.aim_point.frame_id),
            SentryFrame::CAMERA_CV,
            cmd.timestamp
        );
        cmd.aim_point.transform(T_world2camera_cv, std::to_underlying(SentryFrame::CAMERA_CV));
        if (dbg && is_web_running()) {
            dbg->gimbal_cmd.set(cmd);
        }
    });
    auto armor_omni_task = scheduler.register_source<>("armor_omni");
    scheduler.add_rate_source<>("armor_omni_trigger", armor_omni.fps_, [&]() {
        if (navigation_enabled && !navigation_feedback_fresh()) {
            return;
        }
        auto target_armor = armor_target.read();
        if (target_armor.check()) {// 如果有主目标，暂时不使用omni
            return;
        }
        auto_aim::ArmorOmni::One& one = armor_omni.get_next();
        auto img_frame = one.camera->read();// 从感知相机读取一帧图像
        if (img_frame.src_img.empty()) {
            DTS_WARN("Failed to read image from camera.");
            return;
        }

        CommonFrame common_frame;
        common_frame.img_frame = std::move(img_frame);
        common_frame.frame_id = one.cv_frame_id;
        common_frame.id = one.order_id++;
        const auto omni_frame_id = one.frame_id;

        scheduler.runtime_push_source<>(
            armor_omni_task,
            [&, frame_common = std::move(common_frame), omni_frame_id]() {
            static std::unique_ptr<std::counting_semaphore<>> detector_sem;
            if (!detector_sem) {
                detector_sem = std::make_unique<std::counting_semaphore<>>(config["armor_omni"]["max_infer_num"].as<int>());
            }
            {
                auto_aim::ArmorDetection armor_detection { .timestamp = frame_common.img_frame.timestamp,
                                                            .id = frame_common.id, 
                                                            .frame_id = frame_common.frame_id };
                {
                    bool got = detector_sem->try_acquire();
                    tools::SemaphoreGuard guard(*detector_sem, got);
                    if (got) {
                        auto [omni_lights, omni_armors] = armor_omni.detector_->detect(
                            frame_common,
                            cv::Rect(0, 0, frame_common.img_frame.src_img.cols, frame_common.img_frame.src_img.rows)
                        );
                        armor_detection.lights = omni_lights;
                        armor_detection.armors.reserve(omni_armors.size());
                        for (auto& omni_armor: omni_armors) {
                            if ((enemy_color == EnemyColor::BLUE && omni_armor.color == auto_aim::ArmorColor::RED)
                                || (enemy_color == EnemyColor::RED && omni_armor.color == auto_aim::ArmorColor::BLUE))// 过滤掉己方
                            {
                                continue;
                            }
                            armor_detection.armors.push_back(omni_armor);
                        }
                    }
                }
                if (dbg) {
                    // Publish the image and its detections together to keep boxes frame-aligned.
                    auto debug_frame = frame_common.img_frame.clone();
                    if (debug_frame.format == PixelFormat::RGB) {
                        cv::cvtColor(debug_frame.src_img, debug_frame.src_img, cv::COLOR_RGB2BGR);
                        debug_frame.format = PixelFormat::BGR;
                    }
                    armor_detection.draw(debug_frame.src_img);
                    for (auto& armor : armor_detection.armors) {
                        if (!armor.has_tidy) {
                            continue;
                        }
                        const auto& points = armor.key_points.landmarks();
                        using I = auto_aim::ArmorKeyPointsIndex;
                        const std::vector<cv::Point> outline {
                            points[I::LEFT_TOP], points[I::RIGHT_TOP],
                            points[I::RIGHT_BOTTOM], points[I::LEFT_BOTTOM]
                        };
                        cv::polylines(debug_frame.src_img, outline, true,
                                      cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
                    }
                    if (omni_frame_id == std::to_underlying(SentryFrame::OMNI0)) {
                        dbg->omni0_frame.set(std::move(debug_frame));
                    } else if (omni_frame_id == std::to_underlying(SentryFrame::OMNI1)) {
                        dbg->omni1_frame.set(std::move(debug_frame));
                    }
                }
                one.armor_detections_queue->enqueue(armor_detection);
            }
            auto batch_armors = one.armor_detections_queue->dequeue_batch();
            {
                static std::mutex mutex;
                std::lock_guard<std::mutex> lock(mutex);
                for (auto& armor: batch_armors) {
                    auto T_camera_cv2world = tf->pose_a_in_b(
                        SentryFrame(armor.frame_id),
                        SentryFrame::WORLD,
                        armor.timestamp
                    );
                    armor.frame_id = std::to_underlying(SentryFrame::WORLD);

                    one.target = one.tracker->track(armor, one.camera_info, T_camera_cv2world, armor.frame_id);
                    if (!one.target.check()) {
                        one.target.update_count = 0;
                    }
                    one.total_score = one.target.update_count;
                    if (auto best_target = armor_omni.update()) {
                        omni_target.write(*best_target);
                    }
                    auto latency_ms = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - armor.timestamp).count();
                    log_ctx.omni_latency_ms_total += latency_ms;
                    log_ctx.omni_count++;
                    if (dbg) {
#ifdef USE_ROS2
                        rcl::pub_armor_target_marker(
                            rcl_node,
                            SentryFrame_to_str(best_target->get_target_state().frame_id),
                            *best_target
                        );
#endif
                    }
                }
            }
            }
        );
    });
    scheduler.add_rate_source<>("logger", 1.0, [&]() {
        double avg_latency_ms = log_ctx.latency_ms_total / log_ctx.track_count;
        double omni_avg_latency_ms = log_ctx.omni_latency_ms_total / log_ctx.omni_count;
        DTS_INFO(
            "detect: {} track: {} found: {} solve: {} serial: {} camera: {} avg_latency: {:.3} ms omni: {} avg_latency: {:.3} ms ",
            log_ctx.detect_count,
            log_ctx.track_count,
            log_ctx.found_count,
            log_ctx.solve_count,
            log_ctx.serial_count,
            log_ctx.camera_count,
            avg_latency_ms,
            log_ctx.omni_count,
            omni_avg_latency_ms
        );
        if (dbg) {
            dbg->avg_latency_ms.set(avg_latency_ms);
        }
        log_ctx.reset();
    });
    if (dbg) {
        scheduler.add_rate_source<>("debug", 25.0, [&]() {
            if (!is_web_running()) {
                return;
            }
            static Web web;
            dbg->type = (mode == Mode::AutoAim) ? VisionDebugCtx::AUTO_AIM : VisionDebugCtx::AUTO_BUFF;
            auto target_armor = armor_target.read();
            auto target_rune = rune_target.read();
            target_armor.write_log();
            auto target_omni = omni_target.read();
            target_omni.write_log("omni_target");
            Web::write_log("ros_subscription", [&](auto& json) {
                json["topic"] = "/target_point";
                json["enabled"] = navigation_enabled;
                json["feedback_timeout_ms"] = navigation_timeout_ms;
                json["feedback_fresh"] = navigation_enabled && navigation_feedback_fresh();
                json["status"] = "disabled";
#ifdef USE_NAV_ROS2
                json["compiled"] = true;
                if (navigation) {
                    const auto log = navigation->subscription_log();
                    json["received"] = log.received;
                    json["accepted"] = log.accepted;
                    json["rejected"] = log.rejected;
                    json["status"] = log.received == 0 ? "waiting"
                        : navigation_feedback_fresh() ? "receiving" : "stale_or_invalid";
                    json["last_received_age_ms"] = nullptr;
                    if (log.last_message) {
                        json["last_received_age_ms"] = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - log.last_received).count();
                        const auto& msg = *log.last_message;
                        json["last_message"] = nlohmann::json{
                            {"cmd_id", msg.cmd_id},
                            {"head", std::vector<int>{
                                msg.head[0],
                                msg.head[1],
                                msg.head[2]
                            }},
                            {"yaw", msg.yaw},
                            {"pitch", msg.pitch},
                            {"roll", msg.roll},
                            {"big_yaw_in_world", msg.big_yaw_in_world},
                            {"bullet_speed", msg.bullet_speed},
                            {"bullet_count", msg.bullet_count},
                            {"detect_color", msg.detect_color},
                            {"mode", msg.mode},
                            {"operator_yaw_offset", msg.operator_yaw_offset},
                            {"operator_pitch_offset", msg.operator_pitch_offset},
                            {"time_stamp_pc", msg.time_stamp_pc},
                            {"time_stamp_receive_micro", msg.time_stamp_receive_micro},
                            {"time_stamp_send_micro", msg.time_stamp_send_micro}
                        };
                    }
                }
#else
                json["compiled"] = false;
#endif
            });
            target_rune.write_log();
            wheel_odometry.write_log();
            auto img_now = dbg->img_frame.get().timestamp;
            dbg->armor_target.set(target_armor);
            dbg->rune_target.set(target_rune);
            dbg->auto_aim_fsm_mode.set(auto_aim_fsm.get_state());
            auto T_gimbal2gimbal_abs = tf->pose_a_in_b(SentryFrame::GIMBAL, SentryFrame::GIMBAL_ABS, Clock::now());
            auto ypr = tools::matrix2euler(T_gimbal2gimbal_abs.linear(), tools::EulerOrder::ZYX);
            auto gimbal_yaw_pitch = std::make_pair(angles::to_degrees(ypr[0]), -angles::to_degrees(ypr[1]));
            dbg->gimbal_yaw_pitch.set(gimbal_yaw_pitch);
            web.write_debug_data(dbg.value());
            bullet_reappear.update(
                Clock::now(),
                dbg->gimbal_cmd.get().appear ? dbg->gimbal_cmd.get().fly_time : 0.4
            );
            auto bullet_pos = bullet_reappear.get_bullet_positions(img_now, armor_aimer.get_yaw_pitch_offset());
            auto T_world2camera_cv = tf->pose_a_in_b(SentryFrame::WORLD, SentryFrame::CAMERA_CV, img_now);
            for (auto& pos_bullet: bullet_pos) {
                pos_bullet = T_world2camera_cv * pos_bullet;
            }
            dbg->T_world2camera_cv.set(T_world2camera_cv);
            dbg->bullet_positions.set(bullet_pos);
            auto img = dbg->img_frame.get();
            auto debug_img = img.src_img.clone();
            if (img.format == PixelFormat::RGB) {
                cv::cvtColor(debug_img, debug_img, cv::COLOR_RGB2BGR);
            }
            if (!debug_img.empty()) {
                web.draw(debug_img, dbg.value());
                web.write_shm(debug_img);
            }
            auto omni0 = dbg->omni0_frame.get();
            if (!omni0.src_img.empty()) {
                web.write_shm(omni0.src_img, "/dts_vision_frame_omni0");
            }
            auto omni1 = dbg->omni1_frame.get();
            if (!omni1.src_img.empty()) {
                web.write_shm(omni1.src_img, "/dts_vision_frame_omni1");
            }
        });
#ifdef USE_ROS2
        if (debug) {
            scheduler.add_rate_source<>("tf_pub", 100.0, [&]() {
                rcl_tf.pub_robot_tf(*tf, [](SentryFrame frame) {
                    return SentryFrame_to_str(frame);
                });
            });
        }
#endif
#ifdef USE_RERUN
        if (debug) {
            scheduler.add_rate_source<>("rerun_tf", 15.0, [&]() {
                rerun::log_robot_tf(*tf, [](SentryFrame frame) {
                    return SentryFrame_to_str(frame);
                });
            });
        }
#endif
    }
#ifdef USE_ROS2
    auto cmd_sub = rcl_node.make_sub<geometry_msgs::msg::Twist>(
        "cmd_vel",
        rclcpp::QoS(10),
        [&](const geometry_msgs::msg::Twist::SharedPtr msg) {
            SendNavCmdData send;
            send.cmd_ID = 0x02;
            send.vx = msg->linear.x;
            send.vy = msg->linear.y;
            send.wz = msg->angular.z;
            send.turtle_state = (msg->linear.z > 0) ? true : false;
            if (serial) {
                serial->write(tools::to_vector(send));
            }
        }
    );

    rcl_node.push_sub(cmd_sub);
#endif

    if (camera) {
        camera->start<CameraTag>(camera_type);
    }

    if (serial) {
        serial->start<SerialTag>("serial");
    }

    scheduler.build();
    scheduler.run();

#ifdef USE_ROS2
    std::thread([&]() { rcl_node.spin(); }).detach();
#endif
    tools::SignalGuard::spin(std::chrono::milliseconds(1000));
    scheduler.stop();
#ifdef USE_ROS2
    rcl_node.shutdown();
#endif
    return 0;
}
