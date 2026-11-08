#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#ifdef USE_RERUN
    #include "tasks/rerun/recorder.hpp"
    #include "tasks/rerun/tf.hpp"
#endif
#include "backward-cpp/backward.hpp"
#include "video_stream.pb.h"
#include "tasks/auto_aim/armor_control/armor_aimer.hpp"
#include "tasks/auto_aim/armor_detect/armor_detector.hpp"
#include "tasks/auto_aim/armor_track/armor_target.hpp"
#include "tasks/auto_aim/armor_track/armor_tracker.hpp"
#include "tasks/auto_aim/armor_track/motion_model.hpp"
#include "tasks/auto_aim/auto_aim_fsm.hpp"
#include "tasks/auto_aim/type.hpp"
#include "tasks/auto_client/decoder.hpp"
#include "tasks/auto_client/encoder.hpp"
#include "tasks/base/common.hpp"
#include "tasks/base/packet_serial.hpp"
#include "tasks/base/web.hpp"
#include "tasks/base/ballistic_trajectory.hpp"
#include "tasks/base/wheel_odometry.hpp"
#include "tools/common/ascii_banner.hpp"
#include "tools/common/config.hpp"
#include "tools/common/image.hpp"
#include "tools/common/type_common.hpp"
#include "tools/io/camera_factory.hpp"
#include "tools/io/video_save.hpp"
#include "tools/io/serial_driver.hpp"
#include "tools/scheduler/scheduler.hpp"
#include "tools/param_deliver.h"
#include "tools/buffer.hpp"
#include "tools/logger.hpp"
#include "tools/runtime_tf.hpp"
#include "tools/semaphore_guard.hpp"
#include "tools/signal_guard.hpp"
#include "tools/utils.hpp"
namespace backward {
static backward::SignalHandling sh;
}
using namespace dts_vision;
enum class Mode : int { AutoAim = 0, AutoClient = 1 };
Mode str_to_Mode(std::string str) {
    str = tools::to_upper(str);
    if (str == "AUTOAIM") {
        return Mode::AutoAim;
    } else if (str == "AUTOCLIENT") {
        return Mode::AutoClient;
    } else {
        throw std::invalid_argument("Invalid mode string");
    }
}
enum class SimpleFrame : int { WORLD, GIMBAL_ABS, GIMBAL, CAMERA, CAMERA_CV, SHOOT, N };

using SimpleRobotTF = tools::RobotTF<SimpleFrame, static_cast<size_t>(SimpleFrame::N), false>;

std::string SimpleFrame_to_str(int frame) {
    constexpr const char* details[] = { "world",   "gimbal_abs", "gimbal",
                                        "camera", "camera_cv",   "shoot" };
    return std::string(details[frame]);// 从整数转换为字符串
}
std::string SimpleFrame_to_str(SimpleFrame frame) {// 从枚举值转换为字符串
    return SimpleFrame_to_str(std::to_underlying(frame));// 从枚举值转换为整数
}
struct CameraTag {};
struct SerialTag {};
struct DetectTag {};
struct FrameTag {};
/*struct IOPair {
    using first_type = Tag;
    using second_type = Type;
};*/
using CameraIO = tools::Scheduler::IOPair<CameraTag, ImageFrame>;
using SerialIO = tools::Scheduler::IOPair<SerialTag, std::vector<uint8_t>>;
using CommonFrameIo = tools::Scheduler::IOPair<FrameTag, CommonFrame>;
using DetIo = tools::Scheduler::IOPair<DetectTag, std::vector<auto_aim::ArmorDetection>>;
struct LogCtx {
    int camera_count = 0;
    int detect_count = 0;
    int track_count = 0;
    int solve_count = 0;
    int serial_count = 0;
    int found_count = 0;
    double latency_ms_total = 0.0;
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
            cached = (ret == 0);// 1s监测一次web进程是否运行
        },
        std::chrono::duration<double>(1.0)
    );
    bool running = cached.load();
#ifdef USE_RERUN
    running = running || rerun::Recorder::instance().enabled();
#endif
    return running;
}
static constexpr auto RECORD_FOLDER_PATH_ARR = tools::concat(ROOT_DIR, "/record/hero");
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

    tools::Scheduler::Scheduler scheduler;
    EnemyColor enemy_color = enemy_color_from_string(config["common"]["enemy_color"].as<std::string>());
    double bullet_speed = config["common"]["bullet_speed"].as<double>();
    std::unique_ptr<tools::io::SerialDriver> serial;
    if (config["common"]["serial"]["enable"].as<bool>()) {
        serial = std::make_unique<tools::io::SerialDriver>(config["common"]["serial"], scheduler);
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
    Mode mode = Mode::AutoAim;
    CameraInfo camera_info;
    camera_info.load(camera_config["camera_info"]);
    auto_aim::ArmorDetector armor_detector(config["armor_detector"]);
    auto_aim::ArmorTracker armor_tracker(config["armor_tracker"]);
    auto_aim::AutoAimFsm auto_aim_fsm(config["auto_aim_fsm"]);
    auto_aim::ArmorAimer armor_aimer(config["armor_aimer"]);
    tools::OrderedQueue<auto_aim::ArmorDetection> armor_detections_queue;
    tools::SWMR<auto_aim::ArmorTarget> armor_target;
    BulletReappear bullet_reappear(config["common"]["ballistic_trajectory"]);
    LogCtx log_ctx;
    std::optional<VisionDebugCtx> auto_aim_dbg;
    std::pair<double, double> operator_offset = std::make_pair(0, 0);// 操作手调偏置
    if (debug) {
        auto_aim_dbg.emplace();
        auto_aim_dbg->camera_info_ = camera_info;
    }
    WheelOdometry wheel_odometry(config["common"]["wheel_odometry"], Clock::now());
    auto_client::Encoder encoder(config["encoder"]);
    auto tf = SimpleRobotTF::create();
    {
        for (auto [from, to]: std::array {
                 std::pair { SimpleFrame::WORLD, SimpleFrame::GIMBAL_ABS },
                 std::pair { SimpleFrame::GIMBAL_ABS, SimpleFrame::GIMBAL },
                 std::pair { SimpleFrame::GIMBAL, SimpleFrame::CAMERA },
                 std::pair { SimpleFrame::GIMBAL, SimpleFrame::SHOOT },
                 std::pair { SimpleFrame::CAMERA, SimpleFrame::CAMERA_CV },
             })
        {
            tf->add_edge(from, to);
        }
        ISO3 T_camera_cv2camera = ISO3::Identity();
        T_camera_cv2camera.linear() = R_cv2physics;
        tf->push(SimpleFrame::CAMERA, SimpleFrame::CAMERA_CV, Clock::now(), T_camera_cv2camera);
        tf->push(
            SimpleFrame::GIMBAL,
            SimpleFrame::CAMERA,
            Clock::now(),
            tools::load_isometry3(config["common"]["tf"]["T_camera2gimbal"])
        );
        tf->push(
            SimpleFrame::GIMBAL,
            SimpleFrame::SHOOT,
            Clock::now(),
            tools::load_isometry3(config["common"]["tf"]["T_shoot2gimbal"])
        );
    }
    if (video_saver) {
        scheduler.register_task<CameraIO>("save_video", [&](CameraIO::second_type&& image_frame) {
            if (!image_frame.src_img.empty()) {
                video_saver->write_frame(image_frame.src_img);
            }
            return std::make_tuple(std::optional<CameraIO::second_type>(std::nullopt));
        });
    }

    scheduler.register_task<CameraIO, CommonFrameIo>("push_common_frame", [&](CameraIO::second_type&& image_frame) {
        static int current_id = 0;
        if (image_frame.src_img.empty()) {
            return std::make_tuple(std::optional<CommonFrameIo::second_type>(std::nullopt));
        }
        log_ctx.camera_count++;

        CommonFrame common_frame {
            .img_frame = std::move(image_frame),
            .id = current_id++,
            .frame_id = std::to_underlying(SimpleFrame::CAMERA_CV),
        };

        return std::make_tuple(std::optional<CommonFrameIo::second_type>(std::move(common_frame)));
    });
    if (serial) {
        scheduler.register_task<SerialIO>("receive_serial", [&](SerialIO::second_type&& data) {
            static std::mutex mutex;
            std::lock_guard<std::mutex> lock(mutex);
            auto now = Clock::now();

            log_ctx.serial_count++;
            if (auto robo_opt = ReceiveRobotData::create(data); robo_opt.has_value()) {
                auto robo = robo_opt.value();
                static uint32_t last_pc = -1, delay = 0, last_bullet_count = 0;
                if (robo.time_stamp_pc != last_pc) {
                    last_pc = robo.time_stamp_pc;
                    delay = (std::chrono::duration_cast<std::chrono::microseconds>(now - start_tp).count() - robo.time_stamp_pc
                             - (robo.time_stamp_send_micro - robo.time_stamp_receive_micro)) / 2.0;
                }
                auto packet_time = now - std::chrono::microseconds(delay);
                ISO3 T_gimbal2gimbal_abs = ISO3::Identity();
                T_gimbal2gimbal_abs.linear() = tools::euler2matrix(Vec3(
                    angles::from_degrees(robo.yaw),
                    angles::from_degrees(robo.pitch),
                    angles::from_degrees(robo.roll)
                ), tools::EulerOrder::ZYX);
                tf->push(
                    SimpleFrame::GIMBAL_ABS,
                    SimpleFrame::GIMBAL,
                    packet_time,
                    T_gimbal2gimbal_abs
                );
                operator_offset = { angles::from_degrees(robo.operator_yaw_offset),
                                    angles::from_degrees(robo.operator_pitch_offset) };
                ISO3 T_gimbal_abs2world = ISO3::Identity();
                T_gimbal_abs2world.translation() = wheel_odometry.state.pos();
                tf->push(
                    SimpleFrame::WORLD,
                    SimpleFrame::GIMBAL_ABS,
                    packet_time,
                    T_gimbal_abs2world
                );
                enemy_color = EnemyColor(robo.detect_color);
                bullet_speed = robo.bullet_speed;
                if (bullet_speed > 12.2) {
                    bullet_speed = 12.0;
                }
                robo.update_log(delay);
                if (robo.bullet_count > last_bullet_count) {
                    bullet_reappear.push_back(Bullet {
                        .fire_time = Clock::now(),
                        .T_shoot2world = tf->pose_a_in_b(SimpleFrame::SHOOT, SimpleFrame::WORLD, Clock::now()),
                        .speed_in_world = bullet_speed,
                    });
                }
                last_bullet_count = robo.bullet_count;
            }
            if (auto mode_opt = HeroMode::create(data); mode_opt.has_value()) {
                mode = mode_opt.value().mode == 1 ? Mode::AutoClient : Mode::AutoAim;
            }
        });
    }
    if (camera) {
        scheduler.register_task<CommonFrameIo>("auto_exposure", [&](CommonFrameIo::second_type&& common_frame) {
            static std::mutex mutex;
            std::lock_guard<std::mutex> lock(mutex);
            if (mode == Mode::AutoClient) {
                camera->set_ExposureTime(5000);
                // camera->set_AcquisitionFrameRate(30);
                return;
            }
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
                        exposure_time = std::clamp(exposure_time, cfg.exposure_min, cfg.exposure_max);
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
        static std::unique_ptr<std::counting_semaphore<>> detector_sem;
        if (!detector_sem) {
            detector_sem = std::make_unique<std::counting_semaphore<>>(config["common"]["max_infer_num"].as<int>());
        }
        static std::unique_ptr<std::counting_semaphore<>> client_sem;// 创建一个计数信号量，用来限制自定义客户端任务的并发数量
        if (!client_sem) {
            client_sem = std::make_unique<std::counting_semaphore<>>(1);
        }/*1表示这个信号量初始有一个许可：许可数量 = 1，因此它实际上相当于：同一时间最多只允许一个client任务进入*/
        std::optional<cv::Rect> detect_light = std::nullopt;
        auto target_armor = armor_target.read();
        cv::Rect net_focus = cv::Rect(0, 0, common_frame.img_frame.src_img.cols, common_frame.img_frame.src_img.rows);
        if (target_armor.need_focus()) {
            auto T_camera_cv2world = tf->pose_a_in_b(
                SimpleFrame(common_frame.frame_id),
                SimpleFrame(target_armor.get_target_state().frame_id),// world
                common_frame.img_frame.timestamp
            );
            target_armor.set_target_state([&](auto_aim::State& state) {
                state.predict(common_frame.img_frame.timestamp, target_armor.target_type);
            });
            net_focus = target_armor.get_net_focus_roi(
                common_frame.img_frame.timestamp,
                T_camera_cv2world,
                camera_info,
                common_frame.img_frame.src_img.size(),
                armor_detector.get_net_wh_ratio()
            );
            if (target_armor.need_detect_lights()) {
                detect_light = target_armor.expanded( // 送给传统越小越好
                    common_frame.img_frame.timestamp,
                    T_camera_cv2world,
                    camera_info,
                    common_frame.img_frame.src_img.size()
                );
                detect_light->x -= detect_light->width * 0.3;
                detect_light->y -= detect_light->height * 0.3;
                detect_light->width *= 1.6;
                detect_light->height *= 1.6;
                detect_light.value() &= cv::Rect(0, 0, common_frame.img_frame.src_img.cols, common_frame.img_frame.src_img.rows);
            }
        }
        auto_aim::ArmorDetection armor_detection { .timestamp = common_frame.img_frame.timestamp,
                                  .id = common_frame.id,
                                  .frame_id = common_frame.frame_id };
        if (mode == Mode::AutoAim) {
            bool got = detector_sem->try_acquire();
            tools::SemaphoreGuard guard(*detector_sem, got);
            if (got) {
                auto [lights, armors] = armor_detector.detect(common_frame, net_focus, detect_light);
                armor_detection.armors = armors;
                armor_detection.lights = lights;
                log_ctx.detect_count++;
            }
        }
        if (mode == Mode::AutoClient) {
            bool got = client_sem->try_acquire();
            tools::SemaphoreGuard guard(*client_sem, got);
            if (got) {
                encoder.push_frame(common_frame.img_frame.src_img);
                auto_client::BlindSend pkg;
                while (encoder.try_pop_packet(pkg)) {
                    if (serial && config["common"]["serial"]["enable"].as<bool>()) {
                        std::array<uint8_t, auto_client::MAX_PACKET_SIZE> raw {};
                        std::memcpy(raw.data(), &pkg, auto_client::MAX_PACKET_SIZE);

                        doorlock_sniper::CustomByteBlock block;
                        block.set_data(raw.data(), auto_client::MAX_PACKET_SIZE);

                        std::string serialized;
                        if (!block.SerializeToString(&serialized)) {
                            DTS_ERROR("Protobuf serialization failed");
                            continue;
                        }
                        serial->write(std::vector<uint8_t>(serialized.begin(), serialized.end()));
                    }
                }
            }
            return std::make_tuple(std::optional<DetIo::second_type>(std::nullopt));
        }
        armor_detections_queue.enqueue(armor_detection);
        auto batch_armors = armor_detections_queue.dequeue_batch();
        if (auto_aim_dbg && is_web_running()) {
            auto_aim_dbg->expanded.set(net_focus);
            auto_aim_dbg->img_frame.set(std::move(common_frame.img_frame));
        }
        return std::make_tuple(std::optional<DetIo::second_type>(std::move(batch_armors)));
    });

    scheduler.register_task<DetIo>("tracker", [&](DetIo::second_type&& detection) {// hero只有装甲板一种检测对象
        static std::mutex mutex;
        std::lock_guard<std::mutex> lock(mutex);
        for (const auto& armor: detection) {
            auto armor_detection = armor;
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
            auto T_camera_cv2world = tf->pose_a_in_b(SimpleFrame(armor_detection.frame_id), SimpleFrame::WORLD, armor_detection.timestamp);
            armor_detection.frame_id = std::to_underlying(SimpleFrame::WORLD);
            auto target_armor = armor_tracker.track(armor_detection, camera_info, T_camera_cv2world, armor_detection.frame_id);
            auto_aim_fsm.update(
                target_armor.get_target_state().vyaw(),
                target_armor.jumped,
                target_armor.get_target_state().timestamp
            );

            armor_target.write(target_armor);

            auto latency_ms = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - armor_detection.timestamp).count();
            log_ctx.latency_ms_total += latency_ms;
            log_ctx.found_count += armor_tracker.get_count();
            armor_tracker.reset_count();
            if (auto_aim_dbg) {
                auto_aim_dbg->armor_detection.set(armor_detection);
#ifdef USE_ROS2
                rcl::pub_armor_marker(rcl_node, SimpleFrame_to_str(armor_detection.frame_id), armor_detection);
                rcl::pub_armor_target_marker(
                    rcl_node,
                    SimpleFrame_to_str(target_armor.get_target_state().frame_id),
                    target_armor
                );
#endif
            }

            log_ctx.track_count++;
        }
    });
    scheduler.add_rate_source<>("solver", 1000.0, [&]() {
        log_ctx.solve_count++;
        auto target = armor_target.read(); //需要转为相对gimbal_odom
        int old_this_id = target.this_id;
        auto gimbal_abs_state_in_world = wheel_odometry.state;
        gimbal_abs_state_in_world.predict(Clock::now());
        target.set_target_state([&](auto& scheduler) {
            namespace idx = auto_aim::idx;
            const auto pos = gimbal_abs_state_in_world.pos();
            const auto vel = gimbal_abs_state_in_world.vel();
            scheduler.frame_id = std::to_underlying(SimpleFrame::GIMBAL_ABS);
            scheduler.x[idx::CX] -= pos.x();
            scheduler.x[idx::CY] -= pos.y();
            scheduler.x[idx::CZ] -= pos.z();
            scheduler.x[idx::VCX] -= vel.x();
            scheduler.x[idx::VCY] -= vel.y();
            scheduler.x[idx::VCZ] -= vel.z();
        });
        target.this_id = old_this_id;
        GimbalCmd cmd {
            .appear = false,
        };
        auto T_shoot2gimbal_abs =
            tf->pose_a_in_b(SimpleFrame::SHOOT, SimpleFrame::GIMBAL_ABS, Clock::now());
        auto T_gimbal2gimbal_abs =
            tf->pose_a_in_b(SimpleFrame::GIMBAL, SimpleFrame::GIMBAL_ABS, Clock::now());
        if (target.check()) {
            auto auto_aim_fsm_mode = auto_aim_fsm.get_state();
            if (auto_aim_fsm_mode != auto_aim::AutoAimFsmMode::AIM_SINGLE_ARMOR
                && target.target_type == auto_aim::ArmorType::OUTPOST)
            {
                //   auto_aim_fsm_mode = auto_aim::AutoAimFsm::AIM_WHOLE_CAR_CENTER;
            }
            armor_aimer.set_operator_offset(operator_offset);
            cmd = armor_aimer.aim(
                target,
                bullet_speed,
                auto_aim_fsm_mode,
                T_shoot2gimbal_abs,
                T_gimbal2gimbal_abs
            );
        }

        if (serial) {
            if (!cmd.is_valid()) {
                DTS_ERROR(
                    "invalid gimbal command; suppressing NaN/Inf serial packet: yaw={} pitch={} target_yaw={} target_pitch={} v_yaw={} v_pitch={} a_yaw={} a_pitch={}",
                    cmd.yaw, cmd.pitch, cmd.target_yaw, cmd.target_pitch,
                    cmd.v_yaw, cmd.v_pitch, cmd.a_yaw, cmd.a_pitch
                );
                cmd = GimbalCmd {};
            }
            SendRobotCmdData send;
            send.cmd_ID = 0x01;
            send.time_stamp = std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - start_tp).count();
            send.appear = cmd.appear, send.detect_color = std::to_underlying(enemy_color);
            send.yaw = cmd.yaw, send.pitch = cmd.pitch, send.v_yaw = cmd.v_yaw;
            send.target_yaw = cmd.target_yaw, send.target_pitch = cmd.target_pitch;
            send.v_pitch = cmd.v_pitch, send.a_yaw = cmd.a_yaw, send.a_pitch = cmd.a_pitch;
            send.enable_yaw_diff = cmd.enable_yaw_diff;
            send.enable_pitch_diff = cmd.enable_pitch_diff;
            serial->write(std::move(tools::to_vector(send)));
        }
        auto T_world2camera_cv = tf->pose_a_in_b(
            SimpleFrame(cmd.aim_point.frame_id),// world
            SimpleFrame::CAMERA_CV,
            cmd.timestamp
        );
        cmd.aim_point.transform(T_world2camera_cv, std::to_underlying(SimpleFrame::CAMERA_CV));
        if (auto_aim_dbg && is_web_running()) {
            auto_aim_dbg->gimbal_cmd.set(cmd);
        }
    });
    scheduler.add_rate_source<>("logger", 1.0, [&]() {
        double avg_latency_ms = log_ctx.latency_ms_total / log_ctx.track_count;
        DTS_INFO(
            "detect: {} track: {} found: {} solve: {} serial: {} camera: {} avg_latency: {:.3} ms",
            log_ctx.detect_count,
            log_ctx.track_count,
            log_ctx.found_count,
            log_ctx.solve_count,
            log_ctx.serial_count,
            log_ctx.camera_count,
            avg_latency_ms
        );
        if (auto_aim_dbg) {
            auto_aim_dbg->avg_latency_ms.set(avg_latency_ms);
        }
        log_ctx.reset();
    });
    if (auto_aim_dbg) {
        scheduler.add_rate_source<>("debug", 45.0, [&]() {
            if (!is_web_running()) {
                return;
            }
            auto target_armor = armor_target.read();
            target_armor.write_log();
            wheel_odometry.write_log();
            auto img_now = auto_aim_dbg->img_frame.get().timestamp;
            auto_aim_dbg->armor_target.set(target_armor);
            auto_aim_dbg->auto_aim_fsm_mode.set(auto_aim_fsm.get_state());
            auto T_gimbal2gimbal_abs =
                tf->pose_a_in_b(SimpleFrame::GIMBAL, SimpleFrame::GIMBAL_ABS, Clock::now());
            auto ypr = tools::matrix2euler(T_gimbal2gimbal_abs.linear(), tools::EulerOrder::ZYX);
            auto gimbal_yaw_pitch =
                std::make_pair(angles::to_degrees(ypr[0]), -angles::to_degrees(ypr[1]));
            auto_aim_dbg->gimbal_yaw_pitch.set(gimbal_yaw_pitch);
            static Web web;
            web.write_debug_data(auto_aim_dbg.value());
            bullet_reappear.update(
                Clock::now(),
                auto_aim_dbg->gimbal_cmd.get().appear ? auto_aim_dbg->gimbal_cmd.get().fly_time
                                                      : 0.4
            );
            auto bullet_pos = bullet_reappear.get_bullet_positions(img_now, armor_aimer.get_yaw_pitch_offset());
            auto T_world2camera_cv = tf->pose_a_in_b(SimpleFrame::WORLD, SimpleFrame::CAMERA_CV, img_now);
            for (auto& pos_bullet: bullet_pos) {
                pos_bullet = T_world2camera_cv * pos_bullet;
            }// T_world2camera_cv * T_bullet2world = T_bullet2camera_cv
            auto_aim_dbg->T_world2camera_cv.set(T_world2camera_cv);
            auto_aim_dbg->bullet_positions.set(bullet_pos);
            auto img = auto_aim_dbg->img_frame.get();
            auto debug_img = img.src_img;
            if (img.format == PixelFormat::RGB) {
                cv::cvtColor(debug_img, debug_img, cv::COLOR_RGB2BGR);
            }
            static cv::Mat last_draw;
            if (!debug_img.empty() && debug_img.data != last_draw.data) {
                web.draw(debug_img, auto_aim_dbg.value());
                web.write_shm(debug_img);
                last_draw = debug_img;
            }
        });
#ifdef USE_ROS2
        scheduler.add_rate_source<>("tf_pub", 100.0, [&]() {
            rcl_tf.pub_robot_tf(*tf, [](SimpleFrame frame) { return SimpleFrame_to_str(frame); });
        });
#endif
#ifdef USE_RERUN
        scheduler.add_rate_source<>("rerun_tf", 15.0, [&]() {
            rerun::log_robot_tf(*tf, [](SimpleFrame frame) {
                return SimpleFrame_to_str(frame);
            });
        });
#endif
    }

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
