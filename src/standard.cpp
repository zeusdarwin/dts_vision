#include "angles.h"
#include "tasks/daedalus_interface/shm_layout.hpp"
#include "tasks/auto_aim/armor_track/motion_model.hpp"
#include "tasks/auto_buff/rune_control/rune_aimer.hpp"
#include "tasks/auto_buff/rune_detect/rune_detector.hpp"
#include "tasks/auto_buff/rune_track/rune_target.hpp"
#include "tasks/auto_buff/rune_track/rune_tracker.hpp"
#include "tasks/auto_buff/type.hpp"
#include "tasks/base/ballistic_trajectory.hpp"
#include "tasks/base/wheel_odometry.hpp"
#include "tools/io/camera_factory.hpp"
#include "tools/io/video_save.hpp"
#include "tools/io/serial_driver.hpp"
#include "tools/common/ascii_banner.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <opencv2/highgui.hpp>
#include <optional>
#include <string>
#include <utility>
#ifdef USE_RERUN
    #include "tasks/rerun/recorder.hpp"
    #include "tasks/rerun/tf.hpp"
#endif
#ifdef USE_ROS2
    #include "sensor_msgs/msg/camera_info.hpp"
    #include "sensor_msgs/msg/image.hpp"
    #include "tools/_rcl/tf.hpp"
    #include "tools/_rcl/visual/armor.hpp"
    #include "tools/_rcl/visual/armor_target.hpp"
    #include "tools/tools/_rcl/node.hpp"
    #include <rclcpp/qos.hpp>
#endif
#include "backward-cpp/backward.hpp"
#ifdef USE_Daedalus
    #include "tasks/daedalus_interface/shm_client.hpp"
#endif
#include "tasks/auto_aim/armor_control/armor_aimer.hpp"
#include "tasks/auto_aim/armor_detect/armor_detector.hpp"
#include "tasks/auto_aim/armor_track/armor_target.hpp"
#include "tasks/auto_aim/armor_track/armor_tracker.hpp"
#include "tasks/auto_aim/auto_aim_fsm.hpp"
#include "tasks/auto_aim/type.hpp"
#include "tasks/base/common.hpp"
#include "tasks/base/packet_serial.hpp"
#include "tasks/base/web.hpp"
#include "tools/buffer.hpp"
#include "tools/common/config.hpp"
#include "tools/common/image.hpp"
#include "tools/common/type_common.hpp"
#include "tools/io/camera.hpp"
#include "tools/logger.hpp"
#include "tools/param_deliver.h"
#include "tools/runtime_tf.hpp"
#include "tools/scheduler/scheduler.hpp"
#include "tools/semaphore_guard.hpp"
#include "tools/signal_guard.hpp"
#include "tools/utils.hpp"
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
enum class SimpleFrame : int { WORLD, GIMBAL_ABS, GIMBAL, CAMERA, CAMERA_CV, SHOOT, N };

using SimpleRobotTF = tools::RobotTF<SimpleFrame, static_cast<size_t>(SimpleFrame::N), false>;

std::string SimpleFrame_to_str(int simple_frame) {
    constexpr const char* details[] = { "world",  "gimbal_abs", "gimbal",
                                        "camera", "camera_cv",  "shoot" };
    return std::string(details[simple_frame]);
}
std::string SimpleFrame_to_str(SimpleFrame frame) {
    return SimpleFrame_to_str(std::to_underlying(frame));
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
static constexpr auto RECORD_FOLDER_PATH_ARR = tools::concat(ROOT_DIR, "/record/auto_aim");
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
    std::atomic<Mode> mode = str_to_Mode(config["common"]["mode"].as<std::string>());
    std::atomic<EnemyColor> enemy_color = enemy_color_from_string(config["common"]["enemy_color"].as<std::string>());
    std::atomic<double> bullet_speed = config["common"]["bullet_speed"].as<double>();
#ifdef USE_ROS2
    rcl::RclcppNode rcl_node("dts_vision");
    rcl::TF rcl_tf(rcl_node);
#endif
    bool use_daedalus = false;
#ifdef USE_Daedalus
    std::unique_ptr<talos::ipc::ShmClient> daedalus_shm_client;
    if (config["common"]["use_sim"].as<bool>()) {
        auto client = talos::ipc::ShmClient::connect();
        if (!client) {
            DTS_ERROR("Failed to connect to talos::ipc::ShmClient");
            return 1;
        } else {
            use_daedalus = true;
            daedalus_shm_client = std::make_unique<talos::ipc::ShmClient>(std::move(*client));
        }
    }
#endif
    std::unique_ptr<tools::io::SerialDriver> serial;
    if (!use_daedalus && config["common"]["serial"]["enable"].as<bool>()) {
        serial = std::make_unique<tools::io::SerialDriver>(config["common"]["serial"], scheduler);
    }

    auto camera_config = config["common"]["camera"];
    std::unique_ptr<tools::io::Camera> camera;
    auto camera_type = camera_config["type"].as<std::string>();
    tools::SignalGuard::add_callback([&]() {
        if (camera) {
            camera->stop();
        }
    });
    if (!use_daedalus) {
        camera = tools::io::create_camera(camera_config, scheduler, camera_type);
        camera->init();
        if (!camera->is_running()) {
            return 0;
        }
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
    auto_aim::AutoAimFsm auto_aim_fsm(config["auto_aim_fsm"]);
    auto_aim::ArmorAimer armor_aimer(config["armor_aimer"]);
    auto_buff::RuneDetector rune_detector(config["rune_detector"]);
    auto_buff::RuneTracker rune_tracker(config["rune_tracker"]);
    auto_buff::RuneAimer rune_aimer(config["rune_aimer"]);
    tools::OrderedQueue<auto_aim::ArmorDetection> armor_detections_queue;
    tools::OrderedQueue<auto_buff::RuneDetection> rune_detections_queue;
    tools::SWMR<auto_aim::ArmorTarget> armor_target;
    tools::SWMR<auto_buff::RuneTarget> rune_target;
    BulletReappear bullet_reappear(config["common"]["bullet_reappear"]);
    LogCtx log_ctx;
    std::optional<VisionDebugCtx> dbg;
    std::pair<double, double> operator_offset = std::make_pair(0, 0);
    if (debug) {
        dbg.emplace();
        dbg->camera_info_ = camera_info;
    }
    WheelOdometry wheel_odometry(config["common"]["wheel_odometry"], Clock::now());
    auto tf = SimpleRobotTF::create();// 智能指针
    {
        for (auto [from, to]:
             std::array {
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
    auto serial_send_to_image_microseconds = config["common"]["serial"]["serial_send_to_image_microseconds"].as<int>();// 当前收到的下位机云台姿态数据应该对应当前图像帧多久前
#ifdef USE_Daedalus
    if (daedalus_shm_client) {
        auto daedalus_imgs = scheduler.register_source<CameraIO>("daedalus_img");

        auto push_pose = [&](talos::ipc::PoseIndex idx, SimpleFrame parent, SimpleFrame child) {
            if (auto pose = daedalus_shm_client->recv_pose(idx)) {
                ISO3 T = ISO3::Identity();
                T.translation() << pose->x, pose->y, pose->z;
                T.linear() =
                    Quaternion { pose->qw, pose->qx, pose->qy, pose->qz }.toRotationMatrix();

                tf->push(parent, child, TimePoint(std::chrono::nanoseconds(pose->timestamp_ns)), T);
            }
        };

        scheduler.add_rate_source<>("daedalus_tick", 300.0, [&]() {
            static bool has_camera_info = false;

            if (!has_camera_info) {
                auto info = daedalus_shm_client->camera_info();
                has_camera_info = true;
                camera_info.camera_matrix = cv::Mat::eye(3, 3, CV_64F);
                camera_info.camera_matrix.at<double>(0, 0) = info.fx;
                camera_info.camera_matrix.at<double>(1, 1) = info.fy;
                camera_info.camera_matrix.at<double>(0, 2) = info.cx;
                camera_info.camera_matrix.at<double>(1, 2) = info.cy;
                camera_info.distortion_coefficients = cv::Mat(1, 5, CV_64F);
                std::memcpy(
                    camera_info.distortion_coefficients.ptr<double>(),
                    info.distortion,
                    5 * sizeof(double)
                );
            }

            if (auto frame = daedalus_shm_client->recv_image()) {
                ImageFrame img_frame {
                    .src_img = std::move(frame->image),
                    .format = PixelFormat::RGB,
                    .timestamp = TimePoint(std::chrono::nanoseconds(frame->timestamp_ns)),
                };
                scheduler.runtime_push_source<CameraIO>(
                    daedalus_imgs,
                    [f = std::move(img_frame)]() mutable {
                        return std::make_tuple(std::optional<CameraIO::second_type>(std::move(f)));
                    }
                );
            }
            if (auto ground_truth = daedalus_shm_client->recv_ground_truth()) {
                if (dbg) {
                    dbg->ground_truth.set(ground_truth.value());
                }
            }

            push_pose(talos::ipc::PoseIndex::POSE_CAMERA, SimpleFrame::GIMBAL, SimpleFrame::CAMERA);
            push_pose(talos::ipc::PoseIndex::POSE_MUZZLE, SimpleFrame::GIMBAL, SimpleFrame::SHOOT);
            push_pose(
                talos::ipc::PoseIndex::POSE_GIMBAL,
                SimpleFrame::GIMBAL_ABS,
                SimpleFrame::GIMBAL
            );
        });
    }
#endif
    if (video_saver) {
        scheduler.register_task<CameraIO>("save_video", [&](CameraIO::second_type&& img_frame) {
            if (!img_frame.src_img.empty()) {
                video_saver->write_frame(img_frame.src_img);
            }
            return std::make_tuple(std::optional<CameraIO::second_type>(std::nullopt));
        });
    }
    scheduler.register_task<CameraIO, CommonFrameIo>("push_common_frame", [&](CameraIO::second_type&& img_frame) {
        static int current_id = 0;
        if (img_frame.src_img.empty()) {
            return std::make_tuple(std::optional<CommonFrameIo::second_type>(std::nullopt));
        }
        log_ctx.camera_count++;
        CommonFrame common_frame {
            .img_frame = std::move(img_frame),
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
                    delay = (std::chrono::duration_cast<std::chrono::microseconds>(now - start_tp).count() - robo.time_stamp_pc // (now - start_tp) - robo.time_stamp_pc = 上位机接收时刻时间戳 - 上位机发送时刻时间戳，这里面还有一个下位机处理数据的时间段即(robo.time_stamp_send_micro - robo.time_stamp_receive_micro)
                             - (robo.time_stamp_send_micro - robo.time_stamp_receive_micro))// 下位机接收 - 下位机处理数据并变换到云台命令要求的云台位姿 -> 下位机发送
                        / 2.0;// 整个过程时间 = 上位机发送（云台命令） -> 下位机接收 - 下位机处理数据，变换到云台命令要求的云台位姿 -> 下位机发送 -> 上位机接收（云台位姿），再假设上行传输延迟（PC→STM32）与下行传输延迟（STM32→PC）相等，则单向传输延迟为delay/2
                }

                auto packet_time =  now - std::chrono::microseconds(serial_send_to_image_microseconds);// 下位机发送的云台位姿真正对应的时间戳
                ISO3 T_gimbal2gimbal_abs = ISO3::Identity();
                T_gimbal2gimbal_abs.linear() = tools::euler2matrix(
                    Vec3(
                        angles::from_degrees(robo.yaw),
                        angles::from_degrees(robo.pitch),
                        angles::from_degrees(robo.roll)
                    ),
                    tools::EulerOrder::ZYX
                );
                double yaw_robo = angles::from_degrees(robo.yaw);
                // DTS_INFO("yaw = {}", yaw_robo);
                tf->push(
                    SimpleFrame::GIMBAL_ABS,
                    SimpleFrame::GIMBAL,
                    packet_time,
                    T_gimbal2gimbal_abs
                );
                operator_offset = { angles::from_degrees(robo.operator_yaw_offset),
                                    angles::from_degrees(robo.operator_pitch_offset) };
                auto T_gimbal_abs2world = ISO3::Identity();
                // T_gimbal_abs2world.translation() = Vec3(robo.v_x, robo.v_y, robo.v_z);
                // tf->push(
                //     SimpleFrame::WORLD,
                //     SimpleFrame::GIMBAL_ABS,
                //     packet_time,
                //     T_gimbal_abs2world
                // );
                enemy_color = EnemyColor(robo.detect_color);
                bullet_speed = robo.bullet_speed;
                robo.update_log(delay);
                if (robo.bullet_count > last_bullet_count) {
                    bullet_reappear.push_back(
                        Bullet {
                            .fire_time = Clock::now(),
                            .T_shoot2world = tf->pose_a_in_b(
                                SimpleFrame::SHOOT,
                                SimpleFrame::WORLD,
                                Clock::now()
                            ),
                            .speed_in_world = bullet_speed,
                        }
                    );
                }
                last_bullet_count = robo.bullet_count;
            }
        });
    }
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
        cv::Rect net_focus =
            cv::Rect(0, 0, common_frame.img_frame.src_img.cols, common_frame.img_frame.src_img.rows);
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
                    SimpleFrame(common_frame.frame_id),
                    SimpleFrame(target.get_target_state().frame_id),
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
                    armor_detection.lights = lights;
                    armor_detection.armors = armors;
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
                    SimpleFrame(common_frame.frame_id),
                    SimpleFrame(target.get_target_state().frame_id),
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
                auto armor_detection = armors_raw;
                auto is_ally = [&](const auto& obj) {
                    return (enemy_color == EnemyColor::BLUE
                            && obj.color == auto_aim::ArmorColor::RED)
                        || (enemy_color == EnemyColor::RED
                            && obj.color == auto_aim::ArmorColor::BLUE);
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
                    SimpleFrame(armor_detection.frame_id),
                    SimpleFrame::WORLD,
                    armor_detection.timestamp
                ); //转到world
                armor_detection.frame_id = std::to_underlying(SimpleFrame::WORLD);
                auto target_armor = armor_tracker.track(armor_detection, camera_info, T_camera_cv2world, armor_detection.frame_id);
                auto_aim_fsm.update(
                    target_armor.get_target_state().vyaw(),
                    target_armor.jumped,
                    target_armor.get_target_state().timestamp
                );
                armor_target.write(target_armor);
                if (detection.detect_start) {
                    auto latency_ms = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - detection.detect_start.value()).count();
                    log_ctx.latency_ms_total += latency_ms;
                }

                log_ctx.found_count += armor_tracker.get_count();
                armor_tracker.reset_count();
                if (dbg) {
                    dbg->armor_detection.set(armor_detection);
                }

                log_ctx.track_count++;
            }
        } else if (detection.runes) {
            for (auto& rune_detection: detection.runes.value()) {
                auto T_camera_cv2world = tf->pose_a_in_b(
                    SimpleFrame(rune_detection.frame_id),
                    SimpleFrame::WORLD,
                    rune_detection.timestamp
                ); //转到world
                rune_detection.frame_id = std::to_underlying(SimpleFrame::WORLD);

                auto target_rune = rune_tracker.track(rune_detection, camera_info, T_camera_cv2world, rune_detection.frame_id);

                rune_target.write(target_rune);
                if (detection.detect_start) {
                    auto latency_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                          Clock::now() - detection.detect_start.value()
                    ).count();
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
        auto target_armor = armor_target.read(); //需要转为相对gimbal_abs
        int old_this_id = target_armor.this_id;// 1,2,3,4
        auto gimbal_abs_state_in_world = wheel_odometry.state;
        gimbal_abs_state_in_world.predict(Clock::now());
        target_armor.set_target_state([&](auto& scheduler) {
            namespace idx = auto_aim::idx;
            scheduler.frame_id = std::to_underlying(SimpleFrame::GIMBAL_ABS);
            const auto pos = gimbal_abs_state_in_world.pos();
            const auto vel = gimbal_abs_state_in_world.vel();
            scheduler.x[idx::CX] -= pos.x();
            scheduler.x[idx::CY] -= pos.y();
            scheduler.x[idx::CZ] -= pos.z();
            scheduler.x[idx::VCX] -= vel.x();
            scheduler.x[idx::VCY] -= vel.y();
            scheduler.x[idx::VCZ] -= vel.z();
            // scheduler.x[idx::VCX] =0;
            // scheduler.x[idx::VCY] =0;
            // scheduler.x[idx::VCZ] =0;
            // scheduler.x[idx::VYAW] =12.0;
        });
        target_armor.this_id = old_this_id;
        auto target_rune = rune_target.read();
        old_this_id = target_rune.this_id;
        target_rune.set_target_state([&](auto& scheduler) {
            namespace idx = auto_buff::motion_model::idx;
            scheduler.frame_id = std::to_underlying(SimpleFrame::GIMBAL_ABS);
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
        auto T_shoot2gimbal_abs =
            tf->pose_a_in_b(SimpleFrame::SHOOT, SimpleFrame::GIMBAL_ABS, Clock::now());
        auto T_gimbal2gimbal_abs =
            tf->pose_a_in_b(SimpleFrame::GIMBAL, SimpleFrame::GIMBAL_ABS, Clock::now());
        if (target_armor.check()) {
            armor_aimer.set_operator_offset(operator_offset);
            cmd = armor_aimer.aim(
                target_armor,
                bullet_speed,
                auto_aim_fsm.get_state(),
                T_shoot2gimbal_abs,
                T_gimbal2gimbal_abs
            );

        } else if (target_rune.check()) {
            rune_aimer.set_operator_offset(operator_offset);
            cmd = rune_aimer.aim(target_rune, bullet_speed, T_shoot2gimbal_abs, T_gimbal2gimbal_abs);
            bool can_fire = rune_aimer.tick_fire(target_rune, cmd);
            if (can_fire) {
                DTS_INFO("you can fire!");
                cmd.fire_advice = true;
            } else {
                cmd.fire_advice = false;
            }
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
            send.cmd_ID = SendRobotCmdData::ID;
            send.head[0] = 'D', send.head[1] = 'T', send.head[2] = 'S';
            send.time_stamp = std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - start_tp).count();
            send.appear = cmd.appear, send.detect_color = std::to_underlying(enemy_color.load());
            send.yaw = cmd.yaw, send.pitch = cmd.pitch, send.v_yaw = cmd.v_yaw;
            send.target_yaw = cmd.target_yaw, send.target_pitch = cmd.target_pitch;
            send.v_pitch = cmd.v_pitch, send.a_yaw = cmd.a_yaw, send.a_pitch = cmd.a_pitch;
            send.enable_yaw_diff = cmd.enable_yaw_diff;
            send.enable_pitch_diff = cmd.enable_pitch_diff;
            if (tools::is_finite(send)) {
                serial->write(std::move(tools::to_vector(send)));
            }
        }
#ifdef USE_Daedalus
        if (daedalus_shm_client) {
            auto T_gimbal2gimbal_abs =
                tf->pose_a_in_b(SimpleFrame::GIMBAL, SimpleFrame::GIMBAL_ABS, Clock::now());
            auto ypr =
                tools::matrix2euler(T_gimbal2gimbal_abs.linear(), tools::EulerOrder::ZYX);// pitch向下为正
            static TimePoint last_fire;
            bool fire = cmd.fire_advice
                && (std::abs(
                        angles::shortest_angular_distance_degrees(
                            angles::to_degrees(ypr[0]),
                            cmd.target_yaw
                        )
                    ) < cmd.enable_yaw_diff
                    && std::abs(
                           angles::shortest_angular_distance_degrees(
                               angles::to_degrees(-ypr[1]),
                               cmd.target_pitch
                           )
                       ) < cmd.enable_pitch_diff);
            if (fire) {
                last_fire = Clock::now();
            }
            bool send_fire = false;
            if (Clock::now() - last_fire < std::chrono::milliseconds(100)) {
                send_fire = true;
            }
            daedalus_shm_client
                ->send_gimbal_cmd(cmd.yaw, -cmd.pitch, cmd.appear ? 1.0 : -1.0, send_fire);
            // daedalus_shm_client
            //     ->send_gimbal_cmd(cmd.yaw, -cmd.pitch, cmd.appear ? 1.0 : -1.0, false);
        }
#endif
        auto T_world2camera_cv = tf->pose_a_in_b(
            SimpleFrame(cmd.aim_point.frame_id),
            SimpleFrame::CAMERA_CV,
            cmd.timestamp
        );
        cmd.aim_point.transform(T_world2camera_cv, std::to_underlying(SimpleFrame::CAMERA_CV));
        if (dbg && is_web_running()) {
            dbg->gimbal_cmd.set(cmd);
        }
    });
    scheduler.add_rate_source<>("logger", 1.0, [&]() {
        // No camera frames may arrive during a capture/restart gap.
        const double avg_latency_ms = log_ctx.detect_count > 0
            ? log_ctx.latency_ms_total / log_ctx.detect_count
            : 0.0;
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
        if (dbg) {
            dbg->avg_latency_ms.set(avg_latency_ms);
        }
        log_ctx.reset();
    });
    if (dbg) {
            scheduler.add_rate_source<>("debug", 25.0, [&]() {
            auto target_armor = armor_target.read();
            if (is_web_running()) {
                static Web web;
                dbg->type =
                    (mode == Mode::AutoAim) ? VisionDebugCtx::AUTO_AIM : VisionDebugCtx::AUTO_BUFF;
                auto target_rune = rune_target.read();
                target_armor.write_log();
                target_rune.write_log();
                wheel_odometry.write_log();
                auto img_now = dbg->img_frame.get().timestamp;
                dbg->armor_target.set(target_armor);
                dbg->rune_target.set(target_rune);
                dbg->auto_aim_fsm_mode.set(auto_aim_fsm.get_state());
                auto T_gimbal2gimbal_abs =
                    tf->pose_a_in_b(SimpleFrame::GIMBAL, SimpleFrame::GIMBAL_ABS, Clock::now());
                auto ypr = tools::matrix2euler(T_gimbal2gimbal_abs.linear(), tools::EulerOrder::ZYX);
                auto gimbal_yaw_pitch = std::make_pair(angles::to_degrees(ypr[0]), -angles::to_degrees(ypr[1]));
                dbg->gimbal_yaw_pitch.set(gimbal_yaw_pitch);
                bullet_reappear.update(
                    Clock::now(),
                    dbg->gimbal_cmd.get().appear ? dbg->gimbal_cmd.get().fly_time : 0.4
                );
                auto bullet_pos = bullet_reappear.get_bullet_positions(img_now, armor_aimer.get_yaw_pitch_offset());
                auto T_world2camera_cv = tf->pose_a_in_b(SimpleFrame::WORLD, SimpleFrame::CAMERA_CV, img_now);
                for (auto& pos_bullet: bullet_pos) {
                    pos_bullet = T_world2camera_cv * pos_bullet;// 转到camera_cv
                }// 整个bullet_pos变成相对于camera_cv的坐标系
                dbg->T_world2camera_cv.set(T_world2camera_cv);
                dbg->bullet_positions.set(bullet_pos);
                web.write_debug_data(dbg.value());
                auto img = dbg->img_frame.get();
                auto debug_img = img.src_img.clone();
                if (img.format == PixelFormat::RGB) {
                    cv::cvtColor(debug_img, debug_img, cv::COLOR_RGB2BGR);
                }
                if (!debug_img.empty()) {
                    web.draw(debug_img, dbg.value());
                    web.write_shm(debug_img);
                }
            }

#ifdef USE_ROS2
            auto armors = dbg->armor_detection.get();
            rcl::pub_armor_target_marker(
                rcl_node,
                SimpleFrame_to_str(__armor_target.get_target_state().frame_id),
                __armor_target
            );
#endif
        });
#ifdef USE_ROS2
        if (debug) {
            scheduler.add_rate_source<>("tf_pub", 100.0, [&]() {
                rcl_tf.pub_robot_tf(*tf, [](SimpleFrame frame) {
                    return SimpleFrame_to_str(frame);
                });
            });
        }

#endif
#ifdef USE_RERUN
        if (debug) {
            scheduler.add_rate_source<>("rerun_tf", 15.0, [&]() {
                rerun::log_robot_tf(*tf, [](SimpleFrame frame) {
                    return SimpleFrame_to_str(frame);
                });
            });
        }
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
