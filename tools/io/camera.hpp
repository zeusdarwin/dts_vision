#pragma once

#include "tools/common/image.hpp"
#include "tools/logger.hpp"
#include "tools/scheduler/scheduler.hpp"
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <yaml-cpp/node/node.h>

namespace dts_vision::tools::io {

using Scheduler = dts_vision::tools::Scheduler::Scheduler;
template<typename Tag, typename Type>
using IOPair = dts_vision::tools::Scheduler::IOPair<Tag, Type>;

class Camera {
public:
    explicit Camera(Scheduler* scheduler = nullptr): scheduler_(scheduler) {}
    virtual ~Camera() {
        stop_watchdog();
        stop_source_thread();
    }

    virtual void init() {}
    virtual void stop() {
        stop_watchdog();
        stop_source_thread();
    }
    virtual void restart() {}
    virtual bool is_running() const = 0;
    virtual ImageFrame read() = 0;
    virtual bool start_capture() {
        return true;
    }
    virtual double get_ExposureTime() const {
        return 0.0;
    }

    // 相机 SDK(MVS / MindVision)对同一个句柄不保证线程安全: 控制通道和取流通道并发
    // 访问会把取流打死 —— 现象是 MV_CC_GetImageBuffer 一直返回 MV_E_NODATA, 设备却还在
    // 总线上枚举着, 而且再也不恢复。auto_exposure 是注册在 scheduler 里的 task, 跑在
    // 工作线程上, 所以这里只登记请求, 真正的 SDK 下发由采集线程在两帧之间完成。
    // 声明成 final 是为了防止驱动再把直接下发的实现加回来。
    virtual void set_ExposureTime(double exposure_time) final {
        requested_exposure_.store(exposure_time, std::memory_order_relaxed);
        exposure_pending_.store(true, std::memory_order_release);
    }

    template<typename Tag>
    void start(const std::string& source_name) {
        if (!scheduler_) {
            throw std::runtime_error("camera source start requires a Scheduler");
        }
        using IO = IOPair<Tag, ImageFrame>;
        source_snapshot_id_ = scheduler_->register_source<IO>(source_name);
        if (!start_capture()) {
            DTS_ERROR("camera failed to start capture");
            return;
        }
        last_frame_ms_.store(steady_now_ms(), std::memory_order_relaxed);
        source_thread_running_ = true;
        source_thread_ = std::thread(&Camera::run_source_loop<IO>, this);
        start_watchdog();
    }

protected:
    // 由驱动实现真正的下发, 只会在采集线程里被调用。
    virtual void write_exposure_time(double) {
        DTS_WARN("camera does not support exposure control");
    }

    // 驱动可以主动上报"取流已经被 SDK 中止", 这样不必等无帧超时。
    virtual bool stream_faulted() const {
        return false;
    }

    void flush_exposure_request() {
        if (!exposure_pending_.exchange(false, std::memory_order_acquire)) {
            return;
        }
        write_exposure_time(requested_exposure_.load(std::memory_order_relaxed));
    }

    // no_frame_restart_ms: 多久没出帧就重启相机(走驱动的 restart)。
    // no_frame_abort_ms:   多久没出帧就直接让进程非零退出, 由 guard.sh 整体拉起;
    //                      0 表示关闭。采集线程可能卡在 SDK 内部出不来, 这时任何跨线程
    //                      的 SDK 调用只会让情况更糟, 重启进程是唯一可靠的兜底。
    void configure_watchdog(const YAML::Node& config) {
        if (!config) {
            return;
        }
        if (config["no_frame_restart_ms"]) {
            no_frame_restart_ =
                std::chrono::milliseconds(config["no_frame_restart_ms"].as<int>());
        }
        if (config["no_frame_abort_ms"]) {
            no_frame_abort_ = std::chrono::milliseconds(config["no_frame_abort_ms"].as<int>());
        }
    }

    void stop_source_thread() {
        source_thread_running_ = false;
        if (source_thread_.joinable()) {
            source_thread_.join();
        }
    }

    template<typename IO>
    void run_source_loop() {
        auto last_attempt = std::chrono::steady_clock::now();
        last_frame_ms_.store(steady_now_ms(), std::memory_order_relaxed);

        while (source_thread_running_ && is_running()) {
            flush_exposure_request();

            auto frame = read();
            const auto now = std::chrono::steady_clock::now();

            if (!frame.src_img.empty()) {
                last_attempt = now;
                last_frame_ms_.store(steady_now_ms(), std::memory_order_relaxed);
                scheduler_->runtime_push_source<IO>(
                    source_snapshot_id_,
                    [f = std::move(frame)]() mutable {
                        return std::make_tuple(std::optional<typename IO::second_type>(std::move(f))
                        );
                    }
                );
                continue;
            }

            // 按"多久没出帧"判, 不按连续失败次数判: SDK 卡死的时候 read() 的超时参数会
            // 失效(实测单次 GetImageBuffer 阻塞了十几秒), 数到第 11 次失败要等 33s。
            const bool faulted = stream_faulted();
            const auto idle = now - last_attempt;
            if (!faulted && idle < no_frame_restart_) {
                continue;
            }

            DTS_WARN(
                "camera: no frame for {} ms{}, restarting",
                std::chrono::duration_cast<std::chrono::milliseconds>(idle).count(),
                faulted ? " (SDK reported the stream was aborted)" : ""
            );
            restart();
            // 只重置"重启节流"的基准, 不重置真实出帧时间, 否则反复重启会把下面的
            // 硬看门狗一直往后顶, 永远不触发。
            last_attempt = std::chrono::steady_clock::now();
        }
    }

    static int64_t steady_now_ms() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now().time_since_epoch()
        )
            .count();
    }

    void start_watchdog() {
        if (no_frame_abort_.count() <= 0) {
            return;
        }
        watchdog_running_ = true;
        watchdog_thread_ = std::thread([this]() {
            while (watchdog_running_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                if (!watchdog_running_) {
                    break;
                }
                const auto idle = steady_now_ms() - last_frame_ms_.load(std::memory_order_relaxed);
                if (idle < no_frame_abort_.count()) {
                    continue;
                }
                DTS_CRITICAL(
                    "camera watchdog: no frame for {} ms, aborting the process so guard.sh can restart it",
                    idle
                );
                ::dts_vision::tools::logger::get_logger()->flush();
                std::_Exit(EXIT_FAILURE);
            }
        });
    }

    void stop_watchdog() {
        watchdog_running_ = false;
        if (watchdog_thread_.joinable()) {
            watchdog_thread_.join();
        }
    }

    Scheduler* scheduler_ = nullptr;
    size_t source_snapshot_id_ {};
    std::atomic<bool> source_thread_running_ { false };
    std::thread source_thread_;

    std::atomic<double> requested_exposure_ { 0.0 };
    std::atomic<bool> exposure_pending_ { false };

    std::chrono::milliseconds no_frame_restart_ { 1000 };
    std::chrono::milliseconds no_frame_abort_ { 0 };
    std::atomic<int64_t> last_frame_ms_ { 0 };
    std::atomic<bool> watchdog_running_ { false };
    std::thread watchdog_thread_;
};

} // namespace dts_vision::tools::io
