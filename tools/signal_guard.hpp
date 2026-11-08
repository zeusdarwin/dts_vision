#pragma once
#include "tools/logger.hpp"
#include <atomic>
#include <csignal>
#include <functional>
#include <iostream>
#include <mutex>
#include <spdlog/spdlog.h>
#include <thread>
#include <vector>
namespace dts_vision::tools {
class SignalGuard {
public:
    static SignalGuard& instance() {
        static SignalGuard inst;
        return inst;
    }

    static bool running() noexcept {
        return instance().running_.load(std::memory_order_relaxed);
    }
    // 把一个一个信号处理回调注册到全局SignalGuard里
    static void add_callback(std::function<void()> cb) {// 参数是一个无参数、无返回值的函数对象，这个cb可以是普通函数、lambda、绑定后的成员函数等
        std::lock_guard<std::mutex> lock(instance().mtx_);// 这里加锁是为了线程安全，防止多个线程同时注册回调时把callbacks_搞乱
        instance().callbacks_.push_back(std::move(cb));// 把回调存进内部数组
    }
    template<typename Duration, typename Fn = std::function<void()>>
    static void spin(
        Duration dur,
        Fn&& fn = []() {}
    ) {
        while (running()) {
            fn();
            std::this_thread::sleep_for(dur);
        }
    }

private:
    SignalGuard() {
        std::signal(SIGINT, handle_signal);
        std::signal(SIGTERM, handle_signal);
    }

    static void handle_signal(int signum) {
        auto& inst = instance();

        if (!inst.running_)
            return;

        inst.running_ = false;

        DTS_INFO("Signal {} received, stopping...", signum);

        std::lock_guard<std::mutex> lock(inst.mtx_);
        for (auto& cb: inst.callbacks_) {
            if (cb)
                cb();
        }
    }

private:
    std::atomic<bool> running_ { true };
    std::vector<std::function<void()>> callbacks_;
    std::mutex mtx_;
};
} // namespace dts_vision::tools
