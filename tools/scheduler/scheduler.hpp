#pragma once
#include "node.hpp"
#include "tools/logger.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include <tbb/task_arena.h>
#include <tbb/task_group.h>

namespace dts_vision::tools::Scheduler {

class Scheduler {
public:
    using clock = std::chrono::steady_clock;
    inline static unsigned int hardware_concurrency_ = std::thread::hardware_concurrency();// 当前系统支持的硬件线程数

    explicit Scheduler(size_t threads = hardware_concurrency_): // 构造函数
        worker_count_(threads ? threads : 1),
        arena_(static_cast<int>(worker_count_)) {} // 最大并发数为硬件线程数

    ~Scheduler() {
        stop();// 析构rate_threads_和group_
    }

    template<typename InputPair, typename... OutputPairs, typename Fn>
    void register_task(std::string name, Fn&& fn) {
        auto node = TaskNode<InputPair, OutputPairs...>::create(std::forward<Fn>(fn));
        node->name = std::move(name);

        auto input_tags = node->input_tags();
        if (input_tags.empty()) {
            throw std::runtime_error("Task must have input");
        }

        static_tasks_snapshot_[input_tags.front()].push_back(node);// 键是输入tag，值是节点指针
        built_ = false;
    }

    template<typename... OutputPairs>
    [[nodiscard]] size_t register_source(std::string name) {
        auto node = SourceNode<OutputPairs...>::create();
        node->name = std::move(name);

        source_snapshot_.push_back(node);
        built_ = false;
        return source_snapshot_.size() - 1;
    }

    template<typename... OutputPairs, typename Fn>
    void runtime_push_source(size_t snap_id, Fn&& fn) {
        if (!built_)
            build();

        if (!is_running())// 如果调度器没有运行，直接返回
            return;

        if (snap_id >= source_snapshot_.size()) {
            throw std::out_of_range("Invalid source snapshot id");
        }

        static_assert(
            std::is_convertible_v<Fn, typename SourceNode<OutputPairs...>::Func>,
            "Fn must be convertible to SourceNode::Func"
        );

        auto& base_node_ptr = source_snapshot_[snap_id];
        auto local_ptr = base_node_ptr->clone();// 克隆一个新的节点，避免多线程访问同一个节点导致数据竞争，也避免修改原始节点
        auto source_node_ptr = std::static_pointer_cast<SourceNode<OutputPairs...>>(local_ptr);// 转换为SourceNode类型的智能指针
        source_node_ptr->fn = typename SourceNode<OutputPairs...>::Func(std::forward<Fn>(fn));

        schedule(local_ptr);// 把process_queue加入线程池
    }

    template<typename... OutputPairs, typename Fn>
    void add_rate_source(std::string name, double rate, Fn&& fn) {
        if (rate <= 0.0) {
            throw std::invalid_argument("rate must be > 0");
        }

        
        static_assert(std::is_convertible_v<Fn, typename SourceNode<OutputPairs...>::Func>, "Fn must match SourceNode::Func");

        auto node = SourceNode<OutputPairs...>::create(typename SourceNode<OutputPairs...>::Func(std::forward<Fn>(fn)));
        node->name = std::move(name);

        connect(node);

        rate_workers_.emplace_back(RateWorker { node, rate });
    }

    void run() {
        if (running_.exchange(true, std::memory_order_acq_rel))
            return;

        for (auto& rate_worker: rate_workers_) {
            rate_threads_.emplace_back([this, rate_worker](std::stop_token st) {
                const auto period = std::chrono::duration_cast<clock::duration>(
                    std::chrono::duration<double>(1.0 / rate_worker.rate)
                );

                auto next_time = clock::now();

                while (!st.stop_requested() && running_.load(std::memory_order_acquire)) {
                    next_time += period;

                    const auto& next_node = rate_worker.node->execute();
                    for (auto& node: next_node) {
                        schedule(node);
                    }

                    auto now = clock::now();
                    if (now > next_time) {
                        next_time = now;
                        continue;
                    }

                    std::this_thread::sleep_until(next_time);
                }
            });
        }
    }

    void stop() {
        // !true->stop(), !false->return
        if (!running_.exchange(false, std::memory_order_acq_rel))// 防止多个线程进入stop()，多次执行下面的group_.cancel()和group_.wait()。这不仅浪费性能，某些并发场景下甚至会因为重复操作导致死锁或崩溃
            return;

        for (auto& jthread_: rate_threads_) {
            if (jthread_.joinable())
                jthread_.request_stop();
        }
        rate_threads_.clear();

        group_.cancel();// 阻止新的任务继续被捞出来执行，让系统的负载尽快降下来
        group_.wait();// 等待所有任务执行完毕，注意这里可能会阻塞很久，因为有些任务可能会被阻塞在外部资源上，比如网络IO、串口IO等

        DTS_INFO("dts_vision Scheduler stopped");
    }

    void build() {
        if (built_)
            return;

        for (auto& [_, nodes]: static_tasks_snapshot_) {
            for (auto& node: nodes) {
                connect(node);
            }
        }

        for (auto& node: source_snapshot_) {
            connect(node);
        }

        for (auto& [_, nodes]: static_tasks_snapshot_) {
            for (auto& node: nodes) {
                if (node->connected_count == 0) {// 如果节点没有下游节点，说明它是孤立的
                    throw std::runtime_error("Node '" + node->name + "' is isolated");
                }
            }
        }

        built_ = true;
    }

    bool is_running() const {
        return running_.load(std::memory_order_acquire);// 原子读取
    }

private:
    struct RateWorker {
        NodeBase::Ptr node;
        double rate;
    };

    void schedule(NodeBase::Ptr node) {
        if (!node || !running_.load(std::memory_order_acquire))
            return;

        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            task_queue_.push(node);// 指针队列
        }

        // Submit work through the bounded arena so Scheduler tasks cannot
        // expand to TBB's process-wide worker limit.
        arena_.execute([this] {group_.run([this] { process_queue(); });});
    }

    void process_queue() {
        NodeBase::Ptr node;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (task_queue_.empty())
                return;
            node = task_queue_.front();
            task_queue_.pop();
        }

        const auto& next = node->execute();
        for (auto& n: next) {
            schedule(n);
        }
    }

    void connect(NodeBase::Ptr node) {
        for (auto& output_tag: node->output_tags()) {
            auto it = static_tasks_snapshot_.find(output_tag);// 找tag对应的哈希表
            if (it == static_tasks_snapshot_.end())// 没有找到
                continue;

            for (auto& ptr: it->second) {// 遍历tag对应的哈希表的值(指针列表)
                node->connected_count++;
                ptr->connected_count++;// 每个节点都记录了有多少个下游节点，这样在build()时可以判断是否有孤立的节点
                node->add_downstream(output_tag, ptr->clone());// 多态类不能直接拷贝，只能转移指针所有权
            }
        }
    }

private:
    size_t worker_count_;
    tbb::task_arena arena_;

    std::unordered_map<std::type_index, std::vector<NodeBase::Ptr>> static_tasks_snapshot_;
    std::vector<NodeBase::Ptr> source_snapshot_;

    std::vector<RateWorker> rate_workers_;
    std::vector<std::jthread> rate_threads_;

    tbb::task_group group_;

    std::atomic<bool> running_ { false };
    bool built_ { false };

    std::queue<NodeBase::Ptr> task_queue_;
    std::mutex queue_mutex_;
};

} // namespace dts_vision::tools::Scheduler
