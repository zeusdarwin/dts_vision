#pragma once
#include "tools/tf.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace dts_vision::tools {

struct LinkBuffer {
    TimePoseBuffer buffer;
    LinkBuffer(size_t size = 4096): buffer(size) {}// 初始化缓冲区大小
};

// ---------------- Edge ----------------
template<typename FrameEnum>
struct Edge {
    FrameEnum parent;
    FrameEnum child;
};

// ---------------- RobotTF ----------------
template<typename FrameEnum, size_t N, bool Static>
class RobotTF {
public:
    using Ptr = std::shared_ptr<RobotTF>;
    static Ptr create() {
        return std::make_shared<RobotTF>();
    }

    void add_edge(FrameEnum parent, FrameEnum child) {
        std::lock_guard<std::mutex> lock(path_cache_mutex_);
        if (!valid_frame(parent) || !valid_frame(child)) {// 父，子坐标系是否有效
            if constexpr (Static) {
                throw std::out_of_range("Invalid TF edge " + frame_to_string(parent) + " -> " + frame_to_string(child));
            } else {
                return;
            }
        }
        if (has_direct_edge(parent, child))
            return;
        edges_.push_back({ parent, child });
        adjacency_[to_index(parent)].push_back(child);// 建立坐标系连接
        adjacency_[to_index(child)].push_back(parent);
        directed_edge_[to_index(parent)][to_index(child)] = true;// 标记有向边
        path_cache_ = {};
    }

    bool push(FrameEnum parent, FrameEnum child, const TimePoint& t, const ISO3& pose_in_parent) {
        if (!valid_frame(parent) || !valid_frame(child)) {
            if constexpr (Static) {
                throw std::out_of_range("Invalid TF edge " + frame_to_string(parent) + " -> " + frame_to_string(child));
            } else {
                return false;
            }
        }

        if (!has_direct_edge(parent, child)) {
            if constexpr (Static) {
                throw std::runtime_error("Attempt to push pose for undefined edge " + frame_to_string(parent) + " -> " + frame_to_string(child));
            } else {
                return false;
            }
        }

        buffers_[to_index(parent)][to_index(child)].buffer.push(t, pose_in_parent);// 有锁，存的是T_child2parent
        return true;
    }

    ISO3 pose_a_in_b(FrameEnum a, FrameEnum b, const TimePoint& t) const {// T_a2b(a是终点，b是起点)
        if (!valid_frame(a) || !valid_frame(b)) {
            if constexpr (Static) {
                throw std::out_of_range("Invalid TF query " + frame_to_string(a) + " in " + frame_to_string(b));
            } else {
                return ISO3::Identity();
            }
        }

        auto path = find_path(b, a);
        if (!path) {
            if constexpr (Static) {
                throw std::runtime_error("No path from " + frame_to_string(a) + " to " + frame_to_string(b));
            } else {
                std::cout << "No path from " + frame_to_string(a) + " to " + frame_to_string(b) << std::endl;
                return ISO3::Identity();
            }
        }

        ISO3 T = ISO3::Identity();
        for (size_t i = 0; i + 1 < path->size(); ++i) {
            auto start = (*path)[i];
            auto goal = (*path)[i + 1];

            ISO3 T_a2b = ISO3::Identity();
            if (has_direct_edge(start, goal)) {
                T_a2b = buffers_[to_index(start)][to_index(goal)].buffer.get(t);// 按时间戳t获取goal到start的位姿T_goal2start,get有锁
            } else if (has_direct_edge(goal, start)) {
                T_a2b = buffers_[to_index(goal)][to_index(start)].buffer.get(t).inverse();
            } else {
                if constexpr (Static) {
                    throw std::runtime_error( "Broken path edge between " + frame_to_string(start) + " and " + frame_to_string(goal));
                } else {
                    T_a2b = ISO3::Identity();
                }
            }

            T = T * T_a2b;// 经过T_a2b_1 * T_b_12b_2 * ... * T_b_n2b = T_a2b
        }
        return T;
    }

    std::vector<Edge<FrameEnum>> get_edges() const {
        return edges_;
    }

private:
    std::array<std::array<LinkBuffer, N>, N> buffers_;// N个坐标系，一个坐标系可以有N个连接，给每一个连接分配一个缓冲区
    std::vector<Edge<FrameEnum>> edges_;// 存储边
    std::array<std::vector<FrameEnum>, N> adjacency_;// N个坐标系，存储各自的连接
    std::array<std::array<bool, N>, N> directed_edge_ {};// N个坐标系，一个坐标系可以有N个连接，给每一个连接分配一个bool值，表示是否已经建立坐标系联系
    // 坐标系路径缓存，每个std::optional初始化为空状态，第一层std::array放N个起点，第二层std::array放N个std::optional<std::vector<FrameEnum>>，是N条可能存在的路径(因为有N个终点)
    mutable std::array<std::array<std::optional<std::vector<FrameEnum>>, N>, N> path_cache_ {};
    mutable std::mutex path_cache_mutex_;// 读写锁

    bool has_direct_edge(FrameEnum A, FrameEnum B) const {// 判断是否已经建立坐标系联系
        return valid_frame(A) && valid_frame(B) && directed_edge_[to_index(A)][to_index(B)];
    }

    static size_t to_index(FrameEnum frame_enum) {
        using Underlying = std::underlying_type_t<FrameEnum>;
        return static_cast<size_t>(static_cast<Underlying>(frame_enum));
    }

    static bool valid_frame(FrameEnum frame_enum) {
        using Underlying = std::underlying_type_t<FrameEnum>;
        auto value = static_cast<Underlying>(frame_enum);// int
        if constexpr (std::is_signed_v<Underlying>) {
            if (value < 0)
                return false;
        }
        return static_cast<size_t>(value) < N;
    }

    static std::string frame_to_string(FrameEnum frame_enum) {// 是序号
        using Underlying = std::underlying_type_t<FrameEnum>;
        return std::to_string(static_cast<Underlying>(frame_enum));
    }

    std::optional<std::vector<FrameEnum>> find_path(FrameEnum start, FrameEnum goal) const {
        std::lock_guard<std::mutex> lock(path_cache_mutex_);// 读写锁，独占
        auto& path_cached = path_cache_[to_index(start)][to_index(goal)];// 索引是起点终点，看是否已经存在路径
        if (path_cached)
            return path_cached;

        std::array<bool, N> visited {};// N个坐标系，不可重复走过
        std::array<std::optional<FrameEnum>, N> path_node {};// N个坐标系，不可重复走过，记录上一个经过的节点
        std::queue<FrameEnum> frame_enum_queue;

        frame_enum_queue.push(start);// 将起点加入队列
        visited[to_index(start)] = true;// 该起点设为已访问

        while (!frame_enum_queue.empty()) {
            auto node = frame_enum_queue.front();// 取队首元素
            frame_enum_queue.pop();// 移除队首元素
            if (node == goal)// 如果当前节点是终点，说明找到了路径
                break;

            for (auto neighbor: adjacency_[to_index(node)]) {// adjacency_[to_index(node)]是个std::vector<FrameEnum>，放的是当前节点能到达的节点(有边的)
                auto neighbor_index = to_index(neighbor);
                if (visited[neighbor_index])// 避免重复访问和反向
                    continue;
                visited[neighbor_index] = true;// 最后一个应该是终点
                path_node[neighbor_index] = node;// 记录途经的坐标系节点，当前path_node记录的是上一个节点
                frame_enum_queue.push(neighbor);
            }
        }

        if (!visited[to_index(goal)])
            return std::nullopt;

        std::vector<FrameEnum> path;
        for (auto node = goal; ; node = *path_node[to_index(node)]) {// *path_node[to_index(node)]是上一个经过的节点
            path.push_back(node);// 从终点往回放进路径，因为path_node记录的是上一个节点，得从后往前推
            if (node == start)
                break;
        }
        std::reverse(path.begin(), path.end());// 反转路径，因为是从终点往回推的
        path_cached = std::move(path);
        return path_cached;
    }
};

} // namespace dts_vision::tools
