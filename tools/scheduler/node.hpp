#pragma once
#include <algorithm>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <stop_token>
#include <string>
#include <thread>
#include <tuple>
#include <type_traits>
#include <typeindex>
#include <unordered_map>
#include <utility>
#include <vector>
namespace dts_vision::tools::Scheduler {
enum class NodeKind {
    Source,
    Task,
};

template<typename Tag, typename Type>
struct IOPair {
    using first_type = Tag;
    using second_type = Type;
};

template<std::size_t I, typename... Ts>
using nth_type_t = typename std::tuple_element<I, std::tuple<Ts...>>::type;// 获取元组中第I个OutputPairs,是一个IOPair类型

template<typename... Ts>
static std::vector<std::type_index> make_tag_index_vec() {
    return { std::type_index(typeid(Ts))... };
}

struct NodeBase {
    using Ptr = std::shared_ptr<NodeBase>;
    NodeKind kind { NodeKind::Task };// 节点类型
    std::string name;
    int connected_count { 0 };
    virtual ~NodeBase() = default;
    virtual const std::vector<Ptr>& execute() = 0;
    virtual bool receive(std::type_index tag, const void* data, std::type_index value_type) = 0;
    virtual void add_downstream(std::type_index tag, Ptr node) = 0;
    virtual std::vector<std::type_index> input_tags() const = 0;
    virtual std::vector<std::type_index> output_tags() const = 0;
    virtual Ptr clone() const = 0;
};

template<typename Pair>
struct Args {
    using T = typename Pair::second_type;// 真参数类型

    std::optional<T> value_;

    void write(const T& value) {
        value_ = value;
    }
    void write(T&& value) {
        value_ = std::move(value);
    }

    bool has_value() const {
        return value_.has_value();
    }

    std::optional<T> read() const {
        return value_;
    }

    void reset() {
        value_.reset();
    }
};

template<typename InputPair, typename... OutputPairs>
struct TaskNode: NodeBase {
    using InputTag = typename InputPair::first_type;
    using InputType = typename InputPair::second_type;
    using OutputTypeTuple = std::tuple<std::optional<typename OutputPairs::second_type>...>;// 输出类型
    // std::conditional_t<条件, 条件为true时的类型, 条件为false时的类型>
    using Func = std::conditional_t<sizeof...(OutputPairs) == 0, std::function<void(InputType&&)>, std::function<OutputTypeTuple(InputType&&)>>;
    // 如果没有输出类型，则Func类型为void，否则为OutputTypeTuple/void
    Func fn;
    Args<InputPair> inputs;// 输入参数

    std::unordered_map<std::type_index, std::vector<NodeBase::Ptr>> downstream_by_tag;// 由tag标记筛选出的下流节点
    std::vector<NodeBase::Ptr> downstream_all;// 所有下流节点

    TaskNode() {
        kind = NodeKind::Task;
    }

    std::vector<std::type_index> input_tags() const override {
        return { std::type_index(typeid(InputTag)) };
    }

    std::vector<std::type_index> output_tags() const override {
        return make_tag_index_vec<typename OutputPairs::first_type...>();
    }

    NodeBase::Ptr clone() const override {
        auto node = std::make_shared<TaskNode>(*this);// 派生类指针
        node->downstream_all.clear();// 清空新node的downstream_all和downstream_by_tag
        node->downstream_by_tag.clear();
        for (auto& down_node: downstream_by_tag) {
            for (auto& down_node_ptr: down_node.second) {
                node->add_downstream(down_node.first, down_node_ptr->clone());// 把旧node的downstream_all和downstream_by_tag克隆到新node中
            }
        }
        return node;
    }

    template<typename Fn>
    static NodeBase::Ptr create(Fn&& fn) {// 传入fn
        auto node = std::make_shared<TaskNode>();
        node->fn = std::forward<Fn>(fn);
        return node;
    }
    static NodeBase::Ptr create() { // 转移指针所有权
        auto node = std::make_shared<TaskNode>();
        return node;
    }

    void add_downstream(std::type_index tag, NodeBase::Ptr node) override {
        if (!node)
            return;
        downstream_by_tag[tag].push_back(node);// 存指针
        downstream_all.push_back(std::move(node));// 存指针
    }

    bool receive(std::type_index OutputTag, const void* data, std::type_index OutputType) override {
        if (OutputTag != std::type_index(typeid(InputTag)))
            return false;
        if (OutputType != std::type_index(typeid(InputType)))// 输入输出类型匹不匹配
            return false;
        if (inputs.has_value())// 每次接收新输入
            return false;

        inputs.write(*static_cast<const InputType*>(data));// 解引用后是const InputType类型的左值，调用的是write(const T& value)，因为data是地址，所以需要解引用
        return true;
    }

    const std::vector<NodeBase::Ptr>& execute() override {
        if (!inputs.has_value())
            return downstream_all;

        if constexpr (sizeof...(OutputPairs) > 0) {
            auto outputs = fn(std::move(*inputs.value_));
            forward_outputs(outputs);
        } else {
            fn(std::move(*inputs.value_));// 把输入移动给任务函数
        }

        inputs.reset();// 清空输入，以便下次接收新的输入
        return downstream_all;// 返回所有下游节点
    }

private:
    template<typename Pair, typename V>
    void send_one(const V& v) {
        using OutputTag = typename Pair::first_type;
        using OutputType = typename Pair::second_type;

        auto it = downstream_by_tag.find(std::type_index(typeid(OutputTag)));
        if (it == downstream_by_tag.end())// 没有找到
            return;

        for (auto& out: it->second) {// 遍历哈希表的值std::vector<NodeBase::Ptr>
            if (out) {
                out->receive(std::type_index(typeid(OutputTag)), &v, std::type_index(typeid(OutputType)));// 把Out和Input可以匹配的节点的输出参数写入输出参数
            }
        }
    }

    template<std::size_t... I>
    void forward_outputs(OutputTypeTuple& outputs, std::index_sequence<I...>) {
        (dispatch_one<nth_type_t<I, OutputPairs...>>(std::get<I>(outputs)), ...); // 依次调用dispatch_one，把outputs里的值写入inputs里，实际只能写入一个，因为只有一个输入参数，只有一个匹配
    }

    template<typename Pair>
    void dispatch_one(std::optional<typename Pair::second_type>& out) {
        if (!out.has_value())
            return;
        send_one<Pair>(*out);
    }

    void forward_outputs(OutputTypeTuple& outputs) {
        forward_outputs(outputs, std::make_index_sequence<sizeof...(OutputPairs)> {});
    }
};

template<typename... OutputPairs>
struct SourceNode: NodeBase {
    using OutputTypeTuple = std::tuple<std::optional<typename OutputPairs::second_type>...>;
    using Func = std::conditional_t<sizeof...(OutputPairs) == 0, std::function<void()>, std::function<OutputTypeTuple()>>;

    Func fn;

    std::unordered_map<std::type_index, std::vector<NodeBase::Ptr>> downstream_by_tag;
    std::vector<NodeBase::Ptr> downstream_all;

    SourceNode() {
        kind = NodeKind::Source;
    }

    template<typename Fn>
    static NodeBase::Ptr create(Fn&& fn) {
        auto node = std::make_shared<SourceNode>();
        node->fn = std::forward<Fn>(fn);
        return node;
    }
    static NodeBase::Ptr create() {
        auto node = std::make_shared<SourceNode>();
        return node;
    }
    template<typename F>
    bool runtime_set_fn(F&& f) {
        if constexpr (std::is_convertible_v<F, Func>) {// 判断类型Fn能否隐式转换为Func类型
            fn = std::forward<F>(f);
            return true;
        }
        return false;
    }

    NodeBase::Ptr clone() const override {
        auto node = std::make_shared<SourceNode>(*this);
        node->downstream_all.clear();
        node->downstream_by_tag.clear();
        for (auto& down_node: downstream_by_tag) {
            for (auto& down_node_ptr: down_node.second) {
                node->add_downstream(down_node.first, down_node_ptr->clone());
            }
        }

        return node;
    }

    void add_downstream(std::type_index tag, Ptr node) override {
        if (!node)
            return;
        downstream_by_tag[tag].push_back(node);
        downstream_all.push_back(std::move(node));
    }

    bool receive(std::type_index, const void*, std::type_index) override {
        return false;
    }

    std::vector<std::type_index> input_tags() const override {
        return {};
    }

    std::vector<std::type_index> output_tags() const override {
        return make_tag_index_vec<typename OutputPairs::first_type...>();
    }

    const std::vector<NodeBase::Ptr>& execute() override {
        if constexpr (sizeof...(OutputPairs) > 0) {
            auto outputs = fn();
            forward_outputs(outputs);
        } else {
            fn();
        }
        return downstream_all;
    }

private:
    template<typename Pair, typename V>
    void send_one(const V& v) {
        using OutputTag = typename Pair::first_type;
        using OutputType = typename Pair::second_type;

        auto it = downstream_by_tag.find(std::type_index(typeid(OutputTag)));
        if (it == downstream_by_tag.end())
            return;

        for (auto& out: it->second) {
            if (out) {
                out->receive(std::type_index(typeid(OutputTag)), &v, std::type_index(typeid(OutputType)));// 什么都不做
            }
        }
    }

    template<std::size_t... I>
    void forward_outputs(OutputTypeTuple& outputs, std::index_sequence<I...>) {
        (dispatch_one<nth_type_t<I, OutputPairs...>>(std::get<I>(outputs)), ...);
    }

    template<typename Pair>
    void dispatch_one(std::optional<typename Pair::second_type>& out) {
        if (!out.has_value()) {
            return;
        }

        send_one<Pair>(*out);
    }

    void forward_outputs(OutputTypeTuple& outputs) {
        forward_outputs(outputs, std::make_index_sequence<sizeof...(OutputPairs)> {});
    }
};

} // namespace dts_vision::tools::Scheduler
