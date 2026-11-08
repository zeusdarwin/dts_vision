#include "tools/utils.hpp"
#include <optional>
#include <string>
#define DEFINE_CONFIG_PATH(NAME, FILE) \
    static constexpr auto NAME##_ARR = tools::concat(ROOT_DIR, FILE); \
    static constexpr std::string_view NAME(NAME##_ARR.data());
namespace dts_vision {
DEFINE_CONFIG_PATH(HERO_CONFIG_PATH, "/config/hero.yaml")
DEFINE_CONFIG_PATH(INFANTRY_HOLE_CONFIG_PATH, "/config/infantry_hole.yaml")
DEFINE_CONFIG_PATH(INFANTRY_LEG_CONFIG_PATH, "/config/infantry_leg.yaml")
DEFINE_CONFIG_PATH(SENTRY_CONFIG_PATH, "/config/sentry.yaml")
DEFINE_CONFIG_PATH(UAV_CONFIG_PATH, "/config/uav.yaml")
inline std::optional<std::string> get_robot_config_path(std::string name) {
    auto key = tools::to_upper(name);// 将输入的name转换为大写
    if (key == "HERO") {
        return std::string(HERO_CONFIG_PATH);
    } else if (key == "INFANTRY_HOLE") {
        return std::string(INFANTRY_HOLE_CONFIG_PATH);
    } else if (key == "INFANTRY_LEG") {
        return std::string(INFANTRY_LEG_CONFIG_PATH);   
    } else if (key == "SENTRY") {
        return std::string(SENTRY_CONFIG_PATH);
    } else if (key == "UAV") {
        return std::string(UAV_CONFIG_PATH);
    } 
    return std::nullopt;
}
} // namespace dts_vision
