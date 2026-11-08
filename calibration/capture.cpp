#include <fmt/core.h>
#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"  
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

// 命令行参数
const std::string keys =
  "{help h usage ?  |                          | 输出命令行参数说明}"
  "{@config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{output-folder o |      assets/img_with_q   | 输出文件夹路径   }";

// 四元数保存函数
void write_q(const std::string q_path, const Eigen::Quaterniond & q)
{
  std::ofstream q_file(q_path);
  Eigen::Vector4d xyzw = q.coeffs();  
  q_file << fmt::format("{} {} {} {}", xyzw[3], xyzw[0], xyzw[1], xyzw[2]); 
  q_file.close();
}

void capture_loop(const std::string & config_path,  const std::string & output_folder) 
{
  // 读取配置文件中的标定板参数
  auto yaml = tools::load(config_path);
  int pattern_cols = tools::read<int>(yaml, "pattern_cols");
  int pattern_rows = tools::read<int>(yaml, "pattern_rows");

  io::Gimbal gimbal(config_path);  
  io::Camera camera(config_path);
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  int count = 0;

  tools::logger()->info("标定板尺寸：{}列{}行（从配置文件读取）", pattern_cols, pattern_rows);

  while (true) 
  {
    camera.read(img, timestamp);  // 读取图像（分辨率不变）
    Eigen::Quaterniond q = gimbal.q(timestamp);  // 获取对应时刻的云台姿态

    // 克隆图像用于绘制信息
    auto img_with_ypr = img.clone();
    
    // 显示欧拉角（辅助判断姿态稳定性）
    Eigen::Vector3d zyx = tools::eulers(q, 2, 1, 0, 1) * 57.3;  // Z-Y-X欧拉角（度）
    tools::draw_text(img_with_ypr, fmt::format("Yaw: {:.2f}°", zyx[0]), {40, 40}, {0, 0, 255});
    tools::draw_text(img_with_ypr, fmt::format("Pitch: {:.2f}°", zyx[1]), {40, 80}, {0, 0, 255});
    tools::draw_text(img_with_ypr, fmt::format("Roll: {:.2f}°", zyx[2]), {40, 120}, {0, 0, 255});

    // 识别标定板
    std::vector<cv::Point2f> corners;
    cv::Mat gray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    bool success = cv::findChessboardCorners(
      gray, 
      cv::Size(pattern_cols, pattern_rows), 
      corners,
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE
    );

    // 绘制检测结果（无论成功与否都绘制）
    cv::drawChessboardCorners(img_with_ypr, cv::Size(pattern_cols, pattern_rows), corners, success);
    
    // 添加检测状态提示文字
    if (success) {
      tools::draw_text(img_with_ypr, "检测到棋盘格", {40, 160}, {0, 255, 0});  // 绿色提示成功
    } else {
      tools::draw_text(img_with_ypr, "未检测到棋盘格", {40, 160}, {0, 0, 255});  // 红色提示失败
    }

    // 缩小显示（不影响原始图像保存）
    cv::resize(img_with_ypr, img_with_ypr, {}, 0.5, 0.5);
    cv::imshow("Press 's' to save, 'q' to quit", img_with_ypr);

    // 处理按键
    auto key = cv::waitKey(1);
    if (key == 'q') break;
    else if (key != 's') continue;

    // 保存图像和四元数
    count++;
    auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
    auto q_path = fmt::format("{}/{}.txt", output_folder, count);
    if (cv::imwrite(img_path, img)) {
      write_q(q_path, q);
      tools::logger()->info("[{}] 已保存至 {}", count, output_folder);
    } else {
      tools::logger()->error("[{}] 图像保存失败！", count);
    }
  }
}

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) 
  {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto output_folder = cli.get<std::string>("output-folder");

  // 创建输出文件夹（若不存在）
  if (!std::filesystem::exists(output_folder))
  {
    if (!std::filesystem::create_directory(output_folder)) {
      tools::logger()->error("无法创建输出文件夹: {}", output_folder);
      return -1;
    }
  }

  capture_loop(config_path, output_folder);  

  tools::logger()->warn("提示：四元数保存格式为 w x y z");
  cv::destroyAllWindows();  // 关闭所有显示窗口
  return 0;
}
