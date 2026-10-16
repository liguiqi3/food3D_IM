#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include "log.hpp"
#include "measurement.hpp"



namespace {

// Injected by test_package/CMakeLists.txt (link_to_resources); fallback for a standalone compile.
#ifndef RESOURCES_PATH
#define RESOURCES_PATH "."
#endif

const char* const kBaselinePcd = RESOURCES_PATH "/d405_260322274982_20260805_142739.pcd";
const char* const kFoodPcd = RESOURCES_PATH "/d405_260322274982_20260819_180010.pcd";

} // namespace



int main(int argc, char* argv[]) {
#ifndef __ARM_EABI__
    // `main` 是独立 CLI 演示程序，不是 GTest 二进制；覆盖率构建路径会通过
    // gtest_discover_tests 以 `--gtest_list_tests` 探测它。识别到任意 gtest
    // 探测参数时直接干净退出（返回 0、不输出测试名）。
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]).rfind("--gtest", 0) == 0) {
            return 0;
        }
    }

    // 命令行参数：baseline PCD 与 food PCD 路径；未提供时退回内置默认值。
    const std::string baseline_path = (argc > 1) ? argv[1] : kBaselinePcd;
    const std::string food_path = (argc > 2) ? argv[2] : kFoodPcd;
    if (argc < 3) {
        std::cout << "用法: " << argv[0] << " <baseline.pcd> <food.pcd>" << std::endl;
        std::cout << "未提供参数，使用默认: " << baseline_path << " " << food_path << std::endl;
    }

    // 开启日志：五级日志 + 控制台 + 文件（覆盖写），记录算法中间结果与状态。
    log_set_level(LogLevel::kInfo);
    log_set_console(true);
    log_set_file("pcd_im_trace.txt", LogFileMode::kTruncate);

    const PointCloud baseline = load_pcd(baseline_path);
    const PointCloud food = load_pcd(food_path);
    if (baseline.points.empty() || food.points.empty()) {
        std::cerr << "failed to load test point clouds" << std::endl;
        return 1;
    }
    std::cout << "\n=== 加载 PCD ===" << std::endl;
    std::cout << "baseline points: " << baseline.points.size() << std::endl;
    std::cout << "初始点云数量: " << food.points.size() << std::endl;

    FoodVolumeMeasurer measurer;
    const VolumeEstimate est = measurer.set_baseline({baseline}).set_food(food).run();

    std::cout << "\n=== IM 复现完成 ===" << std::endl;
    std::cout << "输入点数: " << est.input_points << std::endl;
    std::cout << "下采样点数: " << est.downsampled_points << std::endl;
    std::cout << "聚类数: " << est.cluster_count << std::endl;
    std::cout << "目标簇标签: [";
    for (std::size_t i = 0; i < est.selected_cluster_labels.size(); ++i) {
        if (i != 0) {
            std::cout << ", ";
        }
        std::cout << est.selected_cluster_labels[i];
    }
    std::cout << "]" << std::endl;
    std::cout << "目标连通块数: " << est.component_count << std::endl;
    std::cout << "目标簇点数: " << est.selected_cluster_points << std::endl;
    std::cout << "顶部 surface cell 数: " << est.top_surface_points << std::endl;
    std::cout << "baseline 帧数: " << est.baseline_frames << std::endl;
    std::cout << "占用网格数: " << est.occupied_cells << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Footprint 占用面积: " << est.footprint_area_m2 << " m^2" << std::endl;
    std::cout << "平均高度: " << est.mean_height_m << " m" << std::endl;
    std::cout << "最大高度: " << est.max_height_m << " m" << std::endl;
    std::cout << "PCD 原始积分体积: " << est.raw_volume_cm3 / 1e6 << " m^3" << std::endl;
    std::cout << "PCD 补洞后积分体积: " << est.volume_cm3 / 1e6 << " m^3" << std::endl;
    std::cout << "AABB 体积: " << est.aabb_volume_m3 << " m^3" << std::endl;
    std::cout << "OBB-Compact 体积: " << est.obb_volume_m3 << " m^3" << std::endl;
    std::cout << "Convex Hull 体积: " << est.convex_hull_volume_m3 << " m^3" << std::endl;
    std::cout << std::defaultfloat;
    std::cout << "未匹配 baseline cell 数: " << est.missing_baseline_cells << std::endl;

    log_close_file();

    return est.status == MeasurementStatus::kSuccess ? 0 : 1;
#else
    (void)argc;
    (void)argv;
    std::cout << "volume measurement unavailable on bare-metal" << std::endl;
    return 0;
#endif
}
