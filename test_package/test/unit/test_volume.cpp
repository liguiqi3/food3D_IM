#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <sstream>
#include <string>
#include <vector>
#include "log.hpp"
#include "measurement.hpp"
#include "types.hpp"



namespace {

constexpr double kCell = 0.005;
constexpr double kCuboidTop = 0.02;
constexpr double kExpectedVolumeCm3 = 200.0;
// A clear height valley (8 mm) between two 20 mm tops; deep enough to force a split.
constexpr double kValleyTop = 0.008;
// A shallow dimple (18 mm) inside a single 20 mm top; too shallow to force a split.
constexpr double kDimpleTop = 0.018;
// A 12 mm saddle joining two 20 mm humps: one connected item, high enough to stay a single seed.
constexpr double kBridgeTop = 0.012;

PointCloud make_baseline(double half = 0.15) {
    PointCloud cloud;
    for (double x = -half; x <= half + 1.0e-9; x += kCell) {
        for (double y = -half; y <= half + 1.0e-9; y += kCell) {
            cloud.points.push_back({static_cast<float>(x), static_cast<float>(y), 0.0F});
        }
    }
    return cloud;
}

PointCloud make_food(double u_min = 0.0025, double u_max = 0.0975, double v_min = 0.0025, double v_max = 0.0975,
                     double hole_u = -1.0, double hole_v = -1.0) {
    PointCloud cloud = make_baseline();
    for (double x = u_min; x <= u_max + 1.0e-9; x += kCell) {
        for (double y = v_min; y <= v_max + 1.0e-9; y += kCell) {
            if (std::fabs(x - hole_u) < 1.0e-6 && std::fabs(y - hole_v) < 1.0e-6) {
                continue;
            }
            cloud.points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(kCuboidTop)});
        }
    }
    return cloud;
}

PointCloud make_food_with_rect_hole(double hole_x_min, double hole_x_max, double hole_y_min, double hole_y_max) {
    PointCloud cloud = make_baseline();
    for (double x = 0.0025; x <= 0.0975 + 1.0e-9; x += kCell) {
        for (double y = 0.0025; y <= 0.0975 + 1.0e-9; y += kCell) {
            if (x >= hole_x_min - 1.0e-9 && x <= hole_x_max + 1.0e-9 && y >= hole_y_min - 1.0e-9 &&
                y <= hole_y_max + 1.0e-9) {
                continue;
            }
            cloud.points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(kCuboidTop)});
        }
    }
    return cloud;
}

// Two separated cuboids on a wide tray; the gap is wide enough that they become
// two distinct footprint clusters.
PointCloud make_two_cuboid_food() {
    PointCloud cloud = make_baseline(0.25);
    for (const double x0 : {0.0025, 0.1125}) {
        for (double x = x0; x <= x0 + 0.095 + 1.0e-9; x += kCell) {
            for (double y = 0.0025; y <= 0.0975 + 1.0e-9; y += kCell) {
                cloud.points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(kCuboidTop)});
            }
        }
    }
    return cloud;
}

MeasurementConfig make_config() {
    MeasurementConfig cfg;
    cfg.voxel_size_m = 0.002F;
    cfg.plane_distance_threshold_m = 0.003F;
    cfg.integration_resolution_m = 0.005F;
    cfg.roi_border_margin_m = 0.02F;
    cfg.min_height_m = 0.0015F;
    cfg.cluster_min_points = 8;
    cfg.foreground_cluster_eps_m = 0.010F;
    // The feature is opt-in; the watershed tests enable it explicitly here.
    cfg.enable_height_watershed = true;
    return cfg;
}

FoodVolumeMeasurer make_measurer() {
    FoodVolumeMeasurer measurer;
    measurer.set_config(make_config());
    return measurer;
}

} // namespace



TEST(VolumeTypes, LengthUnitScale) {
    EXPECT_DOUBLE_EQ(length_unit_to_meter_scale(LengthUnit::kMeter), 1.0);
    EXPECT_DOUBLE_EQ(length_unit_to_meter_scale(LengthUnit::kMillimeter), 0.001);
}



TEST(VolumeTypes, FiniteCheck) {
    const Point3f ok{1.0F, 2.0F, 3.0F};
    EXPECT_TRUE(is_finite(ok));
    const Point3f bad{1.0F, std::numeric_limits<float>::quiet_NaN(), 3.0F};
    EXPECT_FALSE(is_finite(bad));
}



TEST(VolumeTypes, StatusToStringAllValues) {
    for (int i = 0; i <= static_cast<int>(MeasurementStatus::kUnsupportedPlatform); ++i) {
        EXPECT_NE(status_to_string(static_cast<MeasurementStatus>(i)), nullptr);
    }
    // Out-of-range enum values fall through to the switch default branches.
    EXPECT_STREQ(status_to_string(static_cast<MeasurementStatus>(999)), "unknown");
    EXPECT_DOUBLE_EQ(length_unit_to_meter_scale(static_cast<LengthUnit>(999)), 1.0);
}



TEST(Logging, FileSinkWrites) {
    const std::string path = "unit_log_test.txt";
    log_set_level(LogLevel::kInfo);
    log_set_console(false);
    log_set_file(path, LogFileMode::kTruncate);
    log_info("unit log test message");
    log_close_file();

    std::ifstream file(path);
    ASSERT_TRUE(file.is_open());
    std::string line;
    std::getline(file, line);
    EXPECT_NE(line.find("unit log test message"), std::string::npos);
    file.close();
    log_set_level(LogLevel::kOff);
    std::remove(path.c_str());
}



TEST(Logging, LevelFiltersMessages) {
    const std::string path = "unit_log_test_filter.txt";
    log_set_level(LogLevel::kWarning);
    log_set_console(false);
    log_set_file(path, LogFileMode::kTruncate);
    log_info("this info message must be dropped");
    log_warning("this warning message must be kept");
    log_close_file();

    std::ifstream file(path);
    ASSERT_TRUE(file.is_open());
    const std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    file.close();
    EXPECT_EQ(content.find("this info message must be dropped"), std::string::npos);
    EXPECT_NE(content.find("this warning message must be kept"), std::string::npos);
    log_set_level(LogLevel::kOff);
    std::remove(path.c_str());
}



TEST(Logging, GetterRoundTrips) {
    // The getters must report back exactly what the setters stored.
    log_set_level(LogLevel::kDebug);
    EXPECT_EQ(log_get_level(), LogLevel::kDebug);
    log_set_level(LogLevel::kError);
    EXPECT_EQ(log_get_level(), LogLevel::kError);

    log_set_console(true);
    EXPECT_TRUE(log_get_console());
    log_set_console(false);
    EXPECT_FALSE(log_get_console());

    log_set_level(LogLevel::kOff);
}



TEST(Logging, TraceAndDebugLevelsReachTheFile) {
    // Trace and debug sit below the default info level, so they need their own threshold.
    const std::string path = "unit_log_test_levels.txt";
    log_set_console(false);
    log_set_level(LogLevel::kTrace);
    log_set_file(path, LogFileMode::kTruncate);
    log_trace("trace message");
    log_debug("debug message");
    log_close_file();

    std::ifstream file(path);
    ASSERT_TRUE(file.is_open());
    const std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    file.close();
    EXPECT_NE(content.find("[TRACE] trace message"), std::string::npos);
    EXPECT_NE(content.find("[DEBUG] debug message"), std::string::npos);
    log_set_level(LogLevel::kOff);
    std::remove(path.c_str());
}



TEST(Logging, UnknownLevelFallsBackToOffTag) {
    // A level outside the enum must still produce a line rather than an empty tag.
    const std::string path = "unit_log_test_unknown_level.txt";
    log_set_console(false);
    log_set_level(LogLevel::kTrace);
    log_set_file(path, LogFileMode::kTruncate);
    log_write(static_cast<LogLevel>(99), "out of range level");
    log_close_file();

    std::ifstream file(path);
    ASSERT_TRUE(file.is_open());
    const std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    file.close();
    EXPECT_NE(content.find("[OFF] out of range level"), std::string::npos);
    log_set_level(LogLevel::kOff);
    std::remove(path.c_str());
}



TEST(Logging, EmptyPathClosesTheFile) {
    // An empty path is documented as "close the current file"; nothing is written afterwards.
    const std::string path = "unit_log_test_empty_path.txt";
    log_set_console(false);
    log_set_level(LogLevel::kInfo);
    log_set_file(path, LogFileMode::kTruncate);
    log_info("written before closing");
    log_set_file("");
    log_info("written after closing");
    log_close_file();

    std::ifstream file(path);
    ASSERT_TRUE(file.is_open());
    const std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    file.close();
    EXPECT_NE(content.find("written before closing"), std::string::npos);
    EXPECT_EQ(content.find("written after closing"), std::string::npos);
    log_set_level(LogLevel::kOff);
    std::remove(path.c_str());
}



TEST(Logging, ConsoleRoutingSplitsStdoutAndStderr) {
    // Ordinary levels go to stdout; warnings and errors go to stderr.
    std::ostringstream captured_out;
    std::ostringstream captured_err;
    std::streambuf* const old_out = std::cout.rdbuf(captured_out.rdbuf());
    std::streambuf* const old_err = std::cerr.rdbuf(captured_err.rdbuf());

    log_set_level(LogLevel::kInfo);
    log_set_console(true);
    log_info("info goes to stdout");
    log_warning("warning goes to stderr");
    log_error("error goes to stderr");

    std::cout.rdbuf(old_out);
    std::cerr.rdbuf(old_err);
    log_set_console(false);
    log_set_level(LogLevel::kOff);

    EXPECT_NE(captured_out.str().find("[INFO] info goes to stdout"), std::string::npos);
    EXPECT_EQ(captured_out.str().find("warning goes to stderr"), std::string::npos);
    EXPECT_NE(captured_err.str().find("[WARN] warning goes to stderr"), std::string::npos);
    EXPECT_NE(captured_err.str().find("[ERROR] error goes to stderr"), std::string::npos);
}



TEST(Measurer, EmptyBaseline) {
    auto measurer = make_measurer();
    measurer.set_food(make_food());
    const auto est = measurer.run();
    EXPECT_EQ(est.status, MeasurementStatus::kEmptyBaseline);
}



TEST(Measurer, EmptyFoodInput) {
    auto measurer = make_measurer();
    measurer.set_baseline({make_baseline()});
    const auto est = measurer.run();
    EXPECT_EQ(est.status, MeasurementStatus::kEmptyInput);
}



TEST(Measurer, NonFiniteFoodPointsSkipped) {
    // A non-finite depth return must be dropped, not fatal, as long as valid food remains.
    PointCloud cloud = make_food();
    cloud.points.push_back({0.0F, std::numeric_limits<float>::infinity(), 0.0F});
    auto measurer = make_measurer();
    const auto est = measurer.set_baseline({make_baseline()}).set_food(cloud).run();
    EXPECT_EQ(est.status, MeasurementStatus::kSuccess) << est.message;
}



TEST(Measurer, AllNonFiniteFoodInput) {
    PointCloud cloud;
    cloud.points.push_back({0.0F, std::numeric_limits<float>::infinity(), 0.0F});
    cloud.points.push_back({std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F});
    auto measurer = make_measurer();
    const auto est = measurer.set_baseline({make_baseline()}).set_food(cloud).run();
    EXPECT_EQ(est.status, MeasurementStatus::kNonFiniteInput);
}



TEST(Measurer, CuboidEndToEnd) {
    auto measurer = make_measurer();
    const auto est = measurer.set_baseline({make_baseline()}).set_food(make_food()).run();
    ASSERT_EQ(est.status, MeasurementStatus::kSuccess) << est.message;
    EXPECT_NEAR(est.volume_cm3, kExpectedVolumeCm3, 10.0);
    EXPECT_GT(est.coverage_ratio, 0.8);
    EXPECT_EQ(est.baseline_frames, 1u);
    EXPECT_GT(est.baseline_cell_count, 0u);
}



TEST(Measurer, ZeroVoxelSizeSkipsDownsampling) {
    // voxel_size_m = 0 disables voxel downsampling, so every input point survives preprocessing.
    MeasurementConfig cfg = make_config();
    cfg.voxel_size_m = 0.0;

    FoodVolumeMeasurer measurer;
    measurer.set_config(cfg);
    const auto est = measurer.set_baseline({make_baseline()}).set_food(make_food()).run();
    ASSERT_EQ(est.status, MeasurementStatus::kSuccess) << est.message;
    EXPECT_EQ(est.downsampled_points, est.input_points);
    EXPECT_NEAR(est.volume_cm3, kExpectedVolumeCm3, 10.0);
}



TEST(Measurer, NegativeVoxelSizeRejected) {
    // A negative voxel size is still an invalid configuration.
    MeasurementConfig cfg = make_config();
    cfg.voxel_size_m = -0.001;

    FoodVolumeMeasurer measurer;
    measurer.set_config(cfg);
    const auto est = measurer.set_baseline({make_baseline()}).set_food(make_food()).run();
    EXPECT_EQ(est.status, MeasurementStatus::kInvalidConfig);
}



TEST(Measurer, AddBaselineFrameAccumulates) {
    auto measurer = make_measurer();
    measurer.add_baseline_frame(make_baseline());
    measurer.add_baseline_frame(make_baseline());
    measurer.set_food(make_food());
    const auto est = measurer.run();
    ASSERT_EQ(est.status, MeasurementStatus::kSuccess) << est.message;
    EXPECT_EQ(est.baseline_frames, 2u);
    EXPECT_NEAR(est.volume_cm3, kExpectedVolumeCm3, 10.0);
}



TEST(Measurer, EmptyTrayNoFood) {
    auto measurer = make_measurer();
    const auto est = measurer.set_baseline({make_baseline()}).set_food(make_baseline()).run();
    EXPECT_EQ(est.status, MeasurementStatus::kInsufficientCoverage);
}



TEST(Measurer, MillimeterInput) {
    auto baseline = make_baseline();
    auto food = make_food();
    for (auto* cloud : std::vector<PointCloud*>{&baseline, &food}) {
        for (auto& p : cloud->points) {
            p.x *= 1000.0F;
            p.y *= 1000.0F;
            p.z *= 1000.0F;
        }
    }
    auto measurer = make_measurer();
    const auto est = measurer.set_input_unit(LengthUnit::kMillimeter).set_baseline({baseline}).set_food(food).run();
    ASSERT_EQ(est.status, MeasurementStatus::kSuccess) << est.message;
    EXPECT_NEAR(est.volume_cm3, kExpectedVolumeCm3, 10.0);
}



TEST(Measurer, TiltedTrayDetected) {
    const double theta = 0.5;
    const double s = std::sin(theta);
    const double c = std::cos(theta);

    const auto mk = [&](double u, double v, double h) -> Point3f {
        return {static_cast<float>(u * c - h * s), static_cast<float>(v), static_cast<float>(u * s + h * c)};
    };

    PointCloud baseline;
    for (double u = -0.15; u <= 0.15 + 1.0e-9; u += kCell) {
        for (double v = -0.15; v <= 0.15 + 1.0e-9; v += kCell) {
            baseline.points.push_back(mk(u, v, 0.0));
        }
    }
    PointCloud food = baseline;
    for (double u = 0.0025; u <= 0.0975 + 1.0e-9; u += kCell) {
        for (double v = 0.0025; v <= 0.0975 + 1.0e-9; v += kCell) {
            food.points.push_back(mk(u, v, kCuboidTop));
        }
    }

    auto measurer = make_measurer();
    const auto est = measurer.set_baseline({baseline}).set_food(food).run();
    ASSERT_EQ(est.status, MeasurementStatus::kSuccess) << est.message;
    EXPECT_NEAR(est.volume_cm3, kExpectedVolumeCm3, 25.0);
}



TEST(Measurer, RoiCropping) {
    // Food outside the axis-aligned ROI must not contribute to the measured volume.
    AxisAlignedRoi roi;
    roi.min_x = -0.15F;
    roi.max_x = 0.06F; // only the left half of the tray is measured
    roi.min_y = -0.15F;
    roi.max_y = 0.15F;
    roi.min_z = -0.05F;
    roi.max_z = 0.05F;

    auto measurer = make_measurer();
    const auto est = measurer.set_roi(roi).set_baseline({make_baseline()}).set_food(make_food()).run();
    ASSERT_EQ(est.status, MeasurementStatus::kSuccess) << est.message;
    // The cuboid spans u = [0.0025, 0.0975]; the ROI keeps roughly half of it.
    EXPECT_NEAR(est.volume_cm3, 0.5 * kExpectedVolumeCm3, 35.0);
}



TEST(Measurer, HeightCapApplied) {
    // A height cap clamps every cell to max_height_m, bounding the integrated volume.
    auto measurer = make_measurer();
    const auto est =
        measurer.set_height_range(0.0015, 0.01).set_baseline({make_baseline()}).set_food(make_food()).run();
    ASSERT_EQ(est.status, MeasurementStatus::kSuccess) << est.message;
    EXPECT_NEAR(est.max_height_m, 0.01, 1.0e-9);
    // 0.1 m x 0.1 m footprint x 0.01 m cap = 100 cm^3.
    EXPECT_NEAR(est.volume_cm3, 100.0, 15.0);
}



TEST(Measurer, SmallClusterRejected) {
    // A tiny separated cluster below the minimum candidate size must not be reported.
    PointCloud food = make_baseline(0.25);
    for (double x = 0.0025; x <= 0.0975 + 1.0e-9; x += kCell) {
        for (double y = 0.0025; y <= 0.0975 + 1.0e-9; y += kCell) {
            food.points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(kCuboidTop)});
        }
    }
    // A 3x3 speck (~9 points) far from the main cuboid.
    for (double x = -0.15; x <= -0.14 + 1.0e-9; x += kCell) {
        for (double y = -0.15; y <= -0.14 + 1.0e-9; y += kCell) {
            food.points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(kCuboidTop)});
        }
    }

    auto measurer = make_measurer();
    const auto est = measurer.set_baseline({make_baseline(0.25)}).set_food(food).run();
    ASSERT_EQ(est.status, MeasurementStatus::kSuccess) << est.message;
    EXPECT_EQ(est.component_count, 1u);
    EXPECT_NEAR(est.volume_cm3, kExpectedVolumeCm3, 15.0);
}



TEST(Measurer, BumpySingleItemNotSplit) {
    // One item shaped as two 20 mm humps joined by a 12 mm saddle. Seeding from every raw maximum
    // carved this single item into two pieces; plateau seeding keeps it one component.
    PointCloud food = make_baseline(0.25);
    for (double x = 0.0025; x <= 0.1975 + 1.0e-9; x += kCell) {
        for (double y = 0.0025; y <= 0.0975 + 1.0e-9; y += kCell) {
            const bool bridge = std::fabs(x - 0.0975) < 1.0e-6 || std::fabs(x - 0.1025) < 1.0e-6;
            food.points.push_back(
                {static_cast<float>(x), static_cast<float>(y), static_cast<float>(bridge ? kBridgeTop : kCuboidTop)});
        }
    }

    auto measurer = make_measurer();
    const auto est = measurer.set_baseline({make_baseline(0.25)}).set_food(food).run();
    ASSERT_EQ(est.status, MeasurementStatus::kSuccess) << est.message;
    EXPECT_EQ(est.component_count, 1u);
    ASSERT_EQ(est.component_estimates.size(), 1u);
    // Two humps of ~180 cm3 each are one item; the saddle must not add a second component.
    EXPECT_GT(est.volume_cm3, 300.0);
}



TEST(Measurer, MultiComponentVolume) {
    PointCloud food = make_baseline(0.25);
    // Two separated cuboids; the second starts far enough to form its own footprint cluster.
    for (double x = 0.0025; x <= 0.0975 + 1.0e-9; x += kCell) {
        for (double y = 0.0025; y <= 0.0975 + 1.0e-9; y += kCell) {
            food.points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(kCuboidTop)});
        }
    }
    for (double x = 0.1125; x <= 0.2075 + 1.0e-9; x += kCell) {
        for (double y = 0.0025; y <= 0.0975 + 1.0e-9; y += kCell) {
            food.points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(kCuboidTop)});
        }
    }

    auto measurer = make_measurer();
    const auto est = measurer.set_baseline({make_baseline(0.25)}).set_food(food).run();
    ASSERT_EQ(est.status, MeasurementStatus::kSuccess) << est.message;
    EXPECT_EQ(est.component_count, 2u);
    EXPECT_NEAR(est.volume_cm3, 2.0 * kExpectedVolumeCm3, 20.0);

    // Per-component results are exposed aligned with selected_cluster_labels.
    ASSERT_EQ(est.component_estimates.size(), 2u);
    double per_component_sum = 0.0;
    for (const auto& estimate : est.component_estimates) {
        EXPECT_NEAR(estimate.volume_cm3, kExpectedVolumeCm3, 25.0);
        per_component_sum += estimate.volume_cm3;
    }
    EXPECT_NEAR(per_component_sum, est.volume_cm3, 30.0);
}



TEST(Measurer, CloseFoodSplitByHeightValley) {
    // Two 20 mm slabs whose footprints are contiguous (a 5 mm seam) with an 8 mm valley along the
    // seam. DBSCAN merges them into one cluster; the height watershed must split them apart.
    PointCloud food = make_baseline(0.25);
    for (double x = 0.0025; x <= 0.1975 + 1.0e-9; x += kCell) {
        for (double y = 0.0025; y <= 0.0975 + 1.0e-9; y += kCell) {
            const bool valley = std::fabs(x - 0.0975) < 1.0e-6 || std::fabs(x - 0.1025) < 1.0e-6;
            food.points.push_back(
                {static_cast<float>(x), static_cast<float>(y), static_cast<float>(valley ? kValleyTop : kCuboidTop)});
        }
    }

    auto measurer = make_measurer();
    const auto est = measurer.set_baseline({make_baseline(0.25)}).set_food(food).run();
    ASSERT_EQ(est.status, MeasurementStatus::kSuccess) << est.message;
    EXPECT_EQ(est.component_count, 2u);
    // The two slabs sum to about 388 cm3 once the valley columns are accounted for.
    EXPECT_NEAR(est.volume_cm3, 2.0 * kExpectedVolumeCm3, 40.0);
    ASSERT_EQ(est.component_estimates.size(), 2u);
    for (const auto& estimate : est.component_estimates) {
        EXPECT_NEAR(estimate.volume_cm3, kExpectedVolumeCm3, 40.0);
    }
}



TEST(Measurer, HeightWatershedSwitchDisablesSplit) {
    // The same contiguous two-slab scene as CloseFoodSplitByHeightValley. With the switch off the
    // raw DBSCAN cluster is kept, so both items collapse into a single component.
    PointCloud food = make_baseline(0.25);
    for (double x = 0.0025; x <= 0.1975 + 1.0e-9; x += kCell) {
        for (double y = 0.0025; y <= 0.0975 + 1.0e-9; y += kCell) {
            const bool valley = std::fabs(x - 0.0975) < 1.0e-6 || std::fabs(x - 0.1025) < 1.0e-6;
            food.points.push_back(
                {static_cast<float>(x), static_cast<float>(y), static_cast<float>(valley ? kValleyTop : kCuboidTop)});
        }
    }

    MeasurementConfig cfg = make_config();
    cfg.enable_height_watershed = false;
    FoodVolumeMeasurer measurer;
    measurer.set_config(cfg);
    const auto est = measurer.set_baseline({make_baseline(0.25)}).set_food(food).run();
    ASSERT_EQ(est.status, MeasurementStatus::kSuccess) << est.message;
    EXPECT_EQ(est.cluster_count, 1u);
    EXPECT_EQ(est.component_count, 1u);
}



TEST(Measurer, ShallowDimpleNotSplit) {
    // A single 20 mm slab with a 2 mm-deep groove across one column. The valley is far too shallow
    // to be two items, so the watershed must merge the markers back and keep one component.
    PointCloud food = make_baseline(0.25);
    for (double x = 0.0025; x <= 0.0975 + 1.0e-9; x += kCell) {
        for (double y = 0.0025; y <= 0.0975 + 1.0e-9; y += kCell) {
            const bool groove = std::fabs(x - 0.0525) < 1.0e-6;
            food.points.push_back(
                {static_cast<float>(x), static_cast<float>(y), static_cast<float>(groove ? kDimpleTop : kCuboidTop)});
        }
    }

    auto measurer = make_measurer();
    const auto est = measurer.set_baseline({make_baseline()}).set_food(food).run();
    ASSERT_EQ(est.status, MeasurementStatus::kSuccess) << est.message;
    EXPECT_EQ(est.component_count, 1u);
    EXPECT_NEAR(est.volume_cm3, kExpectedVolumeCm3, 15.0);
}



TEST(Measurer, SelectedLabels) {
    PointCloud food = make_baseline(0.25);
    for (double x = 0.0025; x <= 0.0975 + 1.0e-9; x += kCell) {
        for (double y = 0.0025; y <= 0.0975 + 1.0e-9; y += kCell) {
            food.points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(kCuboidTop)});
        }
    }
    for (double x = 0.1125; x <= 0.2075 + 1.0e-9; x += kCell) {
        for (double y = 0.0025; y <= 0.0975 + 1.0e-9; y += kCell) {
            food.points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(kCuboidTop)});
        }
    }

    // A label that does not exist finds no food.
    auto measurer = make_measurer();
    const auto missing = measurer.set_selected_labels({42})
                             .set_baseline({make_baseline(0.25)})
                             .set_food(food)
                             .run();
    EXPECT_EQ(missing.status, MeasurementStatus::kFoodNotFound);

    // Selecting the first real label measures only that component.
    auto manual = make_measurer();
    manual.set_selected_labels({0});
    const auto est = manual.set_baseline({make_baseline(0.25)}).set_food(food).run();
    ASSERT_EQ(est.status, MeasurementStatus::kSuccess) << est.message;
    EXPECT_EQ(est.component_count, 1u);
    EXPECT_NEAR(est.volume_cm3, kExpectedVolumeCm3, 25.0);

    const auto all = manual.set_selected_labels({}).run();
    ASSERT_EQ(all.status, MeasurementStatus::kSuccess) << all.message;
    EXPECT_EQ(all.component_count, 2u);
}



TEST(Measurer, SmallHoleInterpolated) {
    auto measurer = make_measurer();
    const auto est = measurer.set_baseline({make_baseline()})
                         .set_food(make_food(0.0025, 0.0975, 0.0025, 0.0975, 0.0525, 0.0525))
                         .run();
    ASSERT_EQ(est.status, MeasurementStatus::kSuccess) << est.message;
    EXPECT_NEAR(est.volume_cm3, kExpectedVolumeCm3, 10.0);
    EXPECT_EQ(est.interpolated_cells, 1u);
    EXPECT_NEAR(est.interpolated_volume_cm3, kCuboidTop * kCell * kCell * 1.0e6, 0.05);
}



TEST(Measurer, QuadraticHoleCompletion) {
    auto measurer = make_measurer();
    // A 4x4-cell interior gap exceeds the small-hole cap (9 cells) and forces the
    // quadratic-surface (curve fill) path.
    const auto est =
        measurer.set_baseline({make_baseline()}).set_food(make_food_with_rect_hole(0.04, 0.06, 0.04, 0.06)).run();
    ASSERT_EQ(est.status, MeasurementStatus::kSuccess) << est.message;
    EXPECT_EQ(est.interpolated_cells, 16u);
    EXPECT_NEAR(est.volume_cm3, kExpectedVolumeCm3, 10.0);
}



TEST(Measurer, OversizedHoleLeftUnfilled) {
    auto measurer = make_measurer();
    // A 10x10-cell gap (25 cm^2) exceeds the 8 cm^2 curve-fill cap, so it must stay unfilled.
    const auto est =
        measurer.set_baseline({make_baseline()}).set_food(make_food_with_rect_hole(0.025, 0.075, 0.025, 0.075)).run();
    ASSERT_EQ(est.status, MeasurementStatus::kSuccess) << est.message;
    EXPECT_EQ(est.interpolated_cells, 0u);
    EXPECT_EQ(est.unfilled_hole_cells, 100u);
}



TEST(Measurer, ReferenceVolumesReported) {
    // Reference volumes need a cloud with actual thickness that survives the
    // pipeline: a z=0 bottom layer would be removed with the background plane,
    // so the lower face sits at 0.005 m (above the RANSAC threshold).
    PointCloud food = make_baseline();
    for (double x = 0.0025; x <= 0.0975 + 1.0e-9; x += kCell) {
        for (double y = 0.0025; y <= 0.0975 + 1.0e-9; y += kCell) {
            food.points.push_back({static_cast<float>(x), static_cast<float>(y), 0.005F});
            food.points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(kCuboidTop)});
        }
    }

    auto measurer = make_measurer();
    const auto est = measurer.set_baseline({make_baseline()}).set_food(food).run();
    ASSERT_EQ(est.status, MeasurementStatus::kSuccess) << est.message;
    // Reference volumes from the selected components are finite and positive.
    EXPECT_TRUE(std::isfinite(est.aabb_volume_m3));
    EXPECT_TRUE(std::isfinite(est.obb_volume_m3));
    EXPECT_TRUE(std::isfinite(est.convex_hull_volume_m3));
    EXPECT_GT(est.aabb_volume_m3, 0.0);
    EXPECT_GT(est.obb_volume_m3, 0.0);
    EXPECT_GT(est.convex_hull_volume_m3, 0.0);
    // The AABB encloses the slab: 0.095 x 0.095 x (0.02 - 0.005) m.
    EXPECT_NEAR(est.aabb_volume_m3, 0.095 * 0.095 * 0.015, 1.0e-4);
}



TEST(Measurer, LoadConfigFromJsonFile) {
    const std::string path = "unit_config_test.json";
    {
        std::ofstream file(path);
        file << "{\"voxel_size_m\":0.005,\"cluster_min_points\":12,"
                "\"input_unit\":\"millimeter\",\"selected_labels\":[1,2,3]}";
    }
    FoodVolumeMeasurer measurer;
    ASSERT_TRUE(measurer.load_config_from_json(path));
    const MeasurementConfig& cfg = measurer.config();
    EXPECT_DOUBLE_EQ(cfg.voxel_size_m, 0.005);
    EXPECT_EQ(cfg.cluster_min_points, 12);
    EXPECT_EQ(cfg.input_unit, LengthUnit::kMillimeter);
    ASSERT_EQ(cfg.selected_labels.size(), 3u);
    EXPECT_EQ(cfg.selected_labels[0], 1);
    EXPECT_EQ(cfg.selected_labels[2], 3);
    std::remove(path.c_str());
}



TEST(Measurer, MissingConfigFieldsKeepDefaults) {
    const std::string path = "unit_config_partial.json";
    {
        std::ofstream file(path);
        file << "{\"voxel_size_m\":0.007}";
    }
    FoodVolumeMeasurer measurer;
    const double default_eps = measurer.config().foreground_cluster_eps_m;
    const double default_resolution = measurer.config().integration_resolution_m;
    ASSERT_TRUE(measurer.load_config_from_json(path));
    EXPECT_DOUBLE_EQ(measurer.config().voxel_size_m, 0.007);
    EXPECT_DOUBLE_EQ(measurer.config().foreground_cluster_eps_m, default_eps);
    EXPECT_DOUBLE_EQ(measurer.config().integration_resolution_m, default_resolution);
    std::remove(path.c_str());
}



TEST(Measurer, InvalidConfigEnumFails) {
    const std::string path = "unit_config_bad.json";
    {
        std::ofstream file(path);
        file << "{\"input_unit\":\"parsec\"}";
    }
    FoodVolumeMeasurer measurer;
    EXPECT_FALSE(measurer.load_config_from_json(path));
    std::remove(path.c_str());
}



TEST(Measurer, RemovedConfigOptionFails) {
    const std::string path = "unit_config_removed.json";
    {
        std::ofstream file(path);
        file << "{\"save_middle_cloud\":true}";
    }
    FoodVolumeMeasurer measurer;
    EXPECT_FALSE(measurer.load_config_from_json(path));
    std::remove(path.c_str());
}



TEST(Measurer, RoiJsonRoundTripAndClear) {
    const std::string path = "unit_config_roi.json";
    AxisAlignedRoi roi{};
    roi.min_x = 0.01F;
    roi.max_x = 0.09F;
    roi.min_y = 0.02F;
    roi.max_y = 0.08F;
    roi.min_z = -0.01F;
    roi.max_z = 0.06F;

    FoodVolumeMeasurer measurer;
    measurer.set_roi(roi);
    ASSERT_TRUE(measurer.save_config_to_json(path));

    FoodVolumeMeasurer restored;
    ASSERT_TRUE(restored.load_config_from_json(path));
    ASSERT_TRUE(restored.config().roi.has_value());
    EXPECT_FLOAT_EQ(restored.config().roi->max_x, roi.max_x);
    restored.clear_roi();
    EXPECT_FALSE(restored.config().roi.has_value());
    ASSERT_TRUE(restored.save_config_to_json(path));

    restored.set_roi(roi);
    ASSERT_TRUE(restored.load_config_from_json(path));
    EXPECT_FALSE(restored.config().roi.has_value());
    std::remove(path.c_str());
}



TEST(Measurer, MissingConfigFileFails) {
    FoodVolumeMeasurer measurer;
    EXPECT_FALSE(measurer.load_config_from_json("does_not_exist_config.json"));
}



TEST(Measurer, SaveConfigRoundTrip) {
    const std::string path = "unit_config_roundtrip.json";
    auto measurer = make_measurer();
    measurer.set_voxel_size(0.0042).set_plane_distance_threshold(0.0025);
    ASSERT_TRUE(measurer.save_config_to_json(path));

    FoodVolumeMeasurer restored;
    ASSERT_TRUE(restored.load_config_from_json(path));
    EXPECT_DOUBLE_EQ(restored.config().voxel_size_m, 0.0042);
    EXPECT_DOUBLE_EQ(restored.config().plane_distance_threshold_m, 0.0025);
    EXPECT_DOUBLE_EQ(restored.config().integration_resolution_m, measurer.config().integration_resolution_m);
    std::remove(path.c_str());
}

TEST(Measurer, HeightWatershedSwitchJsonRoundTrip) {
    const std::string path = "unit_config_watershed.json";
    MeasurementConfig cfg = make_config();
    // Save the non-default value so the round-trip is proven against the default.
    cfg.enable_height_watershed = true;
    FoodVolumeMeasurer measurer;
    measurer.set_config(cfg);
    ASSERT_TRUE(measurer.save_config_to_json(path));

    FoodVolumeMeasurer restored;
    ASSERT_TRUE(restored.load_config_from_json(path));
    EXPECT_TRUE(restored.config().enable_height_watershed);

    // A file without the key keeps the default, which leaves the split disabled.
    const std::string partial = "unit_config_watershed_default.json";
    {
        std::ofstream file(partial);
        file << "{\"voxel_size_m\":0.007}";
    }
    FoodVolumeMeasurer bare;
    ASSERT_TRUE(bare.load_config_from_json(partial));
    EXPECT_FALSE(bare.config().enable_height_watershed);
}

TEST(Measurer, SaveMiddleCloudWritesStageFiles) {
    // Enabling the dump must leave one PCD per pipeline stage on disk.
    const std::string dir = "unit_middle_cloud_test";
    std::filesystem::remove_all(dir);

    auto measurer = make_measurer();
    const auto est = measurer.set_middle_cloud_dir(dir)
                         .set_baseline({make_baseline()})
                         .set_food(make_food())
                         .run();
    ASSERT_EQ(est.status, MeasurementStatus::kSuccess) << est.message;

    const char* const kExpected[] = {"0_input_food.pcd",       "1_downsampled.pcd",     "2_remaining.pcd",
                                     "3_baseline_surface.pcd", "4_food_components.pcd", "5_top_surface.pcd",
                                     "6_hole_filled_surface.pcd"};
    for (const char* name : kExpected) {
        std::ifstream file(dir + "/" + name);
        EXPECT_TRUE(file.is_open()) << "missing stage cloud: " << name;
    }

    std::filesystem::remove_all(dir);
}



TEST(Measurer, SaveMiddleCloudIncludesFilledHole) {
    const std::string dir = "unit_middle_cloud_hole_test";
    std::filesystem::remove_all(dir);

    auto measurer = make_measurer();
    const auto est = measurer.set_middle_cloud_dir(dir)
                         .set_baseline({make_baseline()})
                         .set_food(make_food(0.0025, 0.0975, 0.0025, 0.0975, 0.0525, 0.0525))
                         .run();
    ASSERT_EQ(est.status, MeasurementStatus::kSuccess) << est.message;
    ASSERT_EQ(est.interpolated_cells, 1u);

    const PointCloud before = load_pcd(dir + "/5_top_surface.pcd");
    const PointCloud after = load_pcd(dir + "/6_hole_filled_surface.pcd");
    EXPECT_EQ(before.points.size(), est.top_surface_points);
    ASSERT_EQ(after.points.size(), est.occupied_cells);
    EXPECT_EQ(after.points.size(), before.points.size() + est.interpolated_cells);

    std::size_t added_points = 0;
    for (const auto& filled : after.points) {
        const bool was_measured = std::any_of(before.points.begin(), before.points.end(), [&](const Point3f& measured) {
            return std::fabs(filled.x - measured.x) < 1.0e-6F &&
                   std::fabs(filled.y - measured.y) < 1.0e-6F &&
                   std::fabs(filled.z - measured.z) < 1.0e-6F;
        });
        if (!was_measured) {
            ++added_points;
        }
    }
    EXPECT_EQ(added_points, est.interpolated_cells);

    std::filesystem::remove_all(dir);
}



TEST(Measurer, SaveMiddleCloudDisabledWritesNothing) {
    const std::string dir = "unit_middle_cloud_off_test";
    std::filesystem::remove_all(dir);

    auto measurer = make_measurer();
    const auto est = measurer.set_middle_cloud_dir(dir)
                         .set_middle_cloud_dir("")
                         .set_baseline({make_baseline()})
                         .set_food(make_food())
                         .run();
    ASSERT_EQ(est.status, MeasurementStatus::kSuccess) << est.message;
    EXPECT_FALSE(std::filesystem::exists(dir));
}



TEST(Measurer, SaveMiddleCloudWritesPerComponentFiles) {
    // Every selected connected component must land in its own PCD, named by its label,
    // so a multi-food frame can be inspected one region at a time.
    const std::string dir = "unit_middle_cloud_components_test";
    std::filesystem::remove_all(dir);

    auto measurer = make_measurer();
    const auto est = measurer.set_middle_cloud_dir(dir)
                         .set_baseline({make_baseline(0.25)})
                         .set_food(make_two_cuboid_food())
                         .run();
    ASSERT_EQ(est.status, MeasurementStatus::kSuccess) << est.message;
    ASSERT_EQ(est.component_count, 2u);
    ASSERT_EQ(est.selected_cluster_labels.size(), est.component_estimates.size());

    for (const int label : est.selected_cluster_labels) {
        std::ostringstream name;
        name << dir << "/4_food_component_label" << std::setw(2) << std::setfill('0') << label << ".pcd";
        std::ifstream file(name.str());
        EXPECT_TRUE(file.is_open()) << "missing component cloud: " << name.str();
    }

    // The merged cloud stays available next to the per-component ones.
    std::ifstream merged(dir + "/4_food_components.pcd");
    EXPECT_TRUE(merged.is_open());

    std::filesystem::remove_all(dir);
}



TEST(Measurer, PrepareBaselineThenRunMatchesInlineBuild) {
    // A prepared baseline must give exactly the same measurement as the inline build.
    auto inline_measurer = make_measurer();
    const auto inline_est = inline_measurer.set_baseline({make_baseline()}).set_food(make_food()).run();
    ASSERT_EQ(inline_est.status, MeasurementStatus::kSuccess) << inline_est.message;

    auto prepared = make_measurer();
    prepared.set_baseline({make_baseline()}).set_food(make_food());
    ASSERT_EQ(prepared.prepare_baseline(), MeasurementStatus::kSuccess);
    EXPECT_TRUE(prepared.has_prepared_baseline());

    const auto est = prepared.run();
    ASSERT_EQ(est.status, MeasurementStatus::kSuccess) << est.message;
    EXPECT_DOUBLE_EQ(est.volume_cm3, inline_est.volume_cm3);
    EXPECT_EQ(est.baseline_cell_count, inline_est.baseline_cell_count);
    EXPECT_EQ(est.component_count, inline_est.component_count);
}



TEST(Measurer, PrepareBaselineReusedAcrossFoodFrames) {
    // One prepared baseline should serve several food frames without being discarded.
    auto measurer = make_measurer();
    measurer.set_baseline({make_baseline()}).set_food(make_food());
    ASSERT_EQ(measurer.prepare_baseline(), MeasurementStatus::kSuccess);

    const auto first = measurer.run();
    ASSERT_EQ(first.status, MeasurementStatus::kSuccess) << first.message;
    EXPECT_TRUE(measurer.has_prepared_baseline());

    // Swapping the food frame keeps the cache: this is the repeated-measurement use case.
    measurer.set_food(make_food_with_rect_hole(0.04, 0.06, 0.04, 0.06));
    EXPECT_TRUE(measurer.has_prepared_baseline());
    const auto second = measurer.run();
    ASSERT_EQ(second.status, MeasurementStatus::kSuccess) << second.message;
    EXPECT_EQ(second.baseline_cell_count, first.baseline_cell_count);
}



TEST(Measurer, PrepareBaselineInvalidatedByBaselineInputs) {
    struct InvalidationCase {
        const char* name;
        std::function<void(FoodVolumeMeasurer&)> mutate;
    };
    const InvalidationCase cases[] = {
        {"set_baseline", [](FoodVolumeMeasurer& m) { m.set_baseline({make_baseline()}); }},
        {"add_baseline_frame", [](FoodVolumeMeasurer& m) { m.add_baseline_frame(make_baseline()); }},
        {"set_input_unit", [](FoodVolumeMeasurer& m) { m.set_input_unit(LengthUnit::kMillimeter); }},
        {"set_voxel_size", [](FoodVolumeMeasurer& m) { m.set_voxel_size(0.003); }},
        {"set_integration_resolution", [](FoodVolumeMeasurer& m) { m.set_integration_resolution(0.004); }},
        {"set_plane_distance_threshold", [](FoodVolumeMeasurer& m) { m.set_plane_distance_threshold(0.004); }},
        {"set_cluster_params", [](FoodVolumeMeasurer& m) { m.set_cluster_params(0.012, 9); }},
        {"set_config", [](FoodVolumeMeasurer& m) { m.set_config(make_config()); }},
    };

    for (const auto& test_case : cases) {
        // A fresh measurer per case: the mutations accumulate otherwise (for example
        // switching to millimetres would reinterpret the metre-valued clouds).
        auto measurer = make_measurer();
        measurer.set_baseline({make_baseline()}).set_food(make_food());

        ASSERT_EQ(measurer.prepare_baseline(), MeasurementStatus::kSuccess) << test_case.name;
        ASSERT_TRUE(measurer.has_prepared_baseline()) << test_case.name;
        test_case.mutate(measurer);
        EXPECT_FALSE(measurer.has_prepared_baseline()) << test_case.name;
    }
}



TEST(Measurer, PrepareBaselineKeptByFoodOnlySetters) {
    // Parameters that only shape the food side must not throw the baseline cache away.
    auto measurer = make_measurer();
    measurer.set_baseline({make_baseline()}).set_food(make_food());
    ASSERT_EQ(measurer.prepare_baseline(), MeasurementStatus::kSuccess);

    measurer.set_height_range(0.002, 0.05)
        .set_selected_labels({0})
        .set_roi(AxisAlignedRoi{})
        .clear_roi();
    EXPECT_TRUE(measurer.has_prepared_baseline());

    const auto est = measurer.run();
    ASSERT_EQ(est.status, MeasurementStatus::kSuccess) << est.message;
}



TEST(Measurer, PrepareBaselineRejectsMissingInputs) {
    auto measurer = make_measurer();

    // No baseline frames yet.
    measurer.set_food(make_food());
    EXPECT_EQ(measurer.prepare_baseline(), MeasurementStatus::kEmptyBaseline);
    EXPECT_FALSE(measurer.has_prepared_baseline());

    // Baseline present, but the food frame needed for orientation is missing.
    auto measurer2 = make_measurer();
    measurer2.set_baseline({make_baseline()});
    EXPECT_EQ(measurer2.prepare_baseline(), MeasurementStatus::kEmptyInput);
    EXPECT_FALSE(measurer2.has_prepared_baseline());
}
