#include <gtest/gtest.h>
#include <cmath>
#include <cstddef>
#include <vector>
#include "measurement.hpp"
#include "types.hpp"



namespace {

constexpr double kStep = 0.003;
constexpr double kFoodTopM = 0.02;

// Large empty-oven tray in the z=0 plane, centred on the origin.
PointCloud make_tray(double half) {
    PointCloud cloud;
    for (double x = -half; x <= half + 1.0e-9; x += kStep) {
        for (double y = -half; y <= half + 1.0e-9; y += kStep) {
            cloud.points.push_back({static_cast<float>(x), static_cast<float>(y), 0.0F});
        }
    }
    return cloud;
}

// Tray plus a broad food top surface, exercising the pipeline at scale.
PointCloud make_food(double half) {
    PointCloud cloud = make_tray(half);
    const double hi = half - kStep;
    for (double x = kStep; x <= hi + 1.0e-9; x += kStep) {
        for (double y = kStep; y <= hi + 1.0e-9; y += kStep) {
            cloud.points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(kFoodTopM)});
        }
    }
    return cloud;
}

} // namespace



TEST(Stress, LargeCloudDownsampling) {
    // A large scene must be voxel-downsampled by the pipeline before clustering.
    FoodVolumeMeasurer measurer;
    const auto est = measurer.set_baseline({make_tray(0.3)}).set_food(make_food(0.3)).run();
    ASSERT_EQ(est.status, MeasurementStatus::kSuccess) << est.message;
    EXPECT_GT(est.input_points, 40000u);
    EXPECT_LT(est.downsampled_points, est.input_points);
    EXPECT_GT(est.downsampled_points, 0u);
}



TEST(Stress, ElongatedComponent) {
    // A long thin food ridge spanning the tray must still resolve as one component.
    PointCloud food = make_tray(0.3);
    for (double x = -0.28; x <= 0.28 + 1.0e-9; x += kStep) {
        for (double y = -0.01; y <= 0.01 + 1.0e-9; y += kStep) {
            food.points.push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(kFoodTopM)});
        }
    }
    FoodVolumeMeasurer measurer;
    const auto est = measurer.set_baseline({make_tray(0.3)}).set_food(food).run();
    ASSERT_EQ(est.status, MeasurementStatus::kSuccess) << est.message;
    EXPECT_EQ(est.component_count, 1u);
    EXPECT_GT(est.volume_cm3, 0.0);
}



TEST(Stress, PipelineLargeScene) {
    const double half = 0.3;
    FoodVolumeMeasurer measurer;
    const auto est = measurer.set_baseline({make_tray(half)}).set_food(make_food(half)).run();
    EXPECT_EQ(est.status, MeasurementStatus::kSuccess);
    EXPECT_GT(est.component_count, 0u);
    EXPECT_GT(est.volume_cm3, 0.0);
    EXPECT_GT(est.coverage_ratio, 0.5);
}
