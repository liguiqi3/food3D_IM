// Conan::ImportStart
#pragma once
#include <cstddef>
#include <vector>
#include "baseline.hpp"
#include "types.hpp"
// Conan::ImportEnd



/**
 * @brief [en] Selected foreground food components: stable public labels plus their point clouds.
 * @brief [zh] 选中的前景食材块：稳定的公开标签及其点云。

 */
struct FoodComponents {
    std::size_t cluster_count = 0;  // total non-noise DBSCAN clusters found before selection
    std::vector<int> labels;        // ascending, one entry per component
    std::vector<PointCloud> clouds; // one cloud per label, same order as `labels`
};



/**
 * @brief [en] Filters food points by baseline-relative height, clusters them in the baseline plane, and selects components.
 * @brief [zh] 按相对基线高度过滤食材点，在基准面内聚类并选择连通块。
 * @param food_m [en] Food points in metres after background-plane removal.
 * @param food_m [zh] 移除背景平面后以米为单位的食材点。
 * @param baseline [en] Empty-oven baseline model.
 * @param baseline [zh] 空炉基线模型。
 * @param cfg [en] Measurement configuration.
 * @param cfg [zh] 测量配置。
 * @param out [en] Selected component labels and their point clouds.
 * @param out [zh] 选中的连通块标签及其点云。
 * @return [en] kSuccess, kInsufficientCoverage, kFoodNotFound, or kInvalidConfig.
 * @return [zh] kSuccess、kInsufficientCoverage、kFoodNotFound 或 kInvalidConfig。

 */
MeasurementStatus extract_food_components(const PointCloud& food_m, const BaselineModel& baseline,
                                          const MeasurementConfig& cfg, FoodComponents& out);
