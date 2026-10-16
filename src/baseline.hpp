// Conan::ImportStart
#pragma once
#include <cstddef>
#include <memory>
#include <vector>
#include "grid.hpp"
#include "types.hpp"
// Conan::ImportEnd



/**
 * @brief [en] An empty-oven baseline for the IM pipeline.
 * @brief [zh] IM 流水线的空炉基线。

 */
struct BaselineModel {
    std::size_t frame_count = 0;
    std::size_t cell_count = 0;
    std::shared_ptr<BaselineData> data;
};



/**
 * @brief [en] Builds the empty-oven baseline model from one or more baseline frames.
 * @brief [zh] 从一帧或多帧空炉点云建立空炉基线模型。
 * @param baseline_frames [en] Empty-oven point clouds in `cfg.input_unit`.
 * @param baseline_frames [zh] 以 `cfg.input_unit` 为单位的空炉点云。
 * @param orientation_points [en] Food-side points used to orient the plane normal; may be empty.
 * @param orientation_points [zh] 用于确定平面法向的食材侧点；可为空。
 * @param cfg [en] Measurement configuration.
 * @param cfg [zh] 测量配置。
 * @param out [en] Built baseline model.
 * @param out [zh] 构建好的基线模型。
 * @return [en] kSuccess, kEmptyBaseline, kNonFiniteBaseline, kInvalidConfig, kPlaneNotFound, or kBaselineNoCells.
 * @return [zh] kSuccess、kEmptyBaseline、kNonFiniteBaseline、kInvalidConfig、kPlaneNotFound 或 kBaselineNoCells。

 */
MeasurementStatus build_baseline_model(const std::vector<PointCloud>& baseline_frames,
                                       const PointCloud& orientation_points, const MeasurementConfig& cfg,
                                       BaselineModel& out);
