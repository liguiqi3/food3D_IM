// Conan::ImportStart
#include <cstddef>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>
#include "measurement.hpp"
#include "baseline.hpp"
#include "component.hpp"
#include "config.hpp"
#include "integrator.hpp"
#include "grid.hpp"
#include "log.hpp"
#include "pointcloudprocess.hpp"
// Conan::ImportEnd


#ifndef __ARM_EABI__
#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <system_error>
#include <Eigen/Dense>
#include <pcl/PolygonMesh.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#endif



// Cached empty-oven baseline model; opaque to the public header.
struct FoodVolumeMeasurer::PreparedBaseline {
    BaselineModel model;
};



namespace {

VolumeEstimate failure(MeasurementStatus status, const std::string& message) {
    log_error("pipeline failed: " + message);
    VolumeEstimate est{};
    est.status = status;
    est.message = message;
    return est;
}

// Concatenates the selected component clouds once for the reference-volume helpers.
PointCloud merged_components_cloud(const FoodComponents& components) {
    PointCloud merged;
    std::size_t total = 0;
    for (const auto& cloud : components.clouds) {
        total += cloud.points.size();
    }
    merged.points.reserve(total);
    for (const auto& cloud : components.clouds) {
        merged.points.insert(merged.points.end(), cloud.points.begin(), cloud.points.end());
    }
    return merged;
}

// Renders plane-frame cell centres back into world coordinates at their stored heights.
PointCloud cells_to_world_cloud(const std::vector<CellKey>& cells, const std::vector<double>& heights_m,
                                const PlaneFrame& frame, double cell_size_m) {
    PointCloud out;
    out.points.reserve(cells.size());
    for (std::size_t i = 0; i < cells.size(); ++i) {
        const double u = (static_cast<double>(cells[i].first) + 0.5) * cell_size_m;
        const double v = (static_cast<double>(cells[i].second) + 0.5) * cell_size_m;
        const double h = heights_m[i];
        out.points.push_back(Point3f{static_cast<float>(frame.ox + u * frame.ux + v * frame.vx + h * frame.nx),
                                     static_cast<float>(frame.oy + u * frame.uy + v * frame.vy + h * frame.ny),
                                     static_cast<float>(frame.oz + u * frame.uz + v * frame.vz + h * frame.nz)});
    }
    return out;
}

// Left-pads a non-negative value with zeros so the per-component files sort in
// component order rather than lexicographically.
std::string zero_padded(int value, int width) {
    std::string text = std::to_string(value);
    if (static_cast<int>(text.size()) < width) {
        text.insert(0, static_cast<std::size_t>(width) - text.size(), '0');
    }
    return text;
}

// Writes one PCD per pipeline stage when the caller asked for the intermediate dump.
#ifndef __ARM_EABI__
void dump_middle_clouds(const MeasurementConfig& cfg, const PointCloud& raw_food, const PointCloud& downsampled,
                        const PointCloud& remaining, const BaselineModel& baseline, const FoodComponents& components,
                        const SurfaceMap& completed_surface) {
    std::error_code ec;
    std::filesystem::create_directories(cfg.middle_cloud_dir, ec);
    if (ec) {
        log_error("cannot create middle cloud dir '" + cfg.middle_cloud_dir + "': " + ec.message());
        return;
    }

    const auto path_of = [&cfg](const char* name) { return cfg.middle_cloud_dir + "/" + name; };
    save_pcd_impl(path_of("0_input_food.pcd"), raw_food);
    save_pcd_impl(path_of("1_downsampled.pcd"), downsampled);
    save_pcd_impl(path_of("2_remaining.pcd"), remaining);

    if (baseline.data) {
        const BaselineData& data = *baseline.data;
        std::vector<CellKey> cells;
        std::vector<double> heights;
        cells.reserve(data.height_by_cell.size());
        heights.reserve(data.height_by_cell.size());
        for (const auto& entry : data.height_by_cell) {
            cells.push_back(entry.first);
            heights.push_back(entry.second);
        }
        save_pcd_impl(path_of("3_baseline_surface.pcd"),
                      cells_to_world_cloud(cells, heights, data.frame, data.cell_size_m));
    }

    // Stage 4: the selected food components. The merged cloud keeps the whole scene
    // together, and every component is additionally written on its own so a
    // multi-food frame can be inspected one connected region at a time.
    save_pcd_impl(path_of("4_food_components.pcd"), merged_components_cloud(components));

    std::size_t component_files = 0;
    for (std::size_t i = 0; i < components.clouds.size(); ++i) {
        const int label = (i < components.labels.size()) ? components.labels[i] : static_cast<int>(i);
        const std::string name = "4_food_component_label" + zero_padded(label, 2) + ".pcd";
        if (save_pcd_impl(path_of(name.c_str()), components.clouds[i])) {
            ++component_files;
        }
    }
    log_debug("wrote " + std::to_string(component_files) + " per-component cloud(s)");

    const SurfaceMap surface = build_top_surface(components, baseline);
    if (baseline.data && !surface.cells.empty()) {
        save_pcd_impl(
            path_of("5_top_surface.pcd"),
            cells_to_world_cloud(surface.cells, surface.heights_m, baseline.data->frame, baseline.data->cell_size_m));
    }
    if (baseline.data && !completed_surface.cells.empty()) {
        save_pcd_impl(path_of("6_hole_filled_surface.pcd"),
                      cells_to_world_cloud(completed_surface.cells, completed_surface.heights_m, baseline.data->frame,
                                           baseline.data->cell_size_m));
    }
    log_info("middle clouds written to " + cfg.middle_cloud_dir);
}
#endif // __ARM_EABI__


// Chooses orientation points from the largest 3D DBSCAN component, or empty when none qualifies.
PointCloud select_largest_cluster(const PointCloud& cloud_m, const MeasurementConfig& cfg) {
#ifdef __ARM_EABI__
    (void)cloud_m;
    (void)cfg;
    return PointCloud{};
#else
    PointCloud out{};
    if (cloud_m.points.empty() || cfg.cluster_min_points <= 0) {
        return out;
    }

    const std::vector<int> labels =
        dbscan_labels(cloud_m.points, volume_defaults::kClusterEpsM, cfg.cluster_min_points);

    std::map<int, std::size_t> counts;
    for (const int label : labels) {
        if (label >= 0) {
            counts[label] += 1;
        }
    }
    if (counts.empty()) {
        return out;
    }

    int best_label = -1;
    std::size_t best_count = 0;
    for (const auto& entry : counts) {
        if (entry.second > best_count) {
            best_label = entry.first;
            best_count = entry.second;
        }
    }

    out.points.reserve(best_count);
    for (std::size_t i = 0; i < labels.size(); ++i) {
        if (labels[i] == best_label) {
            out.points.push_back(cloud_m.points[i]);
        }
    }
    return out;
#endif // __ARM_EABI__
}

} // namespace



/**
 * @brief [en] Loads a PCD point-cloud file into the library's point model.
 * @brief [zh] 将 PCD 点云文件载入到库的点模型。
 * @attacher
 */
PointCloud load_pcd(const std::string& path) { return load_pcd_impl(path); }



/**
 * @brief [en] Replaces the empty-oven baseline frames (in `input_unit`).
 * @brief [zh] 替换空炉基线帧（单位为 `input_unit`）。
 * @attacher
 */
FoodVolumeMeasurer& FoodVolumeMeasurer::set_baseline(const std::vector<PointCloud>& frames) {
    baseline_frames_ = frames;
    invalidate_prepared_baseline();
    return *this;
}



/**
 * @brief [en] Appends one empty-oven baseline frame (in `input_unit`).
 * @brief [zh] 追加一帧空炉基线（单位为 `input_unit`）。
 * @attacher
 */
FoodVolumeMeasurer& FoodVolumeMeasurer::add_baseline_frame(const PointCloud& frame) {
    baseline_frames_.push_back(frame);
    invalidate_prepared_baseline();
    return *this;
}



/**
 * @brief [en] Sets the food point cloud to measure (in `input_unit`).
 * @brief [zh] 设置待测食材点云（单位为 `input_unit`）。
 * @attacher
 */
FoodVolumeMeasurer& FoodVolumeMeasurer::set_food(const PointCloud& cloud) {
    food_ = cloud;
    return *this;
}



/**
 * @brief [en] Sets the linear unit of all input point clouds.
 * @brief [zh] 设置所有输入点云的线性单位。
 * @attacher
 */
FoodVolumeMeasurer& FoodVolumeMeasurer::set_input_unit(LengthUnit unit) {
    cfg_.input_unit = unit;
    invalidate_prepared_baseline();
    return *this;
}



/**
 * @brief [en] Sets the voxel size used for downsampling, in metres.
 * @brief [zh] 设置降采样体素边长（米）。
 * @attacher
 */
FoodVolumeMeasurer& FoodVolumeMeasurer::set_voxel_size(double size_m) {
    cfg_.voxel_size_m = size_m;
    invalidate_prepared_baseline();
    return *this;
}



/**
 * @brief [en] Sets the baseline/integration raster cell size, in metres.
 * @brief [zh] 设置基线/积分栅格边长（米）。
 * @attacher
 */
FoodVolumeMeasurer& FoodVolumeMeasurer::set_integration_resolution(double resolution_m) {
    cfg_.integration_resolution_m = resolution_m;
    invalidate_prepared_baseline();
    return *this;
}



/**
 * @brief [en] Sets the accepted baseline-relative height range of food points, in metres.
 * @brief [zh] 设置食材点相对基线高度的接受范围（米）。
 * @attacher
 */
FoodVolumeMeasurer& FoodVolumeMeasurer::set_height_range(double min_m, double max_m) {
    cfg_.min_height_m = min_m;
    cfg_.max_height_m = max_m;
    return *this;
}



/**
 * @brief [en] Enables an axis-aligned crop region applied before downsampling.
 * @brief [zh] 启用降采样前应用的轴对齐裁剪区域。
 * @attacher
 */
FoodVolumeMeasurer& FoodVolumeMeasurer::set_roi(const AxisAlignedRoi& roi) {
    cfg_.roi = roi;
    return *this;
}



/**
 * @brief [en] Disables the axis-aligned crop region.
 * @brief [zh] 禁用轴对齐裁剪区域。
 * @attacher
 */
FoodVolumeMeasurer& FoodVolumeMeasurer::clear_roi() {
    cfg_.roi.reset();
    return *this;
}



/**
 * @brief [en] Selects component labels; an empty list selects all eligible components.
 * @brief [zh] 选择连通块标签；空列表选择所有符合条件的连通块。
 * @attacher
 */
FoodVolumeMeasurer& FoodVolumeMeasurer::set_selected_labels(const std::vector<int>& labels) {
    cfg_.selected_labels = labels;
    return *this;
}



/**
 * @brief [en] Sets the footprint clustering radius and minimum cluster size.
 * @brief [zh] 设置足迹聚类半径与最小簇规模。
 * @attacher
 */
FoodVolumeMeasurer& FoodVolumeMeasurer::set_cluster_params(double footprint_eps_m, int min_points) {
    cfg_.foreground_cluster_eps_m = footprint_eps_m;
    cfg_.cluster_min_points = min_points;
    invalidate_prepared_baseline();
    return *this;
}



/**
 * @brief [en] Sets the RANSAC inlier distance threshold for background-plane removal.
 * @brief [zh] 设置背景平面移除的 RANSAC 内点距离阈值。
 * @attacher
 */
FoodVolumeMeasurer& FoodVolumeMeasurer::set_plane_distance_threshold(double threshold_m) {
    cfg_.plane_distance_threshold_m = threshold_m;
    invalidate_prepared_baseline();
    return *this;
}



/**
 * @brief [en] Sets the output directory for the intermediate stage point clouds.
 * @brief [zh] 设置中间阶段点云的输出目录。
 * @attacher
 */
FoodVolumeMeasurer& FoodVolumeMeasurer::set_middle_cloud_dir(const std::string& dir) {
    cfg_.middle_cloud_dir = dir;
    return *this;
}



/**
 * @brief [en] Replaces the full measurement configuration, including advanced fields.
 * @brief [zh] 替换完整测量配置（含高级字段）。
 * @attacher
 */
FoodVolumeMeasurer& FoodVolumeMeasurer::set_config(const MeasurementConfig& cfg) {
    cfg_ = cfg;
    invalidate_prepared_baseline();
    return *this;
}



/**
 * @brief [en] Returns the current measurement configuration.
 * @brief [zh] 返回当前测量配置。
 * @attacher
 */
const MeasurementConfig& FoodVolumeMeasurer::config() const { return cfg_; }



/**
 * @brief [en] Loads the configuration from a JSON file, overriding stored values.
 * @brief [zh] 从 JSON 文件载入配置并覆盖已存值。
 * @attacher
 */
bool FoodVolumeMeasurer::load_config_from_json(const std::string& path) {
    const bool ok = ::load_config_from_json(path, cfg_);
    if (ok) {
        invalidate_prepared_baseline();
    }
    return ok;
}



/**
 * @brief [en] Saves the current configuration to a JSON file.
 * @brief [zh] 将当前配置保存到 JSON 文件。
 * @attacher
 */
bool FoodVolumeMeasurer::save_config_to_json(const std::string& path) const {
    return ::save_config_to_json(cfg_, path);
}



/**
 * @brief [en] Builds and caches the empty-oven baseline model so repeated `run` calls skip it.
 * @brief [zh] 构建并缓存空炉基线模型，使后续 `run` 调用无需重复构建。
 * @attacher
 */
MeasurementStatus FoodVolumeMeasurer::prepare_baseline() {
    prepared_baseline_.reset();
#ifdef __ARM_EABI__
    return MeasurementStatus::kUnsupportedPlatform;
#else
    const MeasurementConfig& cfg = cfg_;
    if (baseline_frames_.empty()) {
        return MeasurementStatus::kEmptyBaseline;
    }
    // The food frame is what orients the baseline normal, so it must be known up front.
    if (food_.points.empty()) {
        log_error("prepare_baseline needs a food frame for orientation; call set_food first");
        return MeasurementStatus::kEmptyInput;
    }

    // Reproduce the pipeline's first stages exactly so the cached baseline is identical to
    // the one run() would have built inline.
    PreprocessResult pre{};
    MeasurementStatus st = preprocess_cloud(food_, cfg, pre);
    if (st != MeasurementStatus::kSuccess) {
        return st;
    }

    PointCloud remaining{};
    st = remove_dominant_plane(pre.cloud, cfg, remaining);
    if (st != MeasurementStatus::kSuccess) {
        return st;
    }

    PointCloud orientation = select_largest_cluster(remaining, cfg);
    if (orientation.points.empty()) {
        orientation = remaining;
    }

    BaselineModel model{};
    st = build_baseline_model(baseline_frames_, orientation, cfg, model);
    if (st != MeasurementStatus::kSuccess) {
        return st;
    }

    auto cache = std::make_shared<PreparedBaseline>();
    cache->model = std::move(model);
    prepared_baseline_ = std::move(cache);
    log_info("prepare_baseline: cached baseline frames=" + std::to_string(prepared_baseline_->model.frame_count) +
             " cells=" + std::to_string(prepared_baseline_->model.cell_count));
    return MeasurementStatus::kSuccess;
#endif // __ARM_EABI__
}



/**
 * @brief [en] Returns whether a prepared baseline model is currently cached.
 * @brief [zh] 返回当前是否已缓存了准备好的基线模型。
 * @attacher
 */
bool FoodVolumeMeasurer::has_prepared_baseline() const { return prepared_baseline_ != nullptr; }



void FoodVolumeMeasurer::invalidate_prepared_baseline() { prepared_baseline_.reset(); }



VolumeEstimate FoodVolumeMeasurer::run() const {
    const MeasurementConfig& cfg = cfg_;
#ifdef __ARM_EABI__
    (void)cfg;
    return failure(MeasurementStatus::kUnsupportedPlatform, "PCL volume pipeline is unavailable on bare-metal");
#else
    if (baseline_frames_.empty()) {
        return failure(MeasurementStatus::kEmptyBaseline, status_to_string(MeasurementStatus::kEmptyBaseline));
    }
    if (food_.points.empty()) {
        return failure(MeasurementStatus::kEmptyInput, status_to_string(MeasurementStatus::kEmptyInput));
    }

    // 1. Preprocess the food frame (unit-normalize, optional crop, voxel downsample).
    PreprocessResult pre{};
    MeasurementStatus st = preprocess_cloud(food_, cfg, pre);
    if (st != MeasurementStatus::kSuccess) {
        return failure(st, status_to_string(st));
    }

    // 2. Remove the dominant background plane(s) from the food frame.
    PointCloud remaining{};
    st = remove_dominant_plane(pre.cloud, cfg, remaining);
    if (st != MeasurementStatus::kSuccess) {
        return failure(st, status_to_string(st));
    }

    // 3-4. Baseline model: reuse the one prepared by prepare_baseline() when available,
    // which also lets us skip the orientation DBSCAN pass entirely.
    BaselineModel baseline{};
    if (prepared_baseline_) {
        baseline = prepared_baseline_->model;
        log_debug("run: reusing the prepared baseline model");
    } else {
        // Orientation points: largest DBSCAN component, or all remaining when none qualifies.
        PointCloud orientation = select_largest_cluster(remaining, cfg);
        if (orientation.points.empty()) {
            orientation = remaining;
        }
        st = build_baseline_model(baseline_frames_, orientation, cfg, baseline);
        if (st != MeasurementStatus::kSuccess) {
            return failure(st, status_to_string(st));
        }
    }

    // 5. Extract selected food components.
    FoodComponents components{};
    st = extract_food_components(remaining, baseline, cfg, components);
    if (st != MeasurementStatus::kSuccess) {
        return failure(st, status_to_string(st));
    }

    // 6. Measure the component volume. The per-component breakdown is derived from the
    // same rasterisation pass, so this runs the integration exactly once.
    ComponentVolumeEstimate component_volume{};
    std::vector<ComponentVolumeEstimate> per_component;
    SurfaceMap completed_surface;
    st = measure_component_volume(components, baseline, cfg, component_volume, &per_component,
                                  !cfg.middle_cloud_dir.empty() ? &completed_surface : nullptr);
    if (st != MeasurementStatus::kSuccess) {
        return failure(st, status_to_string(st));
    }

    // 6b. Optional per-stage point-cloud dump, only after the run fully succeeded.
    if (!cfg.middle_cloud_dir.empty()) {
        dump_middle_clouds(cfg, food_, pre.cloud, remaining, baseline, components, completed_surface);
    }

    // 7. Translate the component result and pipeline diagnostics into the public estimate.
    VolumeEstimate est{};
    est.status = MeasurementStatus::kSuccess;
    est.volume_cm3 = component_volume.volume_cm3;
    est.raw_volume_cm3 = component_volume.raw_volume_cm3;
    est.interpolated_volume_cm3 = component_volume.interpolated_volume_cm3;
    est.input_points = pre.input_points;
    est.downsampled_points = pre.retained_points;
    est.cluster_count = components.cluster_count;
    est.selected_cluster_labels = components.labels;
    est.selected_cluster_points = 0;
    for (const auto& cloud : components.clouds) {
        est.selected_cluster_points += cloud.points.size();
    }
    est.baseline_frames = baseline.frame_count;
    est.baseline_cell_count = baseline.cell_count;
    est.component_count = components.labels.size();

    // Per-component breakdown, aligned with `selected_cluster_labels`. The merged total above
    // stays authoritative; the breakdown is a split of the very same grid.
    est.component_estimates = std::move(per_component);

    est.top_surface_points = component_volume.top_surface_points;
    est.measured_cells = component_volume.measured_cells;
    est.interpolated_cells = component_volume.interpolated_cells;
    est.occupied_cells = component_volume.occupied_cells;
    est.bbox_cell_count = component_volume.bbox_cell_count;
    est.missing_baseline_cells = component_volume.missing_baseline_cells;
    est.unfilled_hole_cells = component_volume.unfilled_hole_cells;
    est.footprint_area_m2 = component_volume.footprint_area_m2;
    est.coverage_ratio = component_volume.coverage_ratio;
    est.mean_height_m = component_volume.mean_height_m;
    est.max_height_m = component_volume.max_height_m;
    // Reference volumes from the merged selected-component cloud.
    const PointCloud merged = merged_components_cloud(components);
    est.aabb_volume_m3 = compute_aabb_volume(merged);
    est.obb_volume_m3 = compute_obb_volume(merged);
    est.convex_hull_volume_m3 = compute_convex_hull_volume(merged);
    est.message = status_to_string(MeasurementStatus::kSuccess);
    log_info("result: status=" + std::string(status_to_string(est.status)) +
             " volume_cm3=" + std::to_string(est.volume_cm3));
    return est;
#endif // __ARM_EABI__
}
