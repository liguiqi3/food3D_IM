// Conan::ImportStart
#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>
// Conan::ImportEnd



/**
 * @brief [en] Default parameter values shared by the IM volume pipeline.
 * @brief [zh] IM 体积流水线共享的默认参数值。
 */
namespace volume_defaults {

constexpr double kVoxelSizeM = 0.003;
constexpr double kPlaneDistanceM = 0.003;
constexpr int kPlaneRansacIterations = 1000;
constexpr double kBaselineMaxSurfaceHeightM = 0.03;
constexpr double kClusterEpsM = 0.02;
constexpr double kForegroundClusterEpsM = 0.010;
constexpr int kClusterMinPoints = 8;
constexpr double kIntegrationResolutionM = 0.003;
constexpr double kRoiBorderMarginM = 0.02;
constexpr double kMinHeightM = 0.0015;
constexpr int kBaselineFillRadiusCells = 2;
constexpr int kHoleFillMaxCells = 9;
constexpr int kHoleFillNeighborRadiusCells = 2;
constexpr double kHoleFillMaxNeighborDeltaM = 0.012;
constexpr double kCurveFillMaxHoleAreaCm2 = 8.0;
constexpr double kCurveFillMaxComponentAreaRatio = 0.30;
constexpr double kCurveFillMaxImputedRatio = 0.25;
constexpr int kCurveFillRimRadiusCells = 4;
constexpr int kCurveFillMinRimSamples = 16;
constexpr double kCurveFillMinRimCoverage = 0.75;
constexpr double kCurveFillMaxFitRmseM = 0.004;
constexpr double kCurveFillMaxPredictionRiseM = 0.015;
// Height watershed: two nearly touching items share one footprint cluster and are split
// only where the top surface drops by at least this depth (metres) between their peaks.
constexpr double kComponentSplitMinValleyDepthM = 0.004;
// ...and by at least this fraction of the lower peak's height, so a merely bumpy surface
// (sensor and voxel noise) is never split into pieces.
constexpr double kComponentSplitMinValleyFrac = 0.35;
// Watershed markers are the cells at or above this fraction of a component's peak height, so
// only genuine high plateaus seed a basin and surface noise cannot fragment a single item.
constexpr double kComponentSplitSeedFrac = 0.55;

} // namespace volume_defaults



/**
 * @brief [en] Linear unit of the input point cloud.
 * @brief [zh] 输入点云的线性单位。
 * @exporter
 */
enum class LengthUnit : std::uint8_t {
    kMeter = 0,
    kMillimeter = 1,
};



/**
 * @brief [en] A single 3D point with single precision.
 * @brief [zh] 单精度三维点。
 * @exporter
 */
struct Point3f {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};



/**
 * @brief [en] An ordered collection of 3D points.
 * @brief [zh] 有序三维点集合。
 * @exporter
 */
struct PointCloud {
    std::vector<Point3f> points;
};



/**
 * @brief [en] A plane in Hessian form: dot(n, p) = d, with unit normal n.
 *        Points on the food side satisfy dot(n, p) > d.
 * @brief [zh] Hessian 形式的平面：dot(n,p)=d，法向量 n 为单位向量，食材一侧满足 dot(n,p)>d。
 * @exporter
 */
struct Plane {
    float nx = 0.0F;
    float ny = 0.0F;
    float nz = 1.0F;
    float d = 0.0F;
};



/**
 * @brief [en] Axis-aligned region of interest bounds.
 * @brief [zh] 轴对齐的感兴趣区域边界。
 * @exporter
 */
struct AxisAlignedRoi {
    float min_x = 0.0F;
    float max_x = 0.0F;
    float min_y = 0.0F;
    float max_y = 0.0F;
    float min_z = 0.0F;
    float max_z = 0.0F;
};



/**
 * @brief [en] Outcome status of a volume measurement.
 * @brief [zh] 体积测量的结果状态。
 * @exporter
 */
enum class MeasurementStatus : std::uint8_t {
    kSuccess = 0,
    kEmptyInput = 1,
    kNonFiniteInput = 2,
    kInvalidConfig = 3,
    kEmptyBaseline = 4,
    kNonFiniteBaseline = 5,
    kPlaneNotFound = 6,
    kBaselineNoCells = 8,
    kFoodNotFound = 9,
    kInsufficientCoverage = 10,
    kUnsupportedPlatform = 11,
};



/**
 * @brief [en] Configuration for the IM volume pipeline.
 * @brief [zh] IM 体积流水线的配置。
 * @details [en] Input points use `input_unit`; ROI bounds and every distance field use
 *     metres. An absent `roi` disables cropping, an empty `selected_labels` selects all
 *     eligible components, and an empty `middle_cloud_dir` disables PCD dumps. Start with
 *     defaults and change only values supported by the capture geometry. A `voxel_size_m` of 0
 *     keeps every point (no downsampling).
 *     `enable_height_watershed` (default false) splits an over-merged footprint cluster at a
 *     height valley; enable it to separate nearly touching items.
 * @details [zh] 输入点坐标使用 `input_unit`；ROI 边界与距离字段始终以米计。
 *     `roi` 为空时不裁剪，`selected_labels` 为空时选择全部合格连通块，
 *     `middle_cloud_dir` 为空时不保存中间 PCD。建议从默认值开始，只调整与采集条件有关的值。
 *     `voxel_size_m` 取 0 表示不做降采样、保留全部点。
 *     `enable_height_watershed`（默认 false）在高度凹谷处细分被过度合并的足迹簇；
 *     启用后可区分相邻食材。
 * @exporter
 */
struct MeasurementConfig {
    LengthUnit input_unit = LengthUnit::kMeter;

    std::optional<AxisAlignedRoi> roi;

    // Voxel edge length for downsampling, in metres. A value of 0 disables downsampling.
    double voxel_size_m = volume_defaults::kVoxelSizeM;

    // Baseline plane fitting.
    double plane_distance_threshold_m = volume_defaults::kPlaneDistanceM;

    // Optional second dominant background plane removal.
    bool remove_secondary_plane = false;

    // Foreground footprint clustering (2D).
    double foreground_cluster_eps_m = volume_defaults::kForegroundClusterEpsM;
    int cluster_min_points = volume_defaults::kClusterMinPoints;

    // Split an over-merged footprint cluster where the top surface dips into a height valley
    // between two peaks. Disabled by default; enable it to separate nearly touching items.
    bool enable_height_watershed = false;

    // Foreground component selection.
    // Empty selects all eligible components.
    std::vector<int> selected_labels;

    // Integration.
    double integration_resolution_m = volume_defaults::kIntegrationResolutionM;
    double roi_border_margin_m = volume_defaults::kRoiBorderMarginM;
    double min_height_m = volume_defaults::kMinHeightM;
    double max_height_m = -1.0; // negative disables the cap

    // Small-hole interpolation.
    int hole_fill_max_cells = volume_defaults::kHoleFillMaxCells;

    // Quadratic-surface hole completion.
    double curve_fill_max_hole_area_cm2 = volume_defaults::kCurveFillMaxHoleAreaCm2;

    // Optional per-stage point-cloud dump for debugging and visualisation.
    // Empty disables the dump.
    std::string middle_cloud_dir;
};



/**
 * @brief [en] Per-component IM integration result.
 * @brief [zh] 单个连通块 IM 积分结果。
 * @exporter
 */
struct ComponentVolumeEstimate {
    double raw_volume_cm3 = 0.0;
    double interpolated_volume_cm3 = 0.0;
    double volume_cm3 = 0.0;

    std::size_t top_surface_points = 0;
    std::size_t measured_cells = 0;
    std::size_t interpolated_cells = 0;
    std::size_t occupied_cells = 0;
    std::size_t bbox_cell_count = 0;
    std::size_t missing_baseline_cells = 0;
    std::size_t unfilled_hole_cells = 0;

    double footprint_area_m2 = 0.0;
    double coverage_ratio = 0.0;
    double mean_height_m = 0.0;
    double max_height_m = 0.0;
};



/**
 * @brief [en] Volume estimate and diagnostics from one food frame.
 * @brief [zh] 单帧食材的体积估计与诊断信息。
 * @details [en] Check `status` before reading numeric results; volume and geometry values
 *     are NaN on failure. On success, `volume_cm3` equals `raw_volume_cm3` plus
 *     `interpolated_volume_cm3`. `component_estimates` follows the order of
 *     `selected_cluster_labels`. AABB, OBB, and convex-hull volumes use cubic metres and
 *     are geometric references, not tray-relative integrals. Cell counts remain useful
 *     diagnostics even when an estimate is incomplete.
 * @details [zh] 先检查 `status`；失败时体积及几何数值为 NaN。成功时
 *     `volume_cm3` 等于 `raw_volume_cm3` 与 `interpolated_volume_cm3` 之和。
 *     `component_estimates` 与 `selected_cluster_labels` 顺序一致。AABB、OBB 和凸包体积
 *     使用立方米，是几何参考值，并非相对烤盘的高度积分。栅格计数可用于排查测量质量。
 * @exporter
 */
struct VolumeEstimate {
    MeasurementStatus status = MeasurementStatus::kSuccess;
    double volume_cm3 = std::numeric_limits<double>::quiet_NaN();
    double raw_volume_cm3 = std::numeric_limits<double>::quiet_NaN();
    double interpolated_volume_cm3 = std::numeric_limits<double>::quiet_NaN();

    std::size_t input_points = 0;
    std::size_t downsampled_points = 0;
    std::size_t cluster_count = 0;
    std::vector<int> selected_cluster_labels;
    std::size_t selected_cluster_points = 0;

    /// [en] Per-component integration results, aligned with `selected_cluster_labels`.
    /// [zh] 逐连通块积分结果，与 `selected_cluster_labels` 一一对应。
    std::vector<ComponentVolumeEstimate> component_estimates;

    std::size_t baseline_frames = 0;
    std::size_t baseline_cell_count = 0;
    std::size_t component_count = 0;

    std::size_t top_surface_points = 0;
    std::size_t measured_cells = 0;
    std::size_t interpolated_cells = 0;
    std::size_t occupied_cells = 0;
    std::size_t bbox_cell_count = 0;
    std::size_t missing_baseline_cells = 0;
    std::size_t unfilled_hole_cells = 0;

    double footprint_area_m2 = std::numeric_limits<double>::quiet_NaN();
    double coverage_ratio = std::numeric_limits<double>::quiet_NaN();
    double mean_height_m = std::numeric_limits<double>::quiet_NaN();
    double max_height_m = std::numeric_limits<double>::quiet_NaN();

    double aabb_volume_m3 = std::numeric_limits<double>::quiet_NaN();
    double obb_volume_m3 = std::numeric_limits<double>::quiet_NaN();
    double convex_hull_volume_m3 = std::numeric_limits<double>::quiet_NaN();

    std::string message;
};



/**
 * @brief [en] Converts a length unit to the meters scale factor.
 * @brief [zh] 将长度单位转换为米的比例因子。
 * @param unit [en] The input length unit.
 * @param unit [zh] 输入长度单位。
 * @return [en] The factor that converts a coordinate in `unit` to meters.
 * @return [zh] 把 `unit` 坐标换算成米的因子。
 * @exporter
 */
double length_unit_to_meter_scale(LengthUnit unit);



/**
 * @brief [en] Returns a human-readable description of a measurement status.
 * @brief [zh] 返回测量状态的可读描述。
 * @param status [en] The measurement status.
 * @param status [zh] 测量状态。
 * @return [en] A static description string.
 * @return [zh] 静态描述字符串。
 * @exporter
 */
const char* status_to_string(MeasurementStatus status);



/**
 * @brief [en] Checks whether a point has finite coordinates.
 * @brief [zh] 检查点坐标是否有限。
 * @param p [en] The point to test.
 * @param p [zh] 待检查的点。
 * @return [en] True when all coordinates are finite.
 * @return [zh] 所有坐标都有限时为真。
 * @exporter
 */
inline bool is_finite(const Point3f& p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }
