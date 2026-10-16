// Conan::ImportStart
#pragma once
#include <cstddef>
#include <string>
#include <vector>
#include "types.hpp"
// Conan::ImportEnd



/**
 * @brief [en] Output of the point-cloud preprocessing stage.
 * @brief [zh] 点云预处理阶段的输出。
 * @exporter
 */
struct PreprocessResult {
    PointCloud cloud;
    std::size_t input_points = 0;
    std::size_t retained_points = 0;
};



/**
 * @brief [en] Validates, unit-normalizes, optionally crops, and voxel-downsamples a cloud.
 * @brief [zh] 校验、单位换算、可选裁剪并体素降采样点云。
 * @param input [en] Raw input cloud in `cfg.input_unit`.
 * @param input [zh] 以 `cfg.input_unit` 为单位的原始输入点云。
 * @param cfg [en] Configuration; ROI bounds and voxel size are in metres.
 * @param cfg [zh] 配置；ROI 边界和体素边长均以米为单位。
 * @param out [en] Processed cloud in meters plus point counters.
 * @param out [zh] 以米为单位的处理后点云以及点数统计。
 * @return [en] kSuccess, or an input/config error status.
 * @return [zh] kSuccess，或输入/配置错误状态。
 * @exporter
 */
MeasurementStatus preprocess_cloud(const PointCloud& input, const MeasurementConfig& cfg, PreprocessResult& out);



/**
 * @brief [en] Scales coordinates to metres and drops non-finite points.
 * @brief [zh] 将坐标缩放到米并丢弃非有限点。
 * @param cloud [en] Input cloud in `unit`.
 * @param cloud [zh] 以 `unit` 为单位的输入点云。
 * @param unit [en] Input length unit.
 * @param unit [zh] 输入长度单位。
 * @return [en] Scaled cloud in metres.
 * @return [zh] 以米为单位的缩放后点云。
 * @exporter
 */
PointCloud scale_to_meters(const PointCloud& cloud, LengthUnit unit);



/**
 * @brief [en] Keeps only points inside an axis-aligned box.
 * @brief [zh] 仅保留位于轴对齐包围盒内的点。
 * @param cloud [en] Input cloud.
 * @param cloud [zh] 输入点云。
 * @param roi [en] Axis-aligned bounds in the same units as `cloud`.
 * @param roi [zh] 与 `cloud` 同单位的轴对齐边界。
 * @return [en] Cropped cloud.
 * @return [zh] 裁剪后的点云。
 * @exporter
 */
PointCloud crop_axis_aligned(const PointCloud& cloud, const AxisAlignedRoi& roi);



/**
 * @brief [en] Splits a cloud into plane outliers and inliers for a given Hessian plane.
 * @brief [zh] 按给定 Hessian 平面把点云拆成平面外点与内点。
 * @param cloud [en] Input cloud.
 * @param cloud [zh] 输入点云。
 * @param plane [en] Normalized Hessian plane.
 * @param plane [zh] 归一化的 Hessian 平面。
 * @param distance_threshold_m [en] Inlier distance threshold.
 * @param distance_threshold_m [zh] 内点距离阈值。
 * @param remaining [en] Points whose distance is at or beyond the threshold.
 * @param remaining [zh] 距离不低于阈值的点。
 * @param inlier_indices [en] Indices of points whose distance is below the threshold.
 * @param inlier_indices [zh] 距离低于阈值的点的索引。
 * @exporter
 */
void split_plane_inliers(const PointCloud& cloud, const Plane& plane, double distance_threshold_m,
                         PointCloud& remaining, std::vector<std::size_t>& inlier_indices);



/**
 * @brief [en] Flips the plane normal so the median signed height of `points` is non-negative.
 * @brief [zh] 翻转平面法向，使 `points` 的有符号高度中位数非负。
 * @param plane [en] Normalized Hessian plane.
 * @param plane [zh] 归一化的 Hessian 平面。
 * @param points [en] Points used to decide the orientation.
 * @param points [zh] 用于确定朝向的点。
 * @return [en] The oriented plane.
 * @return [zh] 定向后的平面。
 * @exporter
 */
Plane orient_plane(const Plane& plane, const PointCloud& points);



/**
 * @brief [en] Voxel downsampling: each occupied voxel keeps the centroid of its points.
 * @brief [zh] 体素降采样：每个被占用的体素保留其点的质心。
 * @param cloud [en] Input point cloud.
 * @param cloud [zh] 输入点云。
 * @param voxel_size [en] Voxel edge length in the cloud's units; a value of 0 or less
 *     returns an empty cloud, so callers should skip the call to keep all points.
 * @param voxel_size [zh] 体素边长（点云单位）；取 0 或更小时返回空点云，
 *     调用方应跳过该调用来保留全部点。
 * @return [en] Downsampled point cloud (one centroid per occupied voxel).
 * @return [zh] 降采样后的点云（每个被占用体素一个质心）。
 * @exporter
 */
PointCloud voxel_downsample(const PointCloud& cloud, double voxel_size);



/**
 * @brief [en] Density-based spatial clustering (DBSCAN) matching Open3D `cluster_dbscan`.
 * @brief [zh] 与 Open3D `cluster_dbscan` 语义一致的密度聚类（DBSCAN）。
 * @param points [en] Input points.
 * @param points [zh] 输入点。
 * @param eps [en] Neighborhood radius in the point-cloud coordinate units.
 * @param eps [zh] 邻域半径（点云坐标单位）。
 * @param min_points [en] Minimum neighborhood size, including the point itself, required to form a cluster.
 * @param min_points [zh] 构成簇所需的最小邻域点数（含自身）。
 * @return [en] Per-point cluster label in input order; -1 marks noise.
 * @return [zh] 按输入顺序返回的逐点簇标签；-1 表示噪声。
 * @exporter
 */
std::vector<int> dbscan_labels(const std::vector<Point3f>& points, double eps, int min_points);



/**
 * @brief [en] Fits a dominant plane from a point cloud with RANSAC.
 * @brief [zh] 用 RANSAC 从点云拟合主平面。
 * @param cloud_m [en] Points in metres.
 * @param cloud_m [zh] 以米为单位的点。
 * @param distance_threshold_m [en] RANSAC inlier distance threshold.
 * @param distance_threshold_m [zh] RANSAC 内点距离阈值。
 * @param iterations [en] RANSAC iteration count.
 * @param iterations [zh] RANSAC 迭代次数。
 * @param out_plane [en] Normalized Hessian plane.
 * @param out_plane [zh] 归一化的 Hessian 平面。
 * @param inlier_indices [en] Inlier point indices into `cloud_m`.
 * @param inlier_indices [zh] 指向 `cloud_m` 的内点索引。
 * @return [en] kSuccess or kPlaneNotFound.
 * @return [zh] kSuccess 或 kPlaneNotFound。
 * @exporter
 */
MeasurementStatus fit_plane_ransac(const PointCloud& cloud_m, double distance_threshold_m, int iterations,
                                   Plane& out_plane, std::vector<std::size_t>& inlier_indices);



/**
 * @brief [en] Removes the dominant background plane(s) from a preprocessed food cloud.
 * @brief [zh] 从预处理食物点云中移除主背景平面（及可选的次级平面）。
 * @exporter
 */
MeasurementStatus remove_dominant_plane(const PointCloud& cloud_m, const MeasurementConfig& cfg, PointCloud& remaining);



/**
 * @brief [en] Computes the axis-aligned bounding-box volume of a point cloud in cubic metres.
 * @brief [zh] 计算点云的轴对齐包围盒体积（立方米）。
 * @param cloud [en] Points in metres.
 * @param cloud [zh] 以米为单位的点。
 * @return [en] AABB volume, or NaN when the cloud is empty.
 * @return [zh] AABB 体积；点云为空时为 NaN。
 * @exporter
 */
double compute_aabb_volume(const PointCloud& cloud);



/**
 * @brief [en] Computes the moment-based oriented bounding-box volume of a point cloud in cubic metres.
 * @brief [zh] 计算点云基于矩的定向包围盒体积（立方米）。
 * @param cloud [en] Points in metres.
 * @param cloud [zh] 以米为单位的点。
 * @return [en] OBB volume, or NaN when the cloud is empty.
 * @return [zh] OBB 体积；点云为空时为 NaN。
 * @exporter
 */
double compute_obb_volume(const PointCloud& cloud);



/**
 * @brief [en] Computes the convex-hull volume of a point cloud in cubic metres.
 * @brief [zh] 计算点云的凸包体积（立方米）。
 * @param cloud [en] Points in metres.
 * @param cloud [zh] 以米为单位的点。
 * @return [en] Convex-hull volume, or NaN when it cannot be computed.
 * @return [zh] 凸包体积；无法计算时为 NaN。
 * @exporter
 */
double compute_convex_hull_volume(const PointCloud& cloud);
