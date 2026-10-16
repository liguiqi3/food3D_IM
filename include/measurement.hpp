// Conan::ImportStart
#pragma once
#include <memory>
#include <string>
#include <vector>
#include "types.hpp"
// Conan::ImportEnd



/**
 * @brief [en] Measures food above an observed empty-tray baseline.
 * @brief [zh] 根据实测空烤盘基线计算食材体积。
 *
 * @details [en] Set one or more empty-tray frames and a food frame captured in the same
 *     coordinate system. `run` removes background points, compares food heights with the
 *     observed tray surface, and returns a `VolumeEstimate`. Setters return `*this` for
 *     chaining. Keep the camera and tray fixed between the empty and food captures.
 * @details [zh] 提供同一坐标系下的一帧或多帧空烤盘点云以及食材点云。`run` 去除背景点，
 *     将食材高度与实测烤盘表面比较，并返回 `VolumeEstimate`。setter 返回自身，可链式调用。
 *     空盘与食材采集之间需保持相机和烤盘位置不变。
 * @exporter
 */
class FoodVolumeMeasurer {
  public:
    FoodVolumeMeasurer() = default;

    // ---------- inputs ----------

    /**
     * @brief [en] Replaces the empty-oven baseline frames (in `input_unit`).
     * @brief [zh] 替换空炉基线帧（单位为 `input_unit`）。
     * @exporter
     */
    FoodVolumeMeasurer& set_baseline(const std::vector<PointCloud>& frames);

    /**
     * @brief [en] Appends one empty-oven baseline frame (in `input_unit`).
     * @brief [zh] 追加一帧空炉基线（单位为 `input_unit`）。
     * @exporter
     */
    FoodVolumeMeasurer& add_baseline_frame(const PointCloud& frame);

    /**
     * @brief [en] Sets the food point cloud to measure (in `input_unit`).
     * @brief [zh] 设置待测食材点云（单位为 `input_unit`）。
     * @exporter
     */
    FoodVolumeMeasurer& set_food(const PointCloud& cloud);

    // ---------- common parameters ----------

    /**
     * @brief [en] Sets the linear unit of all input point clouds.
     * @brief [zh] 设置所有输入点云的线性单位。
     * @exporter
     */
    FoodVolumeMeasurer& set_input_unit(LengthUnit unit);

    /**
     * @brief [en] Sets the voxel size used for downsampling, in metres.
     * @brief [zh] 设置降采样体素边长（米）。
     * @details [en] A value of 0 disables voxel downsampling and keeps every point; a negative
     *     value is rejected as an invalid configuration.
     * @details [zh] 取 0 表示关闭体素降采样并保留全部点；负值会被判为无效配置。
     * @exporter
     */
    FoodVolumeMeasurer& set_voxel_size(double size_m);

    /**
     * @brief [en] Sets the baseline/integration raster cell size, in metres.
     * @brief [zh] 设置基线/积分栅格边长（米）。
     * @exporter
     */
    FoodVolumeMeasurer& set_integration_resolution(double resolution_m);

    /**
     * @brief [en] Sets the accepted baseline-relative height range of food points, in metres.
     * @brief [zh] 设置食材点相对基线高度的接受范围（米）。
     * @param min_m [en] Minimum accepted height; must be non-negative.
     * @param min_m [zh] 最小接受高度，必须非负。
     * @param max_m [en] Maximum accepted height; values <= 0 disable the cap.
     * @param max_m [zh] 最大接受高度；<= 0 表示关闭上限。
     * @exporter
     */
    FoodVolumeMeasurer& set_height_range(double min_m, double max_m);

    /**
     * @brief [en] Enables an axis-aligned crop region applied before downsampling.
     * @brief [zh] 启用降采样前应用的轴对齐裁剪区域。
     * @param roi [en] Bounds in metres, even when input points use millimetres.
     * @param roi [zh] 边界以米表示，即使输入点云使用毫米。
     * @exporter
     */
    FoodVolumeMeasurer& set_roi(const AxisAlignedRoi& roi);

    /**
     * @brief [en] Disables the axis-aligned crop region.
     * @brief [zh] 禁用轴对齐裁剪区域。
     * @exporter
     */
    FoodVolumeMeasurer& clear_roi();

    /**
     * @brief [en] Selects component labels; an empty list selects all eligible components.
     * @brief [zh] 选择连通块标签；空列表选择所有符合条件的连通块。
     * @exporter
     */
    FoodVolumeMeasurer& set_selected_labels(const std::vector<int>& labels);

    /**
     * @brief [en] Sets the footprint clustering radius and minimum cluster size.
     * @brief [zh] 设置足迹聚类半径与最小簇规模。
     * @param footprint_eps_m [en] DBSCAN neighbourhood radius in the baseline plane, in metres.
     * @param footprint_eps_m [zh] 基准面内 DBSCAN 邻域半径（米）。
     * @param min_points [en] Minimum cluster size including the point itself; must be positive.
     * @param min_points [zh] 构成簇所需的最小邻域点数（含自身），必须为正。
     * @exporter
     */
    FoodVolumeMeasurer& set_cluster_params(double footprint_eps_m, int min_points);

    /**
     * @brief [en] Sets the RANSAC inlier distance threshold for background-plane removal.
     * @brief [zh] 设置背景平面移除的 RANSAC 内点距离阈值。
     * @exporter
     */
    FoodVolumeMeasurer& set_plane_distance_threshold(double threshold_m);

    /**
     * @brief [en] Sets the intermediate point-cloud directory; an empty path disables the dump.
     * @brief [zh] 设置中间点云目录；空路径禁用保存。
     * @details [en] A successful `run` writes stage PCD files. `5_top_surface.pcd` contains
     *     measured cells; `6_hole_filled_surface.pcd` also contains accepted filled cells.
     *     Files use world coordinates.
     * @details [zh] 成功运行后写出各阶段 PCD。`5_top_surface.pcd` 为实测栅格，
     *     `6_hole_filled_surface.pcd` 还包含通过检查的补洞栅格。文件使用世界坐标。
     * @param dir [en] Directory path, created when missing, or empty to disable.
     * @param dir [zh] 目录路径，不存在时自动创建；空路径表示禁用。
     * @exporter
     */
    FoodVolumeMeasurer& set_middle_cloud_dir(const std::string& dir);

    // ---------- full configuration ----------

    /**
     * @brief [en] Replaces the full measurement configuration, including advanced fields.
     * @brief [zh] 替换完整测量配置（含高级字段）。
     * @exporter
     */
    FoodVolumeMeasurer& set_config(const MeasurementConfig& cfg);

    /**
     * @brief [en] Returns the current measurement configuration.
     * @brief [zh] 返回当前测量配置。
     * @exporter
     */
    const MeasurementConfig& config() const;

    /**
     * @brief [en] Loads the configuration from a JSON file, overriding stored values.
     * @brief [zh] 从 JSON 文件载入配置并覆盖已存值。
     * @details [en] Missing keys retain their current values; unknown and removed keys fail
     *     the load. Check the Boolean return value before running a measurement.
     * @details [zh] 文件中未出现的字段保留原值；未知或已删除的键使加载失败。
     *     测量前应检查返回的布尔值。
     * @exporter
     */
    bool load_config_from_json(const std::string& path);

    /**
     * @brief [en] Saves the current configuration to a JSON file.
     * @brief [zh] 将当前配置保存到 JSON 文件。
     * @exporter
     */
    bool save_config_to_json(const std::string& path) const;

    // ---------- execution ----------

    /**
     * @brief [en] Builds and caches the empty-oven baseline model so repeated `run` calls skip it.
     * @brief [zh] 构建并缓存空炉基线模型，使后续 `run` 调用无需重复构建。
     *
     * @details [en] Call `set_food` first: its points orient the fitted plane normal during
     *     preparation. After preparation, `set_food` can replace the food frame without
     *     discarding the baseline. Keep later frames on the same side of the tray. Changing
     *     baseline frames or baseline-related parameters discards the cache. `run` also works
     *     without preparation and builds the baseline as needed.
     * @details [zh] 准备前先调用 `set_food`：该帧用于确定拟合平面的法向朝向。准备完成后，
     *     `set_food` 可替换食材帧而不丢弃基线；后续食材应位于烤盘同侧。修改空盘帧或基线相关参数
     *     会使缓存失效。未显式准备时，`run` 也会按需构建基线。
     * @return [en] kSuccess, or the first failing stage status.
     * @return [zh] kSuccess，或首个失败阶段的状态。
     * @exporter
     */
    MeasurementStatus prepare_baseline();

    /**
     * @brief [en] Returns whether a prepared baseline model is currently cached.
     * @brief [zh] 返回当前是否已缓存了准备好的基线模型。
     * @exporter
     */
    bool has_prepared_baseline() const;

    /**
     * @brief [en] Runs the IM pipeline and returns the volume estimate.
     * @brief [zh] 运行 IM 流水线并返回体积估计。
     * @details [en] Inspect `status` before using volume fields. An unsuccessful run returns
     *     the first failing stage's status and leaves numeric estimates as NaN.
     * @details [zh] 使用体积字段前先检查 `status`。失败时返回首个失败阶段的状态，
     *     数值估计保持 NaN。
     * @return [en] A VolumeEstimate whose status indicates success or the first failing stage.
     * @return [zh] 一个 VolumeEstimate，其状态指示成功或第一个失败阶段。
     * @exporter
     */
    VolumeEstimate run() const;

  private:
    // Opaque cache holding the prebuilt baseline model; defined in the implementation.
    struct PreparedBaseline;

    // Drops the cached baseline; called by every setter that shapes the baseline inputs.
    void invalidate_prepared_baseline();

    MeasurementConfig cfg_;
    std::vector<PointCloud> baseline_frames_;
    PointCloud food_;
    std::shared_ptr<PreparedBaseline> prepared_baseline_;
};



/**
 * @brief [en] Loads a PCD point-cloud file into the library's point model.
 * @brief [zh] 将 PCD 点云文件载入到库的点模型。
 * @param path [en] Path to the PCD file.
 * @param path [zh] PCD 文件路径。
 * @return [en] The loaded points, or an empty cloud when the file cannot be read.
 * @return [zh] 载入的点；文件无法读取时返回空点云。
 * @exporter
 */
PointCloud load_pcd(const std::string& path);
