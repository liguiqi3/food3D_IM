// Conan::ImportStart
#pragma once
#include <cstddef>
#include <cstdint>
#include <map>
#include <utility>
#include <vector>
#include "types.hpp"
// Conan::ImportEnd



/**
 * @brief [en] A stable integer cell index in the baseline plane frame.
 * @brief [zh] 基准面局部坐标系中的稳定整数栅格索引。

 */
using CellKey = std::pair<std::int64_t, std::int64_t>;



/**
 * @brief [en] A local orthonormal frame on the baseline plane.
 * @brief [zh] 基准面上的局部正交坐标系。

 */
struct PlaneFrame {
    double ox = 0.0;
    double oy = 0.0;
    double oz = 0.0;
    double ux = 1.0;
    double uy = 0.0;
    double uz = 0.0;
    double vx = 0.0;
    double vy = 1.0;
    double vz = 0.0;
    double nx = 0.0;
    double ny = 0.0;
    double nz = 1.0;
};



/**
 * @brief [en] The inset valid region inside the baseline footprint bounding box.
 * @brief [zh] 空炉足迹外接框内缩后的有效区域。

 */
struct PlaneRoi {
    double u_min_m = 0.0;
    double u_max_m = 0.0;
    double v_min_m = 0.0;
    double v_max_m = 0.0;
    double border_margin_m = 0.0;
};



/**
 * @brief [en] Storage of the empty-oven baseline: frame, ROI and per-cell median heights.
 * @brief [zh] 空炉基线的存储：坐标系、ROI 与逐格高度中位数。

 */
struct BaselineData {
    Plane plane;
    PlaneFrame frame;
    PlaneRoi roi;
    double cell_size_m = 0.0;
    std::map<CellKey, double> height_by_cell;
    double bbox_u_min_m = 0.0;
    double bbox_u_max_m = 0.0;
    double bbox_v_min_m = 0.0;
    double bbox_v_max_m = 0.0;
};



/**
 * @brief [en] The baseline-difference height grid plus derived volume statistics.
 * @brief [zh] 基线差分高度栅格及其派生的体积统计。

 */
struct HeightGrid {
    std::vector<CellKey> cells;
    std::vector<double> baseline_heights_m;
    std::vector<double> heights_m;
    std::vector<std::int8_t> is_interpolated;
    std::vector<int> component_labels;

    double cell_size_m = 0.0;
    PlaneFrame frame;

    std::size_t measured_cells = 0;
    std::size_t interpolated_cells = 0;
    std::size_t occupied_cells = 0;
    std::size_t bbox_cells = 0;
    std::size_t missing_baseline_cells = 0;

    /// [en] Cells dropped for a missing baseline entry, attributed to their component label.
    /// [zh] 因基线缺失而被丢弃的格子，按连通块标签归因。
    std::map<int, std::size_t> missing_baseline_by_label;

    double raw_volume_m3 = 0.0;
    double interpolated_volume_m3 = 0.0;
    double volume_m3 = 0.0;
    double footprint_area_m2 = 0.0;
    double coverage_ratio = 0.0;
    double mean_height_m = 0.0;
    double max_height_m = 0.0;
};



/**
 * @brief [en] Statistics of the component-aware hole-filling stage.
 * @brief [zh] 按连通块补洞的统计信息。

 */
struct HoleFillStats {
    std::size_t candidate_hole_count = 0;
    std::size_t filled_hole_count = 0;
    std::size_t filled_cell_count = 0;
    std::size_t small_filled_hole_count = 0;
    std::size_t curve_filled_hole_count = 0;
    std::size_t unfilled_hole_cells = 0;
    double max_component_inferred_ratio = 0.0;

    /// [en] Cells left unfilled, attributed to their component label.
    /// [zh] 未能补全的格子，按连通块标签归因。
    std::map<int, std::size_t> unfilled_by_label;
};



/**
 * @brief [en] Per-cell top surface of the selected components, with each cell's component label.
 * @brief [zh] 选中食材块的逐格顶表面，以及每个格子的连通块标签。

 */
struct SurfaceMap {
    std::vector<CellKey> cells;
    std::vector<double> heights_m;
    std::vector<int> labels;
};



/**
 * @brief [en] Minimum food-relative height accepted when projecting points onto the baseline plane.
 * @brief [zh] 投影到基准面时接受的最小食材相对高度。
 */
constexpr double kFoodMinHeightM = -0.003;



/**
 * @brief [en] Robust median of a value vector.
 * @brief [zh] 数值向量的稳健中位数。

 */
double median(std::vector<double> values);



/**
 * @brief [en] Projects a point into the baseline plane frame: (u, v, height) in metres.
 * @brief [zh] 把点投影到基准面局部坐标系：(u,v,height)，单位为米。

 */
void project_to_plane_frame(const PlaneFrame& frame, const Point3f& p, double& u, double& v, double& height);



/**
 * @brief [en] Converts continuous plane coordinates to integer cell indices by floor.
 * @brief [zh] 用向下取整把连续平面坐标映射为整数栅格索引。

 */
CellKey cell_index_of(double u, double v, double cell_size_m);



/**
 * @brief [en] Tests whether continuous plane coordinates lie inside a plane ROI.
 * @brief [zh] 判断连续平面坐标是否位于平面 ROI 内。

 */
bool inside_roi(double u, double v, const PlaneRoi& roi);



/**
 * @brief [en] Looks up the baseline height for a cell, filling missing cells from bounded Chebyshev neighbours.
 * @brief [zh] 查询某格基线高度，缺失时在有限切比雪夫半径内取邻居中位数补齐。

 */
bool lookup_baseline_height(const BaselineData& baseline, CellKey key, int fill_radius_cells, double& out_height);



// PCD I/O backends, shared by the public `load_pcd` wrapper and the intermediate-cloud debug
// dump. They are PCL-backed and therefore unavailable on bare metal; the definitions live in
// pointcloudprocess.cpp and the only caller is measurement.cpp.
// 公开 `load_pcd` 与中间点云落盘共用的 PCD I/O 后端。依赖 PCL，故裸机不可用；
// 定义在 pointcloudprocess.cpp，唯一调用方是 measurement.cpp。

/**
 * @brief [en] Loads a PCD file into the library's point model.
 * @brief [zh] 将 PCD 文件载入到库的点模型。
 */
PointCloud load_pcd_impl(const std::string& path);



/**
 * @brief [en] Writes a cloud to a binary PCD file.
 * @brief [zh] 将点云写出为二进制 PCD 文件。
 */
bool save_pcd_impl(const std::string& path, const PointCloud& cloud);
