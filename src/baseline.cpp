// Conan::ImportStart
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <map>
#include <memory>
#include <utility>
#include <vector>
#include "baseline.hpp"
#include "grid.hpp"
#include "log.hpp"
#include "pointcloudprocess.hpp"
// Conan::ImportEnd



namespace {

constexpr double kBaselineMinHeightM = -0.003;
constexpr double kTangentEps = 1.0e-8;

double dot3(double ax, double ay, double az, double bx, double by, double bz) { return ax * bx + ay * by + az * bz; }

PlaneFrame make_frame(double nx, double ny, double nz, double ox, double oy, double oz) {
    PlaneFrame f{};
    f.ox = ox;
    f.oy = oy;
    f.oz = oz;
    f.nx = nx;
    f.ny = ny;
    f.nz = nz;

    static const double refs[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    for (const auto& ref : refs) {
        const double t = dot3(ref[0], ref[1], ref[2], nx, ny, nz);
        double tx = ref[0] - t * nx;
        double ty = ref[1] - t * ny;
        double tz = ref[2] - t * nz;
        const double tn = std::sqrt(tx * tx + ty * ty + tz * tz);
        if (tn > kTangentEps) {
            const double ux = tx / tn;
            const double uy = ty / tn;
            const double uz = tz / tn;
            double vx = ny * uz - nz * uy;
            double vy = nz * ux - nx * uz;
            double vz = nx * uy - ny * ux;
            const double vn = std::sqrt(vx * vx + vy * vy + vz * vz);
            if (vn > kTangentEps) {
                vx /= vn;
                vy /= vn;
                vz /= vn;
            }
            f.ux = ux;
            f.uy = uy;
            f.uz = uz;
            f.vx = vx;
            f.vy = vy;
            f.vz = vz;
            return f;
        }
    }
    return f;
}

} // namespace



namespace {

// Plane-frame construction, baseline rasterisation and the plane ROI, used only by
// build_baseline_model below. 平面坐标系构建、基线栅格化与平面 ROI，仅供下方使用。

/**
 * @brief [en] Builds the local orthonormal (u,v,n) frame for a plane and an origin.
 * @brief [zh] 为平面和原点构建局部正交 (u,v,n) 坐标系。
 * @attacher
 */
PlaneFrame build_plane_frame(const Plane& plane, const Point3f& origin) {
    return make_frame(plane.nx, plane.ny, plane.nz, origin.x, origin.y, origin.z);
}



/**
 * @brief [en] Projects baseline frames into a plane frame and rasterizes per-cell median heights.
 * @brief [zh] 把基线帧投影到平面坐标系，并栅格化出逐格高度中位数。
 * @attacher
 */
MeasurementStatus rasterize_baseline(const std::vector<PointCloud>& baseline_frames, const PlaneFrame& frame,
                                     double cell_size_m, double max_surface_height_m, BaselineData& out) {
    out.height_by_cell.clear();
    out.frame = frame;
    out.cell_size_m = cell_size_m;

    std::map<CellKey, std::vector<double>> samples_by_cell;
    for (const auto& frame_cloud : baseline_frames) {
        for (const auto& p : frame_cloud.points) {
            double u = 0.0;
            double v = 0.0;
            double h = 0.0;
            project_to_plane_frame(frame, p, u, v, h);
            if (!std::isfinite(h) || h < kBaselineMinHeightM || h > max_surface_height_m) {
                continue;
            }
            samples_by_cell[cell_index_of(u, v, cell_size_m)].push_back(h);
        }
    }

    if (samples_by_cell.empty()) {
        return MeasurementStatus::kBaselineNoCells;
    }

    bool first_cell = true;
    for (const auto& entry : samples_by_cell) {
        const CellKey key = entry.first;
        const double center_u = (static_cast<double>(key.first) + 0.5) * cell_size_m;
        const double center_v = (static_cast<double>(key.second) + 0.5) * cell_size_m;
        out.height_by_cell[key] = median(entry.second);
        if (first_cell) {
            out.bbox_u_min_m = center_u;
            out.bbox_u_max_m = center_u;
            out.bbox_v_min_m = center_v;
            out.bbox_v_max_m = center_v;
            first_cell = false;
        } else {
            out.bbox_u_min_m = std::min(out.bbox_u_min_m, center_u);
            out.bbox_u_max_m = std::max(out.bbox_u_max_m, center_u);
            out.bbox_v_min_m = std::min(out.bbox_v_min_m, center_v);
            out.bbox_v_max_m = std::max(out.bbox_v_max_m, center_v);
        }
    }
    return MeasurementStatus::kSuccess;
}



/**
 * @brief [en] Computes the inset plane ROI from a baseline footprint and stores it in the baseline.
 * @brief [zh] 从基线足迹计算内缩平面 ROI 并写回基线。
 * @attacher
 */
bool build_plane_roi(BaselineData& baseline, double border_margin_m) {
    if (border_margin_m < 0.0 || baseline.height_by_cell.empty()) {
        return false;
    }

    const double u_min = baseline.bbox_u_min_m + border_margin_m;
    const double u_max = baseline.bbox_u_max_m - border_margin_m;
    //const double v_min = baseline.bbox_v_min_m + border_margin_m;
    //const double v_max = baseline.bbox_v_max_m - border_margin_m;
    const double v_min = baseline.bbox_v_min_m;
    const double v_max = baseline.bbox_v_max_m;
    if (u_min >= u_max || v_min >= v_max) {
        return false;
    }

    baseline.roi.u_min_m = u_min;
    baseline.roi.u_max_m = u_max;
    baseline.roi.v_min_m = v_min;
    baseline.roi.v_max_m = v_max;
    baseline.roi.border_margin_m = border_margin_m;
    return true;
}
} // namespace
/**
 * @brief [en] Builds the empty-oven baseline model from one or more baseline frames.
 * @brief [zh] 从一帧或多帧空炉点云建立空炉基线模型。
 * @attacher
 */
MeasurementStatus build_baseline_model(const std::vector<PointCloud>& baseline_frames,
                                       const PointCloud& orientation_points, const MeasurementConfig& cfg,
                                       BaselineModel& out) {
#ifdef __ARM_EABI__
    (void)baseline_frames;
    (void)orientation_points;
    (void)cfg;
    (void)out;
    return MeasurementStatus::kUnsupportedPlatform;
#else
    out = BaselineModel{};
    auto data = std::make_shared<BaselineData>();

    if (baseline_frames.empty()) {
        return MeasurementStatus::kEmptyBaseline;
    }
    if (!(cfg.integration_resolution_m > 0.0F) || !(cfg.voxel_size_m >= 0.0F) ||
        !(cfg.plane_distance_threshold_m > 0.0F) || volume_defaults::kPlaneRansacIterations <= 0 ||
        !(volume_defaults::kBaselineMaxSurfaceHeightM > 0.0F)) {
        return MeasurementStatus::kInvalidConfig;
    }
    for (const auto& frame : baseline_frames) {
        if (frame.points.empty()) {
            return MeasurementStatus::kEmptyBaseline;
        }
        // A frame is rejected only when it contributes nothing usable at all.
        if (std::none_of(frame.points.begin(), frame.points.end(), [](const Point3f& p) { return is_finite(p); })) {
            return MeasurementStatus::kNonFiniteBaseline;
        }
    }

    // Work in metres and reuse the shared point-cloud operators.
    std::vector<PointCloud> frames_m;
    frames_m.reserve(baseline_frames.size());
    for (const auto& frame : baseline_frames) {
        frames_m.push_back(scale_to_meters(frame, cfg.input_unit));
    }

    // The reference plane comes from the first frame; a zero voxel size skips downsampling.
    const PointCloud first_m =
        (cfg.voxel_size_m > 0.0F) ? voxel_downsample(frames_m[0], cfg.voxel_size_m) : frames_m[0];

    Plane plane{};
    std::vector<std::size_t> inliers;
    MeasurementStatus st = fit_plane_ransac(first_m, cfg.plane_distance_threshold_m,
                                            volume_defaults::kPlaneRansacIterations, plane, inliers);
    if (st != MeasurementStatus::kSuccess) {
        return st;
    }

    // Origin is the mean of the plane inliers.
    double ox = 0.0;
    double oy = 0.0;
    double oz = 0.0;
    for (const std::size_t idx : inliers) {
        ox += first_m.points[idx].x;
        oy += first_m.points[idx].y;
        oz += first_m.points[idx].z;
    }
    const double inv_n = 1.0 / static_cast<double>(inliers.size());
    ox *= inv_n;
    oy *= inv_n;
    oz *= inv_n;

    // Orient the normal toward the food side when orientation points are available.
    plane = orient_plane(plane, orientation_points);

    data->plane = plane;
    data->frame =
        build_plane_frame(plane, Point3f{static_cast<float>(ox), static_cast<float>(oy), static_cast<float>(oz)});
    data->cell_size_m = cfg.integration_resolution_m;

    st = rasterize_baseline(frames_m, data->frame, data->cell_size_m, volume_defaults::kBaselineMaxSurfaceHeightM,
                            *data);
    if (st != MeasurementStatus::kSuccess) {
        return st;
    }

    if (!build_plane_roi(*data, cfg.roi_border_margin_m)) {
        return MeasurementStatus::kInvalidConfig;
    }

    out.frame_count = baseline_frames.size();
    out.cell_count = data->height_by_cell.size();
    out.data = std::move(data);
    log_info("baseline: frames=" + std::to_string(out.frame_count) + " cells=" + std::to_string(out.cell_count));
    return MeasurementStatus::kSuccess;
#endif // __ARM_EABI__
}
