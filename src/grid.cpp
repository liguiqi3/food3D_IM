// Conan::ImportStart
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>
#include "grid.hpp"
// Conan::ImportEnd



double median(std::vector<double> values) {
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const std::size_t n = values.size();
    if (n % 2 == 1) {
        return values[n / 2];
    }
    return 0.5 * (values[n / 2 - 1] + values[n / 2]);
}



void project_to_plane_frame(const PlaneFrame& frame, const Point3f& p, double& u, double& v, double& height) {
    const double dx = static_cast<double>(p.x) - frame.ox;
    const double dy = static_cast<double>(p.y) - frame.oy;
    const double dz = static_cast<double>(p.z) - frame.oz;
    u = dx * frame.ux + dy * frame.uy + dz * frame.uz;
    v = dx * frame.vx + dy * frame.vy + dz * frame.vz;
    height = dx * frame.nx + dy * frame.ny + dz * frame.nz;
}



CellKey cell_index_of(double u, double v, double cell_size_m) {
    return CellKey{static_cast<std::int64_t>(std::floor(u / cell_size_m)),
                   static_cast<std::int64_t>(std::floor(v / cell_size_m))};
}



bool inside_roi(double u, double v, const PlaneRoi& roi) {
    return u >= roi.u_min_m && u <= roi.u_max_m && v >= roi.v_min_m && v <= roi.v_max_m;
}



bool lookup_baseline_height(const BaselineData& baseline, CellKey key, int fill_radius_cells, double& out_height) {
    const auto direct = baseline.height_by_cell.find(key);
    if (direct != baseline.height_by_cell.end()) {
        out_height = direct->second;
        return true;
    }

    for (int radius = 1; radius <= fill_radius_cells; ++radius) {
        std::vector<double> neighbors;
        for (std::int64_t du = -radius; du <= radius; ++du) {
            for (std::int64_t dv = -radius; dv <= radius; ++dv) {
                const auto it = baseline.height_by_cell.find(CellKey{key.first + du, key.second + dv});
                if (it != baseline.height_by_cell.end()) {
                    neighbors.push_back(it->second);
                }
            }
        }
        if (!neighbors.empty()) {
            out_height = median(std::move(neighbors));
            return true;
        }
    }
    return false;
}
