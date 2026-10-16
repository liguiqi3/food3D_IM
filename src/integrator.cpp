// Conan::ImportStart
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <set>
#include <utility>
#include <vector>
#include "integrator.hpp"
#include "grid.hpp"
#include "log.hpp"
#ifndef __ARM_EABI__
#include <Eigen/Dense>
#endif
// Conan::ImportEnd



#ifndef __ARM_EABI__

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kM3ToCm3 = 1.0e6;

// Finds enclosed 4-connected empty regions of `component_cells` when `blocked_cells`
// are treated as occupied walls. Mirrors the Python exterior-flood-fill approach.
std::vector<std::vector<CellKey>> find_enclosed_holes(const std::vector<CellKey>& component_cells,
                                                      const std::vector<CellKey>& blocked_cells) {
    if (component_cells.empty()) {
        return {};
    }

    // The search box is sized from the component's own cells plus a one-cell padding. Blocked cells
    // are walls inside it but must not enlarge it; anything outside the box cannot enclose a hole.
    std::int64_t lo_i = component_cells.front().first;
    std::int64_t hi_i = component_cells.front().first;
    std::int64_t lo_j = component_cells.front().second;
    std::int64_t hi_j = component_cells.front().second;
    const auto expand = [&](const CellKey& key) {
        lo_i = std::min(lo_i, key.first);
        hi_i = std::max(hi_i, key.first);
        lo_j = std::min(lo_j, key.second);
        hi_j = std::max(hi_j, key.second);
    };
    for (const auto& key : component_cells) {
        expand(key);
    }

    lo_i -= 1;
    hi_i += 1;
    lo_j -= 1;
    hi_j += 1;
    const std::int64_t rows = hi_i - lo_i + 1;
    const std::int64_t cols = hi_j - lo_j + 1;

    std::vector<std::vector<bool>> occupied(static_cast<std::size_t>(rows),
                                            std::vector<bool>(static_cast<std::size_t>(cols), false));
    const auto mark = [&](const CellKey& key) {
        if (key.first < lo_i || key.first > hi_i || key.second < lo_j || key.second > hi_j) {
            return;
        }
        occupied[static_cast<std::size_t>(key.first - lo_i)][static_cast<std::size_t>(key.second - lo_j)] = true;
    };
    for (const auto& key : component_cells) {
        mark(key);
    }
    for (const auto& key : blocked_cells) {
        mark(key);
    }

    std::vector<std::vector<bool>> exterior(static_cast<std::size_t>(rows),
                                            std::vector<bool>(static_cast<std::size_t>(cols), false));
    std::deque<CellKey> pending;
    const auto seed_border = [&](std::int64_t r, std::int64_t c) {
        if (!occupied[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)] &&
            !exterior[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)]) {
            exterior[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)] = true;
            pending.push_back(CellKey{r, c});
        }
    };
    for (std::int64_t r = 0; r < rows; ++r) {
        seed_border(r, 0);
        seed_border(r, cols - 1);
    }
    for (std::int64_t c = 0; c < cols; ++c) {
        seed_border(0, c);
        seed_border(rows - 1, c);
    }

    const std::int64_t dr[4] = {-1, 1, 0, 0};
    const std::int64_t dc[4] = {0, 0, -1, 1};
    while (!pending.empty()) {
        const CellKey cur = pending.front();
        pending.pop_front();
        for (int k = 0; k < 4; ++k) {
            const std::int64_t nr = cur.first + dr[k];
            const std::int64_t nc = cur.second + dc[k];
            if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) {
                continue;
            }
            if (!occupied[static_cast<std::size_t>(nr)][static_cast<std::size_t>(nc)] &&
                !exterior[static_cast<std::size_t>(nr)][static_cast<std::size_t>(nc)]) {
                exterior[static_cast<std::size_t>(nr)][static_cast<std::size_t>(nc)] = true;
                pending.push_back(CellKey{nr, nc});
            }
        }
    }

    std::vector<std::vector<CellKey>> holes;
    for (std::int64_t r = 0; r < rows; ++r) {
        for (std::int64_t c = 0; c < cols; ++c) {
            if (occupied[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)] ||
                exterior[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)]) {
                continue;
            }
            std::vector<CellKey> hole;
            std::deque<CellKey> queue;
            queue.push_back(CellKey{r, c});
            occupied[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)] = true;
            while (!queue.empty()) {
                const CellKey cur = queue.front();
                queue.pop_front();
                hole.push_back(CellKey{cur.first + lo_i, cur.second + lo_j});
                for (int k = 0; k < 4; ++k) {
                    const std::int64_t nr = cur.first + dr[k];
                    const std::int64_t nc = cur.second + dc[k];
                    if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) {
                        continue;
                    }
                    if (!occupied[static_cast<std::size_t>(nr)][static_cast<std::size_t>(nc)] &&
                        !exterior[static_cast<std::size_t>(nr)][static_cast<std::size_t>(nc)]) {
                        occupied[static_cast<std::size_t>(nr)][static_cast<std::size_t>(nc)] = true;
                        queue.push_back(CellKey{nr, nc});
                    }
                }
            }
            holes.push_back(std::move(hole));
        }
    }
    return holes;
}

std::set<int> hole_boundary_component_labels(const std::vector<CellKey>& hole,
                                             const std::map<CellKey, int>& label_by_cell) {
    std::set<CellKey> hole_cells(hole.begin(), hole.end());
    std::set<int> labels;
    for (const auto& cell : hole) {
        for (std::int64_t du = -1; du <= 1; ++du) {
            for (std::int64_t dv = -1; dv <= 1; ++dv) {
                if (du == 0 && dv == 0) {
                    continue;
                }
                const CellKey candidate{cell.first + du, cell.second + dv};
                if (hole_cells.find(candidate) != hole_cells.end()) {
                    continue;
                }
                const auto it = label_by_cell.find(candidate);
                if (it != label_by_cell.end()) {
                    labels.insert(it->second);
                }
            }
        }
    }
    return labels;
}

std::vector<CellKey> collect_rim_cells(const std::vector<CellKey>& hole, const std::map<CellKey, int>& label_by_cell,
                                       int component_label, int radius_cells) {
    std::set<CellKey> hole_cells(hole.begin(), hole.end());
    std::set<CellKey> rim;
    for (const auto& cell : hole) {
        for (std::int64_t du = -radius_cells; du <= radius_cells; ++du) {
            for (std::int64_t dv = -radius_cells; dv <= radius_cells; ++dv) {
                if (du == 0 && dv == 0) {
                    continue;
                }
                const CellKey candidate{cell.first + du, cell.second + dv};
                if (hole_cells.find(candidate) != hole_cells.end()) {
                    continue;
                }
                const auto it = label_by_cell.find(candidate);
                if (it != label_by_cell.end() && it->second == component_label) {
                    rim.insert(candidate);
                }
            }
        }
    }
    return std::vector<CellKey>(rim.begin(), rim.end());
}

struct SmallAddition {
    CellKey cell;
    double baseline_height_m;
    double height_m;
};

bool interpolate_small_hole(const std::vector<CellKey>& hole, const std::map<CellKey, double>& height_by_cell,
                            const std::map<CellKey, int>& label_by_cell, int component_label, int neighbor_radius_cells,
                            double max_neighbor_height_delta_m, const BaselineData& baseline,
                            int baseline_fill_radius_cells, std::vector<SmallAddition>& additions) {
    additions.clear();
    for (const auto& cell : hole) {
        std::vector<std::pair<double, double>> samples; // (height, weight)
        for (std::int64_t du = -neighbor_radius_cells; du <= neighbor_radius_cells; ++du) {
            for (std::int64_t dv = -neighbor_radius_cells; dv <= neighbor_radius_cells; ++dv) {
                if (du == 0 && dv == 0) {
                    continue;
                }
                const CellKey candidate{cell.first + du, cell.second + dv};
                const auto lit = label_by_cell.find(candidate);
                if (lit == label_by_cell.end() || lit->second != component_label) {
                    continue;
                }
                const auto hit = height_by_cell.find(candidate);
                if (hit == height_by_cell.end()) {
                    continue;
                }
                const double distance = std::sqrt(static_cast<double>(du * du + dv * dv));
                samples.emplace_back(hit->second, 1.0 / std::max(distance, 1.0e-6));
            }
        }
        if (samples.size() < 3) {
            return false;
        }
        double min_h = samples.front().first;
        double max_h = samples.front().first;
        double weighted = 0.0;
        double weight_sum = 0.0;
        for (const auto& sample : samples) {
            min_h = std::min(min_h, sample.first);
            max_h = std::max(max_h, sample.first);
            weighted += sample.first * sample.second;
            weight_sum += sample.second;
        }
        if (max_h - min_h > max_neighbor_height_delta_m) {
            return false;
        }

        double baseline_height = 0.0;
        if (!lookup_baseline_height(baseline, cell, baseline_fill_radius_cells, baseline_height)) {
            return false;
        }
        additions.push_back(SmallAddition{cell, baseline_height, weighted / weight_sum});
    }
    return true;
}

bool fit_quadratic_hole(const std::vector<CellKey>& hole, const std::map<CellKey, double>& height_by_cell,
                        const std::map<CellKey, int>& label_by_cell, int component_label, double cell_size_m,
                        int rim_radius_cells, int min_rim_samples, double min_rim_coverage, double max_fit_rmse_m,
                        double max_prediction_rise_m, double min_height_m, bool has_max_height, double max_height_m,
                        std::vector<double>& predictions) {
    predictions.clear();

    const std::vector<CellKey> rim = collect_rim_cells(hole, label_by_cell, component_label, rim_radius_cells);
    if (static_cast<int>(rim.size()) < min_rim_samples) {
        return false;
    }

    double hole_center_u = 0.0;
    double hole_center_v = 0.0;
    for (const auto& cell : hole) {
        hole_center_u += (static_cast<double>(cell.first) + 0.5) * cell_size_m;
        hole_center_v += (static_cast<double>(cell.second) + 0.5) * cell_size_m;
    }
    hole_center_u /= static_cast<double>(hole.size());
    hole_center_v /= static_cast<double>(hole.size());

    const std::size_t n = rim.size();
    Eigen::MatrixXd design(static_cast<Eigen::Index>(n), 6);
    Eigen::VectorXd observed(static_cast<Eigen::Index>(n));
    std::vector<double> angles;
    angles.reserve(n);
    for (std::size_t idx = 0; idx < n; ++idx) {
        const double ru = (static_cast<double>(rim[idx].first) + 0.5) * cell_size_m;
        const double rv = (static_cast<double>(rim[idx].second) + 0.5) * cell_size_m;
        const double x = ru - hole_center_u;
        const double y = rv - hole_center_v;
        design(static_cast<Eigen::Index>(idx), 0) = 1.0;
        design(static_cast<Eigen::Index>(idx), 1) = x;
        design(static_cast<Eigen::Index>(idx), 2) = y;
        design(static_cast<Eigen::Index>(idx), 3) = x * x;
        design(static_cast<Eigen::Index>(idx), 4) = x * y;
        design(static_cast<Eigen::Index>(idx), 5) = y * y;
        observed(static_cast<Eigen::Index>(idx)) = height_by_cell.at(rim[idx]);
        angles.push_back(std::atan2(y, x));
    }

    std::set<int> occupied_bins;
    for (const double angle : angles) {
        const double shifted = (angle + kPi) / (2.0 * kPi) * 8.0;
        const int bin = static_cast<int>(std::floor(shifted)) % 8;
        if (bin < 0) {
            occupied_bins.insert(bin + 8);
        } else {
            occupied_bins.insert(bin);
        }
    }
    if (static_cast<double>(occupied_bins.size()) / 8.0 < min_rim_coverage) {
        return false;
    }

    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(design);
    Eigen::VectorXd coefficients = qr.solve(observed);
    if (qr.rank() < 6) {
        return false;
    }

    Eigen::VectorXd residuals = observed - design * coefficients;
    std::vector<double> residual_vec(residuals.data(), residuals.data() + n);
    const double median_residual = median(residual_vec);

    std::vector<double> abs_dev;
    abs_dev.reserve(n);
    for (const double r : residual_vec) {
        abs_dev.push_back(std::fabs(r - median_residual));
    }
    const double mad = median(std::move(abs_dev));
    const double robust_limit = std::max(0.0015, 3.0 * 1.4826 * mad);

    std::vector<bool> inlier_mask(n, false);
    std::size_t inlier_count = 0;
    for (std::size_t idx = 0; idx < n; ++idx) {
        if (std::fabs(residual_vec[idx] - median_residual) <= robust_limit) {
            inlier_mask[idx] = true;
            ++inlier_count;
        }
    }

    if (inlier_count >= static_cast<std::size_t>(min_rim_samples) && inlier_count < n) {
        Eigen::MatrixXd sub(static_cast<Eigen::Index>(inlier_count), 6);
        Eigen::VectorXd sub_obs(static_cast<Eigen::Index>(inlier_count));
        std::size_t row = 0;
        for (std::size_t idx = 0; idx < n; ++idx) {
            if (inlier_mask[idx]) {
                sub.row(static_cast<Eigen::Index>(row)) = design.row(static_cast<Eigen::Index>(idx));
                sub_obs(static_cast<Eigen::Index>(row)) = observed(static_cast<Eigen::Index>(idx));
                ++row;
            }
        }
        Eigen::ColPivHouseholderQR<Eigen::MatrixXd> sub_qr(sub);
        coefficients = sub_qr.solve(sub_obs);
        if (sub_qr.rank() < 6) {
            return false;
        }
        residuals = sub_obs - sub * coefficients;
    }

    const double rmse = std::sqrt(residuals.squaredNorm() / static_cast<double>(residuals.size()));
    if (rmse > max_fit_rmse_m) {
        return false;
    }

    double obs_min = observed(0);
    double obs_max = observed(0);
    for (std::size_t idx = 1; idx < n; ++idx) {
        obs_min = std::min(obs_min, observed(static_cast<Eigen::Index>(idx)));
        obs_max = std::max(obs_max, observed(static_cast<Eigen::Index>(idx)));
    }

    const double lower = std::max(0.0, obs_min - max_prediction_rise_m);
    const double upper = obs_max + max_prediction_rise_m;

    predictions.reserve(hole.size());
    for (const auto& cell : hole) {
        const double hu = (static_cast<double>(cell.first) + 0.5) * cell_size_m;
        const double hv = (static_cast<double>(cell.second) + 0.5) * cell_size_m;
        const double x = hu - hole_center_u;
        const double y = hv - hole_center_v;
        const double pred = coefficients(0) + coefficients(1) * x + coefficients(2) * y + coefficients(3) * x * x +
                            coefficients(4) * x * y + coefficients(5) * y * y;
        if (pred < lower || pred > upper) {
            return false;
        }
        if (pred < min_height_m) {
            return false;
        }
        if (has_max_height && pred > max_height_m) {
            return false;
        }
        predictions.push_back(pred);
    }
    return true;
}

void recompute_grid_stats(HeightGrid& grid) {
    const double cell_area = grid.cell_size_m * grid.cell_size_m;
    double raw = 0.0;
    double interp = 0.0;
    double sum = 0.0;
    double max_h = 0.0;
    std::size_t measured = 0;
    std::size_t interpolated = 0;

    std::int64_t min_i = 0;
    std::int64_t max_i = 0;
    std::int64_t min_j = 0;
    std::int64_t max_j = 0;
    bool first = true;
    for (std::size_t idx = 0; idx < grid.cells.size(); ++idx) {
        const double h = grid.heights_m[idx];
        sum += h;
        max_h = std::max(max_h, h);
        if (grid.is_interpolated[idx]) {
            interp += h;
            ++interpolated;
        } else {
            raw += h;
            ++measured;
        }
        if (first) {
            min_i = grid.cells[idx].first;
            max_i = grid.cells[idx].first;
            min_j = grid.cells[idx].second;
            max_j = grid.cells[idx].second;
            first = false;
        } else {
            min_i = std::min(min_i, grid.cells[idx].first);
            max_i = std::max(max_i, grid.cells[idx].first);
            min_j = std::min(min_j, grid.cells[idx].second);
            max_j = std::max(max_j, grid.cells[idx].second);
        }
    }

    grid.raw_volume_m3 = raw * cell_area;
    grid.interpolated_volume_m3 = interp * cell_area;
    grid.volume_m3 = grid.raw_volume_m3 + grid.interpolated_volume_m3;
    grid.measured_cells = measured;
    grid.interpolated_cells = interpolated;
    grid.occupied_cells = grid.cells.size();
    grid.bbox_cells = static_cast<std::size_t>((max_i - min_i + 1) * (max_j - min_j + 1));
    grid.footprint_area_m2 = static_cast<double>(grid.occupied_cells) * cell_area;
    grid.coverage_ratio = grid.bbox_cells > 0 ? static_cast<double>(grid.occupied_cells) / grid.bbox_cells : 0.0;
    grid.mean_height_m = grid.cells.empty() ? 0.0 : sum / static_cast<double>(grid.cells.size());
    grid.max_height_m = max_h;
}

// Builds per-cell top-surface heights and component labels from selected component clouds.
void build_surface_maps(const FoodComponents& components, const BaselineData& baseline,
                        std::map<CellKey, double>& surface_by_cell, std::map<CellKey, int>& label_by_cell) {
    surface_by_cell.clear();
    label_by_cell.clear();

    for (std::size_t c = 0; c < components.labels.size(); ++c) {
        const int label = components.labels[c];
        std::map<CellKey, double> component_surface;
        for (const auto& p : components.clouds[c].points) {
            double u = 0.0;
            double v = 0.0;
            double h = 0.0;
            project_to_plane_frame(baseline.frame, p, u, v, h);
            if (!std::isfinite(h) || h < kFoodMinHeightM) {
                continue;
            }
            const CellKey key = cell_index_of(u, v, baseline.cell_size_m);
            const double center_u = (static_cast<double>(key.first) + 0.5) * baseline.cell_size_m;
            const double center_v = (static_cast<double>(key.second) + 0.5) * baseline.cell_size_m;
            if (!inside_roi(center_u, center_v, baseline.roi)) {
                continue;
            }
            const auto it = component_surface.find(key);
            if (it == component_surface.end() || h > it->second) {
                component_surface[key] = h;
            }
        }
        for (const auto& entry : component_surface) {
            const auto it = surface_by_cell.find(entry.first);
            if (it == surface_by_cell.end() || entry.second > it->second) {
                surface_by_cell[entry.first] = entry.second;
                label_by_cell[entry.first] = label;
            }
        }
    }
}

// Builds the baseline-difference height grid from a food top-surface map.
MeasurementStatus build_height_grid(const std::map<CellKey, double>& surface_by_cell,
                                    const std::map<CellKey, int>& label_by_cell, const BaselineData& baseline,
                                    const MeasurementConfig& cfg, HeightGrid& out) {
    out = HeightGrid{};

    if (!(baseline.cell_size_m > 0.0) || !(cfg.min_height_m >= 0.0F)) {
        return MeasurementStatus::kInvalidConfig;
    }

    out.frame = baseline.frame;
    out.cell_size_m = baseline.cell_size_m;

    std::int64_t min_i = 0;
    std::int64_t max_i = 0;
    std::int64_t min_j = 0;
    std::int64_t max_j = 0;
    bool first = true;

    for (const auto& entry : surface_by_cell) {
        const CellKey key = entry.first;
        const double food_height = entry.second;

        double baseline_height = 0.0;
        if (!lookup_baseline_height(baseline, key, volume_defaults::kBaselineFillRadiusCells, baseline_height)) {
            ++out.missing_baseline_cells;
            const auto missing_label = label_by_cell.find(key);
            if (missing_label != label_by_cell.end()) {
                ++out.missing_baseline_by_label[missing_label->second];
            }
            continue;
        }

        double diff = food_height - baseline_height;
        if (diff < cfg.min_height_m) {
            continue;
        }
        if (cfg.max_height_m > 0.0F && diff > cfg.max_height_m) {
            diff = cfg.max_height_m;
        }

        out.cells.push_back(key);
        out.baseline_heights_m.push_back(baseline_height);
        out.heights_m.push_back(diff);
        out.is_interpolated.push_back(0);
        const auto label = label_by_cell.find(key);
        out.component_labels.push_back(label != label_by_cell.end() ? label->second : 0);

        if (first) {
            min_i = key.first;
            max_i = key.first;
            min_j = key.second;
            max_j = key.second;
            first = false;
        } else {
            min_i = std::min(min_i, key.first);
            max_i = std::max(max_i, key.first);
            min_j = std::min(min_j, key.second);
            max_j = std::max(max_j, key.second);
        }
    }

    if (out.cells.empty()) {
        return MeasurementStatus::kInsufficientCoverage;
    }

    const double cell_area = out.cell_size_m * out.cell_size_m;
    double sum = 0.0;
    double max_h = 0.0;
    for (const double h : out.heights_m) {
        sum += h;
        max_h = std::max(max_h, h);
    }

    out.bbox_cells = static_cast<std::size_t>((max_i - min_i + 1) * (max_j - min_j + 1));
    out.occupied_cells = out.cells.size();
    out.measured_cells = out.cells.size();
    out.interpolated_cells = 0;
    out.raw_volume_m3 = sum * cell_area;
    out.interpolated_volume_m3 = 0.0;
    out.volume_m3 = out.raw_volume_m3;
    out.footprint_area_m2 = static_cast<double>(out.occupied_cells) * cell_area;
    out.coverage_ratio = out.bbox_cells > 0 ? static_cast<double>(out.occupied_cells) / out.bbox_cells : 0.0;
    out.mean_height_m = sum / static_cast<double>(out.cells.size());
    out.max_height_m = max_h;
    return MeasurementStatus::kSuccess;
}

// Completes enclosed holes conservatively inside single food components.
MeasurementStatus complete_component_aware_holes(HeightGrid& grid, const BaselineData& baseline,
                                                 const MeasurementConfig& cfg, HoleFillStats& stats) {
    stats = HoleFillStats{};

    // Only the knobs that are still configurable need checking; the retired ones are
    // compile-time constants in volume_defaults and are already known to be usable.
    if (cfg.hole_fill_max_cells < 0 || !(cfg.curve_fill_max_hole_area_cm2 >= 0.0F)) {
        return MeasurementStatus::kInvalidConfig;
    }

    const bool has_max_height = cfg.max_height_m > 0.0F;
    const double cell_area = grid.cell_size_m * grid.cell_size_m;

    std::map<CellKey, double> height_by_cell;
    std::map<CellKey, int> label_by_cell;
    for (std::size_t idx = 0; idx < grid.cells.size(); ++idx) {
        height_by_cell[grid.cells[idx]] = grid.heights_m[idx];
        label_by_cell[grid.cells[idx]] = grid.component_labels[idx];
    }

    std::set<int> component_labels;
    for (const int label : grid.component_labels) {
        component_labels.insert(label);
    }

    std::vector<SmallAddition> additions;
    std::vector<int> addition_labels;
    std::set<CellKey> unfilled;
    double max_component_inferred_ratio = 0.0;

    for (const int component_label : component_labels) {
        std::vector<CellKey> component_cells;
        std::vector<CellKey> blocked_cells;
        component_cells.reserve(grid.cells.size());
        blocked_cells.reserve(grid.cells.size());
        for (std::size_t idx = 0; idx < grid.cells.size(); ++idx) {
            if (grid.component_labels[idx] == component_label) {
                component_cells.push_back(grid.cells[idx]);
            } else {
                blocked_cells.push_back(grid.cells[idx]);
            }
        }

        const std::size_t component_measured = component_cells.size();
        std::size_t component_added = 0;

        // Records a hole as unfilled and attributes its cells to this component, so the
        // per-component breakdown can report unfilled cells without a second pipeline pass.
        const auto mark_unfilled = [&](const std::vector<CellKey>& hole) {
            unfilled.insert(hole.begin(), hole.end());
            stats.unfilled_by_label[component_label] += hole.size();
        };

        const auto holes = find_enclosed_holes(component_cells, blocked_cells);
        for (const auto& hole : holes) {
            stats.candidate_hole_count += 1;

            if (hole_boundary_component_labels(hole, label_by_cell) != std::set<int>{component_label}) {
                mark_unfilled(hole);
                continue;
            }

            std::vector<SmallAddition> local;
            bool completion_kind_small = false;
            if (cfg.hole_fill_max_cells > 0 && hole.size() <= static_cast<std::size_t>(cfg.hole_fill_max_cells)) {
                completion_kind_small = true;
                if (interpolate_small_hole(hole, height_by_cell, label_by_cell, component_label,
                                           volume_defaults::kHoleFillNeighborRadiusCells,
                                           volume_defaults::kHoleFillMaxNeighborDeltaM, baseline,
                                           volume_defaults::kBaselineFillRadiusCells, local)) {
                    // Small interpolation succeeded.
                } else {
                    local.clear();
                    completion_kind_small = false;
                }
            }

            if (local.empty()) {
                // Quadratic-surface completion for gaps that small interpolation could not fill.
                const double hole_area_cm2 = static_cast<double>(hole.size()) * cell_area * 1.0e4;
                const double component_area_m2 = static_cast<double>(component_cells.size()) * cell_area;
                if (!(cfg.curve_fill_max_hole_area_cm2 > 0.0) || hole_area_cm2 > cfg.curve_fill_max_hole_area_cm2 ||
                    static_cast<double>(hole.size()) * cell_area >
                        component_area_m2 * volume_defaults::kCurveFillMaxComponentAreaRatio) {
                    mark_unfilled(hole);
                    continue;
                }
                std::vector<double> predictions;
                if (!fit_quadratic_hole(
                        hole, height_by_cell, label_by_cell, component_label, grid.cell_size_m,
                        volume_defaults::kCurveFillRimRadiusCells, volume_defaults::kCurveFillMinRimSamples,
                        volume_defaults::kCurveFillMinRimCoverage, volume_defaults::kCurveFillMaxFitRmseM,
                        volume_defaults::kCurveFillMaxPredictionRiseM, cfg.min_height_m, has_max_height,
                        cfg.max_height_m, predictions)) {
                    mark_unfilled(hole);
                    continue;
                }
                for (std::size_t idx = 0; idx < hole.size(); ++idx) {
                    double baseline_height = 0.0;
                    if (!lookup_baseline_height(baseline, hole[idx], volume_defaults::kBaselineFillRadiusCells,
                                                baseline_height)) {
                        local.clear();
                        break;
                    }
                    local.push_back(SmallAddition{hole[idx], baseline_height, predictions[idx]});
                }
                if (local.empty()) {
                    mark_unfilled(hole);
                    continue;
                }
                stats.curve_filled_hole_count += 1;
            }

            // Per-component inferred-cell ceiling keeps the repaired result trustworthy.
            const std::size_t projected_inferred = component_added + local.size();
            const std::size_t projected_total = component_measured + projected_inferred;
            if (projected_total == 0 || static_cast<double>(projected_inferred) / static_cast<double>(projected_total) >
                                            volume_defaults::kCurveFillMaxImputedRatio) {
                mark_unfilled(hole);
                continue;
            }

            for (const auto& addition : local) {
                additions.push_back(addition);
                addition_labels.push_back(component_label);
            }
            component_added += local.size();
            stats.filled_hole_count += 1;
            stats.filled_cell_count += local.size();
            if (completion_kind_small) {
                stats.small_filled_hole_count += 1;
            }
        }

        const double inferred_ratio =
            static_cast<double>(component_added) / static_cast<double>(component_measured + component_added);
        max_component_inferred_ratio = std::max(max_component_inferred_ratio, inferred_ratio);
    }

    stats.unfilled_hole_cells = unfilled.size();
    stats.max_component_inferred_ratio = max_component_inferred_ratio;

    if (additions.empty()) {
        return MeasurementStatus::kSuccess;
    }

    for (const auto& addition : additions) {
        grid.cells.push_back(addition.cell);
        grid.baseline_heights_m.push_back(addition.baseline_height_m);
        grid.heights_m.push_back(addition.height_m);
        grid.is_interpolated.push_back(1);
    }
    grid.component_labels.insert(grid.component_labels.end(), addition_labels.begin(), addition_labels.end());
    recompute_grid_stats(grid);
    return MeasurementStatus::kSuccess;
}

// Derives one estimate per selected component from a single completed height grid. Every grid cell
// already carries its component label, so the per-component breakdown costs one pass over the grid
// instead of a second full rasterisation / hole-completion run per component.
void split_grid_by_component(const std::vector<int>& labels, const HeightGrid& grid, const SurfaceMap& surface,
                             const HoleFillStats& stats, std::vector<ComponentVolumeEstimate>& out) {
    const double cell_area = grid.cell_size_m * grid.cell_size_m;

    for (const int label : labels) {
        ComponentVolumeEstimate est;

        double raw_m = 0.0;
        double interpolated_m = 0.0;
        double sum_h = 0.0;
        double max_h = 0.0;
        std::size_t measured = 0;
        std::size_t interpolated = 0;
        std::int64_t min_i = 0;
        std::int64_t max_i = 0;
        std::int64_t min_j = 0;
        std::int64_t max_j = 0;
        bool first = true;

        for (std::size_t idx = 0; idx < grid.cells.size(); ++idx) {
            if (grid.component_labels[idx] != label) {
                continue;
            }
            const double h = grid.heights_m[idx];
            sum_h += h;
            max_h = std::max(max_h, h);
            if (grid.is_interpolated[idx] != 0) {
                interpolated_m += h;
                ++interpolated;
            } else {
                raw_m += h;
                ++measured;
            }

            if (first) {
                min_i = grid.cells[idx].first;
                max_i = grid.cells[idx].first;
                min_j = grid.cells[idx].second;
                max_j = grid.cells[idx].second;
                first = false;
            } else {
                min_i = std::min(min_i, grid.cells[idx].first);
                max_i = std::max(max_i, grid.cells[idx].first);
                min_j = std::min(min_j, grid.cells[idx].second);
                max_j = std::max(max_j, grid.cells[idx].second);
            }
        }

        // Cells of this component's top surface, i.e. before the height filter dropped any.
        std::size_t top_surface_points = 0;
        for (const int surface_label : surface.labels) {
            if (surface_label == label) {
                ++top_surface_points;
            }
        }

        const std::size_t occupied = measured + interpolated;
        const std::size_t bbox = first ? 0 : static_cast<std::size_t>((max_i - min_i + 1) * (max_j - min_j + 1));

        est.raw_volume_cm3 = raw_m * cell_area * kM3ToCm3;
        est.interpolated_volume_cm3 = interpolated_m * cell_area * kM3ToCm3;
        est.volume_cm3 = est.raw_volume_cm3 + est.interpolated_volume_cm3;
        est.top_surface_points = top_surface_points;
        est.measured_cells = measured;
        est.interpolated_cells = interpolated;
        est.occupied_cells = occupied;
        est.bbox_cell_count = bbox;
        est.footprint_area_m2 = static_cast<double>(occupied) * cell_area;
        est.coverage_ratio = bbox > 0 ? static_cast<double>(occupied) / static_cast<double>(bbox) : 0.0;
        est.mean_height_m = occupied > 0 ? sum_h / static_cast<double>(occupied) : 0.0;
        est.max_height_m = max_h;

        const auto missing = grid.missing_baseline_by_label.find(label);
        est.missing_baseline_cells = missing != grid.missing_baseline_by_label.end() ? missing->second : 0;
        const auto unfilled = stats.unfilled_by_label.find(label);
        est.unfilled_hole_cells = unfilled != stats.unfilled_by_label.end() ? unfilled->second : 0;

        out.push_back(est);
    }
}

} // namespace

#endif // __ARM_EABI__



/**
 * @brief [en] Builds the per-cell top surface of the selected components.
 * @brief [zh] 构建选中连通块的逐格顶表面。
 * @attacher
 */
SurfaceMap build_top_surface(const FoodComponents& components, const BaselineModel& baseline) {
#ifdef __ARM_EABI__
    (void)components;
    (void)baseline;
    return SurfaceMap{};
#else
    SurfaceMap out;
    if (!baseline.data) {
        return out;
    }
    std::map<CellKey, double> surface_by_cell;
    std::map<CellKey, int> label_by_cell;
    build_surface_maps(components, *baseline.data, surface_by_cell, label_by_cell);
    out.cells.reserve(surface_by_cell.size());
    out.heights_m.reserve(surface_by_cell.size());
    out.labels.reserve(surface_by_cell.size());
    for (const auto& entry : surface_by_cell) {
        out.cells.push_back(entry.first);
        out.heights_m.push_back(entry.second);
        out.labels.push_back(label_by_cell[entry.first]);
    }
    return out;
#endif // __ARM_EABI__
}



namespace {

// Grid construction, hole completion and grid-to-estimate conversion. Only
// measure_component_volume below uses them; build_top_surface stays in the header because the
// intermediate-cloud dump needs it. 栅格构建、补洞与转估计，仅供下方积分使用。

/**
 * @brief [en] Builds the baseline-difference height grid from a top-surface map.
 * @brief [zh] 从顶表面图构建基线差分高度栅格。
 * @attacher
 */
MeasurementStatus build_height_grid(const SurfaceMap& surface, const BaselineModel& baseline,
                                    const MeasurementConfig& cfg, HeightGrid& out) {
#ifdef __ARM_EABI__
    (void)surface;
    (void)baseline;
    (void)cfg;
    (void)out;
    return MeasurementStatus::kUnsupportedPlatform;
#else
    if (!baseline.data) {
        return MeasurementStatus::kInvalidConfig;
    }
    std::map<CellKey, double> surface_by_cell;
    std::map<CellKey, int> label_by_cell;
    for (std::size_t i = 0; i < surface.cells.size(); ++i) {
        surface_by_cell[surface.cells[i]] = surface.heights_m[i];
        label_by_cell[surface.cells[i]] = surface.labels[i];
    }
    return build_height_grid(surface_by_cell, label_by_cell, *baseline.data, cfg, out);
#endif // __ARM_EABI__
}



/**
 * @brief [en] Completes enclosed holes conservatively inside single food components, in place.
 * @brief [zh] 在单一食材块内部保守地补全封闭孔（原地修改）。
 * @attacher
 */
MeasurementStatus complete_holes(HeightGrid& grid, const BaselineModel& baseline, const MeasurementConfig& cfg,
                                 HoleFillStats& stats) {
#ifdef __ARM_EABI__
    (void)grid;
    (void)baseline;
    (void)cfg;
    (void)stats;
    return MeasurementStatus::kUnsupportedPlatform;
#else
    if (!baseline.data) {
        return MeasurementStatus::kInvalidConfig;
    }
    return complete_component_aware_holes(grid, *baseline.data, cfg, stats);
#endif // __ARM_EABI__
}



/**
 * @brief [en] Derives a component volume estimate from a height grid.
 * @brief [zh] 从高度栅格派生连通块体积估计。
 * @attacher
 */
ComponentVolumeEstimate compute_grid_estimate(const HeightGrid& grid) {
#ifdef __ARM_EABI__
    (void)grid;
    return ComponentVolumeEstimate{};
#else
    ComponentVolumeEstimate out;
    out.raw_volume_cm3 = grid.raw_volume_m3 * kM3ToCm3;
    out.interpolated_volume_cm3 = grid.interpolated_volume_m3 * kM3ToCm3;
    out.volume_cm3 = grid.volume_m3 * kM3ToCm3;
    out.measured_cells = grid.measured_cells;
    out.interpolated_cells = grid.interpolated_cells;
    out.occupied_cells = grid.occupied_cells;
    out.bbox_cell_count = grid.bbox_cells;
    out.missing_baseline_cells = grid.missing_baseline_cells;
    out.footprint_area_m2 = grid.footprint_area_m2;
    out.coverage_ratio = grid.coverage_ratio;
    out.mean_height_m = grid.mean_height_m;
    out.max_height_m = grid.max_height_m;
    return out;
#endif // __ARM_EABI__
}
} // namespace
/**
 * @brief [en] Measures component volume by integrating baseline-relative heights with conservative hole completion.
 * @brief [zh] 通过积分相对基线高度并保守补洞来测量连通块体积。
 * @attacher
 */
MeasurementStatus measure_component_volume(const FoodComponents& components, const BaselineModel& baseline,
                                           const MeasurementConfig& cfg, ComponentVolumeEstimate& out,
                                           std::vector<ComponentVolumeEstimate>* per_component,
                                           SurfaceMap* completed_surface) {
#ifdef __ARM_EABI__
    (void)components;
    (void)baseline;
    (void)cfg;
    (void)out;
    (void)per_component;
    (void)completed_surface;
    return MeasurementStatus::kUnsupportedPlatform;
#else
    out = ComponentVolumeEstimate{};
    if (per_component != nullptr) {
        per_component->clear();
    }
    if (completed_surface != nullptr) {
        *completed_surface = SurfaceMap{};
    }

    if (components.labels.empty() || components.labels.size() != components.clouds.size()) {
        return MeasurementStatus::kInvalidConfig;
    }
    if (!baseline.data) {
        return MeasurementStatus::kInvalidConfig;
    }
    const BaselineData& data = *baseline.data;
    if (!(data.cell_size_m > 0.0) || !(cfg.min_height_m >= 0.0F)) {
        return MeasurementStatus::kInvalidConfig;
    }

    SurfaceMap surface = build_top_surface(components, baseline);
    if (surface.cells.empty()) {
        return MeasurementStatus::kFoodNotFound;
    }

    HeightGrid grid{};
    MeasurementStatus st = build_height_grid(surface, baseline, cfg, grid);
    if (st != MeasurementStatus::kSuccess) {
        return st;
    }

    HoleFillStats hole_stats{};
    st = complete_holes(grid, baseline, cfg, hole_stats);
    if (st != MeasurementStatus::kSuccess) {
        return st;
    }

    if (completed_surface != nullptr) {
        completed_surface->cells = grid.cells;
        completed_surface->labels = grid.component_labels;
        completed_surface->heights_m.reserve(grid.cells.size());
        for (std::size_t i = 0; i < grid.cells.size(); ++i) {
            completed_surface->heights_m.push_back(grid.baseline_heights_m[i] + grid.heights_m[i]);
        }
    }

    out = compute_grid_estimate(grid);
    out.top_surface_points = surface.cells.size();
    out.unfilled_hole_cells = hole_stats.unfilled_hole_cells;
    log_info("integrate: measured=" + std::to_string(out.measured_cells) + " interpolated=" +
             std::to_string(out.interpolated_cells) + " volume_cm3=" + std::to_string(out.volume_cm3));

    if (per_component != nullptr) {
        split_grid_by_component(components.labels, grid, surface, hole_stats, *per_component);
    }
    return MeasurementStatus::kSuccess;
#endif // __ARM_EABI__
}
