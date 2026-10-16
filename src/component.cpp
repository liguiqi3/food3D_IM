// Conan::ImportStart
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <tuple>
#include <utility>
#include <vector>
#include "component.hpp"
#include "grid.hpp"
#include "log.hpp"
#include "pointcloudprocess.hpp"
// Conan::ImportEnd



namespace {

// Filters food points by baseline-relative height and the inset plane ROI, producing the dense foreground cloud.
MeasurementStatus build_valid_difference_points(const PointCloud& food_m, const BaselineData& baseline,
                                                const PlaneRoi& roi, const MeasurementConfig& cfg,
                                                PointCloud& dense_out) {
    dense_out = PointCloud{};

    if (volume_defaults::kBaselineFillRadiusCells < 0 || !(cfg.min_height_m >= 0.0F)) {
        return MeasurementStatus::kInvalidConfig;
    }

    dense_out.points.reserve(food_m.points.size());
    for (const auto& p : food_m.points) {
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
        if (!inside_roi(center_u, center_v, roi)) {
            continue;
        }

        double baseline_height = 0.0;
        if (!lookup_baseline_height(baseline, key, volume_defaults::kBaselineFillRadiusCells, baseline_height)) {
            continue;
        }

        double diff = h - baseline_height;
        if (diff < cfg.min_height_m) {
            continue;
        }
        if (cfg.max_height_m > 0.0F && diff > cfg.max_height_m) {
            diff = cfg.max_height_m;
        }
        (void)diff;
        dense_out.points.push_back(p);
    }

    if (dense_out.points.empty()) {
        return MeasurementStatus::kInsufficientCoverage;
    }
    return MeasurementStatus::kSuccess;
}

// Defined further down but called by extract_food_components above, so they are declared here.
// 定义在下方，但被上方的 extract_food_components 调用，故在此前置声明。
MeasurementStatus filter_baseline_difference(const PointCloud& food_m, const BaselineModel& baseline,
                                             const MeasurementConfig& cfg, PointCloud& dense_out);
PointCloud project_to_plane(const PointCloud& cloud, const BaselineModel& baseline);
std::vector<int> split_labels_by_height_watershed(const PointCloud& dense, const std::vector<int>& dbscan_labels,
                                                  const BaselineData& baseline);
MeasurementStatus select_components(const std::vector<int>& labels, const PointCloud& cloud,
                                    const MeasurementConfig& cfg, std::size_t cluster_count, FoodComponents& out);

} // namespace



/**
 * @brief [en] Filters food points by baseline-relative height, clusters them in the baseline plane, and selects components.
 * @brief [zh] 按相对基线高度过滤食材点，在基准面内聚类并选择连通块。
 * @attacher
 */
MeasurementStatus extract_food_components(const PointCloud& food_m, const BaselineModel& baseline,
                                          const MeasurementConfig& cfg, FoodComponents& out) {
#ifdef __ARM_EABI__
    (void)food_m;
    (void)baseline;
    (void)cfg;
    (void)out;
    return MeasurementStatus::kUnsupportedPlatform;
#else
    out = FoodComponents{};

    if (!baseline.data) {
        return MeasurementStatus::kInvalidConfig;
    }
    if (food_m.points.empty()) {
        return MeasurementStatus::kInsufficientCoverage;
    }
    if (!(cfg.foreground_cluster_eps_m > 0.0F) || cfg.cluster_min_points <= 0 ||
        volume_defaults::kBaselineFillRadiusCells < 0 || !(cfg.min_height_m >= 0.0F)) {
        return MeasurementStatus::kInvalidConfig;
    }

    PointCloud dense{};
    MeasurementStatus st = filter_baseline_difference(food_m, baseline, cfg, dense);
    if (st != MeasurementStatus::kSuccess) {
        return st;
    }

    // Project valid points onto the baseline plane and cluster with Open3D-equivalent DBSCAN.
    const PointCloud projected = project_to_plane(dense, baseline);
    const std::vector<int> dbscan =
        dbscan_labels(projected.points, cfg.foreground_cluster_eps_m, cfg.cluster_min_points);

    // Two items that nearly touch share one footprint cluster even so. When enabled, split each
    // cluster where the top surface dips between its peaks, so the reported component count matches
    // the real items; otherwise the raw DBSCAN clusters are kept as-is.
    std::set<int> distinct_clusters;
    for (const int label : dbscan) {
        if (label >= 0) {
            distinct_clusters.insert(label);
        }
    }
    const std::vector<int> labels =
        cfg.enable_height_watershed ? split_labels_by_height_watershed(dense, dbscan, *baseline.data) : dbscan;

    return select_components(labels, dense, cfg, distinct_clusters.size(), out);
#endif // __ARM_EABI__
}


namespace {

// Foreground filtering and footprint clustering, used only by extract_food_components above.
// 前景过滤与足迹聚类，仅供上方的 extract_food_components 使用。

MeasurementStatus filter_baseline_difference(const PointCloud& food_m, const BaselineModel& baseline,
                                             const MeasurementConfig& cfg, PointCloud& dense_out) {
#ifdef __ARM_EABI__
    (void)food_m;
    (void)baseline;
    (void)cfg;
    (void)dense_out;
    return MeasurementStatus::kUnsupportedPlatform;
#else
    if (!baseline.data) {
        return MeasurementStatus::kInvalidConfig;
    }
    const BaselineData& data = *baseline.data;
    return build_valid_difference_points(food_m, data, data.roi, cfg, dense_out);
#endif // __ARM_EABI__
}



PointCloud project_to_plane(const PointCloud& cloud, const BaselineModel& baseline) {
#ifdef __ARM_EABI__
    (void)cloud;
    (void)baseline;
    return PointCloud{};
#else
    PointCloud out;
    if (!baseline.data) {
        return out;
    }
    const PlaneFrame& frame = baseline.data->frame;
    out.points.reserve(cloud.points.size());
    for (const auto& p : cloud.points) {
        double u = 0.0;
        double v = 0.0;
        double h = 0.0;
        project_to_plane_frame(frame, p, u, v, h);
        out.points.push_back(Point3f{static_cast<float>(u), static_cast<float>(v), 0.0F});
    }
    return out;
#endif // __ARM_EABI__
}



#ifndef __ARM_EABI__
namespace {

// Splits one 8-connected footprint component into height-separated sub-components. Seeding only
// from high plateaus keeps sensor noise from carving a single item into pieces, while genuinely
// separate items still seed their own basin.
std::vector<int> split_one_component(const std::vector<CellKey>& cells, const std::vector<double>& heights_m,
                                     double min_valley_depth_m, double min_valley_frac, double min_seed_frac) {
    const std::size_t cell_count = cells.size();
    std::vector<int> sub_labels(cell_count, 0);
    if (cell_count <= 1) {
        return sub_labels;
    }

    std::map<CellKey, std::size_t> index_of;
    for (std::size_t i = 0; i < cell_count; ++i) {
        index_of[cells[i]] = i;
    }

    constexpr int kNeighborCount = 8;
    constexpr int kNeighborDu[kNeighborCount] = {-1, -1, -1, 0, 0, 1, 1, 1};
    constexpr int kNeighborDv[kNeighborCount] = {-1, 0, 1, -1, 1, -1, 0, 1};
    std::vector<std::size_t> neighbors;
    const auto collect_neighbors = [&](std::size_t index) {
        neighbors.clear();
        for (int k = 0; k < kNeighborCount; ++k) {
            const CellKey candidate{cells[index].first + kNeighborDu[k], cells[index].second + kNeighborDv[k]};
            const auto it = index_of.find(candidate);
            if (it != index_of.end()) {
                neighbors.push_back(it->second);
            }
        }
    };

    // Markers are the high plateaus: cells at or above a fraction of this component's peak.
    double hmax = -std::numeric_limits<double>::infinity();
    for (const double h : heights_m) {
        hmax = std::max(hmax, h);
    }
    const double seed_threshold = min_seed_frac * hmax;

    std::vector<int> marker(cell_count, -1);
    int marker_count = 0;
    for (std::size_t seed = 0; seed < cell_count; ++seed) {
        if (heights_m[seed] < seed_threshold || marker[seed] >= 0) {
            continue;
        }
        const int id = marker_count++;
        marker[seed] = id;
        std::vector<std::size_t> stack{seed};
        while (!stack.empty()) {
            const std::size_t current = stack.back();
            stack.pop_back();
            collect_neighbors(current);
            for (const std::size_t j : neighbors) {
                if (marker[j] < 0 && heights_m[j] >= seed_threshold) {
                    marker[j] = id;
                    stack.push_back(j);
                }
            }
        }
    }

    if (marker_count <= 1) {
        return sub_labels;
    }

    // Assign every cell to the geodesically nearest marker. A pure height-first flood lets a large,
    // lower basin race along a low trailing edge and steal the other item's rim; the uniform-cost
    // geodesic assignment puts the boundary between the markers instead.
    std::vector<int> label(cell_count, -1);
    std::vector<double> dist(cell_count, std::numeric_limits<double>::infinity());
    using FloodNode = std::tuple<double, std::size_t, int>;
    std::priority_queue<FloodNode, std::vector<FloodNode>, std::greater<>> frontier;
    for (std::size_t i = 0; i < cell_count; ++i) {
        if (marker[i] >= 0) {
            label[i] = marker[i];
            dist[i] = 0.0;
            frontier.emplace(0.0, i, marker[i]);
        }
    }
    while (!frontier.empty()) {
        const FloodNode node = frontier.top();
        frontier.pop();
        const std::size_t index = std::get<1>(node);
        if (std::get<0>(node) > dist[index]) {
            continue;
        }
        const int id = std::get<2>(node);
        collect_neighbors(index);
        for (const std::size_t j : neighbors) {
            const bool diagonal = cells[j].first != cells[index].first && cells[j].second != cells[index].second;
            const double step = diagonal ? std::sqrt(2.0) : 1.0;
            const double next = dist[index] + step;
            if (next < dist[j]) {
                dist[j] = next;
                label[j] = id;
                frontier.emplace(next, j, id);
            }
        }
    }

    // Valley floor between two markers: the lowest cell on their shared boundary. The drop from the
    // lower peak down to this floor is the separating valley's depth.
    std::map<std::pair<int, int>, double> valley_floor_m;
    for (std::size_t i = 0; i < cell_count; ++i) {
        if (label[i] < 0) {
            continue;
        }
        collect_neighbors(i);
        for (const std::size_t j : neighbors) {
            if (label[j] < 0 || label[j] == label[i]) {
                continue;
            }
            const int low = std::min(label[i], label[j]);
            const int high = std::max(label[i], label[j]);
            const double pass = std::min(heights_m[i], heights_m[j]);
            const auto it = valley_floor_m.find(std::make_pair(low, high));
            if (it == valley_floor_m.end()) {
                valley_floor_m.emplace(std::make_pair(low, high), pass);
            } else {
                it->second = std::min(it->second, pass);
            }
        }
    }

    // Merge markers whose separating valley is too shallow to be two distinct items.
    std::vector<int> parent(static_cast<std::size_t>(marker_count));
    std::vector<double> peak(static_cast<std::size_t>(marker_count), -std::numeric_limits<double>::infinity());
    for (int i = 0; i < marker_count; ++i) {
        parent[static_cast<std::size_t>(i)] = i;
    }
    for (std::size_t i = 0; i < cell_count; ++i) {
        if (label[i] >= 0) {
            const auto slot = static_cast<std::size_t>(label[i]);
            peak[slot] = std::max(peak[slot], heights_m[i]);
        }
    }
    const auto find_root = [&parent](int node) {
        while (parent[static_cast<std::size_t>(node)] != node) {
            node = parent[static_cast<std::size_t>(node)];
        }
        return node;
    };
    std::vector<std::tuple<double, int, int>> edges;
    edges.reserve(valley_floor_m.size());
    for (const auto& entry : valley_floor_m) {
        edges.emplace_back(entry.second, entry.first.first, entry.first.second);
    }
    std::sort(edges.begin(), edges.end());
    for (const auto& edge : edges) {
        const double pass = std::get<0>(edge);
        const int root_a = find_root(std::get<1>(edge));
        const int root_b = find_root(std::get<2>(edge));
        if (root_a == root_b) {
            continue;
        }
        const double peak_a = peak[static_cast<std::size_t>(root_a)];
        const double peak_b = peak[static_cast<std::size_t>(root_b)];
        // Prominence: a valley splits two items only when it drops well below the lower peak, both in
        // absolute terms (above sensor noise) and relative terms (a real gap, not a bumpy surface).
        const double required = std::max(min_valley_depth_m, min_valley_frac * std::min(peak_a, peak_b));
        const double margin = std::min(peak_a, peak_b) - pass;
        if (margin >= required) {
            continue;
        }
        parent[static_cast<std::size_t>(root_a)] = root_b;
        peak[static_cast<std::size_t>(root_b)] = std::max(peak_a, peak_b);
    }

    std::map<int, int> compact;
    for (std::size_t i = 0; i < cell_count; ++i) {
        const int root = label[i] >= 0 ? find_root(label[i]) : 0;
        const auto it = compact.find(root);
        if (it == compact.end()) {
            const int next = static_cast<int>(compact.size());
            compact.emplace(root, next);
            sub_labels[i] = next;
        } else {
            sub_labels[i] = it->second;
        }
    }
    return sub_labels;
}

} // namespace
#endif // __ARM_EABI__



// Splits each DBSCAN footprint cluster into height-separated sub-components. Points keep their
// order, so the result lines up one-to-one with `dense`; non-food points stay labelled -1.
std::vector<int> split_labels_by_height_watershed(const PointCloud& dense, const std::vector<int>& dbscan_labels,
                                                  const BaselineData& baseline) {
#ifdef __ARM_EABI__
    (void)dense;
    (void)baseline;
    return dbscan_labels;
#else
    std::vector<int> labels = dbscan_labels;
    const std::size_t point_count = dense.points.size();
    if (point_count == 0 || !(baseline.cell_size_m > 0.0)) {
        return labels;
    }

    // Per point: the cell it falls in; per cluster: the tallest baseline-relative height per cell.
    std::vector<CellKey> point_cell(point_count, CellKey{0, 0});
    std::vector<char> point_has_cell(point_count, 0);
    std::map<int, std::map<CellKey, double>> height_by_cluster;
    for (std::size_t i = 0; i < point_count; ++i) {
        if (dbscan_labels[i] < 0) {
            continue;
        }
        double u = 0.0;
        double v = 0.0;
        double h = 0.0;
        project_to_plane_frame(baseline.frame, dense.points[i], u, v, h);
        if (!std::isfinite(h)) {
            continue;
        }
        const CellKey key = cell_index_of(u, v, baseline.cell_size_m);
        const double center_u = (static_cast<double>(key.first) + 0.5) * baseline.cell_size_m;
        const double center_v = (static_cast<double>(key.second) + 0.5) * baseline.cell_size_m;
        if (!inside_roi(center_u, center_v, baseline.roi)) {
            continue;
        }
        double baseline_height = 0.0;
        if (!lookup_baseline_height(baseline, key, volume_defaults::kBaselineFillRadiusCells, baseline_height)) {
            continue;
        }
        point_cell[i] = key;
        point_has_cell[i] = 1;
        auto& cells = height_by_cluster[dbscan_labels[i]];
        const double diff = h - baseline_height;
        const auto it = cells.find(key);
        if (it == cells.end() || diff > it->second) {
            cells[key] = diff;
        }
    }

    // Split every cluster in the height domain; sub-components are numbered per cluster.
    std::map<int, std::map<CellKey, int>> split_of_cell;
    for (const auto& cluster : height_by_cluster) {
        std::vector<CellKey> cells;
        std::vector<double> heights;
        cells.reserve(cluster.second.size());
        heights.reserve(cluster.second.size());
        for (const auto& entry : cluster.second) {
            cells.push_back(entry.first);
            heights.push_back(entry.second);
        }

        std::map<CellKey, std::size_t> index_of;
        for (std::size_t i = 0; i < cells.size(); ++i) {
            index_of[cells[i]] = i;
        }

        // Group the footprint into 8-connected components first.
        std::vector<int> group(cells.size(), -1);
        int group_count = 0;
        for (std::size_t seed = 0; seed < cells.size(); ++seed) {
            if (group[seed] >= 0) {
                continue;
            }
            const int id = group_count++;
            group[seed] = id;
            std::vector<std::size_t> stack{seed};
            while (!stack.empty()) {
                const std::size_t current = stack.back();
                stack.pop_back();
                for (int du = -1; du <= 1; ++du) {
                    for (int dv = -1; dv <= 1; ++dv) {
                        if (du == 0 && dv == 0) {
                            continue;
                        }
                        const auto it = index_of.find(CellKey{cells[current].first + du, cells[current].second + dv});
                        if (it == index_of.end() || group[it->second] >= 0) {
                            continue;
                        }
                        group[it->second] = id;
                        stack.push_back(it->second);
                    }
                }
            }
        }

        std::vector<std::vector<std::size_t>> members(static_cast<std::size_t>(group_count));
        for (std::size_t i = 0; i < cells.size(); ++i) {
            members[static_cast<std::size_t>(group[i])].push_back(i);
        }

        auto& cell_split = split_of_cell[cluster.first];
        int next_sub = 0;
        for (const auto& group_members : members) {
            std::vector<CellKey> group_cells;
            std::vector<double> group_heights;
            group_cells.reserve(group_members.size());
            group_heights.reserve(group_members.size());
            for (const std::size_t index : group_members) {
                group_cells.push_back(cells[index]);
                group_heights.push_back(heights[index]);
            }
            const std::vector<int> sub = split_one_component(
                group_cells, group_heights, volume_defaults::kComponentSplitMinValleyDepthM,
                volume_defaults::kComponentSplitMinValleyFrac, volume_defaults::kComponentSplitSeedFrac);
            int sub_count = 0;
            for (const int value : sub) {
                sub_count = std::max(sub_count, value + 1);
            }
            for (std::size_t i = 0; i < group_cells.size(); ++i) {
                cell_split[group_cells[i]] = next_sub + sub[i];
            }
            next_sub += sub_count;
        }
    }

    // Map every point to the sub-component of its own cluster's cell.
    std::map<std::pair<int, int>, int> global_of_local;
    for (std::size_t i = 0; i < point_count; ++i) {
        if (dbscan_labels[i] < 0 || point_has_cell[i] == 0) {
            labels[i] = -1;
            continue;
        }
        const auto cluster_it = split_of_cell.find(dbscan_labels[i]);
        int local = 0;
        if (cluster_it != split_of_cell.end()) {
            const auto cell_it = cluster_it->second.find(point_cell[i]);
            if (cell_it != cluster_it->second.end()) {
                local = cell_it->second;
            }
        }
        const auto key = std::make_pair(dbscan_labels[i], local);
        const auto it = global_of_local.find(key);
        if (it == global_of_local.end()) {
            const int next = static_cast<int>(global_of_local.size());
            global_of_local.emplace(key, next);
            labels[i] = next;
        } else {
            labels[i] = it->second;
        }
    }
    return labels;
#endif // __ARM_EABI__
}



MeasurementStatus select_components(const std::vector<int>& labels, const PointCloud& cloud,
                                    const MeasurementConfig& cfg, std::size_t cluster_count, FoodComponents& out) {
#ifdef __ARM_EABI__
    (void)labels;
    (void)cloud;
    (void)cfg;
    (void)cluster_count;
    (void)out;
    return MeasurementStatus::kUnsupportedPlatform;
#else
    out = FoodComponents{};
    if (labels.size() != cloud.points.size() || cfg.cluster_min_points <= 0) {
        return MeasurementStatus::kInvalidConfig;
    }

    const int min_candidate_size = std::max(32, cfg.cluster_min_points * 4);
    std::map<int, std::size_t> counts;
    for (const int label : labels) {
        if (label >= 0) {
            counts[label] += 1;
        }
    }
    if (counts.empty()) {
        return MeasurementStatus::kFoodNotFound;
    }

    // Keeps the DBSCAN cluster count, independent of any height-watershed sub-splitting.
    out.cluster_count = cluster_count;

    std::vector<int> selected;
    if (cfg.selected_labels.empty()) {
        for (const auto& entry : counts) {
            if (entry.second >= static_cast<std::size_t>(min_candidate_size)) {
                selected.push_back(entry.first);
            }
        }
    } else {
        for (const int label : cfg.selected_labels) {
            const auto it = counts.find(label);
            if (it != counts.end() && it->second >= static_cast<std::size_t>(min_candidate_size)) {
                selected.push_back(label);
            }
        }
        std::sort(selected.begin(), selected.end());
    }
    if (selected.empty()) {
        return MeasurementStatus::kFoodNotFound;
    }

    std::map<int, std::size_t> slot_by_label;
    for (std::size_t i = 0; i < selected.size(); ++i) {
        slot_by_label[selected[i]] = i;
    }

    std::vector<std::size_t> per_point_slot(cloud.points.size(), static_cast<std::size_t>(-1));
    for (std::size_t i = 0; i < labels.size(); ++i) {
        if (labels[i] >= 0) {
            const auto it = slot_by_label.find(labels[i]);
            if (it != slot_by_label.end()) {
                per_point_slot[i] = it->second;
            }
        }
    }

    out.labels = selected;
    out.clouds.resize(selected.size());
    std::vector<std::size_t> counts_per_slot(selected.size(), 0);
    for (const std::size_t slot : per_point_slot) {
        if (slot != static_cast<std::size_t>(-1)) {
            counts_per_slot[slot] += 1;
        }
    }
    for (std::size_t s = 0; s < selected.size(); ++s) {
        out.clouds[s].points.reserve(counts_per_slot[s]);
    }
    for (std::size_t i = 0; i < per_point_slot.size(); ++i) {
        if (per_point_slot[i] != static_cast<std::size_t>(-1)) {
            out.clouds[per_point_slot[i]].points.push_back(cloud.points[i]);
        }
    }
    log_info("components: clusters=" + std::to_string(out.cluster_count) +
             " selected=" + std::to_string(out.labels.size()));
    return MeasurementStatus::kSuccess;
#endif // __ARM_EABI__
}
} // namespace
