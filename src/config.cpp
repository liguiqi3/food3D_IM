// Conan::ImportStart
#include <fstream>
#include <string>
#include <nlohmann/json.hpp>
#include "config.hpp"
#include "log.hpp"
// Conan::ImportEnd



namespace {

using Json = nlohmann::json;

bool parse_length_unit(const std::string& text, LengthUnit& out) {
    if (text == "meter") {
        out = LengthUnit::kMeter;
        return true;
    }
    if (text == "millimeter") {
        out = LengthUnit::kMillimeter;
        return true;
    }
    return false;
}

const char* length_unit_to_string(LengthUnit unit) {
    switch (unit) {
    case LengthUnit::kMeter:
        return "meter";
    case LengthUnit::kMillimeter:
        return "millimeter";
    }
    return "meter";
}

// Reads a scalar member only when the key is present; leaves the target untouched otherwise.
// A type-mismatched field is ignored (the target keeps its previous value).
template <typename T> void read_if_present(const Json& j, const char* key, T& target) {
    if (!j.contains(key)) {
        return;
    }
    try {
        j.at(key).get_to(target);
    } catch (const Json::exception&) {
        // 类型不匹配：忽略该字段，保留原值。
    }
}

} // namespace



bool load_config_from_json(const std::string& path, MeasurementConfig& out) {
    std::ifstream file(path);
    if (!file.is_open()) {
        log_error("load_config_from_json: cannot open file " + path);
        return false;
    }

    Json root;
    try {
        file >> root;
    } catch (const Json::parse_error& err) {
        log_error("load_config_from_json: parse error " + std::string(err.what()));
        return false;
    }
    if (!root.is_object()) {
        log_error("load_config_from_json: root is not a JSON object");
        return false;
    }

    // Reject unsupported keys so old or misspelled options cannot be silently ignored.
    for (auto it = root.begin(); it != root.end(); ++it) {
        bool supported = false;
        for (const char* key :
             {"input_unit", "roi", "voxel_size_m", "plane_distance_threshold_m", "remove_secondary_plane",
              "foreground_cluster_eps_m", "cluster_min_points", "selected_labels", "integration_resolution_m",
              "roi_border_margin_m", "min_height_m", "max_height_m", "hole_fill_max_cells",
              "curve_fill_max_hole_area_cm2", "middle_cloud_dir", "enable_height_watershed"}) {
            if (it.key() == key) {
                supported = true;
                break;
            }
        }
        if (!supported) {
            log_error("load_config_from_json: unsupported option '" + it.key() + "'");
            return false;
        }
    }

    // Enum fields (string -> enum).
    if (root.contains("input_unit")) {
        const Json& unit_json = root.at("input_unit");
        if (!unit_json.is_string()) {
            log_error("load_config_from_json: input_unit must be a string");
            return false;
        }
        LengthUnit unit{};
        if (!parse_length_unit(unit_json.get<std::string>(), unit)) {
            log_error("load_config_from_json: invalid input_unit");
            return false;
        }
        out.input_unit = unit;
    }
    // Nested ROI object.
    if (root.contains("roi")) {
        const Json& roi = root.at("roi");
        if (roi.is_null()) {
            out.roi.reset();
        } else if (roi.is_object()) {
            AxisAlignedRoi bounds = out.roi.value_or(AxisAlignedRoi{});
            read_if_present(roi, "min_x", bounds.min_x);
            read_if_present(roi, "max_x", bounds.max_x);
            read_if_present(roi, "min_y", bounds.min_y);
            read_if_present(roi, "max_y", bounds.max_y);
            read_if_present(roi, "min_z", bounds.min_z);
            read_if_present(roi, "max_z", bounds.max_z);
            out.roi = bounds;
        } else {
            log_error("load_config_from_json: roi must be an object or null");
            return false;
        }
    }

    // Scalar fields.
    read_if_present(root, "voxel_size_m", out.voxel_size_m);
    read_if_present(root, "plane_distance_threshold_m", out.plane_distance_threshold_m);
    read_if_present(root, "remove_secondary_plane", out.remove_secondary_plane);
    read_if_present(root, "foreground_cluster_eps_m", out.foreground_cluster_eps_m);
    read_if_present(root, "cluster_min_points", out.cluster_min_points);
    read_if_present(root, "enable_height_watershed", out.enable_height_watershed);
    read_if_present(root, "integration_resolution_m", out.integration_resolution_m);
    read_if_present(root, "roi_border_margin_m", out.roi_border_margin_m);
    read_if_present(root, "min_height_m", out.min_height_m);
    read_if_present(root, "max_height_m", out.max_height_m);
    read_if_present(root, "hole_fill_max_cells", out.hole_fill_max_cells);
    read_if_present(root, "curve_fill_max_hole_area_cm2", out.curve_fill_max_hole_area_cm2);
    read_if_present(root, "middle_cloud_dir", out.middle_cloud_dir);

    if (root.contains("selected_labels")) {
        if (!root.at("selected_labels").is_array()) {
            log_error("load_config_from_json: selected_labels must be an array");
            return false;
        }
        root.at("selected_labels").get_to(out.selected_labels);
    }

    return true;
}



bool save_config_to_json(const MeasurementConfig& cfg, const std::string& path) {
    Json root = Json::object();

    root["input_unit"] = length_unit_to_string(cfg.input_unit);
    root["roi"] = cfg.roi ? Json{{"min_x", cfg.roi->min_x}, {"max_x", cfg.roi->max_x}, {"min_y", cfg.roi->min_y},
                                 {"max_y", cfg.roi->max_y}, {"min_z", cfg.roi->min_z}, {"max_z", cfg.roi->max_z}}
                          : Json(nullptr);
    root["voxel_size_m"] = cfg.voxel_size_m;
    root["plane_distance_threshold_m"] = cfg.plane_distance_threshold_m;
    root["remove_secondary_plane"] = cfg.remove_secondary_plane;
    root["foreground_cluster_eps_m"] = cfg.foreground_cluster_eps_m;
    root["cluster_min_points"] = cfg.cluster_min_points;
    root["enable_height_watershed"] = cfg.enable_height_watershed;
    root["selected_labels"] = cfg.selected_labels;
    root["integration_resolution_m"] = cfg.integration_resolution_m;
    root["roi_border_margin_m"] = cfg.roi_border_margin_m;
    root["min_height_m"] = cfg.min_height_m;
    root["max_height_m"] = cfg.max_height_m;
    root["hole_fill_max_cells"] = cfg.hole_fill_max_cells;
    root["curve_fill_max_hole_area_cm2"] = cfg.curve_fill_max_hole_area_cm2;
    root["middle_cloud_dir"] = cfg.middle_cloud_dir;

    std::ofstream file(path);
    if (!file.is_open()) {
        log_error("save_config_to_json: cannot open file " + path);
        return false;
    }
    file << root.dump(2);
    return true;
}
