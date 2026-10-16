// Conan::ImportStart
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <string>
#include <vector>

#include "log.hpp"
#include "measurement.hpp"
#include "types.hpp"
// Conan::ImportEnd



namespace py = pybind11;



namespace {



VolumeEstimate measure_from_pcd(const std::vector<std::string>& baseline_paths, const std::string& food_path,
                                const MeasurementConfig& cfg) {
    std::vector<PointCloud> baseline_frames;
    baseline_frames.reserve(baseline_paths.size());
    for (const auto& path : baseline_paths) {
        baseline_frames.push_back(load_pcd(path));
    }
    const PointCloud food = load_pcd(food_path);
    FoodVolumeMeasurer measurer;
    measurer.set_config(cfg).set_baseline(baseline_frames).set_food(food);
    return measurer.run();
}



void bind_enums(py::module& m) {
    py::enum_<LengthUnit>(m, "LengthUnit")
        .value("kMeter", LengthUnit::kMeter)
        .value("kMillimeter", LengthUnit::kMillimeter)
        .export_values();

    py::enum_<MeasurementStatus>(m, "MeasurementStatus")
        .value("kSuccess", MeasurementStatus::kSuccess)
        .value("kEmptyInput", MeasurementStatus::kEmptyInput)
        .value("kNonFiniteInput", MeasurementStatus::kNonFiniteInput)
        .value("kInvalidConfig", MeasurementStatus::kInvalidConfig)
        .value("kEmptyBaseline", MeasurementStatus::kEmptyBaseline)
        .value("kNonFiniteBaseline", MeasurementStatus::kNonFiniteBaseline)
        .value("kPlaneNotFound", MeasurementStatus::kPlaneNotFound)
        .value("kBaselineNoCells", MeasurementStatus::kBaselineNoCells)
        .value("kFoodNotFound", MeasurementStatus::kFoodNotFound)
        .value("kInsufficientCoverage", MeasurementStatus::kInsufficientCoverage)
        .value("kUnsupportedPlatform", MeasurementStatus::kUnsupportedPlatform)
        .export_values();

    py::enum_<LogLevel>(m, "LogLevel")
        .value("kTrace", LogLevel::kTrace)
        .value("kDebug", LogLevel::kDebug)
        .value("kInfo", LogLevel::kInfo)
        .value("kWarning", LogLevel::kWarning)
        .value("kError", LogLevel::kError)
        .value("kOff", LogLevel::kOff)
        .export_values();

    py::enum_<LogFileMode>(m, "LogFileMode")
        .value("kAppend", LogFileMode::kAppend)
        .value("kTruncate", LogFileMode::kTruncate)
        .export_values();
}



void bind_structures(py::module& m) {
    py::class_<Point3f>(m, "Point3f")
        .def(py::init<>())
        .def(py::init<float, float, float>(), py::arg("x"), py::arg("y"), py::arg("z"))
        .def_readwrite("x", &Point3f::x)
        .def_readwrite("y", &Point3f::y)
        .def_readwrite("z", &Point3f::z)
        .def("__repr__", [](const Point3f& p) {
            return "Point3f(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ", " + std::to_string(p.z) + ")";
        });

    py::class_<PointCloud>(m, "PointCloud")
        .def(py::init<>())
        .def(py::init([](const std::vector<Point3f>& points) {
                 PointCloud cloud;
                 cloud.points = points;
                 return cloud;
             }),
             py::arg("points"))
        .def_readwrite("points", &PointCloud::points)
        .def(
            "append", [](PointCloud& cloud, const Point3f& point) { cloud.points.push_back(point); }, py::arg("point"))
        .def("__len__", [](const PointCloud& cloud) { return cloud.points.size(); });

    py::class_<AxisAlignedRoi>(m, "AxisAlignedRoi")
        .def(py::init<>())
        .def_readwrite("min_x", &AxisAlignedRoi::min_x)
        .def_readwrite("max_x", &AxisAlignedRoi::max_x)
        .def_readwrite("min_y", &AxisAlignedRoi::min_y)
        .def_readwrite("max_y", &AxisAlignedRoi::max_y)
        .def_readwrite("min_z", &AxisAlignedRoi::min_z)
        .def_readwrite("max_z", &AxisAlignedRoi::max_z);

    py::class_<MeasurementConfig>(m, "MeasurementConfig")
        .def(py::init<>())
        .def_readwrite("input_unit", &MeasurementConfig::input_unit)
        .def_readwrite("roi", &MeasurementConfig::roi)
        .def_readwrite("voxel_size_m", &MeasurementConfig::voxel_size_m)
        .def_readwrite("plane_distance_threshold_m", &MeasurementConfig::plane_distance_threshold_m)
        .def_readwrite("remove_secondary_plane", &MeasurementConfig::remove_secondary_plane)
        .def_readwrite("foreground_cluster_eps_m", &MeasurementConfig::foreground_cluster_eps_m)
        .def_readwrite("cluster_min_points", &MeasurementConfig::cluster_min_points)
        .def_readwrite("enable_height_watershed", &MeasurementConfig::enable_height_watershed)
        .def_readwrite("selected_labels", &MeasurementConfig::selected_labels)
        .def_readwrite("integration_resolution_m", &MeasurementConfig::integration_resolution_m)
        .def_readwrite("roi_border_margin_m", &MeasurementConfig::roi_border_margin_m)
        .def_readwrite("min_height_m", &MeasurementConfig::min_height_m)
        .def_readwrite("max_height_m", &MeasurementConfig::max_height_m)
        .def_readwrite("hole_fill_max_cells", &MeasurementConfig::hole_fill_max_cells)
        .def_readwrite("curve_fill_max_hole_area_cm2", &MeasurementConfig::curve_fill_max_hole_area_cm2)
        .def_readwrite("middle_cloud_dir", &MeasurementConfig::middle_cloud_dir);

    py::class_<VolumeEstimate>(m, "VolumeEstimate")
        .def_readonly("status", &VolumeEstimate::status)
        .def_readonly("volume_cm3", &VolumeEstimate::volume_cm3)
        .def_readonly("raw_volume_cm3", &VolumeEstimate::raw_volume_cm3)
        .def_readonly("interpolated_volume_cm3", &VolumeEstimate::interpolated_volume_cm3)
        .def_readonly("input_points", &VolumeEstimate::input_points)
        .def_readonly("downsampled_points", &VolumeEstimate::downsampled_points)
        .def_readonly("cluster_count", &VolumeEstimate::cluster_count)
        .def_readonly("selected_cluster_labels", &VolumeEstimate::selected_cluster_labels)
        .def_readonly("component_estimates", &VolumeEstimate::component_estimates)
        .def_readonly("selected_cluster_points", &VolumeEstimate::selected_cluster_points)
        .def_readonly("baseline_frames", &VolumeEstimate::baseline_frames)
        .def_readonly("baseline_cell_count", &VolumeEstimate::baseline_cell_count)
        .def_readonly("component_count", &VolumeEstimate::component_count)
        .def_readonly("top_surface_points", &VolumeEstimate::top_surface_points)
        .def_readonly("measured_cells", &VolumeEstimate::measured_cells)
        .def_readonly("interpolated_cells", &VolumeEstimate::interpolated_cells)
        .def_readonly("occupied_cells", &VolumeEstimate::occupied_cells)
        .def_readonly("bbox_cell_count", &VolumeEstimate::bbox_cell_count)
        .def_readonly("missing_baseline_cells", &VolumeEstimate::missing_baseline_cells)
        .def_readonly("unfilled_hole_cells", &VolumeEstimate::unfilled_hole_cells)
        .def_readonly("footprint_area_m2", &VolumeEstimate::footprint_area_m2)
        .def_readonly("coverage_ratio", &VolumeEstimate::coverage_ratio)
        .def_readonly("mean_height_m", &VolumeEstimate::mean_height_m)
        .def_readonly("max_height_m", &VolumeEstimate::max_height_m)
        .def_readonly("aabb_volume_m3", &VolumeEstimate::aabb_volume_m3)
        .def_readonly("obb_volume_m3", &VolumeEstimate::obb_volume_m3)
        .def_readonly("convex_hull_volume_m3", &VolumeEstimate::convex_hull_volume_m3)
        .def_readonly("message", &VolumeEstimate::message)
        .def("__repr__", [](const VolumeEstimate& estimate) {
            return "<VolumeEstimate status=" + std::string(status_to_string(estimate.status)) +
                   " volume_cm3=" + std::to_string(estimate.volume_cm3) + ">";
        });

    py::class_<ComponentVolumeEstimate>(m, "ComponentVolumeEstimate")
        .def(py::init<>())
        .def_readonly("raw_volume_cm3", &ComponentVolumeEstimate::raw_volume_cm3)
        .def_readonly("interpolated_volume_cm3", &ComponentVolumeEstimate::interpolated_volume_cm3)
        .def_readonly("volume_cm3", &ComponentVolumeEstimate::volume_cm3)
        .def_readonly("top_surface_points", &ComponentVolumeEstimate::top_surface_points)
        .def_readonly("measured_cells", &ComponentVolumeEstimate::measured_cells)
        .def_readonly("interpolated_cells", &ComponentVolumeEstimate::interpolated_cells)
        .def_readonly("occupied_cells", &ComponentVolumeEstimate::occupied_cells)
        .def_readonly("bbox_cell_count", &ComponentVolumeEstimate::bbox_cell_count)
        .def_readonly("missing_baseline_cells", &ComponentVolumeEstimate::missing_baseline_cells)
        .def_readonly("unfilled_hole_cells", &ComponentVolumeEstimate::unfilled_hole_cells)
        .def_readonly("footprint_area_m2", &ComponentVolumeEstimate::footprint_area_m2)
        .def_readonly("coverage_ratio", &ComponentVolumeEstimate::coverage_ratio)
        .def_readonly("mean_height_m", &ComponentVolumeEstimate::mean_height_m)
        .def_readonly("max_height_m", &ComponentVolumeEstimate::max_height_m);
}



void bind_functions(py::module& m) {
    m.def("length_unit_to_meter_scale", &length_unit_to_meter_scale, py::arg("unit"));
    m.def("status_to_string", &status_to_string, py::arg("status"));
    m.def("is_finite", &is_finite, py::arg("point"));
    m.def("load_pcd", &load_pcd, py::arg("path"));
    m.def("measure_from_pcd", &measure_from_pcd, py::arg("baseline_pcd_paths"), py::arg("food_pcd_path"),
          py::arg("cfg") = MeasurementConfig{});

    py::class_<FoodVolumeMeasurer>(m, "FoodVolumeMeasurer")
        .def(py::init<>())
        .def("set_baseline", &FoodVolumeMeasurer::set_baseline, py::arg("frames"))
        .def("add_baseline_frame", &FoodVolumeMeasurer::add_baseline_frame, py::arg("frame"))
        .def("set_food", &FoodVolumeMeasurer::set_food, py::arg("cloud"))
        .def("set_input_unit", &FoodVolumeMeasurer::set_input_unit, py::arg("unit"))
        .def("set_voxel_size", &FoodVolumeMeasurer::set_voxel_size, py::arg("size_m"))
        .def("set_integration_resolution", &FoodVolumeMeasurer::set_integration_resolution, py::arg("resolution_m"))
        .def("set_height_range", &FoodVolumeMeasurer::set_height_range, py::arg("min_m"), py::arg("max_m"))
        .def("set_roi", &FoodVolumeMeasurer::set_roi, py::arg("roi"))
        .def("clear_roi", &FoodVolumeMeasurer::clear_roi)
        .def("set_selected_labels", &FoodVolumeMeasurer::set_selected_labels, py::arg("labels"))
        .def("set_cluster_params", &FoodVolumeMeasurer::set_cluster_params, py::arg("footprint_eps_m"),
             py::arg("min_points"))
        .def("set_plane_distance_threshold", &FoodVolumeMeasurer::set_plane_distance_threshold, py::arg("threshold_m"))
        .def("set_middle_cloud_dir", &FoodVolumeMeasurer::set_middle_cloud_dir, py::arg("dir"))
        .def("set_config", &FoodVolumeMeasurer::set_config, py::arg("cfg"))
        .def("config", &FoodVolumeMeasurer::config, py::return_value_policy::reference_internal)
        .def("load_config_from_json", &FoodVolumeMeasurer::load_config_from_json, py::arg("path"))
        .def("save_config_to_json", &FoodVolumeMeasurer::save_config_to_json, py::arg("path"))
        .def("run", &FoodVolumeMeasurer::run);

    // Logging.
    m.def("log_set_level", &log_set_level, py::arg("level"));
    m.def("log_get_level", &log_get_level);
    m.def("log_set_console", &log_set_console, py::arg("enabled"));
    m.def("log_get_console", &log_get_console);
    m.def("log_set_file", &log_set_file, py::arg("path"), py::arg("mode") = LogFileMode::kAppend);
    m.def("log_close_file", &log_close_file);
    m.def("log_write", &log_write, py::arg("level"), py::arg("message"));
    m.def("log_trace", &log_trace, py::arg("message"));
    m.def("log_debug", &log_debug, py::arg("message"));
    m.def("log_info", &log_info, py::arg("message"));
    m.def("log_warning", &log_warning, py::arg("message"));
    m.def("log_error", &log_error, py::arg("message"));
}



} // namespace



PYBIND11_MODULE(food3d_im_python, m) {
    m.doc() = "Python bindings for the vm IM food volume measurement library";
    m.attr("__version__") = "0.1.0";

    bind_enums(m);
    bind_structures(m);
    bind_functions(m);
}
