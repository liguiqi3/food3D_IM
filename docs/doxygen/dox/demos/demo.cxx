// Example: one empty tray frame and one food frame in the same camera coordinates.
#include "measurement.hpp"

#include <iostream>

int main() {
    const PointCloud empty = load_pcd("empty_tray.pcd");
    const PointCloud food = load_pcd("food.pcd");
    if (empty.points.empty() || food.points.empty()) {
        std::cerr << "Could not read the input PCD files\n";
        return 1;
    }

    FoodVolumeMeasurer measurer;
    // For millimetre PCD coordinates, add:
    // measurer.set_input_unit(LengthUnit::kMillimeter);
    const VolumeEstimate result = measurer.set_baseline({empty}).set_food(food).run();
    if (result.status != MeasurementStatus::kSuccess) {
        std::cerr << status_to_string(result.status) << '\n';
        return 1;
    }

    std::cout << "measured: " << result.raw_volume_cm3 << " cm3\n"
              << "filled:   " << result.interpolated_volume_cm3 << " cm3\n"
              << "total:    " << result.volume_cm3 << " cm3\n"
              << "filled cells: " << result.interpolated_cells << '\n'
              << "missing baseline cells: " << result.missing_baseline_cells << '\n';
}
