// Conan::ImportStart
#include "types.hpp"
// Conan::ImportEnd



double length_unit_to_meter_scale(LengthUnit unit) {
    // Any unit other than millimetres is treated as metres, including an out-of-range value.
    return unit == LengthUnit::kMillimeter ? 1.0e-3 : 1.0;
}



const char* status_to_string(MeasurementStatus status) {
    switch (status) {
    case MeasurementStatus::kSuccess:
        return "success";
    case MeasurementStatus::kEmptyInput:
        return "empty input";
    case MeasurementStatus::kNonFiniteInput:
        return "non-finite input";
    case MeasurementStatus::kInvalidConfig:
        return "invalid configuration";
    case MeasurementStatus::kEmptyBaseline:
        return "empty baseline";
    case MeasurementStatus::kNonFiniteBaseline:
        return "non-finite baseline";
    case MeasurementStatus::kPlaneNotFound:
        return "baseline plane not found";
    case MeasurementStatus::kBaselineNoCells:
        return "no valid baseline cells";
    case MeasurementStatus::kFoodNotFound:
        return "food component not found";
    case MeasurementStatus::kInsufficientCoverage:
        return "insufficient grid coverage";
    case MeasurementStatus::kUnsupportedPlatform:
        return "unsupported platform";
    }
    return "unknown";
}
