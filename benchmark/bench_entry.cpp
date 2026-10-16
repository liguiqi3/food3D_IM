/**
 * bench_entry.c  –  Algorithm Benchmark Entry
 *
 * ============================================================
 *  HOW TO USE (algorithm engineer)
 * ============================================================
 *  1. Set MODULE_NAME to identify this build (e.g. "MyAlgo-v2.0").
 *  2. Include your algorithm header(s) below.
 *  3. Add static wrapper functions (one per test item).
 *  4. Add entries to bench_table[].
 *  5. Build with:  conan build . -pr:h profiles/<target>.profile
 *
 *  Everything else (linker script, descriptor, RAM init, timing)
 *  is handled automatically – do NOT modify below the separator.
 * ============================================================
 */
#include <stdint.h>
#include <cmath>
#include "core/het_bench_core.h"
#include "pointcloudprocess.hpp"


/* ============================================================
 * USER ZONE (algorithm engineer edits only this block)
 * ============================================================ */

constexpr const char *MODULE_NAME = "food3d_im";
constexpr double kReferenceWidthM = 0.095;
constexpr double kReferenceDepthM = 0.095;
constexpr double kReferenceHeightM = 0.020;
constexpr double kReferenceCellM = 0.005;
constexpr double kReferenceVolumeM3 =
    kReferenceWidthM * kReferenceDepthM * kReferenceHeightM;

namespace {

PointCloud &reference_food_cloud()
{
    static PointCloud cloud;
    return cloud;
}

}  // namespace

static void bench_prepare_input(void)
{
    PointCloud &cloud = reference_food_cloud();
    cloud.points.clear();
    cloud.points.reserve(800U);

    // A closed 9.5 cm × 9.5 cm × 2 cm prism is deterministic and does not
    // require test PCD assets to be present in the transferred bundle.
    for (double x = 0.0; x <= kReferenceWidthM + 1.0e-9; x += kReferenceCellM) {
        for (double y = 0.0; y <= kReferenceDepthM + 1.0e-9; y += kReferenceCellM) {
            cloud.points.push_back(
                {static_cast<float>(x), static_cast<float>(y), 0.0F});
            cloud.points.push_back({
                static_cast<float>(x),
                static_cast<float>(y),
                static_cast<float>(kReferenceHeightM),
            });
        }
    }
}

static int bench_aabb_volume_case(const void * const ctx)
{
    (void)ctx;
    const double volume = compute_aabb_volume(reference_food_cloud());
    return std::isfinite(volume) &&
           std::fabs(volume - kReferenceVolumeM3) <= 1.0e-8;
}

static int bench_convex_hull_volume_case(const void * const ctx)
{
    (void)ctx;
    const double volume = compute_convex_hull_volume(reference_food_cloud());
    return std::isfinite(volume) &&
           std::fabs(volume - kReferenceVolumeM3) <= 1.0e-6;
}

static const Case bench_table[] = {
    BENCHMARK_CASE_IMPLEMENTATION("aabb_prism_volume", nullptr,
                                  bench_aabb_volume_case, 100U),
    BENCHMARK_CASE_IMPLEMENTATION("convex_hull_prism_volume", nullptr,
                                  bench_convex_hull_volume_case, 20U),
};


BENCHMARK_IMPLEMENTATION(MODULE_NAME, bench_table);
