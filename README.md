# food3d_im

Built with Conan 2 and CMake. Targets C++17, with optional C++20/23 features such as modules.

## Badges

![License](https://img.shields.io/github/license/HeT-FTI/food3d_im?color=blue&label=license)
![CodeFactor Grade](https://img.shields.io/codefactor/grade/github/HeT-FTI/food3d_im?label=code%20quality&logo=codefactor)
![C++](https://img.shields.io/badge/C%2B%2B-17%2F20%2F23-blue?logo=c%2B%2B&logoColor=white)
![Modules](https://img.shields.io/badge/modules-C%2B%2B23%20experimental-purple?logo=c%2B%2B&logoColor=white)
![CI](https://github.com/HeT-FTI/food3d_im/actions/workflows/ci-build-test.yml/badge.svg)
![CI](https://github.com/HeT-FTI/food3d_im/actions/workflows/docs-build.yml/badge.svg)
![CMake](https://img.shields.io/badge/cmake-3.28%2B-orange?logo=cmake)
![Python](https://img.shields.io/badge/python-3.10%2B-blue?logo=python)
![Doxygen](https://img.shields.io/badge/docs-Doxygen-green?logo=doxygen&logoColor=white)
![Sphinx](https://img.shields.io/badge/docs-Sphinx-blue?logo=readthedocs&logoColor=white)

## Project Overview

- **Language**: C/C++
- **Build**: Conan 2 + CMake
- **Docs**: Doxygen, Graphviz, Sphinx
- **Modules**: optional, enabled from C++23 up
- **Metadata-Driven**: `metadata.json` holds the build, dependency, docs and CI settings
- **Skills & Agent**: library-level `het-*` skills + routing agent (`.github/skills/`)
- **Layout**:
    - headers and sources are named in pairs
    - the suffix tells the languages apart: (.h, .c) for C, (.hpp, .cpp) for C++
    - docs use .dox for doc-only pages and .cxx for example code

## Library: IM Food Volume Measurement

`food3d_im` measures food volume in cubic centimetres from a fixed oven-tray depth camera.
The IM (baseline-plane height-difference integral) pipeline is:

1. Build an empty-oven **baseline height map**: fit the tray plane from the first empty frame,
   project every empty frame into the same local `(u, v)` frame, and take the per-cell median height.
2. Remove the dominant background plane(s) from the food frame, then filter food points by their
   baseline-relative height difference.
3. Cluster the surviving points in **baseline-plane (u, v) coordinates** to separate multiple food items.
   With `enable_height_watershed` enabled (opt-in, off by default) a cluster whose top surface dips into
   a **height valley** between two peaks is split further, so two nearly touching items are reported separately.
4. Keep one top-surface point per cell and integrate
   `volume = Σ (food_top − baseline_height) × cell_size²`.
5. Conservatively complete enclosed depth holes **inside a single component** with inverse-distance
   interpolation (small, smooth holes) or a guarded quadratic surface fit (larger specular holes).
   Measured and interpolated cells are reported separately.

```cpp
FoodVolumeMeasurer measurer;          // defaults match the IM reference
measurer.set_baseline({empty_oven_frame}) // one or more empty-oven frames
    .set_food(food_frame)                 // the food frame
    .set_input_unit(LengthUnit::kMillimeter); // optional: inputs in mm
VolumeEstimate est = measurer.run();
// est.volume_cm3, est.raw_volume_cm3, est.interpolated_volume_cm3, est.component_count, ...
```

- All geometry is in metres internally; set the input unit when inputs are millimetres.
- At least one baseline frame is required; a baseline cell that is missing from the empty-oven map
  is never replaced with an ideal zero plane.
- Hole filling is intentionally conservative: only holes whose boundary belongs to exactly one food
  component are considered, and several area / rim / fit-safety guards must pass.

### Configuring the measurement

`FoodVolumeMeasurer` has chainable setters for the common parameters, `set_config` for a full
`MeasurementConfig`, and JSON load/save for its configurable fields. JSON files may contain only
the values to override; the hole-completion safety guards remain internal constants:

```cpp
FoodVolumeMeasurer measurer;
measurer.set_voxel_size(0.003)                       // downsampling voxel edge (m); 0 disables downsampling
    .set_integration_resolution(0.003)               // raster cell edge (m)
    .set_height_range(0.0015, 0.05)                  // accepted food height (m); max <= 0 disables the cap
    .set_cluster_params(0.010, 8)                    // footprint DBSCAN radius + min points
    .set_plane_distance_threshold(0.003)             // background-plane RANSAC threshold (m)
    .set_roi(roi)                                    // optional axis-aligned crop before downsampling
    .load_config_from_json("measurement_config.json"); // or set everything from JSON
```

Implementation-only details (plane-local frames, grid keys, ROI bounds, height maps, hole
candidates, baseline rasters) stay private to `src/` and are never part of the installed
headers.

`include/pointcloudprocess.hpp` additionally exposes the fine-grained point-cloud
operators the pipeline is built from — unit scaling, axis-aligned cropping, RANSAC plane
fitting and orientation, voxel downsampling, DBSCAN labelling, background-plane removal, and
the AABB / OBB / convex-hull reference volumes — so they can be reused and tested on their own.

An ROI object enables cropping; an empty `selected_labels` selects all eligible food components.
The secondary-plane removal uses `plane_distance_threshold_m`. Setting `middle_cloud_dir`
enables stage PCD output; an empty path disables it. `5_top_surface.pcd` contains the measured
top-surface cells before hole completion, and `6_hole_filled_surface.pcd` contains the full top
surface after completion. Both use world coordinates.

## Features

- Dependency management with Conan
- Supports C++ standards 17, 20, and 23
- Automatic module file generation (`.ixx`/`.cppm`) from headers/sources
- Builds on Windows, Linux and macOS
- Dual C and C++ interfaces with separate linkage targets
- Doxygen annotation support for object exporting (`@exporter`, `@attacher`)
- Automated docs via Doxygen and Sphinx
- Import, derivation and call graphs via Graphviz
- `metadata.json` as the one place holding build / deps / docs / CI settings
- Library-level VS Code skills + routing agent (`.github/skills/`, `/het-*` slash commands)
- Metadata + gitmoji driven CI/CD orchestration (GitHub Actions)
- Cross-compilation & on-board benchmark framework (`benchmark/`)
- Agentic Coding workspace (`/workspace/`, git-ignored)

## Agentic Coding Workspace

This repository reserves `/workspace/` (declared in `.gitignore`) for agent-generated work products produced
during Agentic Coding sessions — e.g., implementation plans, test contracts, audit reports, and other
intermediate artifacts. Keep such working files under `/workspace/` so they never pollute the tracked
source tree.

## Skills & Agent (VS Code)

This repository ships a library-level skills system under `.github/skills/` — **15 `het-*` skills plus one
routing agent**, all invocable from Copilot Chat by typing `/`:

| Family | Skills | Audience |
|--------|--------|----------|
| IaC usage (S0–S6) | `het-guide`, `het-build`, `het-release`, `het-docs`, `het-quality`, `het-board`, `het-fix-ci` | CI/CD novices |
| Dev automation (N1–N8) | `het-deps`, `het-module`, `het-setup`, `het-testgen`, `het-commit`, `het-audit`, `het-preflight`, `het-patent` | Developers |
| Routing agent (A1) | `het-agent` | Everyone |

Examples: `/het-guide` for onboarding, `/het-testgen` to generate tests, `/het-agent` for natural-language
composite tasks (e.g. *"add a module, test it, and commit"*). See `.github/skills/README.md` and
`PLAN-skills.md` for the full execution plan.

## Build Requirements

- Python 3.10+ (Conan tooling)
- Conan 2.0+
- A C/C++ compiler:
    - GCC
    - Clang
    - MSVC
- CMake (auto-installed by Conan)

## Documenting Requirements

- Doxygen
- Graphviz
- sphinx
- sphinx-intl
- sphinx-rtd-theme

## Test Requirements

- GTest

## CI/CD (metadata-driven)

All GitHub Actions workflows are orchestrated from `metadata.json` (entry: `ci-orchestrator.yml`, decision:
`metadata-controller.yml`). Each pipeline is gated by a `workflow_triggers.*` switch and triggered by a
gitmoji in the commit message. The canonical commit form is `<type>(<emoji>): <description>` — the emoji
sits in the parentheses right after the commit word (e.g. `feat(:fire:): ...`, `chore(:package:): ...`);
as a **soft rule** the emoji also triggers from anywhere in the message:

| Pipeline | gitmoji in commit message | Gate (`metadata.json`) |
|----------|---------------------------|------------------------|
| Build | `:building_construction:` | `workflow_triggers.build` |
| Tests + coverage | `:beer:` | `trigger_tests` / `activate_code_coverage` (needs `build_type=Debug`) |
| Release | `(:package:):` | `workflow_triggers.release` (needs `build_type=Release`) |
| Docs | `:book:` | `workflow_triggers.docs` |
| Security / lint | `:shield:` | `workflow_triggers.security_scan` (also runs on every PR) |
| Board cross-build | `:fire:` (or `🔥`) | hetai self-hosted runner |

> **Note**: `workflow_triggers.build` / `.tests` / `.security_scan` are enabled by default
> (commit-lint & schema gates always run on push/PR; build/tests/security shift-left on PRs).
> `release` and `docs` require both the gitmoji and the switch (`build_type` must match too).

## Build Cheat Sheet

### 1. Build the package and run the consumer check

From the repository root:

```bash
conan create . -s build_type=Debug --build=missing -c tools.build:jobs=4
```

The full CTest suite runs only when `trigger_tests` is enabled in `metadata.json`.

cross-build to host device (assume toolchain and profile are ready):

```bash
conan create . -pr:b=default -pr:h=arm_profile -s build_type=Debug --build=missing \
    -c tools.build:jobs=4 -tf=""
```

### 2. Build documentations

```bash
python ./docs/build.py
```

### 3. One-lined build automation 

Unix-like platforms (Linux, MacOS):

```bash
bash ./build
```

Windows:

```powershell
Get-Content "build" | Invoke-Expression
```

### 4. Add requirements

Add your required libraries in *conandata.yml* where dependency graph is automatically computed from, then 
modify the **dependencies** field in *metadata.json* to config proper package names and associated targets to 
link (no need modification on *CMakeLists.txt*).

Requirements for your project can be the package archived on [Conan Center](https://conan.io/center), or user 
built ones. If the later one, at least you need a locale Conan server for managing your libraries.

### 5. Run MegaLinter locally (advisory SAST)

MegaLinter mirrors the CI `megalinter` job (`.github/workflows/security-linters.yml`): same config
(`.github/misc/.mega-linter.yml`), same full image, and the same version (`v8.8.0`, pinned to match the
CI action). It runs the **advisory** SAST layer — gitleaks / semgrep / checkov / devskim, plus clang-format
& cppcheck. The **required** gates (clang-format / clang-tidy / gitleaks with pinned tool versions) are the
native `quality-gates` job of the same workflow.

Prerequisites: Docker (or Podman) + Node.js ≥ 20.

```bash
bash .github/misc/run-megalinter.sh             # lint the whole codebase
bash .github/misc/run-megalinter.sh --fix       # auto-apply fixes (e.g. clang-format)
bash .github/misc/run-megalinter.sh src/foo.c   # lint selected files only
CONTAINER_ENGINE=podman bash .github/misc/run-megalinter.sh   # Podman users
```

- The first run pulls the `v8.8.0` image (a few GB — cached on later runs).
- Reports are written to `megalinter-reports/` (git-ignored); per-linter details are in
  `megalinter-reports/linters_logs/ERROR-*.log`.
- The runner (`mega-linter-runner`) is a devDependency — `npm ci` installs it, or it is fetched
  automatically on first use.

## All-in-one Project Structure

```
project-root/
├── conanfile.py              # Conan recipe
├── CMakeLists.txt            # CMake build framework
├── metadata.json             # Package identity and all build/deps/docs switches
├── conandata.yml             # Dependency specifications, Conan plugin support
├── package.json              # Node-side tooling deps (commitlint / semantic-release / ajv)
├── CHANGELOG.md              # Auto-generated by semantic-release (init tag required)
├── build                     # One-liner local build script
├── LICENSE                   # Apache v2 Project license
├── NOTICE                    # Notice file of Apache v2
├── .vscode/                  # Recommended editor config (extensions / settings / tasks)
├── .devcontainer/            # Reproducible dev environment (Dockerfile)
├── .github/                  # CI/CD + library-level skills
│   ├── workflows/            # ci-orchestrator / metadata-controller / build / test / docs / release / security
│   ├── skills/               # 16 het-* skills + het-agent + _shared + manifest.json
│   ├── misc/                 # clang-format(-c/cpp) / clang-tidy / gitleaks / releaserc / commitlint /
│   │                         # metadata.schema / mega-linter / pre-commit / labels / Codegen-Starter.txt
│   ├── CODEOWNERS            # Required reviewers
│   ├── CONTRIBUTING.md       # Contribution & commit conventions
│   ├── dependabot.yml        # Grouped monthly dependency updates
│   └── PULL_REQUEST_TEMPLATE.md
├── api/                      # Interface to advanced programming language
│    └── python_bindings.cpp  # Python bindings interface
├── include/                  # Public headers
│   ├── *.h                   # C interface headers
│   └── *.hpp                 # C++ interface headers
├── src/                      # Implementation files
│   ├── *.c                   # C sources
│   ├── *.cpp                 # C++ sources
│   └── *.ixx/*.cppm          # Auto-generated Module files (in experimental)
├── benchmark/                # Cross-compile & on-board benchmark framework (Cortex-M / Cortex-A)
├── .hetai/                   # hetai package matrix (cross-compile targets)
├── wokspace/                 # Agentic Coding work products (git-ignored)
├── docs/                     # Documentations root
│   ├── doxygen/              # Doxygen system main root
│   │   ├── dox/              # Pure documentations' folder
│   │   │   ├── demos/        # Examples catalogue
│   │   │   │   ├── *.dox     # Documenting docstring
│   │   │   │   └── *.cxx     # Example codes
│   │   │   └── *.dox         # Main pages and etc
│   │   └── ...
│   ├── sphinx/               # Sphinx system main root
│   │   ├── source/           # Source files of sphinx system
│   │   ├── locales/          # Pot files for internalization
│   │   └── ...
│   └── images/               # Static images for doxygen/sphinx system
└── test_package/             # Test project
    ├── export/               # Log for testing results
    ├── resources/            # Test resources for test_package/ programs
    ├── stress/
    │   └── *.cpp             # Scripts for stress testing
    ├── unit/
    │   └── *.cpp             # Scripts for unit testing
    ├── main.cpp              # Validation program for package
    ├── conanfile.py          # Conan recipe for test_package
    └── CMakeLists.txt        # CMake build workflow for test_package
```

## Module Generation (experimental)

When `generate_modules_inplace` is enabled in `metadata.json`:

1. Header/source pairs automatically generate module files
2. `#include` directives are converted to `import` statements
3. Doxygen annotations control symbol visibility:
    - `@exporter`: Exports symbols in modules
    - `@attacher`: Attaches symbols to modules

This feature is experimental now, however, the specific syntax can make the existing project a ease 
migration to fit the future C++ standard.

## Compiler Support Matrix

| Feature          | MSVC | Clang | GCC | Apple-Clang |
|------------------|------|-------|-----|-------------|
| C++ Modules      | ✓    | ✓     | ✓   | ✓           |
| C Compatibility  | ✓    | ✓     | ✓   | ✓           |
| Automatic Export | ✓    | ✓     | ✓   | ✓           |

## Platforms Support

- **Desktop**: Windows, Linux, MacOS
- **Mobile**: arm-linux, risc-v

## Benchmark (on-board)

`benchmark/` cross-compiles the library to real hardware and measures performance on-board:

- **Cortex-M (baremetal)**: flash via JLink / OpenOCD / PyOCD, timing via SYSTICK
- **Cortex-A (Linux)**: deploy via ADB / SSH, timing via `clock_gettime`

Edit `benchmark/bench_config.json` (board parameters) and `benchmark/bench_entry.c` (algorithm cases),
then run `python benchmark/script/run_bench.py` (add `--no-flash` to build only). Results follow the
`BENCHMARK_START / RESULT|name|cycles / BENCHMARK_END` protocol. See `benchmark/README.md`.

> Role split: `test_package/` runs **host-side** GTest verification (desktop/x86_64, even when the library
> is cross-compiled to baremetal), while `benchmark/` runs **target-side** on-board verification.

## Commit Convention & Versioning

Commit style uses the [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/)
specification, extended with the template's private **emoji superset** (see the CI/CD table above) so a
single message both bumps the version and triggers the right pipeline.

**Canonical commit form** — the emoji is placed in the parentheses right after the commit word:

```
<type>(<emoji>): <description>
```

Examples: `feat(:fire:): cross-compile support`, `test(:beer:): vector add cases`,
`chore(:package:): prepare release`. As a soft rule, the emoji anywhere in the message also triggers the
pipeline, but the parenthesized placement is the recommended convention.

Versioning is driven by **semantic-release** (`semver-release.yml` + `.github/misc/.releaserc.json`): the commit prefix
decides the jump — `feat` → minor, `fix`/`perf` → patch, `BREAKING CHANGE`/`!` → major. A release
generates `CHANGELOG.md` and rewrites the `version` in `metadata.json`. (The legacy
commit-base-versioning mechanism has been removed.)

## To Do Things

Possible frame design/validation on Apple Clang compiler (raised from dlib requirement).

## License

[Apache-2.0] - See included LICENSE file for details.
