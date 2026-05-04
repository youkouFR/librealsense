# AGENTS.md — librealsense

## Project

Intel RealSense SDK 2.0 — C++ cross-platform library (`realsense2`) for depth cameras (D400/D500 series). Public API is C; C++ wrapper provided. Wrappers for Python, C#, ROS 2, Unity, OpenCV, PCL, and more.

**Upstream:** `https://github.com/realsenseai/librealsense` (migrated from IntelRealSense org)

## Branch Policy

- `master` = stable (includes beta releases)
- `development` = integration branch; **all PRs target this branch**
- Feature branches branch off `development`

## Build

CMake-based. Out-of-source build required.

```bash
mkdir build && cd build
cmake .. [-D<OPTIONS>]
cmake --build . -- -j$(nproc)   # Linux/macOS
cmake --build . --config Release -- -m  # Windows (VS generator)
```

### Key CMake Options (see `CMake/lrs_options.cmake`)

| Flag | Default | Notes |
|---|---|---|
| `BUILD_SHARED_LIBS` | ON | Shared vs static library |
| `BUILD_EXAMPLES` | ON | Non-graphical examples |
| `BUILD_GRAPHICAL_EXAMPLES` | ON | Viewer & DQT (implies `BUILD_GLSL_EXTENSIONS`) |
| `BUILD_UNIT_TESTS` | OFF | C++ unit test executables |
| `BUILD_PYTHON_BINDINGS` | OFF | pyrealsense2 (pybind11) |
| `BUILD_TOOLS` | ON | realsense-viewer, fw-updater, etc. |
| `FORCE_RSUSB_BACKEND` | OFF | Required for Win7/macOS/Android; on Linux CI (GHA) needed when no V4L2 available |
| `BUILD_WITH_DDS` | OFF | Requires CMake >= 3.16.3 |
| `BUILD_ASAN` | OFF | AddressSanitizer (hidden option) |
| `ENABLE_SECURITY_FLAGS` | OFF | Extra compiler security flags |
| `UNIT_TESTS_ARGS` | "" | Args forwarded to `unit-test-config.py`, e.g. `"--not-live --context=linux"` |

### Linux Dependencies (for CI reference)

```bash
sudo apt-get install build-essential libusb-1.0-0-dev libgtk-3-dev libglfw3-dev \
  xorg-dev libgl1-mesa-dev libglu1-mesa-dev libglew-dev libglm-dev
```

### ccache

Enabled by default (`ENABLE_CCACHE=ON`). Speeds up rebuilds.

## Testing

Two-tier test system:

1. **C++ unit tests** (Catch2) — built when `BUILD_UNIT_TESTS=ON`, compiled as `test-*` executables
2. **Python test orchestrator** — `unit-tests/run-unit-tests.py` discovers and runs both C++ exes and Python test scripts

### Running tests (no hardware)

```bash
# Build with tests
cmake .. -DBUILD_UNIT_TESTS=ON -DBUILD_PYTHON_BINDINGS=ON \
  -DUNIT_TESTS_ARGS="--not-live --context=linux"
cmake --build . -- -j$(nproc)

# Run tests
python3 unit-tests/run-unit-tests.py --not-live --context "linux"
```

### Running tests (with hardware)

```bash
python3 unit-tests/run-unit-tests.py --context "linux"
# Use --live for device-required tests, --not-live for offline tests
# Use -t <tag> to filter by tag, -r <regex> to filter by name
```

### Test framework details

- Tests live in `unit-tests/` subdirs: `func/`, `log/`, `types/`, `algo/`, `live/`, `dds/`, etc.
- Test source files are named `test-*.cpp` or `test-*.py`
- Tests use `test:device` directives to declare hardware requirements
- `--context` controls which tests run (e.g., `linux`, `windows`, `dds`, `gha`)
- `--not-live` skips tests requiring physical cameras
- Python test support modules are in `unit-tests/py/rspy/`

## Pre-PR Checks

Run these before submitting a PR:

```bash
./scripts/api_check.sh    # Verifies every API header compiles in isolation (C++11)
./scripts/pr_check.sh     # Checks copyright, license, tabs, line endings
./scripts/pr_check.sh --fix  # Auto-fixes copyright/line-ending issues
```

CI runs `api_check.sh` on every job and `pr_check.sh` on Linux jobs.

## Architecture

```
include/librealsense2/   → Public C/C++ API (rs.h, rs.hpp) — C++11 compatible
src/                     → Core library implementation — C++14
  core/                  → Core types, context, device, sensor
  ds/                    → Depth camera device code (D400/D500)
  proc/                  → Processing blocks (align, pointcloud, etc.)
  pipeline/              → High-level pipeline API
  platform/              → OS abstraction layer
  linux/ uvc/ usb/ hid/  → Linux-specific backends
  win/ mf/               → Windows-specific backends
  proc/sse/ proc/neon/ proc/cuda/ → SIMD-optimized processing
common/                  → Shared UI code (viewer, tools)
examples/                → Sample applications
tools/                   → Utilities (realsense-viewer, fw-updater, enumerate-devices, etc.)
wrappers/                → Language bindings (python, csharp, unity, opencv, pcl, etc.)
unit-tests/              → Test framework and test cases
third-party/             → Vendored deps (rsutils, realsense-file, glfw, json, etc.)
CMake/                   → Build configuration modules
```

### Key classes

- `rs2::pipeline` — high-level streaming API
- `rs2::context` — device discovery and management
- `rs2::device` / `rs2::sensor` — hardware abstractions
- `rs2::frame` / `rs2::frameset` — frame data
- `librealsense::context` (internal) — implementation in `src/context.cpp`

### Threading

- Context/device/sensor objects are thread-safe
- Frame callbacks run on internal streaming threads — keep them fast
- One streaming thread per active sensor

## Naming Conventions

- Files: kebab-case (`backend-v4l2.cpp`)
- Classes/functions: snake_case (`uvc_device`, `get_device_count()`)
- Constants: UPPER_CASE (`RS2_CAMERA_INFO_NAME`)
- Public C API enums: `rs2_*` prefix (`rs2_format`, `rs2_stream`)
- Interfaces: `*_interface` suffix (`device_interface`)
- Namespaces: `librealsense` (main), `librealsense::platform` (platform layer)

## Important Gotchas

- **C++ standard**: Core compiles as C++14; public API requires only C++11
- **CMake min version**: 3.10 (3.16.3 if `BUILD_WITH_DDS=ON`)
- **No formatter enforced**: Follow surrounding file style
- **Enum changes**: When adding values to `rs2_option` or similar enums, update all wrappers (Matlab, C#, Python, Android, Unreal) — see CONTRIBUTING.md
- **GHA Linux CI**: Cannot access `/sys/class/video4linux`, so `FORCE_RSUSB_BACKEND=ON` is needed for DDS tests
- **Windows CI**: Builds in `C:/lrs_build` due to disk space constraints on D: drive
- **Python bindings**: `pyrealsense2` and `pyrealsense2-beta` are separate PyPI packages; both import as `pyrealsense2`
- **Firmware**: `IMPORT_DEPTH_CAM_FW=ON` (default) downloads firmware from cloud; requires internet
- **rsutils** (`third-party/rsutils/`) is a foundational utility library, publicly linked into `realsense2`

## CI

GitHub Actions (` .github/workflows/buildsCI.yaml`) builds across:
- Windows 2025 (shared/static, Python, C#, DDS)
- Ubuntu 22.04/24.04 (shared/static, Python, DDS)
- macOS 15 (Clang, DDS)
- Android (NDK cross-compile)

Each job runs `api_check.sh`; Linux jobs also run `pr_check.sh`.
