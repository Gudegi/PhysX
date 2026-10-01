# NVIDIA PhysX SDK for macOS

This port supports PhysX 5.8 CPU simulation on Apple Silicon using static libraries.
The validated build configurations are `checked` and `release`.

## Location of Binaries:

* SDK libraries: `build/macos-arm64-<config>/bin/mac.arm64/<config>`
* Platform test: `build/macos-arm64-<config>/platform_tests/physx_platform_smoke`

Paths are relative to the repository root and use the build settings below.

## Required packages to generate projects:

* macOS on Apple Silicon (ARM64)
* Xcode command-line tools
* CMake, minimum version 3.16
* Ninja

### Compilers and C++ Standard:

* Tested with AppleClang 21.0.0.21000099 and macOS SDK 26.4 on macOS 26.6.2.
* Tested with CMake 4.0.1 and Ninja 1.12.1.
* Uses C++14 and ARM NEON intrinsics.
* The deployment target is not explicitly overridden by these commands;
  compatibility with older macOS versions has not been validated.

## Generating Ninja projects:

This macOS build uses CMake and Ninja directly; Python, Packman and
`generate_projects.sh` are not required.

Run CMake from the repository root. Choose `checked` or `release`:

```sh
physx_config=release
physx_build="$PWD/build/macos-arm64-$physx_config"
cmake -S physx/compiler/public -B "$physx_build" -G Ninja \
  -DPHYSX_ROOT_DIR="$PWD/physx" \
  -DTARGET_BUILD_PLATFORM=mac \
  -DCMAKE_OSX_ARCHITECTURES=arm64 \
  -DCMAKE_BUILD_TYPE="$physx_config" \
  -DPX_GENERATE_STATIC_LIBRARIES=ON \
  -DPX_GENERATE_GPU_PROJECTS=OFF \
  -DPX_GENERATE_GPU_PROJECTS_ONLY=OFF \
  -DPX_BUILDSNIPPETS=OFF \
  -DPX_BUILDPVDRUNTIME=OFF \
  -DPX_BUILD_PLATFORM_TESTS=ON \
  -DPX_OUTPUT_LIB_DIR="$physx_build" \
  -DPX_OUTPUT_BIN_DIR="$physx_build" \
  -DCMAKE_INSTALL_PREFIX="$PWD/install/macos-arm64-$physx_config"
```

## Building SDK:

From the same shell:

```sh
cmake --build "$physx_build" -j 6
```

* Clean project: `cmake --build "$physx_build" --target clean`
* Installation packaging has not been validated. Use the source headers and
  matching build libraries together.
* CMake generates `physx/include/PxConfig.h` in the source tree. Configure and
  build sequentially for these identical static CPU options. Use separate source
  worktrees for incompatible build options or SDK versions.

## Running Platform Tests:

```sh
ctest --test-dir "$physx_build" --output-on-failure
```

The headless test covers NEON vector splats, PGS/TGS simulation, articulation
static loads and joint moments, external force reactions, freefall, revolute
drive tracking, and sphere-ground contact. Checks remain active in release builds.
This test does not cover the full SDK feature set.

## Supported Features and Limitations:

* This build configuration supports native ARM64 CPU static libraries.
* Intel Mac, universal binaries, shared libraries, GPU acceleration, OmniPVD
  runtime and rendering snippets are outside this port's initial scope.
* Builds may emit upstream compiler and static-library linker warnings.
