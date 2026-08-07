# Repository Guidelines

## Project Structure & Module Organization

The SDK is organized by radio-stack function. `cuPHY/` contains CUDA-accelerated L1 code and GoogleTest-based tests under `cuPHY/test/`. Control-plane and integration services live in `cuPHY-CP/`, including the fronthaul driver, PHY controller, RU emulator, and test MAC. `cuMAC/` and `cuMAC-CP/` implement L2 scheduling and control. Python bindings, notebooks, and pytest suites are in `pyaerial/`; MATLAB reference models are in `5GModel/`. Use `testBenches/` for system/performance workflows, `testVectors/` for validation data, and `cubb_scripts/` for automation. Treat `build*` directories as generated output.

## Development Platform

The current development and validation platform is NVIDIA DGX Spark. Account for this platform when documenting setup steps, reproducing results, or reporting platform-specific issues.

## Build, Test, and Development Commands

Development is expected inside the Aerial container:

```bash
./cuPHY-CP/container/run_aerial.sh
./testBenches/phase4_test_scripts/build_aerial_sdk.sh
```

For a smaller Ninja build, use `cmake --preset minimal-x86` followed by `cmake --build --preset minimal-x86` (or the `minimal-arm` equivalents). These presets disable tests. For a test-enabled native build:

```bash
cmake -B build -GNinja -DCMAKE_TOOLCHAIN_FILE=cuPHY/cmake/toolchains/native -DENABLE_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

Run pyAerial checks with `pyaerial/scripts/run_static_tests.sh` and `pyaerial/scripts/run_unit_tests.sh`.

## Coding Style & Naming Conventions

CMake enforces C++20 and CUDA 17. C/CUDA code uses four spaces, no tabs, left-aligned pointers, and the nearest component’s `.clang-format` (for example, `cuPHY/.clang-format`). Run `clang-format -i <files>` on edited C/CUDA sources. Python static checks include flake8, pylint, mypy, and interrogate; documented Python APIs must maintain 100% docstring coverage. Follow surrounding naming conventions and name new tests `test_<feature>.py` or `test_<feature>.cpp`. Preserve existing SPDX headers.

## Testing Guidelines

Add C++/CUDA tests to the local `CMakeLists.txt` with GoogleTest and `add_test`; add Python tests under `pyaerial/tests/` using pytest. Keep tests focused on the changed component, then run the broader relevant suite. Pull Git LFS data before testing. pyAerial test vectors default to `/mnt/cicd_tvs/develop/GPU_test_input/`; override this with `TEST_VECTOR_DIR`.

## Commit & Pull Request Guidelines

Recent history uses concise, capitalized subjects such as `Append -cubb to install script version`; keep each commit scoped and use an imperative summary. The repository currently states that external contributions are not accepted. For authorized internal changes, include the motivation, affected modules, test commands/results, linked issue, and relevant GPU architecture or container configuration.

## Security

Never report vulnerabilities in public issues. Follow `SECURITY.md` and use NVIDIA’s private product-security reporting process.
