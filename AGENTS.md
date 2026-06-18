# Repository Guidelines

## Project Structure & Module Organization
`include/` contains public C99 headers, with most exported symbols using the `HYD_` prefix. `src/` holds core motion-control modules such as `motion_control.c`, `motion_planner.c`, and diagnostics helpers; simulator code is isolated under `src/sim/` and builds into `HydroSimLib`. `tests/` contains standalone C regression tests named `test_*.c`, a Python layout check, fixtures, and a small PLC demo under `tests/plcdemo/`. Build output belongs under `out/`; historical notes and retired docs live in `archive/`.

## Build, Test, and Development Commands
Configure and build the default toolchain with `cmake --preset unixgcc` and `cmake --build --preset unixgcc`. Run the full automated suite with `ctest --test-dir out/build/unixgcc --output-on-failure`. Run one target directly when iterating, for example `./out/build/unixgcc/test_motion_planner`. Generate coverage with `./scripts/coverage.sh` or `./scripts/coverage.sh --html`. Produce the embedded production package with `./scripts/deploy_embedded_prod.sh`. Performance work should use `./out/build/unixgcc/benchmark_performance`, which is intentionally not part of `ctest`.

## Coding Style & Naming Conventions
Use C99 and follow the existing style: 4-space indentation, opening braces on the same line, and `snake_case` filenames. Keep public APIs in `include/` mirrored by implementation files in `src/`. Prefer `static` for internal helpers and preserve the established `HYD_TypeName` / `HYD_FunctionName` naming pattern. No formatter is configured in-repo, so keep edits consistent with nearby code and avoid unrelated reformatting.

## Testing Guidelines
Add or update regression tests for every behavior change. New C tests should live in `tests/`, use `assert`, and be registered in `CMakeLists.txt` with both `add_executable(...)` and `add_test(...)`. Name files after the behavior under test, for example `test_fault_recovery.c`. When touching simulator or interface layout behavior, also check the relevant specialized tests such as `test_hydro_sim_fb` or `test_interface_layout_consistency.py`.

## Commit & Pull Request Guidelines
Recent history favors short, focused subjects that describe the behavioral change, in either English or Chinese, for example `reduce max_segment to 1` or `修改压力仿真模型`. Keep commits narrowly scoped and use imperative phrasing. PRs should explain the user-visible or control-behavior impact, list changed modules, include exact verification commands, and attach logs or screenshots only when outputs, diagnostics, or tooling UX changed.
The remote repository is hosted on gitee.com. 
Username: liurundan
Password: bajx2019