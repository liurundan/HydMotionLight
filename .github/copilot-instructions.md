# Project Guidelines

## Code Style
Use C99 standard with `HDY_` prefix for all symbols. Follow PLCopen function block patterns with input/output structs. Reference [include/common_types.h](include/common_types.h) for type definitions and [include/motion_control.h](include/motion_control.h) for API examples.

## Architecture
PLCopen-inspired function block design separating process layer (recipe management, segment switching) from motion control layer (pure planning calculations). Motion control acts as a calculator for pump commands, not hardware control. See [项目需求与设计说明书.md](项目需求与设计说明书.md) for detailed architecture and component boundaries.

## Build and Test
- Configure: `cmake --preset unixgcc`
- Build: `cmake --build out/build/unixgcc`
- Test: `ctest` in build directory
Agents will run these automatically. See [README.md](README.md) for simple build examples.

## Conventions
Use static arrays for fixed-size data, clamps for safety limits, and diagnostic structs for error reporting. No dynamic allocation or exceptions. Differ from common C++ by using pure C with typedefs and enums.