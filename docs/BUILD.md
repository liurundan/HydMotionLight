# 构建配置说明

本项目使用CMake和CMakePresets.json进行跨平台构建管理。

## 构建预设

### Windows (MinGW-w64)
```bash
# 配置
cmake --preset mingw-w64

# 编译
cmake --build out/build/mingw-w64

# 运行测试
ctest --test-dir out/build/mingw-w64 --output-on-failure
```

**编译器路径**: `D:\mingw64\bin\gcc.exe`  
**Make工具**: `D:\mingw64\bin\mingw32-make.exe`  
**工具链文件**: `cmake/mingw_w64_toolchain.cmake`

### Linux (Unix GCC)
```bash
# 配置
cmake --preset unixgcc

# 编译
cmake --build out/build/unixgcc

# 运行测试
ctest --test-dir out/build/unixgcc --output-on-failure
```

### 代码覆盖率 (Linux)
```bash
cmake --preset coverage
cmake --build out/build/coverage
ctest --test-dir out/build/coverage
```

## Claude Code配置

`.claude/settings.local.json`已配置构建命令权限：
- `cmake --preset *` - 配置项目
- `cmake --build *` - 编译项目
- `ctest *` - 运行测试
- `D:/mingw64/bin/gcc *` - 直接调用编译器
- `D:/mingw64/bin/mingw32-make *` - 直接调用make

## 快速开始

### Windows开发
```bash
# 1. 清理并重新配置
rm -rf out/build/mingw-w64
cmake --preset mingw-w64

# 2. 编译
cmake --build out/build/mingw-w64

# 3. 运行RBF-PID相关测试
ctest --test-dir out/build/mingw-w64 -R "rbf|pressure" --output-on-failure
```

### Linux开发
```bash
# 1. 清理并重新配置
rm -rf out/build/unixgcc
cmake --preset unixgcc

# 2. 编译
cmake --build out/build/unixgcc

# 3. 运行所有测试
ctest --test-dir out/build/unixgcc --output-on-failure
```

## 常见问题

### Q: CMake找不到编译器
**A**: 确保MinGW-w64已安装在`D:\mingw64\bin`，或修改`cmake/mingw_w64_toolchain.cmake`中的路径。

### Q: 编译错误 "generator mismatch"
**A**: 清理构建目录：
```bash
rm -rf out/build/mingw-w64/CMakeCache.txt out/build/mingw-w64/CMakeFiles
```

### Q: 测试找不到可执行文件
**A**: 先完整编译项目：
```bash
cmake --build out/build/mingw-w64 --target all
```

## CMakeLists.txt说明

核心库源文件使用GLOB_RECURSE自动扫描`src/*.c`：
```cmake
file(GLOB_RECURSE HYDRO_CORE_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
)
```

新增源文件（如`src/rbf_preset_configs.c`）会自动包含，无需手动修改CMakeLists.txt。

## 编译输出

- 静态库: `out/build/*/libHydroMotionLib.a`
- 测试可执行文件: `out/build/*/test_*.exe`
- 主程序: `out/build/*/main.exe`
