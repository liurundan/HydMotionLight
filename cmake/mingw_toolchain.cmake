# cmake/mingw_toolchain.cmake
# MinGW-w64 (Windows) 工具链：使用 D:/mingw64/bin 下的 gcc/g++ 编译本项目。
# 对应 CMakePresets.json 中的 "mingw" preset。

set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR x86_64)

# 使用绝对路径指定编译器，避免与系统其它 gcc/g++ 冲突
set(CMAKE_C_COMPILER   "D:/mingw64/bin/gcc.exe"   CACHE PATH "C compiler")
set(CMAKE_CXX_COMPILER "D:/mingw64/bin/g++.exe"   CACHE PATH "C++ compiler")
set(CMAKE_RC_COMPILER  "D:/mingw64/bin/windres.exe" CACHE PATH "RC compiler")

# 编译/调试选项与 unixgcc 保持一致（-g 调试信息，-O0 关闭优化，-Wall 告警）
set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type")
set(CMAKE_C_FLAGS   "-g -O0 -Wall" CACHE STRING "C compile flags")
set(CMAKE_CXX_FLAGS "-g -O0 -Wall" CACHE STRING "C++ compile flags")

# 生成控制台程序（非 GUI 窗口），与 Linux 下行为保持一致
set(CMAKE_WIN32_EXECUTABLE OFF)

# 在主机环境中查找构建工具（mingw32-make 等）
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# 头文件与库在目标（MinGW）环境中查找
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
