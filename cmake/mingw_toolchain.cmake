# toolchain-mingw.cmake
set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR x86_64)
# 指定编译器路径（本机 MinGW64）
set(CMAKE_C_COMPILER "D:/mingw64/bin/gcc.exe" CACHE PATH "C compiler")
set(CMAKE_CXX_COMPILER "D:/mingw64/bin/g++.exe" CACHE PATH "C++ compiler")

# 核心：添加调试信息（-g）+ 关闭优化（-O0，避免调试时代码乱序）
set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type")
set(CMAKE_C_FLAGS "-g -O0 -Wall" CACHE STRING "C compile flags")
set(CMAKE_CXX_FLAGS "-g -O0 -Wall" CACHE STRING "C++ compile flags")

set(TARGET_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/include        # 项目自定义头文件
    CACHE INTERNAL "Target include directories"
)
