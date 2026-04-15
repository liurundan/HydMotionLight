# toolchain-mingw.cmake
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR x86_64)
# 指定编译器路径（替换为你的实际路径）
set(CMAKE_C_COMPILER "/usr/bin/gcc" CACHE PATH "C compiler")
set(CMAKE_CXX_COMPILER "/usr/bin/g++" CACHE PATH "C++ compiler")

# 核心：添加调试信息（-g）+ 关闭优化（-O0，避免调试时代码乱序）
set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type")
set(CMAKE_C_FLAGS "-g -O0 -Wall" CACHE STRING  "C compile flags")
set(CMAKE_CXX_FLAGS "-g -O0 -Wall" CACHE STRING  "C++ compile flags")

# Optional: Set paths if needed
# set(CMAKE_FIND_ROOT_PATH "C:/mingw")
set(TARGET_INCLUDE_DIRS 
    ${CMAKE_SOURCE_DIR}/include        # 项目自定义头文件
    /usr/include
    /usr/local/include
    CACHE INTERNAL "Target include directories"
)
set(CMAKE_WIN32_EXECUTABLE OFF)
# Search for programs in the host environment
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# Search for libraries and headers in the target environment
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)