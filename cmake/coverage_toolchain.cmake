# cmake/coverage_toolchain.cmake
# Coverage toolchain: identical to unixgcc_toolchain.cmake but with --coverage
# for gcov instrumentation and -O0 -g (required for accurate coverage data).

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR x86_64)
set(CMAKE_C_COMPILER "/usr/bin/gcc" CACHE PATH "C compiler")
set(CMAKE_CXX_COMPILER "/usr/bin/g++" CACHE PATH "C++ compiler")

set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type")
set(CMAKE_C_FLAGS "-g -O0 -Wall --coverage" CACHE STRING "C compile flags")
set(CMAKE_CXX_FLAGS "-g -O0 -Wall --coverage" CACHE STRING "C++ compile flags")
set(CMAKE_EXE_LINKER_FLAGS "--coverage" CACHE STRING "Linker flags")
set(CMAKE_SHARED_LINKER_FLAGS "--coverage" CACHE STRING "Linker flags")

set(TARGET_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/include
    /usr/include
    /usr/local/include
    CACHE INTERNAL "Target include directories"
)
set(CMAKE_WIN32_EXECUTABLE OFF)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
