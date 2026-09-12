# cmake/mingw_w64_toolchain.cmake
# MinGW-w64 toolchain for Windows cross-platform builds

set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR x86_64)

# MinGW-w64 compiler paths (use forward slashes for CMake compatibility)
set(CMAKE_C_COMPILER "D:/mingw64/bin/gcc.exe" CACHE PATH "C compiler")
set(CMAKE_CXX_COMPILER "D:/mingw64/bin/g++.exe" CACHE PATH "C++ compiler")

# MinGW make program (required for MinGW Makefiles generator)
set(CMAKE_MAKE_PROGRAM "D:/mingw64/bin/mingw32-make.exe" CACHE PATH "Make program")

# Debug build with optimization disabled for accurate debugging
set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type")
set(CMAKE_C_FLAGS "-g -O0 -Wall" CACHE STRING "C compile flags")
set(CMAKE_CXX_FLAGS "-g -O0 -Wall" CACHE STRING "C++ compile flags")

# Include directories (MinGW provides standard C library headers)
set(TARGET_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/include
    CACHE INTERNAL "Target include directories"
)

# Windows executable settings
set(CMAKE_WIN32_EXECUTABLE OFF)

# Search for programs in the host environment
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# Search for libraries and headers in the target environment
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)
