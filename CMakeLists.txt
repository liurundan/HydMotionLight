cmake_minimum_required(VERSION 3.10)
project(HydroMotionLib C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(HYDRO_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include

)

# set(HYDRO_CORE_SRCS
#     ${CMAKE_CURRENT_SOURCE_DIR}/src 
# )

# 自动搜索 src 目录下所有 .c 源文件（推荐）
file(GLOB_RECURSE HYDRO_CORE_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
) 

add_library(HydroMotionLib STATIC ${HYDRO_CORE_SRCS})
target_include_directories(HydroMotionLib PUBLIC ${HYDRO_INCLUDE_DIRS})
target_link_libraries(HydroMotionLib PUBLIC m)

 

add_executable(test_motion_planner tests/test_motion_planner.c)
target_link_libraries(test_motion_planner PRIVATE HydroMotionLib)

 
 
