cmake_minimum_required(VERSION 3.16)
project(slam_v1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Eigen3
find_package(Eigen3 REQUIRED)
# Eigen3 cmake might set Eigen3_DIR but not the include dir directly
get_filename_component(EIGEN3_INCLUDE_DIR "${Eigen3_DIR}/../../../include/eigen3" ABSOLUTE)
include_directories(${EIGEN3_INCLUDE_DIR})

# 头文件目录
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

# 源文件
set(SOURCES
    src/types/frame.cpp
    src/types/map_point.cpp
    src/types/key_frame.cpp
    src/core/detector.cpp
    src/core/matcher.cpp
    src/core/pose_estimator.cpp
    src/backend/map.cpp
    src/frontend/frontend.cpp
)

# 库
add_library(slam_core STATIC ${SOURCES})

# 可执行文件
add_executable(run_slam practice/main.cpp)
target_link_libraries(run_slam slam_core ${OpenCV_LIBS})