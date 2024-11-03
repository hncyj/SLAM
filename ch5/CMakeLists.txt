cmake_minimum_required(VERSION 3.10)

project(use_opencv VERSION 1.0)

set(CMAKE_BUILD_TYPE "Debug")
# 添加c++ 11标准支持
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Eigen
include_directories("/usr/include/eigen3")

# 寻找OpenCV库
find_package(OpenCV REQUIRED)
# 添加头文件
include_directories(${OpenCV_INCLUDE_DIRS})

# add_subdirectory(imageBasics)
# add_subdirectory(stereo)
add_subdirectory(rgbd)