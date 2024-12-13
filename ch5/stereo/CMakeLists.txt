cmake_minimum_required(VERSION 3.10)

project(stereoVison VERSION 1.0)

find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)

add_executable(stereoVision stereoVision.cpp)
target_link_libraries(stereoVision ${OpenCV_LIBS} ${Pangolin_LIBRARIES})