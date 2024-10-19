cmake_minimum_required(VERSION 3.10)

project(CoordinateTransform VERSION 1.0)

find_package(Eigen3 REQUIRED)

add_executable(
    Coo
    coordinateTransform.cpp
)

target_link_directories(
    Coo
    PRIVATE
    ${Eigen3_INCLUDE_DIRS}
)

target_link_libraries(
    Coo
    PRIVATE
    Eigen3::Eigen
)