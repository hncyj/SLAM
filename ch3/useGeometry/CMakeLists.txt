cmake_minimum_required(VERSION 3.10)

project(geometry VERSION 1.0)

find_package(Eigen3 REQUIRED)

add_executable(
    useGeometry
    geometry.cpp
)

target_link_libraries(useGeometry Eigen3::Eigen)




