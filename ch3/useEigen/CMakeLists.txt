cmake_minimum_required(VERSION 3.10)

project(useEigen)

find_package(Eigen3 REQUIRED)

add_executable(
    eigenMatrix
    eigenMatrix.cpp
)

target_link_libraries(eigenMatrix Eigen3::Eigen)