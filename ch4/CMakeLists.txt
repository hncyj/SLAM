cmake_minimum_required(VERSION 3.10)

project(useSophus VERSION 1.0)

find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${SOPHUS_INCLUDE_DIR}
)

add_executable(
    useSophus
    useSophus.cpp
)

target_link_libraries(
    useSophus
    Sophus::Sophus
)

add_subdirectory(examples)