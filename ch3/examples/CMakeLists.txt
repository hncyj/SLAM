cmake_minimum_required(VERSION 3.10)

project(CoordinateTransform VERSION 1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)

add_executable(
    Coo
    coordinateTransform.cpp
)

add_executable(
    Draw
    TrajectoryDemo.cpp
)

target_include_directories(
    Coo
    PRIVATE
    ${Eigen3_INCLUDE_DIRS}
)

target_include_directories(
    Draw
    PRIVATE
    ${Eigen3_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIRS}
)

target_link_libraries(
    Coo
    PRIVATE
    Eigen3::Eigen
)

target_link_libraries(
    Draw
    PRIVATE
    Eigen3::Eigen
    GLEW::GLEW
    OpenGL::GL
    ${Pangolin_LIBRARIES}
)