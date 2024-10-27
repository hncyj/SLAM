cmake_minimum_required(VERSION 3.10)

project(examples VERSION 1.0)

option(USE_UBUNTU_24 "Set to ON if you are using Ubuntu 24.04" OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Pangolin REQUIRED)

set(CMAKE_BUILD_TYPE Debug)

if(USE_UBUNTU_24)
    message("You are using Ubuntu 24.04, fmt::fmt will be linked")
    find_package(fmt REQUIRED)
    set(FMT_LIBRARIES fmt::fmt)
endif()

include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(trajectoryError trajectoryError.cpp)

target_link_libraries(trajectoryError ${Pangolin_LIBRARIES} ${FMT_LIBRARIES})

