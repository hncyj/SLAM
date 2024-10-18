cmake_minimum_required(VERSION 3.10)
project(visualize VERSION 1.0)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置 Pangolin 的路径
# set(Pangolin_DIR "/home/chenyinjie/github/Pangolin/build")

# 查找 Eigen3 库
find_package(Eigen3 REQUIRED)
if(Eigen3_FOUND)
    message(STATUS "Found Eigen3: ${EIGEN3_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "Eigen3 not found")
endif()

# 查找 Pangolin 库
find_package(Pangolin REQUIRED)
if(Pangolin_FOUND)
    message(STATUS "Found Pangolin: ${Pangolin_INCLUDE_DIRS}")
    message(STATUS "Pangolin libraries: ${Pangolin_LIBRARIES}")  # 添加此行以调试
else()
    message(FATAL_ERROR "Pangolin not found")
endif()

# 查找 GLEW 库
find_package(GLEW REQUIRED)
if(GLEW_FOUND)
    message(STATUS "Found GLEW: ${GLEW_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "GLEW not found")
endif()

# 查找 OpenGL 库
find_package(OpenGL REQUIRED)
if(OpenGL_FOUND)
    message(STATUS "Found OpenGL: ${OPENGL_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "OpenGL not found")
endif()

# 添加可执行文件
add_executable(visualize visual.cpp)

# 包含目录
target_include_directories(
    visualize
    PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIRS}
    ${OPENGL_INCLUDE_DIRS}
)

# 链接库
target_link_libraries(
    visualize
    PRIVATE
    Eigen3::Eigen
    GLEW::GLEW
    OpenGL::GL
    ${Pangolin_LIBRARIES}
)