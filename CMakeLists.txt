cmake_minimum_required(VERSION 3.13)
project(ICPAlgorithm)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# libigl
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use IMGUI"          ON)

find_package(LIBIGL REQUIRED QUIET)

add_executable(ICP src/main.cpp src/mytools.cpp)

target_link_libraries(ICP igl::core igl::opengl_glfw igl::opengl_glfw_imgui)
