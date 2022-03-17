cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

find_package(OpenCV)

set(CMAKE_CXX_STANDARD 17)

include_directories(/usr/local/include/)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp Triangle.hpp Triangle.cpp)

target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})