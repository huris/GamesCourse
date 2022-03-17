cmake_minimum_required(VERSION 3.6)
project(eigentest)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


include_directories(/usr/local/include/)
set(SOURCE_FILES main.cpp)
add_executable(Transformation ${SOURCE_FILES})