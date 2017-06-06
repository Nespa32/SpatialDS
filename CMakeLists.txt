cmake_minimum_required(VERSION 2.8.9)

project(GeometricDS)

set (CMAKE_CXX_FLAGS "-std=c++11 -O3 -ggdb")

include_directories(${GeometricDS_SOURCE_DIR}/src/algorithm)
include_directories(${GeometricDS_SOURCE_DIR}/src/util)

file(GLOB SOURCES "src/*.cpp")

add_executable(runnable ${SOURCES})

