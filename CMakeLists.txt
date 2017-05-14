cmake_minimum_required(VERSION 2.8.9)

project(GeometricDS)

set (CMAKE_CXX_FLAGS "-std=c++11 -ggdb")

file(GLOB SOURCES "src/*.cpp")

add_executable(runnable ${SOURCES})

