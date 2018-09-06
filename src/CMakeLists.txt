# CMake VERSION
cmake_minimum_required(VERSION 3.5.1)

# Project Name
project (ArucoGen)

# Shared Library
set ( PROJECT_LINK_LIBS libDense3DMT.so libDense3D.so libDuo.so)
link_directories(../lib)

# Include Headers
include_directories(../include)

# Executable
add_executable(ArucoGen aruco_gen.cpp)
