cmake_minimum_required(VERSION 3.17)
project(ESKF_CPP)

set(CMAKE_CXX_STANDARD 11)

#Eigen
find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(/usr/include/eigen3)

#sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

FILE(GLOB_RECURSE LibFiles "include/*.h")
add_custom_target(headers SOURCES ${LibFiles})

add_executable(ESKF_CPP  utils.cpp ESKF.cpp main.cpp)