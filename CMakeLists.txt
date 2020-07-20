cmake_minimum_required(VERSION 3.17)
project(ESKF_CPP)

set(CMAKE_CXX_STANDARD 11)

include_directories(include)

#Eigen
find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(/usr/include/eigen3)

#sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

File(GLOB_RECURSE srcs "src/*.cpp")
FILE(GLOB_RECURSE header "include/*.h")
add_custom_target(header)

add_executable(ESKF_CPP main.cpp ${srcs})