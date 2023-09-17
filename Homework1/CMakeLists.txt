cmake_minimum_required(VERSION 3.24)
project(DeepBlueProject)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

include_directories(include)

set(SOURCES
        src/GradientDescentSolver.cpp)

add_executable(DeepBlueProject
        main.cpp
        src/GradientDescentSolver.cpp
        include/GradientDescentSolver.h)

target_link_libraries(DeepBlueProject Eigen3::Eigen)

