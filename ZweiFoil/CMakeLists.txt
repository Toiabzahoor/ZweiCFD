cmake_minimum_required(VERSION 3.10)

project(ZweiFoil CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

add_executable(ZweiFoil
    src/main.cpp
    src/airfoil.cpp
    src/solver.cpp
)