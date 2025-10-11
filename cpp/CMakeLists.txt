cmake_minimum_required(VERSION 3.12)
project(dpcbf_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include)

add_executable(dpcbf_sim
    sim.cpp
    src/bicycle_model.cc
    src/config_loader.cc
    src/dpcbf.cc
    src/qp_controller.cc
)
target_link_libraries(dpcbf_sim PRIVATE Eigen3::Eigen OsqpEigen::OsqpEigen yaml-cpp::yaml-cpp)