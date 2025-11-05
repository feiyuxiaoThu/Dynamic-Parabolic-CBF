cmake_minimum_required(VERSION 3.12)
project(dpcbf_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GFLAGS REQUIRED gflags)

include_directories(include)

# 统一仿真器库
add_library(unified_simulator
    src/unified_simulator.cpp
    src/bicycle_model.cc
    src/gflags_config.cc
    src/dpcbf.cc
    src/qp_controller.cc
    src/multiple_shooting_controller.cpp
    src/advanced_qp_solver.cpp
)
target_link_libraries(unified_simulator PUBLIC Eigen3::Eigen OsqpEigen::OsqpEigen ${GFLAGS_LIBRARIES})
target_include_directories(unified_simulator PUBLIC ${GFLAGS_INCLUDE_DIRS})

# 统一示例程序
add_executable(unified_examples
    unified_examples.cpp
)
target_link_libraries(unified_examples PRIVATE unified_simulator)

# Test for all obstacles functionality
add_executable(test_all_obstacles
    test_all_obstacles.cpp
)
target_link_libraries(test_all_obstacles PRIVATE unified_simulator)

# Multiple-Shooting 测试程序
add_executable(test_multiple_shooting
    test_multiple_shooting.cpp
)
target_link_libraries(test_multiple_shooting PRIVATE unified_simulator)

# 控制器性能基准测试
add_executable(benchmark_controllers
    benchmark_controllers.cpp
)
target_link_libraries(benchmark_controllers PRIVATE unified_simulator)

