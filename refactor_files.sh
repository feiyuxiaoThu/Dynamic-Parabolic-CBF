#!/bin/bash

# 移动 sim.cpp 到 cpp/ 根目录
mv cpp/src/sim.cpp cpp/sim.cpp

# 将 include 目录下的 .hpp 文件重命名为 .h
mv cpp/include/bicycle_model.hpp cpp/include/bicycle_model.h
mv cpp/include/config_loader.hpp cpp/include/config_loader.h
mv cpp/include/dpcbf.hpp cpp/include/dpcbf.h
mv cpp/include/qp_controller.hpp cpp/include/qp_controller.h

# 将 src 目录下的 .cpp 文件重命名为 .cc
mv cpp/src/bicycle_model.cpp cpp/src/bicycle_model.cc
mv cpp/src/config_loader.cpp cpp/src/config_loader.cc
mv cpp/src/dpcbf.cpp cpp/src/dpcbf.cc
mv cpp/src/qp_controller.cpp cpp/src/qp_controller.cc

echo "文件重命名和移动完成。"

