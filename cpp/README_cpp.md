# DPCBF C++ 重构与可视化

## 构建与运行（使用 OSQP-Eigen）

在 WSL(Ubuntu) 环境下，先安装依赖（示例）：
```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake pkg-config
# 安装 OSQP（若未安装）
sudo apt-get install -y libosqp-dev
# 安装 Eigen3（若未安装）
sudo apt-get install -y libeigen3-dev
```

安装 OSQP-Eigen（两种方式之一）：
- 方式A：系统已有包（若可用）
```bash
sudo apt-get install -y libosqp-eigen-dev
```
- 方式B：源码安装
```bash
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
sudo cmake --build . --target install
```

构建本工程：
```bash
mkdir -p build
cd build
cmake ..
cmake --build . -j
./dpcbf_sim
```

生成 `output_dpcbf.csv` 后，用 Python 可视化：
```bash
python3 viz/plot.py
```

## 文件说明
- `cpp/include/bicycle_model.hpp`：简化 Kinematic Bicycle 2D 离散动力学与名义控制。
- `cpp/include/dpcbf.hpp`：DPCBF 连续/离散 h 的实现（LoS 坐标与抛物线边界）。
- `cpp/include/qp_controller.hpp`：基于 OSQP-Eigen 的 CBF-QP 控制器（带松弛变量、变量盒约束与离散约束线性化）。
- `cpp/src/sim.cpp`：仿真主程序，加载障碍与路点，循环输出数据。
- `viz/plot.py`：Python 可视化脚本。
- `CMakeLists.txt`：CMake 构建配置。

## 备注
- 若 CBF 约束较紧，可适度增大 `rho`（松弛权重）与 `gamma`，或选择最近障碍子集以提升可行性。
- 线性化采用单次在 `u_ref` 处的数值梯度；如需更稳健，可采用迭代线性化（Sequential QP）。+++++++ REPLACE

## 文件说明
- `cpp/include/bicycle_model.hpp`：简化 Kinematic Bicycle 2D 离散动力学与名义控制。
- `cpp/include/dpcbf.hpp`：DPCBF 连续/离散 h 的实现（LoS 坐标与抛物线边界）。
- `cpp/include/cbf_controller.hpp`：不依赖外部 QP 的迭代修正器，近似满足离散 CBF 约束。
- `cpp/src/sim.cpp`：仿真主程序，加载障碍与路点，循环输出数据。
- `viz/plot.py`：Python 可视化脚本。

## 备注
- 若需严格的 QP 解算，可接入 OSQP/qpOASES 等库，并将约束线性化为 `A u <= b` 形式；本示例采用数值线性化的迭代修正，便于零依赖快速运行。
- 参数 `k_lambda, k_mu, margin, gamma` 可调，以适配不同的速度/距离工况与保守性。