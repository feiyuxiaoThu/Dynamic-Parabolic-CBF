# DPCBF vs C3BF：算法构造、约束形式与在 Kinematic Bicycle 2D 中的使用

说明与来源：
- 代码依据：`dynamic_env/kinematic_bicycle2D_dpcbf.py`, `dynamic_env/kinematic_bicycle2D_c3bf.py`, `dynamic_env/robot.py`, `dynamic_env/main.py`, `dynamic_env/README.md`
- 论文 HTML：<https://arxiv.org/html/2510.01402v1>，本文档参考其中 DPCBF 的 LoS 构造与动态参数设计进行对齐。若您提供论文关键公式/段落，可进一步严格校准符号与推导。

## 1. 模型与场景

- 机器人模型：Kinematic Bicycle 2D  
  状态 x = [X, Y, θ, v]，控制 u 含转向与加速度（基类 step(x,u) 推进）。
- 障碍物：圆形，支持速度分量 obs = [ox, oy, r, vx, vy]；机器人半径 robot_radius。
- 控制框架：每周期生成名义控制 u_ref（朝向路点），通过 CBF-QP/约束修正得到安全控制 u，再用动力学推进。

共同相对量与安全半径：
- 相对位置：p_rel = [ox − X, oy − Y]
- 相对速度：v_rel = [vx − v cos θ, vy − v sin θ]
- 合成安全半径：ego_dim = (r_obs + r_robot) × margin（margin > 1 提供安全缓冲）

## 2. C3BF（Collision Cone CBF）

- 代码位置：`dynamic_env/kinematic_bicycle2D_c3bf.py`
- 思想：用碰撞圆锥（Collision Cone）刻画危险方向。设
  - 距离：p = ||p_rel||，速度：v = ||v_rel||
  - φ = arcsin(ego_dim / p)，cos φ = sqrt(max(p^2 − ego_dim^2, ε)) / (p + ε)
- 连续时间候选函数：
  - h_C3BF(x) = ⟨p_rel, v_rel⟩ + p · v · cos φ
  - h ≥ 0 视为安全；h < 0 落入圆锥危险域。
- 实现细节：
  - 提供 h 与 dh/dx（用于线性化约束）。
  - 离散接口 agent_barrier_dt：返回 h_k 与 d_h = h_{k+1} − h_k。

可视化（`dynamic_env/robot.py`）：
- 绘制碰撞圆锥边界与相对速度箭头；箭头指向圆锥内部则风险更高。

## 3. DPCBF（Dynamic Parabolic CBF）

- 代码位置：`dynamic_env/kinematic_bicycle2D_dpcbf.py`
- 创新点：在视线坐标（LoS frame）下，用“动态抛物线”刻画安全边界，并随距离/相对速度自适应调整曲率与偏移。
  1) LoS 旋转：将 x 轴对准 p_rel，v_rel_new = R v_rel = [v_x, v_y]
  2) 安全距离：
     - d_safe = max(||p_rel||^2 − ego_dim^2, ε)
  3) 动态参数（与论文思路一致，结合安全系数 margin 与 ego_dim 缩放）：
     - 令 scale = sqrt(margin^2 − 1) / ego_dim
     - λ(x) = (k_lambda · scale) · sqrt(d_safe) / ||v_rel||
     - μ(x) = (k_mu · scale) · sqrt(d_safe)
  4) 抛物线型 CBF：
     - h_DPCBF(x) = v_x + λ(x) · v_y^2 + μ(x)
- 直觉：
  - v_x：LoS 方向的接近速度；越大越安全（远离）
  - λ · v_y^2：横向速度的二次惩罚，抛物线开口可调
  - μ：随安全距离前向平移边界，提供缓冲
- 实现细节：
  - 提供 h 与 dh/dx（用于连续时间线性化）
  - 离散接口 agent_barrier_dt：返回 h_k 与 d_h（便于离散约束）

可视化（`dynamic_env/robot.py`）：
- 在 LoS 坐标系绘制抛物线边界；相对速度矢量应落在抛物线外（h ≥ 0）。

## 4. CBF-QP 约束形式（连续/离散）

- 连续时间常见约束：
  - ḣ(x,u) = ∂h/∂x · f(x) + ∂h/∂x · g(x) · u ≥ −α h(x)，α > 0
- 离散时间约束（贴合代码的离散推进）：
  - h(x_{k+1}) − h(x_k) + γ h(x_k) ≥ 0，γ ≥ 0
- QP 目标（典型）：
  - min ||u − u_ref||^2 + ρ ||δ||^2  
    s.t. 对每个障碍的线性化 CBF 约束；δ 为松弛量（若需要）。

在本仓库：
- C3BF 与 DPCBF 都提供 h 与离散增量接口。
- DPCBF 提供 LoS 变换与抛物线绘制辅助。

## 5. DPCBF 与 C3BF 对比

- 几何基元
  - C3BF：圆锥（角度 φ 由距离与安全半径决定），边界刚性
  - DPCBF：抛物线（λ、μ 动态可调），形状与偏移自适应
- 坐标参考
  - C3BF：原始坐标基于夹角关系
  - DPCBF：LoS 坐标显式解耦，便于分解 v_x/v_y 构造二次边界
- 可调性与鲁棒性
  - DPCBF 通过 λ、μ 随 d_safe、||v_rel|| 调整，更能适应近距离/高速场景
- 复杂度
  - 两者均为相对度 1 的 CBF；DPCBF 的梯度更复杂，但安全边界更灵活

## 6. C++ 重构与可视化（本仓库新增）

- 文件：
  - `cpp/include/bicycle_model.hpp`：简化 Kinematic Bicycle 2D 动力学与名义控制
  - `cpp/include/dpcbf.hpp`：DPCBF 连续/离散 h 的实现（含 LoS 坐标）
  - `cpp/include/qp_controller.hpp`：基于 OSQP-Eigen 的标准 CBF-QP 控制器（含松弛变量与盒约束）。
  - `cpp/src/sim.cpp`：仿真主程序，输出 `output_dpcbf.csv`
  - `viz/plot.py`：Python 可视化脚本读取 CSV 绘图
  - `CMakeLists.txt` 与 `README_cpp.md`：构建运行说明（含 OSQP/OSQP-Eigen 安装提示）

- 运行（WSL/Ubuntu，OSQP-Eigen QP）：
  ```bash
  mkdir -p build
  cd build
  cmake ..
  cmake --build . -j
  ./dpcbf_sim
  python3 ../viz/plot.py
  ```

- 备注：
  - 若需严格 QP 解算，可接入 OSQP/qpOASES，将约束线性化为 A u ≤ b。
  - 关键参数（`k_lambda, k_mu, margin, gamma`）可调，平衡保守性与可行性。

## 7. 后续改进建议

- 引入外部 QP 库以获得更稳定的约束求解。
- 在 C++ 中加入 C3BF 的实现与对照仿真。
- 支持多障碍选择策略（最近障碍子集/可行性筛选）。
- 将 λ、μ 与速度上限、制动距离模型耦合，增强物理一致性。