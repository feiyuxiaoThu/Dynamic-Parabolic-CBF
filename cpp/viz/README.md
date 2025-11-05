# CBF-QP 仿真结果可视化工具

本目录包含增强的可视化工具，用于分析和展示 Multiple-Shooting CBF-QP 控制器的仿真结果。

## 功能特性

### 📊 增强静态分析
- **轨迹分析**: 主车轨迹、参考轨迹、障碍物轨迹
- **速度曲线**: 实际速度 vs 参考速度
- **控制输入**: 转向角和加速度变化
- **安全距离**: CBF 安全边界和危险区域
- **方向角变化**: 车辆朝向分析
- **障碍物距离**: 到各障碍物的距离变化

### 🎬 增强动画
- **实时动画**: 展示车辆运动过程
- **安全圆圈**: 动态显示安全区域
- **信息面板**: 实时显示仿真状态
- **轨迹尾迹**: 显示历史运动轨迹

### 🔍 控制器对比
- **多控制器对比**: 自动识别并对比不同控制器的性能
- **性能指标**: 轨迹、速度、安全性对比
- **统计图表**: 综合性能分析

## 使用方法

### 1. 基本使用

```bash
# 自动检测并可视化所有CSV文件
python3 enhanced_visualizer.py --auto

# 指定特定场景类型
python3 enhanced_visualizer.py --scenario straight

# 指定具体CSV文件
python3 enhanced_visualizer.py --csv test_ms_straight_output.csv

# 只生成静态分析图
python3 enhanced_visualizer.py --scenario straight --static-only

# 只生成动画
python3 enhanced_visualizer.py --scenario straight --animation-only
```

### 2. 控制器性能对比

```bash
# 生成控制器对比分析（需要多个CSV文件）
python3 enhanced_visualizer.py --comparison --scenario straight

# 自动模式下会自动生成对比分析
python3 enhanced_visualizer.py --auto
```

### 3. 高级使用

```bash
# 交叉路口场景可视化
python3 enhanced_visualizer.py --scenario intersection

# 查看帮助信息
python3 enhanced_visualizer.py --help
```

## CSV 文件格式

可视化脚本支持以下CSV列格式：

```
t,x,y,theta,v,steer,a,h_min,ref_x,ref_y,obs0_ox,obs0_oy,obs0_r,obs0_vx,obs0_vy,obs1_ox,obs1_oy,obs1_r,obs1_vx,obs1_vy,...
```

**必需列：**
- `t`: 时间 (秒)
- `x`, `y`: 车辆位置 (米)
- `v`: 速度 (米/秒)
- `steer`: 转向角 (弧度)
- `a`: 加速度 (米/秒²)

**可选列：**
- `theta`: 方向角 (弧度)
- `h_min`: 最小安全距离
- `ref_x`, `ref_y`: 参考轨迹位置
- `obs{i}_ox`, `obs{i}_oy`: 障碍物i的位置
- `obs{i}_r`: 障碍物i的半径
- `obs{i}_vx`, `obs{i}_vy`: 障碍物i的速度

## 输出文件

脚本会自动生成以下文件：

### 静态分析图
- `{filename}_enhanced_analysis.png`: 6面板综合分析图
- `{filename}_controller_comparison.png`: 控制器对比图

### 动画
- `{filename}_enhanced_animation.gif`: 增强动画效果

### 文件命名规则
- 自动检测时：基于原始CSV文件名
- 手动指定时：基于场景类型和参数

## 示例输出

### 1. 直线避障场景
```bash
# 生成数据
./test_multiple_shooting --controller=multiple --scenario=straight

# 可视化
python3 viz/enhanced_visualizer.py --auto
```

**预期输出：**
- 车辆成功避开4个动态障碍物
- 展示多步预测的安全保证
- 控制输入平滑稳定

### 2. 路口转弯场景
```bash
# 生成数据
./test_multiple_shooting --controller=adaptive --scenario=intersection

# 可视化
python3 viz/enhanced_visualizer.py --scenario intersection
```

**预期输出：**
- 复杂路口环境的避障行为
- 自适应控制器切换
- 转弯过程中的安全约束

### 3. 控制器性能对比
```bash
# 生成多种控制器数据
./test_multiple_shooting --controller=single --scenario=straight
./test_multiple_shooting --controller=multiple --scenario=straight
./test_multiple_shooting --controller=adaptive --scenario=straight

# 对比分析
python3 viz/enhanced_visualizer.py --auto
```

**预期输出：**
- 三种控制器的轨迹对比
- 安全性和性能指标对比
- 计算时间对比

## 安装依赖

```bash
pip install pandas matplotlib numpy pillow
```

## 故障排除

### 1. 找不到CSV文件
```bash
# 检查当前目录下的CSV文件
ls -la *output*.csv

# 使用完整路径
python3 viz/enhanced_visualizer.py --csv /path/to/your/file.csv
```

### 2. 动画生成失败
- 确保 `pillow` 库已安装：`pip install pillow`
- 检查磁盘空间是否充足
- 尝试使用 `--static-only` 参数

### 3. 字体显示问题
- 在Linux上可能需要安装中文字体
- 或者设置 `plt.rcParams` 使用系统默认字体

### 4. 性能问题
- 对于大型数据集，使用 `--static-only` 避免动画
- 减少动画帧数：修改脚本中的 `interval` 参数

## 高级功能

### 1. 自定义颜色主题
在 `enhanced_visualizer.py` 中修改颜色设置：
```python
# 修改障碍物颜色
obstacle_colors = plt.cm.Set2(np.linspace(0, 1, num_obstacles))

# 修改主车颜色
ego_vehicle, = ax.plot([], [], 'o', color='blue', ...)
```

### 2. 添加自定义指标
可以扩展 `print_enhanced_statistics()` 方法：
```python
# 添加自定义性能指标
def print_enhanced_statistics(self):
    # 现有统计...

    # 自定义指标
    if 'custom_metric' in self.df.columns:
        print(f"Custom metric: {self.df['custom_metric'].mean():.3f}")
```

### 3. 批量处理
创建批量处理脚本：
```python
import glob
import subprocess

csv_files = glob.glob("*output*.csv")
for csv_file in csv_files:
    subprocess.run(["python3", "enhanced_visualizer.py", "--csv", csv_file, "--static-only"])
```

## 技术细节

### 动画性能优化
- 使用轨迹尾迹而非完整轨迹
- 限制动画帧率到20 FPS
- 异步渲染优化

### 内存管理
- 大数据集分块处理
- 及时释放不需要的图形对象
- 使用生成器模式处理大规模数据

### 可扩展性
- 模块化设计，易于添加新的分析维度
- 支持自定义数据格式
- 插件式架构支持

## 贡献指南

欢迎提交 Pull Request 来改进可视化工具：

1. **新功能**: 添加新的分析维度或可视化类型
2. **性能优化**: 提高动画生成速度或内存使用效率
3. **用户体验**: 改进界面交互或错误处理
4. **文档完善**: 更新说明文档或添加示例

---

**版本**: 1.0
**最后更新**: 2024年10月
**兼容性**: Python 3.6+