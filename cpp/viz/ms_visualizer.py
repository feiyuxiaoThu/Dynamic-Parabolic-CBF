#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import argparse
import os
import glob

# 设置字体以支持中文显示
plt.rcParams['font.sans-serif'] = ['DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

class MSVisualizer:
    def __init__(self, csv_file=None, scenario_type="straight"):
        self.csv_file = csv_file
        self.scenario_type = scenario_type

        # 如果没有指定文件，尝试自动查找
        if self.csv_file is None:
            self.auto_find_csv_file()

        if self.csv_file is None:
            raise FileNotFoundError("Could not find any CSV file to visualize")

        # 根据文件名自动检测场景类型
        if self.scenario_type == "auto":
            self.detect_scenario_type()

    def auto_find_csv_file(self):
        """自动查找CSV文件"""
        # 按优先级查找可能的文件名
        candidates = [
            "output_dpcbf.csv",           # 原始文件名
            "intersection_output.csv",   # 路口场景
            "test_ms_intersection_output.csv",  # MS测试文件
            "test_ms_straight_output.csv",    # MS测试文件
            "*straight*output*.csv",         # 直线场景
            "*intersection*output*.csv",      # 路口场景
            "*output*.csv"                   # 任何output文件
        ]

        for pattern in candidates:
            if pattern.startswith("*"):
                matches = glob.glob(pattern)
            else:
                matches = [pattern] if os.path.exists(pattern) else []

            if matches:
                # 选择最新修改的文件
                self.csv_file = max(matches, key=os.path.getmtime)
                print(f"Auto-detected CSV file: {self.csv_file}")
                return

    def detect_scenario_type(self):
        """从文件名检测场景类型"""
        filename = self.csv_file.lower()

        if "intersection" in filename or "turn" in filename:
            self.scenario_type = "intersection"
        elif "straight" in filename or "line" in filename:
            self.scenario_type = "straight"
        else:
            # 默认设为straight
            self.scenario_type = "straight"

        print(f"Auto-detected scenario type: {self.scenario_type}")

    def load_data(self):
        """加载仿真数据"""
        if not os.path.exists(self.csv_file):
            raise FileNotFoundError(f"Data file not found: {self.csv_file}")

        self.df = pd.read_csv(self.csv_file)
        print(f"Loaded {len(self.df)} data points from {self.csv_file}")

        # 检测数据列
        self.check_data_columns()

    def check_data_columns(self):
        """检查数据列的完整性"""
        required_cols = ['t', 'x', 'y', 'v', 'steer', 'a']
        missing_cols = [col for col in required_cols if col not in self.df.columns]

        if missing_cols:
            raise ValueError(f"Missing required columns: {missing_cols}")

        # 检测可选列
        self.has_theta = 'theta' in self.df.columns
        self.has_h_min = 'h_min' in self.df.columns
        self.has_reference = 'ref_x' in self.df.columns and 'ref_y' in self.df.columns

        # 检测障碍物数量
        obstacle_cols = [col for col in self.df.columns if col.startswith('obs') and col.endswith('_ox')]
        self.num_obstacles = len(obstacle_cols)

        print(f"Data columns detected:")
        print(f"  Required columns: ✓")
        print(f"  Theta data: {'✓' if self.has_theta else '✗'}")
        print(f"  Safety data: {'✓' if self.has_h_min else '✗'}")
        print(f"  Reference trajectory: {'✓' if self.has_reference else '✗'}")
        print(f"  Obstacles: {self.num_obstacles}")

    def create_static_analysis(self):
        """创建静态分析图"""
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 12))

        # 1. 轨迹图
        self.plot_trajectory(ax1)

        # 2. 速度曲线
        self.plot_velocity(ax2)

        # 3. 安全距离
        if self.has_h_min:
            self.plot_safety_distance(ax3)
        else:
            ax3.text(0.5, 0.5, 'Safety data not available',
                    ha='center', va='center', transform=ax3.transAxes,
                    fontsize=12, bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.5))
            ax3.set_title('Safety Distance')
            ax3.grid(True, alpha=0.3)

        # 4. 控制输入
        self.plot_control_inputs(ax4)

        plt.tight_layout()

        # 保存图片
        output_file = self.get_output_filename("_analysis.png")
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Static analysis saved to: {output_file}")

        plt.show()

    def plot_trajectory(self, ax):
        """绘制轨迹图（使用原始方法）"""
        # 主车轨迹
        ax.plot(self.df['x'].values, self.df['y'].values, 'b-', linewidth=2, label='Ego Vehicle')

        # 参考轨迹
        if self.has_reference:
            ax.plot(self.df['ref_x'].values, self.df['ref_y'].values, 'r--', linewidth=1, label='Reference')

        # 障碍物轨迹
        obstacle_colors = ['g', 'm', 'c', 'orange', 'purple']

        for i in range(self.num_obstacles):
            ox_col = f'obs{i}_ox'
            oy_col = f'obs{i}_oy'
            if ox_col in self.df.columns and oy_col in self.df.columns:
                ax.plot(self.df[ox_col].values, self.df[oy_col].values,
                       color=obstacle_colors[i % len(obstacle_colors)],
                       alpha=0.7, label=f'Vehicle {i+1}')

                # 标记障碍物起点
                ax.plot(self.df[ox_col].iloc[0], self.df[oy_col].iloc[0],
                       'o', color=obstacle_colors[i % len(obstacle_colors)], markersize=6)

        # 标记起点和终点
        ax.plot(self.df['x'].iloc[0], self.df['y'].iloc[0], 'go', markersize=8, label='Start')
        ax.plot(self.df['x'].iloc[-1], self.df['y'].iloc[-1], 'ro', markersize=8, label='End')

        # 场景特定的装饰
        if self.scenario_type == "intersection":
            # 绘制路口区域
            intersection_x = [0, 20, 20, 0, 0]
            intersection_y = [0, 0, 20, 20, 0]
            ax.plot(intersection_x, intersection_y, 'k--', alpha=0.5, label='Intersection')

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(f'{self.get_scenario_title()} - Trajectory')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.axis('equal')

    def plot_velocity(self, ax):
        """绘制速度曲线（使用原始方法）"""
        ax.plot(self.df['t'].values, self.df['v'].values, 'b-', linewidth=2, label='Actual Velocity')

        # 添加参考速度线
        ref_velocity = self.df['v'].mean()
        ax.axhline(y=ref_velocity, color='r', linestyle='--', alpha=0.7,
                   label=f'Avg Velocity: {ref_velocity:.2f} m/s')

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (m/s)')
        ax.set_title('Velocity Profile')
        ax.legend()
        ax.grid(True, alpha=0.3)

    def plot_safety_distance(self, ax):
        """绘制安全距离（使用原始方法）"""
        ax.plot(self.df['t'].values, self.df['h_min'].values, 'r-', linewidth=2)
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.7, label='Safety Boundary')

        # 标记不安全区域
        unsafe_mask = self.df['h_min'].values < 0
        if unsafe_mask.any():
            ax.fill_between(self.df['t'].values, self.df['h_min'].values, 0,
                           where=unsafe_mask, alpha=0.3, color='red', label='Unsafe Region')

            # 标记最危险的时刻
            min_idx = self.df['h_min'].idxmin()
            min_time = self.df.loc[min_idx, 't']
            min_value = self.df.loc[min_idx, 'h_min']
            ax.plot(min_time, min_value, 'ro', markersize=8,
                   label=f'Min: {min_value:.3f}')

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Min Safety Distance h_min')
        ax.set_title('Safety Distance')
        ax.legend()
        ax.grid(True, alpha=0.3)

    def plot_control_inputs(self, ax):
        """绘制控制输入（使用原始方法）"""
        ax.plot(self.df['t'].values, self.df['steer'].values, 'g-', linewidth=2, label='Steering (rad)')
        ax.plot(self.df['t'].values, self.df['a'].values, 'b-', linewidth=2, label='Acceleration (m/s²)')

        # 添加控制限制线
        ax.axhline(y=0.5, color='g', linestyle=':', alpha=0.5)
        ax.axhline(y=-0.5, color='g', linestyle=':', alpha=0.5)
        ax.axhline(y=5.0, color='b', linestyle=':', alpha=0.5)
        ax.axhline(y=-5.0, color='b', linestyle=':', alpha=0.5)

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Control Input')
        ax.set_title('Control Inputs')
        ax.legend()
        ax.grid(True, alpha=0.3)

    def create_animation(self):
        """创建动画（使用原始方法）"""
        fig, ax = plt.subplots(figsize=(12, 8))

        # 设置坐标轴范围
        x_min, x_max = self.df['x'].min() - 5, self.df['x'].max() + 5
        y_min, y_max = self.df['y'].min() - 5, self.df['y'].max() + 5

        # 确保坐标轴比例合适
        x_range = x_max - x_min
        y_range = y_max - y_min
        if x_range > y_range:
            y_center = (y_min + y_max) / 2
            y_min = y_center - x_range / 2
            y_max = y_center + x_range / 2
        else:
            x_center = (x_min + x_max) / 2
            x_min = x_center - y_range / 2
            x_max = x_center + y_range / 2

        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_aspect('equal')

        # 绘制参考轨迹
        if self.has_reference:
            ax.plot(self.df['ref_x'].values, self.df['ref_y'].values, 'r--',
                   linewidth=1, alpha=0.7, label='Reference')

        # 场景特定的背景
        if self.scenario_type == "intersection":
            intersection_x = [0, 20, 20, 0, 0]
            intersection_y = [0, 0, 20, 20, 0]
            ax.plot(intersection_x, intersection_y, 'k--', alpha=0.3)

        # 初始化动画元素
        ego_vehicle, = ax.plot([], [], 'bo', markersize=8, label='Ego Vehicle')
        ego_trail, = ax.plot([], [], 'b-', alpha=0.5, linewidth=1)

        # 障碍物
        obstacle_vehicles = []
        obstacle_trails = []
        obstacle_colors = ['g', 'm', 'c', 'orange', 'purple']

        for i in range(self.num_obstacles):
            vehicle, = ax.plot([], [], 'o', markersize=6,
                             color=obstacle_colors[i % len(obstacle_colors)],
                             label=f'Vehicle {i+1}')
            trail, = ax.plot([], [], '-', alpha=0.3, linewidth=1,
                           color=obstacle_colors[i % len(obstacle_colors)])
            obstacle_vehicles.append(vehicle)
            obstacle_trails.append(trail)

        # 信息文本
        info_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                           verticalalignment='top', fontsize=10,
                           bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(f'{self.get_scenario_title()} - Animation')
        ax.legend()
        ax.grid(True, alpha=0.3)

        def animate(frame):
            if frame >= len(self.df):
                return [ego_vehicle, ego_trail, info_text] + obstacle_vehicles + obstacle_trails

            # 更新主车
            ego_x, ego_y = self.df['x'].iloc[frame], self.df['y'].iloc[frame]
            ego_vehicle.set_data([ego_x], [ego_y])

            # 更新主车轨迹
            trail_start = max(0, frame - 50)  # 显示最近50个点的轨迹
            ego_trail.set_data(self.df['x'].iloc[trail_start:frame+1],
                              self.df['y'].iloc[trail_start:frame+1])

            # 更新障碍物
            for i in range(self.num_obstacles):
                ox_col = f'obs{i}_ox'
                oy_col = f'obs{i}_oy'
                if ox_col in self.df.columns and oy_col in self.df.columns:
                    obs_x, obs_y = self.df[ox_col].iloc[frame], self.df[oy_col].iloc[frame]
                    obstacle_vehicles[i].set_data([obs_x], [obs_y])
                    obstacle_trails[i].set_data(self.df[ox_col].iloc[trail_start:frame+1],
                                              self.df[oy_col].iloc[trail_start:frame+1])

            # 更新信息文本
            t = self.df['t'].iloc[frame]
            v = self.df['v'].iloc[frame]
            h_min = self.df['h_min'].iloc[frame] if self.has_h_min else 0
            safety_status = "SAFE" if h_min > 0 else "UNSAFE"
            info_text.set_text(f'Time: {t:.2f}s\nVelocity: {v:.2f}m/s\nSafety: {h_min:.3f} ({safety_status})')

            return [ego_vehicle, ego_trail, info_text] + obstacle_vehicles + obstacle_trails

        # 创建动画
        anim = animation.FuncAnimation(fig, animate, frames=len(self.df),
                                     interval=50, blit=True, repeat=True)

        # 保存动画
        output_file = self.get_output_filename("_animation.gif")
        print(f"Saving animation to: {output_file}")
        anim.save(output_file, writer='pillow', fps=20)
        print(f"Animation saved successfully!")

        plt.show()

    def print_statistics(self):
        """打印统计信息"""
        print(f"\n=== {self.get_scenario_title()} Analysis ===")
        print(f"Data source: {self.csv_file}")
        print(f"Simulation duration: {self.df['t'].iloc[-1]:.2f} seconds")
        print(f"Final position: ({self.df['x'].iloc[-1]:.2f}, {self.df['y'].iloc[-1]:.2f})")
        print(f"Average velocity: {self.df['v'].mean():.2f} m/s")
        print(f"Max steering angle: {abs(self.df['steer']).max():.3f} rad")
        print(f"Max acceleration: {abs(self.df['a']).max():.3f} m/s²")

        # 分析关键时刻
        if self.has_h_min:
            critical_moments = self.df[self.df['h_min'] < 0.5]
            if not critical_moments.empty():
                print(f"\nCritical avoidance moments:")
                print(f"Time range: {critical_moments['t'].min():.2f}s - {critical_moments['t'].max():.2f}s")
                print(f"Average velocity during avoidance: {critical_moments['v'].mean():.2f} m/s")

            # 安全性分析
            unsafe_moments = self.df[self.df['h_min'] < 0]
            if not unsafe_moments.empty():
                print(f"\nWARNING: Unsafe moments detected!")
                print(f"Unsafe time range: {unsafe_moments['t'].min():.2f}s - {unsafe_moments['t'].max():.2f}s")
                print(f"Minimum safety distance: {unsafe_moments['h_min'].min():.3f}")
            else:
                print(f"\nSafety: All constraints satisfied throughout simulation")

    def get_output_filename(self, suffix=""):
        """生成输出文件名"""
        base_name = os.path.splitext(os.path.basename(self.csv_file))[0]
        return f"{base_name}{suffix}"

    def get_scenario_title(self):
        """获取场景标题"""
        if self.scenario_type == "straight":
            return "Straight Line Avoidance Scenario"
        elif self.scenario_type == "intersection":
            return "Right Turn Intersection Scenario"
        else:
            return f"Scenario: {os.path.basename(self.csv_file)}"

def main():
    parser = argparse.ArgumentParser(description='Multiple-Shooting DPCBF Simulation Visualizer')
    parser.add_argument('--csv', type=str, help='Specific CSV file to visualize')
    parser.add_argument('--scenario', choices=['straight', 'intersection', 'auto'],
                       default='auto', help='Scenario type')
    parser.add_argument('--static-only', action='store_true',
                       help='Generate only static analysis plots')
    parser.add_argument('--animation-only', action='store_true',
                       help='Generate only animation')
    parser.add_argument('--list', action='store_true',
                       help='List available CSV files')

    args = parser.parse_args()

    # 列出可用的CSV文件
    if args.list:
        csv_files = glob.glob("*output*.csv")
        if not csv_files:
            print("No CSV files found")
            return

        print("Available CSV files:")
        for i, file in enumerate(csv_files):
            print(f"  {i+1}. {file}")
        return

    try:
        visualizer = MSVisualizer(args.csv, args.scenario)
        visualizer.load_data()
        visualizer.print_statistics()

        if not args.animation_only:
            visualizer.create_static_analysis()

        if not args.static_only:
            visualizer.create_animation()

    except FileNotFoundError as e:
        print(f"Error: {e}")
        print("Please run the simulation first with:")
        print("  ./test_multiple_shooting --scenario <type> --save_csv=true")
        print("Then visualize with:")
        print("  python3 viz/ms_visualizer.py")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()