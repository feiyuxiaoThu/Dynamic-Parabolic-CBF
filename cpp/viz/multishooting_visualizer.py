#!/usr/bin/env python3
"""
Multiple-Shooting DPCBF Controller Visualization Tool
Fixed version for better compatibility with modern pandas/numpy versions
"""

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import argparse
import os
import glob
from pathlib import Path

# 设置字体和绘图参数
plt.rcParams['font.sans-serif'] = ['DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False
plt.rcParams['figure.dpi'] = 100

class MultishootingVisualizer:
    def __init__(self, csv_file=None, scenario_type="auto"):
        self.csv_file = csv_file
        self.scenario_type = scenario_type

        # 数据检测标志
        self.has_theta = False
        self.has_h_min = False
        self.has_reference = False
        self.num_obstacles = 0

        # 自动查找CSV文件
        if self.csv_file is None:
            self.auto_find_csv_file()

        if self.csv_file is None:
            raise FileNotFoundError("Could not find any CSV file to visualize")

        # 自动检测场景类型
        if self.scenario_type == "auto":
            self.detect_scenario_type()

    def auto_find_csv_file(self):
        """自动查找CSV文件"""
        candidates = [
            "test_ms_intersection_output.csv",
            "test_ms_straight_output.csv",
            "intersection_output.csv",
            "output_dpcbf.csv",
            "*straight*output*.csv",
            "*intersection*output*.csv",
            "*output*.csv"
        ]

        for pattern in candidates:
            if pattern.startswith("*"):
                matches = glob.glob(pattern)
            else:
                matches = [pattern] if os.path.exists(pattern) else []

            if matches:
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
            self.scenario_type = "straight"
        print(f"Auto-detected scenario type: {self.scenario_type}")

    def load_data(self):
        """加载和验证数据"""
        if not os.path.exists(self.csv_file):
            raise FileNotFoundError(f"Data file not found: {self.csv_file}")

        self.df = pd.read_csv(self.csv_file)
        print(f"Loaded {len(self.df)} data points from {self.csv_file}")

        self.check_data_columns()

    def check_data_columns(self):
        """检查数据列完整性"""
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
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        fig.suptitle(f'{self.get_scenario_title()} - Comprehensive Analysis', fontsize=16, fontweight='bold')

        # 展平axes数组以便索引
        axes = axes.flatten()

        # 1. 轨迹图
        self.plot_trajectory(axes[0])

        # 2. 速度曲线
        self.plot_velocity(axes[1])

        # 3. 安全距离
        if self.has_h_min:
            self.plot_safety_distance(axes[2])
        else:
            axes[2].text(0.5, 0.5, 'Safety data not available',
                         ha='center', va='center', transform=axes[2].transAxes,
                         fontsize=12, bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.5))
            axes[2].set_title('Safety Distance')
            axes[2].grid(True, alpha=0.3)

        # 4. 控制输入
        self.plot_control_inputs(axes[3])

        # 5. 方向角变化（如果有theta数据）
        if self.has_theta:
            self.plot_orientation(axes[4])
        else:
            axes[4].text(0.5, 0.5, 'Orientation data not available',
                         ha='center', va='center', transform=axes[4].transAxes,
                         fontsize=12, bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.5))
            axes[4].set_title('Vehicle Orientation')
            axes[4].grid(True, alpha=0.3)

        # 6. 障碍物距离
        if self.num_obstacles > 0:
            self.plot_obstacle_distances(axes[5])
        else:
            axes[5].text(0.5, 0.5, 'No obstacle data',
                         ha='center', va='center', transform=axes[5].transAxes,
                         fontsize=12, bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.5))
            axes[5].set_title('Obstacle Distances')
            axes[5].grid(True, alpha=0.3)

        plt.tight_layout()

        # 保存图片
        output_file = self.get_output_filename("_analysis.png")
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Static analysis saved to: {output_file}")

        plt.show()

    def plot_trajectory(self, ax):
        """绘制轨迹图"""
        # 主车轨迹
        ax.plot(self.df['x'].values, self.df['y'].values, 'b-', linewidth=2.5,
                label='Ego Vehicle', alpha=0.8)

        # 参考轨迹
        if self.has_reference:
            ax.plot(self.df['ref_x'].values, self.df['ref_y'].values, 'r--',
                    linewidth=1.5, alpha=0.7, label='Reference')

        # 障碍物轨迹
        obstacle_colors = plt.cm.Set1(np.linspace(0, 1, min(self.num_obstacles, 9)))

        for i in range(self.num_obstacles):
            ox_col = f'obs{i}_ox'
            oy_col = f'obs{i}_oy'
            if ox_col in self.df.columns and oy_col in self.df.columns:
                color = obstacle_colors[i % len(obstacle_colors)]
                ax.plot(self.df[ox_col].values, self.df[oy_col].values,
                       color=color, alpha=0.7, linewidth=2, label=f'Vehicle {i+1}')

                # 标记障碍物起点和终点
                ax.plot(self.df[ox_col].iloc[0], self.df[oy_col].iloc[0],
                       'o', color=color, markersize=8)
                ax.plot(self.df[ox_col].iloc[-1], self.df[oy_col].iloc[-1],
                       's', color=color, markersize=8)

        # 标记主车起点和终点
        ax.plot(self.df['x'].iloc[0], self.df['y'].iloc[0], 'go',
                markersize=12, label='Start', zorder=5)
        ax.plot(self.df['x'].iloc[-1], self.df['y'].iloc[-1], 'ro',
                markersize=12, label='End', zorder=5)

        # 场景特定装饰
        if self.scenario_type == "intersection":
            # 绘制路口区域
            intersection_x = [0, 20, 20, 0, 0]
            intersection_y = [0, 0, 20, 20, 0]
            ax.plot(intersection_x, intersection_y, 'k--', alpha=0.3, linewidth=1)

        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title('Vehicle Trajectory', fontsize=14, fontweight='bold')
        ax.legend(loc='best', fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.axis('equal')

    def plot_velocity(self, ax):
        """绘制速度曲线"""
        ax.plot(self.df['t'].values, self.df['v'].values, 'b-', linewidth=2, label='Velocity')

        # 添加参考线
        avg_velocity = self.df['v'].mean()
        ax.axhline(y=avg_velocity, color='r', linestyle='--', alpha=0.7,
                   label=f'Average: {avg_velocity:.2f} m/s')

        ax.fill_between(self.df['t'].values, 0, self.df['v'].values, alpha=0.3, color='blue')

        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Velocity (m/s)', fontsize=12)
        ax.set_title('Velocity Profile', fontsize=14, fontweight='bold')
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)

    def plot_safety_distance(self, ax):
        """绘制安全距离"""
        ax.plot(self.df['t'].values, self.df['h_min'].values, 'r-', linewidth=2, label='Safety Distance')
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.7, linewidth=2, label='Safety Boundary')

        # 标记不安全区域
        unsafe_mask = self.df['h_min'].values < 0
        if unsafe_mask.any():
            ax.fill_between(self.df['t'].values, self.df['h_min'].values, 0,
                           where=unsafe_mask, alpha=0.3, color='red', label='Unsafe Region')

            # 标记最危险点
            min_idx = self.df['h_min'].idxmin()
            min_time = self.df.loc[min_idx, 't']
            min_value = self.df.loc[min_idx, 'h_min']
            ax.plot(min_time, min_value, 'ro', markersize=10,
                   label=f'Min: {min_value:.3f}')

        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Safety Distance h_min', fontsize=12)
        ax.set_title('Safety Analysis', fontsize=14, fontweight='bold')
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)

    def plot_control_inputs(self, ax):
        """绘制控制输入"""
        # 创建双y轴
        ax2 = ax.twinx()

        # 转向角（左轴）
        line1 = ax.plot(self.df['t'].values, self.df['steer'].values, 'g-',
                       linewidth=2, label='Steering (rad)', alpha=0.8)

        # 加速度（右轴）
        line2 = ax2.plot(self.df['t'].values, self.df['a'].values, 'b-',
                        linewidth=2, label='Acceleration (m/s²)', alpha=0.8)

        # 添加控制限制线
        ax.axhline(y=0.5, color='g', linestyle=':', alpha=0.5)
        ax.axhline(y=-0.5, color='g', linestyle=':', alpha=0.5)
        ax2.axhline(y=5.0, color='b', linestyle=':', alpha=0.5)
        ax2.axhline(y=-5.0, color='b', linestyle=':', alpha=0.5)

        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Steering Angle (rad)', fontsize=12, color='g')
        ax2.set_ylabel('Acceleration (m/s²)', fontsize=12, color='b')
        ax.tick_params(axis='y', labelcolor='g')
        ax2.tick_params(axis='y', labelcolor='b')

        # 合并图例
        lines = line1 + line2
        labels = [l.get_label() for l in lines]
        ax.legend(lines, labels, loc='best', fontsize=10)

        ax.set_title('Control Inputs', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)

    def plot_orientation(self, ax):
        """绘制方向角变化"""
        if self.has_theta:
            ax.plot(self.df['t'].values, np.degrees(self.df['theta'].values),
                   'purple', linewidth=2, label='Heading Angle')

            # 计算角速度
            theta_deg = np.degrees(self.df['theta'].values)
            dt_diff = self.df['t'].diff().dropna().values
            angular_velocity = np.diff(theta_deg) / dt_diff
            ax2 = ax.twinx()
            ax2.plot(self.df['t'].values[1:], angular_velocity,
                    'orange', linewidth=1.5, alpha=0.7, label='Angular Velocity')

            ax.set_xlabel('Time (s)', fontsize=12)
            ax.set_ylabel('Heading Angle (degrees)', fontsize=12, color='purple')
            ax2.set_ylabel('Angular Velocity (deg/s)', fontsize=12, color='orange')
            ax.tick_params(axis='y', labelcolor='purple')
            ax2.tick_params(axis='y', labelcolor='orange')

            ax.legend(loc='upper left', fontsize=10)
            ax2.legend(loc='upper right', fontsize=10)

        ax.set_title('Vehicle Orientation', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)

    def plot_obstacle_distances(self, ax):
        """绘制障碍物距离"""
        ego_x = self.df['x'].values
        ego_y = self.df['y'].values

        for i in range(self.num_obstacles):
            ox_col = f'obs{i}_ox'
            oy_col = f'obs{i}_oy'
            if ox_col in self.df.columns and oy_col in self.df.columns:
                obs_x = self.df[ox_col].values
                obs_y = self.df[oy_col].values

                # 计算距离
                distances = np.sqrt((ego_x - obs_x)**2 + (ego_y - obs_y)**2)
                ax.plot(self.df['t'].values, distances, linewidth=2,
                       label=f'Vehicle {i+1} distance')

        ax.axhline(y=2.0, color='r', linestyle='--', alpha=0.7,
                  label='Safety Threshold')

        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Distance (m)', fontsize=12)
        ax.set_title('Obstacle Distances', fontsize=14, fontweight='bold')
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)

    def create_animation(self):
        """创建动画"""
        fig, ax = plt.subplots(figsize=(12, 10))

        # 设置坐标轴范围
        x_min, x_max = self.df['x'].min() - 5, self.df['x'].max() + 5
        y_min, y_max = self.df['y'].min() - 5, self.df['y'].max() + 5

        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        ax.set_aspect('equal')

        # 绘制参考轨迹
        if self.has_reference:
            ax.plot(self.df['ref_x'].values, self.df['ref_y'].values, 'r--',
                   linewidth=1.5, alpha=0.7, label='Reference')

        # 场景背景
        if self.scenario_type == "intersection":
            intersection_x = [0, 20, 20, 0, 0]
            intersection_y = [0, 0, 20, 20, 0]
            ax.plot(intersection_x, intersection_y, 'k--', alpha=0.3, linewidth=1)

        # 初始化动画元素
        ego_vehicle, = ax.plot([], [], 'o', markersize=10, color='blue',
                              label='Ego Vehicle', zorder=5)
        ego_trail, = ax.plot([], [], '-', alpha=0.5, linewidth=2, color='blue')

        # 障碍物
        obstacle_vehicles = []
        obstacle_trails = []
        obstacle_colors = plt.cm.Set1(np.linspace(0, 1, min(self.num_obstacles, 9)))

        for i in range(self.num_obstacles):
            vehicle, = ax.plot([], [], 'o', markersize=8,
                             color=obstacle_colors[i % len(obstacle_colors)],
                             label=f'Vehicle {i+1}', zorder=4)
            trail, = ax.plot([], [], '-', alpha=0.3, linewidth=1.5,
                           color=obstacle_colors[i % len(obstacle_colors)])
            obstacle_vehicles.append(vehicle)
            obstacle_trails.append(trail)

        # 信息文本
        info_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                           verticalalignment='top', fontsize=11,
                           bbox=dict(boxstyle='round', facecolor='white', alpha=0.9))

        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title(f'{self.get_scenario_title()} - Animation', fontsize=14, fontweight='bold')
        ax.legend(loc='upper right', fontsize=10)
        ax.grid(True, alpha=0.3)

        def animate(frame):
            if frame >= len(self.df):
                return [ego_vehicle, ego_trail, info_text] + obstacle_vehicles + obstacle_trails

            # 更新主车
            ego_x, ego_y = self.df['x'].iloc[frame], self.df['y'].iloc[frame]
            ego_vehicle.set_data([ego_x], [ego_y])

            # 更新主车轨迹
            trail_start = max(0, frame - 30)
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
            steer = self.df['steer'].iloc[frame]
            a = self.df['a'].iloc[frame]

            info_lines = [
                f'Time: {t:.2f}s',
                f'Velocity: {v:.2f} m/s',
                f'Steering: {steer:.3f} rad',
                f'Acceleration: {a:.2f} m/s²'
            ]

            if self.has_h_min:
                h_min = self.df['h_min'].iloc[frame]
                safety_status = "SAFE" if h_min > 0 else "UNSAFE"
                info_lines.append(f'Safety: {h_min:.3f} ({safety_status})')

            info_text.set_text('\n'.join(info_lines))

            return [ego_vehicle, ego_trail, info_text] + obstacle_vehicles + obstacle_trails

        # 创建动画
        anim = animation.FuncAnimation(fig, animate, frames=len(self.df),
                                     interval=50, blit=True, repeat=True)

        # 保存动画
        output_file = self.get_output_filename("_animation.gif")
        print(f"Saving animation to: {output_file}")
        try:
            anim.save(output_file, writer='pillow', fps=20)
            print(f"Animation saved successfully!")
        except Exception as e:
            print(f"Warning: Could not save animation: {e}")
            print("Please install pillow: pip install pillow")

        plt.show()

    def print_statistics(self):
        """打印详细统计信息"""
        print(f"\n{'='*60}")
        print(f"MULTIPLE-SHOOTING ANALYSIS: {self.get_scenario_title().upper()}")
        print(f"{'='*60}")
        print(f"Data source: {self.csv_file}")
        print(f"Simulation duration: {self.df['t'].iloc[-1]:.2f} seconds")
        print(f"Data points: {len(self.df)}")
        print(f"Number of obstacles: {self.num_obstacles}")

        # 轨迹分析
        print(f"\nTRAJECTORY ANALYSIS:")
        print(f"  Start position: ({self.df['x'].iloc[0]:.2f}, {self.df['y'].iloc[0]:.2f})")
        print(f"  End position: ({self.df['x'].iloc[-1]:.2f}, {self.df['y'].iloc[-1]:.2f})")

        # 计算总距离
        distances = np.sqrt(np.diff(self.df['x'].values)**2 + np.diff(self.df['y'].values)**2)
        total_distance = np.sum(distances)
        print(f"  Total distance: {total_distance:.2f} m")
        print(f"  Average velocity: {self.df['v'].mean():.2f} m/s")
        print(f"  Max velocity: {self.df['v'].max():.2f} m/s")
        print(f"  Min velocity: {self.df['v'].min():.2f} m/s")

        # 控制输入分析
        print(f"\nCONTROL INPUT ANALYSIS:")
        print(f"  Max steering: {abs(self.df['steer']).max():.3f} rad ({np.degrees(abs(self.df['steer']).max()):.1f}°)")
        print(f"  Max acceleration: {abs(self.df['a']).max():.3f} m/s²")
        print(f"  Steering std: {self.df['steer'].std():.3f} rad")
        print(f"  Acceleration std: {self.df['a'].std():.3f} m/s²")

        # 安全性分析
        if self.has_h_min:
            print(f"\nSAFETY ANALYSIS:")
            print(f"  Min safety distance: {self.df['h_min'].min():.3f}")
            print(f"  Mean safety distance: {self.df['h_min'].mean():.3f}")

            critical_moments = self.df[self.df['h_min'] < 0.5]
            if not critical_moments.empty:
                print(f"  Critical moments (h < 0.5): {len(critical_moments)} points")
                print(f"  Critical time range: {critical_moments['t'].min():.2f}s - {critical_moments['t'].max():.2f}s")

            unsafe_moments = self.df[self.df['h_min'] < 0]
            if not unsafe_moments.empty:
                print(f"  WARNING: {len(unsafe_moments)} unsafe moments detected!")
                print(f"  Unsafe time range: {unsafe_moments['t'].min():.2f}s - {unsafe_moments['t'].max():.2f}s")
                print(f"  Minimum safety distance: {unsafe_moments['h_min'].min():.3f}")
            else:
                print(f"  Safety: All constraints satisfied throughout simulation")

        # 方向角分析
        if self.has_theta:
            print(f"\nORIENTATION ANALYSIS:")
            theta_deg = np.degrees(self.df['theta'].values)
            print(f"  Heading change: {abs(theta_deg[-1] - theta_deg[0]):.1f}°")
            if len(theta_deg) > 1:
                dt_diff = self.df['t'].diff().dropna().values  # 移除NaN值
                angular_velocity = np.diff(theta_deg) / dt_diff
                print(f"  Max angular velocity: {abs(angular_velocity).max():.1f}°/step")

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
    parser = argparse.ArgumentParser(description='Multiple-Shooting DPCBF Visualization Tool')
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

    # 列出可用CSV文件
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
        # 创建可视化器
        visualizer = MultishootingVisualizer(args.csv, args.scenario)

        # 加载数据
        visualizer.load_data()

        # 打印统计信息
        visualizer.print_statistics()

        # 生成可视化
        if not args.animation_only:
            visualizer.create_static_analysis()

        if not args.static_only:
            visualizer.create_animation()

    except FileNotFoundError as e:
        print(f"Error: {e}")
        print("\nPlease run the simulation first:")
        print("  ./test_multiple_shooting --scenario <type> --save_csv=true")
        print("Then visualize with:")
        print("  python3 viz/multishooting_visualizer.py")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()