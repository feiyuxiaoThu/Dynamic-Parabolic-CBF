#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import argparse
import os
import glob
from pathlib import Path

# 设置字体以支持中文显示
plt.rcParams['font.sans-serif'] = ['DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

class EnhancedVisualizer:
    def __init__(self, csv_file=None, scenario_type="straight"):
        self.csv_file = csv_file
        self.scenario_type = scenario_type

        # 如果没有指定文件，尝试自动查找
        if self.csv_file is None:
            self.auto_find_csv_file()

        if self.csv_file is None:
            raise FileNotFoundError("Could not find any CSV file to visualize")

    def auto_find_csv_file(self):
        """自动查找CSV文件"""
        # 定义可能的文件名模式
        patterns = [
            "test_ms_{}_output.csv".format(self.scenario_type),
            "output_dpcbf.csv",
            "intersection_output.csv",
            "*output*.csv"
        ]

        for pattern in patterns:
            matches = glob.glob(pattern)
            if matches:
                # 选择最新修改的文件
                self.csv_file = max(matches, key=os.path.getmtime)
                print(f"Auto-detected CSV file: {self.csv_file}")
                return

        # 如果还找不到，尝试在当前目录和上级目录查找
        for search_dir in [".", "..", "../build"]:
            for pattern in patterns:
                search_pattern = os.path.join(search_dir, pattern)
                matches = glob.glob(search_pattern)
                if matches:
                    self.csv_file = max(matches, key=os.path.getmtime)
                    print(f"Auto-detected CSV file: {self.csv_file}")
                    return

    def load_data(self):
        """加载仿真数据"""
        if not os.path.exists(self.csv_file):
            raise FileNotFoundError(f"Data file not found: {self.csv_file}")

        self.df = pd.read_csv(self.csv_file)
        print(f"Loaded {len(self.df)} data points from {self.csv_file}")

        # 检测数据格式
        self.detect_data_format()

    def detect_data_format(self):
        """检测数据格式和特征"""
        self.num_obstacles = 0
        obstacle_cols = [col for col in self.df.columns if col.startswith('obs') and col.endswith('_ox')]
        self.num_obstacles = len(obstacle_cols)

        print(f"Detected {self.num_obstacles} obstacles")
        print(f"Data columns: {list(self.df.columns)}")

        # 检测是否有特殊字段
        self.has_reference = 'ref_x' in self.df.columns and 'ref_y' in self.df.columns
        self.has_theta = 'theta' in self.df.columns
        self.has_safety = 'h_min' in self.df.columns

        print(f"Has reference trajectory: {self.has_reference}")
        print(f"Has orientation data: {self.has_theta}")
        print(f"Has safety data: {self.has_safety}")

    def create_enhanced_static_analysis(self):
        """创建增强的静态分析图"""
        fig = plt.figure(figsize=(16, 10))

        # 创建网格布局
        gs = fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)

        # 1. 主轨迹图（占据左上2x2区域）
        ax1 = fig.add_subplot(gs[0:2, 0:2])
        self.plot_trajectory(ax1)

        # 2. 速度曲线
        ax2 = fig.add_subplot(gs[0, 2])
        self.plot_velocity(ax2)

        # 3. 控制输入
        ax3 = fig.add_subplot(gs[1, 2])
        self.plot_control_inputs(ax3)

        # 4. 安全距离
        ax4 = fig.add_subplot(gs[2, 0])
        if self.has_safety:
            self.plot_safety_distance(ax4)
        else:
            ax4.text(0.5, 0.5, 'Safety data not available',
                    ha='center', va='center', transform=ax4.transAxes)
            ax4.set_title('Safety Distance')

        # 5. 轨向角变化
        ax5 = fig.add_subplot(gs[2, 1])
        if self.has_theta:
            self.plot_orientation(ax5)
        else:
            ax5.text(0.5, 0.5, 'Orientation data not available',
                    ha='center', va='center', transform=ax5.transAxes)
            ax5.set_title('Orientation')

        # 6. 障碍物距离
        ax6 = fig.add_subplot(gs[2, 2])
        self.plot_obstacle_distances(ax6)

        plt.suptitle(f'{self.get_scenario_title()} - Enhanced Analysis', fontsize=16)

        # 保存图片
        output_file = self.get_output_filename("_enhanced_analysis.png")
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Enhanced analysis saved to: {output_file}")

        plt.show()

    def create_controller_comparison(self):
        """创建控制器对比分析（如果有多个CSV文件）"""
        # 查找所有相关的CSV文件
        pattern = "*{}*output*.csv".format(self.scenario_type)
        csv_files = glob.glob(pattern)

        if len(csv_files) < 2:
            print("Not enough CSV files for comparison analysis")
            return

        fig, axes = plt.subplots(2, 2, figsize=(15, 10))

        for csv_file in csv_files:
            df = pd.read_csv(csv_file)
            controller_name = self.extract_controller_name(csv_file)

            # 轨迹对比
            axes[0, 0].plot(df['x'], df['y'], label=controller_name, alpha=0.7)

            # 速度对比
            axes[0, 1].plot(df['t'], df['v'], label=controller_name, alpha=0.7)

            # 安全距离对比
            if 'h_min' in df.columns:
                axes[1, 0].plot(df['t'], df['h_min'], label=controller_name, alpha=0.7)

            # 控制输入对比
            axes[1, 1].plot(df['t'], df['steer'], label=f'{controller_name} steer', alpha=0.7)
            axes[1, 1].plot(df['t'], df['a'], label=f'{controller_name} accel', alpha=0.7)

        # 设置图表标题和标签
        axes[0, 0].set_title('Trajectory Comparison')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].axis('equal')

        axes[0, 1].set_title('Velocity Comparison')
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)

        axes[1, 0].set_title('Safety Distance Comparison')
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)

        axes[1, 1].set_title('Control Input Comparison')
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)

        plt.tight_layout()

        output_file = self.get_output_filename("_controller_comparison.png")
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Controller comparison saved to: {output_file}")

        plt.show()

    def extract_controller_name(self, csv_file):
        """从文件名中提取控制器名称"""
        if 'single' in csv_file.lower():
            return 'Single-Shooting'
        elif 'multiple' in csv_file.lower() or 'ms_' in csv_file.lower():
            return 'Multiple-Shooting'
        elif 'adaptive' in csv_file.lower():
            return 'Adaptive'
        else:
            return os.path.basename(csv_file).replace('.csv', '')

    def plot_trajectory(self, ax):
        """绘制轨迹图"""
        # 主车轨迹
        ax.plot(self.df['x'].values, self.df['y'].values, 'b-', linewidth=2.5, label='Ego Vehicle', zorder=5)

        # 参考轨迹
        if self.has_reference:
            ax.plot(self.df['ref_x'].values, self.df['ref_y'].values, 'r--', linewidth=1.5, alpha=0.7, label='Reference', zorder=3)

        # 障碍物轨迹
        obstacle_colors = plt.cm.Set3(np.linspace(0, 1, self.num_obstacles))

        for i in range(self.num_obstacles):
            ox_col = f'obs{i}_ox'
            oy_col = f'obs{i}_oy'
            if ox_col in self.df.columns and oy_col in self.df.columns:
                # 绘制障碍物轨迹
                ax.plot(self.df[ox_col].values, self.df[oy_col].values,
                       color=obstacle_colors[i], alpha=0.6, linewidth=1.5,
                       label=f'Obstacle {i+1}', zorder=2)

                # 标记障碍物起点和终点
                ax.plot(self.df[ox_col].iloc[0], self.df[oy_col].iloc[0],
                       'o', color=obstacle_colors[i], markersize=6, zorder=4)
                ax.plot(self.df[ox_col].iloc[-1], self.df[oy_col].iloc[-1],
                       's', color=obstacle_colors[i], markersize=6, zorder=4)

        # 标记主车起点和终点
        ax.plot(self.df['x'].iloc[0], self.df['y'].iloc[0], 'go', markersize=10,
               label='Start', zorder=6, markeredgecolor='darkgreen', markeredgewidth=2)
        ax.plot(self.df['x'].iloc[-1], self.df['y'].iloc[-1], 'ro', markersize=10,
               label='End', zorder=6, markeredgecolor='darkred', markeredgewidth=2)

        # 场景特定的装饰
        if self.scenario_type == "intersection":
            # 绘制路口区域
            intersection_x = [0, 20, 20, 0, 0]
            intersection_y = [0, 0, 20, 20, 0]
            ax.plot(intersection_x, intersection_y, 'k--', alpha=0.5, linewidth=2, label='Intersection')

            # 添加道路标线
            ax.axhline(y=10, color='gray', linestyle=':', alpha=0.5)
            ax.axvline(x=10, color='gray', linestyle=':', alpha=0.5)

        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title(f'{self.get_scenario_title()} - Trajectory', fontsize=14, fontweight='bold')
        ax.legend(loc='best', fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.axis('equal')

        # 添加比例尺
        if self.scenario_type == "straight":
            ax.plot([2, 12], [self.df['y'].min() - 3, self.df['y'].min() - 3], 'k-', linewidth=3)
            ax.text(7, self.df['y'].min() - 4, '10 m', ha='center', fontsize=10, fontweight='bold')

    def plot_velocity(self, ax):
        """绘制速度曲线"""
        ax.plot(self.df['t'].values, self.df['v'].values, 'b-', linewidth=2, label='Actual Velocity')

        # 添加参考速度线
        ref_velocity = self.df['v'].mean()
        ax.axhline(y=ref_velocity, color='r', linestyle='--', alpha=0.7,
                  label=f'Avg: {ref_velocity:.2f} m/s')

        # 添加安全区域
        ax.fill_between(self.df['t'].values, 0, self.df['v'].values, alpha=0.3, color='blue')

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (m/s)')
        ax.set_title('Velocity Profile')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_ylim(bottom=0)

    def plot_safety_distance(self, ax):
        """绘制安全距离"""
        ax.plot(self.df['t'].values, self.df['h_min'].values, 'r-', linewidth=2, label='Min Safety Distance')
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.7, label='Safety Boundary')

        # 标记不安全区域
        unsafe_mask = self.df['h_min'] < 0
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
        """绘制控制输入"""
        ax.plot(self.df['t'].values, self.df['steer'].values, 'g-', linewidth=2, label='Steering (rad)')
        ax.plot(self.df['t'].values, self.df['a'].values, 'b-', linewidth=2, label='Acceleration (m/s²)')

        # 添加控制限制线
        ax.axhline(y=0.5, color='g', linestyle=':', alpha=0.5, label='Steer limit')
        ax.axhline(y=-0.5, color='g', linestyle=':', alpha=0.5)
        ax.axhline(y=5.0, color='b', linestyle=':', alpha=0.5, label='Accel limit')
        ax.axhline(y=-5.0, color='b', linestyle=':', alpha=0.5)

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Control Input')
        ax.set_title('Control Inputs')
        ax.legend()
        ax.grid(True, alpha=0.3)

    def plot_orientation(self, ax):
        """绘制方向角变化"""
        if self.has_theta:
            # 转换为角度制
            theta_deg = np.degrees(self.df['theta'])
            ax.plot(self.df['t'].values, theta_deg, 'purple', linewidth=2, label='Heading Angle')

            # 标记转向点
            theta_diff = np.diff(theta_deg)
            turn_indices = np.where(np.abs(theta_diff) > 1)[0]
            for idx in turn_indices:
                ax.plot(self.df.loc[idx, 't'], theta_deg[idx], 'ro', markersize=4)

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Heading Angle (degrees)')
        ax.set_title('Orientation')
        ax.legend()
        ax.grid(True, alpha=0.3)

    def plot_obstacle_distances(self, ax):
        """绘制到障碍物的距离"""
        times = []
        distances = []
        obstacle_labels = []

        for i in range(self.num_obstacles):
            ox_col = f'obs{i}_ox'
            oy_col = f'obs{i}_oy'

            if ox_col in self.df.columns and oy_col in self.df.columns:
                # 计算到障碍物的距离
                distances_to_obs = np.sqrt((self.df['x'] - self.df[ox_col])**2 +
                                         (self.df['y'] - self.df[oy_col])**2)

                ax.plot(self.df['t'], distances_to_obs, label=f'Obs {i+1}', alpha=0.7)

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Distance to Obstacle (m)')
        ax.set_title('Obstacle Distances')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_ylim(bottom=0)

    def create_enhanced_animation(self):
        """创建增强的动画"""
        fig, ax = plt.subplots(figsize=(14, 10))

        # 设置坐标轴范围
        x_min, x_max = self.df['x'].min() - 3, self.df['x'].max() + 3
        y_min, y_max = self.df['y'].min() - 3, self.df['y'].max() + 3

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
            # 绘制路口
            rect = plt.Rectangle((0, 0), 20, 20, linewidth=2,
                               edgecolor='black', facecolor='lightgray', alpha=0.3)
            ax.add_patch(rect)

            # 添加道路标线
            ax.axhline(y=10, color='white', linestyle='--', alpha=0.8, linewidth=2)
            ax.axvline(x=10, color='white', linestyle='--', alpha=0.8, linewidth=2)

        # 初始化动画元素
        ego_vehicle, = ax.plot([], [], 'o', markersize=10, label='Ego Vehicle',
                              markeredgecolor='darkblue', markeredgewidth=2, zorder=10)
        ego_trail, = ax.plot([], [], '-', alpha=0.6, linewidth=2, zorder=5)

        # 安全区域圆圈
        safety_circle = plt.Circle((0, 0), 1, fill=False, edgecolor='red',
                                  linestyle='--', alpha=0.5, zorder=3)
        ax.add_patch(safety_circle)

        # 障碍物
        obstacle_vehicles = []
        obstacle_trails = []
        obstacle_circles = []
        obstacle_colors = plt.cm.Set3(np.linspace(0, 1, self.num_obstacles))

        for i in range(self.num_obstacles):
            vehicle, = ax.plot([], [], 's', markersize=8,
                             color=obstacle_colors[i], label=f'Obstacle {i+1}',
                             markeredgecolor='black', markeredgewidth=1, zorder=9)
            trail, = ax.plot([], [], '-', alpha=0.4, linewidth=1.5,
                           color=obstacle_colors[i], zorder=4)

            # 障碍物安全圆圈
            circle = plt.Circle((0, 0), 0.5, fill=True, facecolor=obstacle_colors[i],
                               alpha=0.3, zorder=2)
            ax.add_patch(circle)
            obstacle_circles.append(circle)

            obstacle_vehicles.append(vehicle)
            obstacle_trails.append(trail)

        # 信息面板
        info_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                           verticalalignment='top', fontsize=11,
                           bbox=dict(boxstyle='round', facecolor='white', alpha=0.9),
                           family='monospace')

        # 图例
        ax.legend(loc='upper right', fontsize=10)

        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title(f'{self.get_scenario_title()} - Enhanced Animation', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)

        def animate(frame):
            if frame >= len(self.df):
                return [ego_vehicle, ego_trail, safety_circle, info_text] + obstacle_vehicles + obstacle_trails + obstacle_circles

            # 更新主车
            ego_x, ego_y = self.df['x'].iloc[frame], self.df['y'].iloc[frame]
            ego_vehicle.set_data([ego_x], [ego_y])

            # 更新主车轨迹
            trail_start = max(0, frame - 30)
            ego_trail.set_data(self.df['x'].iloc[trail_start:frame+1],
                              self.df['y'].iloc[trail_start:frame+1])

            # 更新安全圆圈
            safety_circle.center = (ego_x, ego_y)

            # 更新障碍物
            for i in range(self.num_obstacles):
                ox_col = f'obs{i}_ox'
                oy_col = f'obs{i}_oy'
                if ox_col in self.df.columns and oy_col in self.df.columns:
                    obs_x, obs_y = self.df[ox_col].iloc[frame], self.df[oy_col].iloc[frame]
                    obstacle_vehicles[i].set_data([obs_x], [obs_y])
                    obstacle_trails[i].set_data(self.df[ox_col].iloc[trail_start:frame+1],
                                              self.df[oy_col].iloc[trail_start:frame+1])

                    # 更新障碍物安全圆圈
                    obstacle_circles[i].center = (obs_x, obs_y)

            # 更新信息文本
            t = self.df['t'].iloc[frame]
            v = self.df['v'].iloc[frame]
            steer = self.df['steer'].iloc[frame]
            accel = self.df['a'].iloc[frame]

            info_lines = [
                f"Time: {t:6.2f}s",
                f"Speed: {v:5.2f} m/s",
                f"Steer: {steer:6.3f} rad",
                f"Accel: {accel:6.2f} m/s²"
            ]

            if self.has_safety:
                h_min = self.df['h_min'].iloc[frame]
                safety_status = "SAFE" if h_min > 0 else "UNSAFE"
                safety_color = "green" if h_min > 0 else "red"
                info_lines.append(f"Safety: {h_min:6.3f} ({safety_status})")

            info_text.set_text('\n'.join(info_lines))

            # 根据安全状态改变安全圆圈颜色
            if self.has_safety:
                h_min = self.df['h_min'].iloc[frame]
                safety_circle.set_edgecolor('red' if h_min < 0 else 'green')
                safety_circle.set_linewidth(2 if h_min < 0 else 1)

            return [ego_vehicle, ego_trail, safety_circle, info_text] + obstacle_vehicles + obstacle_trails + obstacle_circles

        # 创建动画
        anim = animation.FuncAnimation(fig, animate, frames=len(self.df),
                                     interval=50, blit=True, repeat=True)

        # 保存动画
        output_file = self.get_output_filename("_enhanced_animation.gif")
        print(f"Saving enhanced animation to: {output_file}")
        anim.save(output_file, writer='pillow', fps=20)
        print(f"Enhanced animation saved successfully!")

        plt.show()

    def get_output_filename(self, suffix=""):
        """生成输出文件名"""
        base_name = os.path.splitext(os.path.basename(self.csv_file))[0]
        return f"{base_name}{suffix}"

    def get_scenario_title(self):
        """获取场景标题"""
        if "straight" in self.csv_file.lower():
            return "Straight Line Avoidance Scenario"
        elif "intersection" in self.csv_file.lower():
            return "Right Turn Intersection Scenario"
        else:
            return f"Scenario: {os.path.basename(self.csv_file)}"

    def print_enhanced_statistics(self):
        """打印增强统计信息"""
        print(f"\n{'='*60}")
        print(f"Enhanced Analysis: {self.get_scenario_title()}")
        print(f"{'='*60}")
        print(f"Data source: {self.csv_file}")
        print(f"Simulation duration: {self.df['t'].iloc[-1]:.2f} seconds")
        print(f"Data points: {len(self.df)}")
        print(f"Number of obstacles: {self.num_obstacles}")

        # 位置统计
        start_pos = (self.df['x'].iloc[0], self.df['y'].iloc[0])
        end_pos = (self.df['x'].iloc[-1], self.df['y'].iloc[-1])
        total_distance = 0
        for i in range(1, len(self.df)):
            dx = self.df['x'].iloc[i] - self.df['x'].iloc[i-1]
            dy = self.df['y'].iloc[i] - self.df['y'].iloc[i-1]
            total_distance += np.sqrt(dx**2 + dy**2)

        print(f"\nTrajectory Analysis:")
        print(f"  Start position: ({start_pos[0]:.2f}, {start_pos[1]:.2f})")
        print(f"  End position: ({end_pos[0]:.2f}, {end_pos[1]:.2f})")
        print(f"  Total distance: {total_distance:.2f} m")
        print(f"  Average velocity: {self.df['v'].mean():.2f} m/s")
        print(f"  Max velocity: {self.df['v'].max():.2f} m/s")
        print(f"  Min velocity: {self.df['v'].min():.2f} m/s")

        # 控制输入统计
        print(f"\nControl Input Analysis:")
        print(f"  Max steering: {self.df['steer'].abs().max():.3f} rad ({np.degrees(self.df['steer'].abs().max()):.1f}°)")
        print(f"  Max acceleration: {self.df['a'].abs().max():.3f} m/s²")
        print(f"  Steering std: {self.df['steer'].std():.3f} rad")
        print(f"  Acceleration std: {self.df['a'].std():.3f} m/s²")

        # 安全性分析
        if self.has_safety:
            print(f"\nSafety Analysis:")
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
                print(f"  Safety: All constraints satisfied ✓")

        # 方向分析
        if self.has_theta:
            print(f"\nOrientation Analysis:")
            theta_deg = np.degrees(self.df['theta'])
            print(f"  Heading change: {theta_deg.iloc[-1] - theta_deg.iloc[0]:.1f}°")
            print(f"  Max angular velocity: {np.degrees(np.diff(theta_deg)).max():.1f}°/step")

def main():
    parser = argparse.ArgumentParser(description='Enhanced DPCBF Simulation Visualizer')
    parser.add_argument('--csv', type=str, help='Specific CSV file to visualize')
    parser.add_argument('--scenario', choices=['straight', 'intersection'],
                       default='straight', help='Scenario type (used if --csv not specified)')
    parser.add_argument('--static-only', action='store_true',
                       help='Generate only static analysis plots')
    parser.add_argument('--animation-only', action='store_true',
                       help='Generate only animation')
    parser.add_argument('--comparison', action='store_true',
                       help='Generate controller comparison plots')
    parser.add_argument('--auto', action='store_true',
                       help='Auto-detect and visualize all available CSV files')

    args = parser.parse_args()

    if args.auto:
        # 自动模式：处理所有可用的CSV文件
        csv_files = glob.glob("*output*.csv")
        if not csv_files:
            print("No CSV files found for auto-visualization")
            return

        print(f"Auto-detected {len(csv_files)} CSV files:")
        for csv_file in csv_files:
            print(f"  - {csv_file}")

        # 首先生成对比分析
        if len(csv_files) >= 2:
            try:
                visualizer = EnhancedVisualizer(csv_files[0])
                visualizer.create_controller_comparison()
            except Exception as e:
                print(f"Could not generate comparison: {e}")

        # 为每个文件生成单独的分析
        for csv_file in csv_files:
            print(f"\n{'='*50}")
            print(f"Processing: {csv_file}")
            print(f"{'='*50}")

            try:
                visualizer = EnhancedVisualizer(csv_file)
                visualizer.load_data()
                visualizer.print_enhanced_statistics()

                if not args.animation_only:
                    visualizer.create_enhanced_static_analysis()

                if not args.static_only:
                    visualizer.create_enhanced_animation()

            except Exception as e:
                print(f"Error processing {csv_file}: {e}")
    else:
        # 单文件模式
        try:
            visualizer = EnhancedVisualizer(args.csv, args.scenario)
            visualizer.load_data()
            visualizer.print_enhanced_statistics()

            if args.comparison:
                visualizer.create_controller_comparison()
            elif not args.animation_only:
                visualizer.create_enhanced_static_analysis()

            if not args.static_only and not args.comparison:
                visualizer.create_enhanced_animation()

        except FileNotFoundError as e:
            print(f"Error: {e}")
            print("Please run the simulation first or specify the correct CSV file")
        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    main()