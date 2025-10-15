#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import argparse
import os
from pathlib import Path

# 设置字体以支持中文显示
plt.rcParams['font.sans-serif'] = ['DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

class UnifiedVisualizer:
    def __init__(self, scenario_type="straight"):
        self.scenario_type = scenario_type
        self.setup_file_paths()
        
    def setup_file_paths(self):
        """设置文件路径"""
        if self.scenario_type == "straight":
            self.csv_file = "output_dpcbf.csv"
            self.output_prefix = "straight_line"
        elif self.scenario_type == "intersection":
            self.csv_file = "intersection_output.csv"
            self.output_prefix = "intersection"
        else:
            raise ValueError(f"Unknown scenario type: {self.scenario_type}")
    
    def load_data(self):
        """加载仿真数据"""
        if not os.path.exists(self.csv_file):
            raise FileNotFoundError(f"Data file not found: {self.csv_file}")
        
        self.df = pd.read_csv(self.csv_file)
        print(f"Loaded {len(self.df)} data points from {self.csv_file}")
        
    def create_static_analysis(self):
        """创建静态分析图"""
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 12))
        
        # 1. 轨迹图
        self.plot_trajectory(ax1)
        
        # 2. 速度曲线
        self.plot_velocity(ax2)
        
        # 3. 安全距离
        self.plot_safety_distance(ax3)
        
        # 4. 控制输入
        self.plot_control_inputs(ax4)
        
        plt.tight_layout()
        
        # 保存图片
        output_file = f"{self.output_prefix}_analysis.png"
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Static analysis saved to: {output_file}")
        
        plt.show()
        
    def plot_trajectory(self, ax):
        """绘制轨迹图"""
        # 主车轨迹
        ax.plot(self.df['x'].values, self.df['y'].values, 'b-', linewidth=2, label='Ego Vehicle')
        ax.plot(self.df['ref_x'].values, self.df['ref_y'].values, 'r--', linewidth=1, label='Reference')
        
        # 障碍物轨迹
        obstacle_colors = ['g', 'm', 'c', 'orange', 'purple']
        num_obstacles = self.get_num_obstacles()
        
        for i in range(num_obstacles):
            ox_col = f'obs{i}_ox'
            oy_col = f'obs{i}_oy'
            if ox_col in self.df.columns and oy_col in self.df.columns:
                ax.plot(self.df[ox_col].values, self.df[oy_col].values, 
                       color=obstacle_colors[i % len(obstacle_colors)], 
                       alpha=0.7, label=f'Vehicle {i+1}')
        
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
        """绘制速度曲线"""
        ax.plot(self.df['t'].values, self.df['v'].values, 'b-', linewidth=2, label='Actual Velocity')
        ax.axhline(y=6.0, color='r', linestyle='--', alpha=0.7, label='Reference Velocity')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (m/s)')
        ax.set_title('Velocity Profile')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
    def plot_safety_distance(self, ax):
        """绘制安全距离"""
        ax.plot(self.df['t'].values, self.df['h_min'].values, 'r-', linewidth=2)
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.7, label='Safety Boundary')
        ax.fill_between(self.df['t'].values, self.df['h_min'].values, 0, 
                       where=(self.df['h_min'].values < 0), alpha=0.3, color='red', label='Unsafe Region')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Min Safety Distance h_min')
        ax.set_title('Safety Distance')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
    def plot_control_inputs(self, ax):
        """绘制控制输入"""
        ax.plot(self.df['t'].values, self.df['steer'].values, 'g-', linewidth=2, label='Steering (rad)')
        ax.plot(self.df['t'].values, self.df['a'].values, 'b-', linewidth=2, label='Acceleration (m/s²)')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Control Input')
        ax.set_title('Control Inputs')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
    def create_animation(self):
        """创建动画"""
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
        num_obstacles = self.get_num_obstacles()
        obstacle_vehicles = []
        obstacle_trails = []
        obstacle_colors = ['g', 'm', 'c', 'orange', 'purple']
        
        for i in range(num_obstacles):
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
            for i in range(num_obstacles):
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
            h_min = self.df['h_min'].iloc[frame]
            info_text.set_text(f'Time: {t:.2f}s\nVelocity: {v:.2f}m/s\nSafety: {h_min:.3f}')
            
            return [ego_vehicle, ego_trail, info_text] + obstacle_vehicles + obstacle_trails
        
        # 创建动画
        anim = animation.FuncAnimation(fig, animate, frames=len(self.df), 
                                     interval=50, blit=True, repeat=True)
        
        # 保存动画
        output_file = f"{self.output_prefix}_animation.gif"
        print(f"Saving animation to: {output_file}")
        anim.save(output_file, writer='pillow', fps=20)
        print(f"Animation saved successfully!")
        
        plt.show()
        
    def print_statistics(self):
        """打印统计信息"""
        print(f"\n=== {self.get_scenario_title()} Analysis ===")
        print(f"Simulation duration: {self.df['t'].iloc[-1]:.2f} seconds")
        print(f"Final position: ({self.df['x'].iloc[-1]:.2f}, {self.df['y'].iloc[-1]:.2f})")
        print(f"Average velocity: {self.df['v'].mean():.2f} m/s")
        print(f"Min safety distance: {self.df['h_min'].min():.3f}")
        print(f"Max steering angle: {self.df['steer'].abs().max():.3f} rad")
        print(f"Max acceleration: {self.df['a'].abs().max():.3f} m/s²")
        
        # 分析关键时刻
        critical_moments = self.df[self.df['h_min'] < 0.5]
        if not critical_moments.empty:
            print(f"\nCritical avoidance moments:")
            print(f"Time range: {critical_moments['t'].min():.2f}s - {critical_moments['t'].max():.2f}s")
            print(f"Average velocity during avoidance: {critical_moments['v'].mean():.2f} m/s")
        
        # 安全性分析
        unsafe_moments = self.df[self.df['h_min'] < 0]
        if not unsafe_moments.empty:
            print(f"\nWARNING: Unsafe moments detected!")
            print(f"Unsafe time range: {unsafe_moments['t'].min():.2f}s - {unsafe_moments['t'].max():.2f}s")
            print(f"Minimum safety distance: {unsafe_moments['h_min'].min():.3f}")
        else:
            print(f"\nSafety: All constraints satisfied throughout simulation")
    
    def get_num_obstacles(self):
        """获取障碍物数量"""
        obstacle_cols = [col for col in self.df.columns if col.startswith('obs') and col.endswith('_ox')]
        return len(obstacle_cols)
    
    def get_scenario_title(self):
        """获取场景标题"""
        if self.scenario_type == "straight":
            return "Straight Line Avoidance Scenario"
        elif self.scenario_type == "intersection":
            return "Right Turn Intersection Scenario"
        else:
            return "Unknown Scenario"

def main():
    parser = argparse.ArgumentParser(description='Unified DPCBF Simulation Visualizer')
    parser.add_argument('--scenario', choices=['straight', 'intersection', 'both'], 
                       default='straight', help='Scenario type to visualize')
    parser.add_argument('--static-only', action='store_true', 
                       help='Generate only static analysis plots')
    parser.add_argument('--animation-only', action='store_true', 
                       help='Generate only animation')
    
    args = parser.parse_args()
    
    scenarios = ['straight', 'intersection'] if args.scenario == 'both' else [args.scenario]
    
    for scenario in scenarios:
        print(f"\n{'='*50}")
        print(f"Processing {scenario} scenario...")
        print(f"{'='*50}")
        
        try:
            visualizer = UnifiedVisualizer(scenario)
            visualizer.load_data()
            visualizer.print_statistics()
            
            if not args.animation_only:
                visualizer.create_static_analysis()
            
            if not args.static_only:
                visualizer.create_animation()
                
        except FileNotFoundError as e:
            print(f"Error: {e}")
            print(f"Please run the simulation first with: --scenario {scenario}")
        except Exception as e:
            print(f"Error processing {scenario} scenario: {e}")

if __name__ == "__main__":
    main()