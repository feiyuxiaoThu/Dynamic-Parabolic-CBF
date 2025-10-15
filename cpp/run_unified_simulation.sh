#!/bin/bash

# 统一仿真运行脚本
# 使用方法: ./run_unified_simulation.sh [scenario] [visualize]
# scenario: straight, intersection, both (默认: both)
# visualize: true, false (默认: true)

set -e

SCENARIO=${1:-both}
VISUALIZE=${2:-true}

echo "=========================================="
echo "Dynamic Parabolic CBF 统一仿真系统"
echo "=========================================="

# 检查构建目录
if [ ! -d "build" ]; then
    echo "创建构建目录..."
    mkdir build
fi

cd build

# 构建项目
echo "构建项目..."
cmake ..
make -j$(nproc)

echo "构建完成!"

# 运行仿真
echo "运行仿真场景: $SCENARIO"

if [ "$SCENARIO" = "straight" ] || [ "$SCENARIO" = "both" ]; then
    echo ""
    echo ">>> 运行直线避障场景..."
    ./unified_examples --scenario straight
    
    if [ "$VISUALIZE" = "true" ]; then
        echo "生成直线场景可视化..."
        cd ..
        python3 viz/unified_visualizer.py --scenario straight
        cd build
    fi
fi

if [ "$SCENARIO" = "intersection" ] || [ "$SCENARIO" = "both" ]; then
    echo ""
    echo ">>> 运行路口转弯场景..."
    ./unified_examples --scenario intersection
    
    if [ "$VISUALIZE" = "true" ]; then
        echo "生成路口场景可视化..."
        cd ..
        python3 viz/unified_visualizer.py --scenario intersection
        cd build
    fi
fi

echo ""
echo "=========================================="
echo "仿真完成!"
echo "=========================================="

# 显示输出文件
cd ..
echo "生成的文件:"
ls -la *.csv *.png *.gif 2>/dev/null || echo "没有找到输出文件"

echo ""
echo "使用说明:"
echo "1. 查看静态分析图: *_analysis.png"
echo "2. 查看动画: *_animation.gif"
echo "3. 查看原始数据: *.csv"
echo ""
echo "高级用法:"
echo "./run_unified_simulation.sh straight false  # 只运行直线场景，不生成可视化"
echo "./run_unified_simulation.sh intersection true  # 只运行路口场景，生成可视化"
echo "python3 viz/unified_visualizer.py --scenario both  # 对两个场景都生成可视化"