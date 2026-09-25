#!/bin/bash
# Go2+Piper Gazebo + MoveIt 工作空间设置
# 用法: source setup_workspace.sh

WS_SRC="/home/lili/3d-navi/src"
GAZEBO_DIR="/home/lili/LeggedManip_Lab/gazebo"

# 链接包到工作空间
ln -sf "$GAZEBO_DIR/go2_piper_description" "$WS_SRC/go2_piper_description" 2>/dev/null
ln -sf "$GAZEBO_DIR/go2_piper_moveit_config" "$WS_SRC/go2_piper_moveit_config" 2>/dev/null
ln -sf "/home/lili/piper_ws/src/piper_urdf/robot_description/piper_l_description" "$WS_SRC/piper_l_description" 2>/dev/null
ln -sf "/home/lili/piper_ws/src/piper_urdf/robot_description/realsense2_description" "$WS_SRC/realsense2_description" 2>/dev/null

echo "[INFO] 已链接包到 $WS_SRC/"
echo "[INFO] 编译: cd /home/lili/3d-navi && catkin_make"
echo ""
echo "启动命令:"
echo "  roslaunch go2_piper_moveit_config gazebo_moveit.launch    # Gazebo+MoveIt"
echo "  roslaunch go2_piper_moveit_config demo.launch              # MoveIt+RViz (无Gazebo)"
echo "  python gazebo/go2_piper_moveit_config/scripts/grasp_pipeline.py --grasp --x 0.4 --z 0.2"
