#!/bin/bash
# LeggedManip Lab - Docker 环境管理脚本
# 已自动加载到 ~/.bashrc，开箱即用
# 命令: isaaclab_enter / isaaclab_train / isaaclab_play / isaaclab_stop

CONTAINER_NAME="isaac-sim"
ISAAC_SIM_IMAGE="nvcr.io/nvidia/isaac-sim:5.1.0"
PROJECT_ROOT="/home/lili/LeggedManip_Lab"
GO2ARM_ROOT="/home/lili/Go2Arm_sim2sim"
GO2ARM_MOUNT_PATH="/workspace/Go2Arm_sim2sim"
ISAACLAB_ROOT="/home/lili/IsaacLab"
ISAAC_SIM_PATH="/isaac-sim"
ISAACLAB_MOUNT_PATH="/workspace/IsaacLab"
PROJECT_MOUNT_PATH="/workspace/LeggedManip_Lab"

isaaclab_start() {
    if docker inspect -f '{{.State.Running}}' $CONTAINER_NAME 2>/dev/null | grep -q true; then
        return 0
    fi
    echo "[isaaclab] 启动 Isaac Sim 容器..."
    docker rm -f $CONTAINER_NAME 2>/dev/null
    # Read-only access is only used to build the Isaac-specific VBC URDF;
    # the existing Gazebo/ROS workspace remains untouched.
    docker run --gpus all -d \
        --name $CONTAINER_NAME \
        --network host \
        -e DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        -e ACCEPT_EULA=Y \
        -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
        -v $PROJECT_ROOT:$PROJECT_MOUNT_PATH \
        -v $GO2ARM_ROOT:/workspace/Go2Arm_sim2sim:ro \
        -v $ISAACLAB_ROOT:$ISAACLAB_MOUNT_PATH \
        --ipc=host \
        $ISAAC_SIM_IMAGE sleep infinity
}

isaaclab_setup() {
    isaaclab_start
    docker exec $CONTAINER_NAME bash -c '
        source '$ISAAC_SIM_PATH'/setup_python_env.sh
        export PATH='$ISAAC_SIM_PATH'/kit/python/bin:$PATH
        export ISAAC_PATH='$ISAAC_SIM_PATH'
        export EXP_PATH='$ISAAC_SIM_PATH'/apps
        export CARB_APP_PATH='$ISAAC_SIM_PATH'/kit
        python3 '$PROJECT_MOUNT_PATH'/scripts/prepare_go2_piper_vbc_urdf.py \
            --input '$GO2ARM_MOUNT_PATH'/ros2_ws/src/go2_piper_description/urdf/go2_piper_ros2.urdf \
            --output /tmp/go2_piper_vbc.urdf \
            --mesh-root '$GO2ARM_MOUNT_PATH'/ros2_ws/src/go2_piper_description/legacy/meshes
        python3 -m ensurepip --upgrade --default-pip 2>/dev/null
        if [ ! -f /tmp/IsaacLab/.installed ]; then
            rm -rf /tmp/IsaacLab 2>/dev/null
            cp -r '$ISAACLAB_MOUNT_PATH' /tmp/IsaacLab
            cd /tmp/IsaacLab && ln -sf '$ISAAC_SIM_PATH' _isaac_sim
            python3 -m pip install -q -e source/isaaclab -e source/isaaclab_assets -e source/isaaclab_mimic -e source/isaaclab_rl -e source/isaaclab_tasks
            python3 -m pip install -q rsl-rl-lib mujoco scipy pyyaml psutil prettytable gymnasium
            touch /tmp/IsaacLab/.installed
        fi
        if [ ! -f /tmp/LeggedManip_Lab/.installed ]; then
            rm -rf /tmp/LeggedManip_Lab 2>/dev/null
            cp -r '$PROJECT_MOUNT_PATH'/source/LeggedManip_Lab /tmp/LeggedManip_Lab
            cp /tmp/go2_piper_vbc.urdf /tmp/LeggedManip_Lab/LeggedManip_Lab/assets/go2_piper/go2_piper_vbc.urdf
            python3 -m pip install -q -e /tmp/LeggedManip_Lab
            touch /tmp/LeggedManip_Lab/.installed
        fi
    ' 2>&1 | grep -v "WARNING\|Warning\|No protocol"
}

isaaclab_enter() {
    isaaclab_setup
    docker exec -it $CONTAINER_NAME bash -c '
        source '$ISAAC_SIM_PATH'/setup_python_env.sh
        export PATH='$ISAAC_SIM_PATH'/kit/python/bin:$PATH
        export ISAAC_PATH='$ISAAC_SIM_PATH'
        export EXP_PATH='$ISAAC_SIM_PATH'/apps
        export CARB_APP_PATH='$ISAAC_SIM_PATH'/kit
        export GIT_PYTHON_REFRESH=quiet
        cd '$PROJECT_MOUNT_PATH'
        exec bash
    '
}

isaaclab_run() {
    isaaclab_setup
    docker exec $CONTAINER_NAME bash -c '
        source '$ISAAC_SIM_PATH'/setup_python_env.sh
        export PATH='$ISAAC_SIM_PATH'/kit/python/bin:$PATH
        export ISAAC_PATH='$ISAAC_SIM_PATH'
        export EXP_PATH='$ISAAC_SIM_PATH'/apps
        export CARB_APP_PATH='$ISAAC_SIM_PATH'/kit
        export GIT_PYTHON_REFRESH=quiet
        cd '$PROJECT_MOUNT_PATH'
        '"$1"'
    '
}

isaaclab_train() {
    local task="${1:-GO2-PIPER-Flat}"
    local headless="${2:-true}"
    local num_envs="${3:-4096}"
    local max_iter="${4:-5000}"
    local headless_flag=""
    [ "$headless" = "true" ] && headless_flag="--headless"
    isaaclab_run "python3 scripts/rsl_rl/train.py --task $task --num_envs $num_envs $headless_flag --max_iterations $max_iter"
}

isaaclab_play() {
    local task="${1:-GO2-PIPER-Flat}"
    local headless="${2:-true}"
    local headless_flag=""
    [ "$headless" = "true" ] && headless_flag="--headless"
    isaaclab_run "python3 scripts/rsl_rl/play.py --task $task $headless_flag"
}

isaaclab_list_envs() {
    isaaclab_run "python3 scripts/list_envs.py"
}

isaaclab_stop() {
    docker stop $CONTAINER_NAME 2>/dev/null
    docker rm $CONTAINER_NAME 2>/dev/null
    echo "[isaaclab] 容器已停止"
}

echo "[isaaclab] 已加载 (isaaclab_enter / train / play / stop)"
