FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-cv-bridge \
    ros-humble-image-geometry \
    ros-humble-image-transport \
    ros-humble-tf-transformations \
    ros-humble-nav2-bringup \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-twist-mux \
    libopen3d-dev \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-opencv \
    python3-pip \
    python3-pytest \
    xterm \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir torch==2.2.2 --index-url https://download.pytorch.org/whl/cpu \
    && python3 -m pip install --no-cache-dir open3d -i https://pypi.tuna.tsinghua.edu.cn/simple

ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
WORKDIR /workspace/LeggedManip_Lab/ros2_ws

COPY entrypoint.sh /ros2_entrypoint.sh
RUN chmod +x /ros2_entrypoint.sh
ENTRYPOINT ["/ros2_entrypoint.sh"]
CMD ["bash"]
