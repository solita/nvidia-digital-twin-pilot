FROM ros:humble

RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-rmw-fastrtps-cpp \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Build warehouse_msgs
WORKDIR /ros2_ws/src
COPY warehouse_msgs/ warehouse_msgs/
COPY vehicle-controller/ vehicle_controller/

WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install && \
    echo 'source /ros2_ws/install/setup.bash' >> /etc/bash.bashrc"

RUN pip3 install --no-cache-dir redis

WORKDIR /ros2_ws
