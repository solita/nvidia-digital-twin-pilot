FROM ros:humble

RUN apt-get update && apt-get install -y \
    ros-humble-rmw-fastrtps-cpp \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Build warehouse_msgs
WORKDIR /ros2_ws/src
COPY warehouse_msgs/ warehouse_msgs/

WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --packages-select warehouse_msgs && \
    echo 'source /ros2_ws/install/setup.bash' >> /etc/bash.bashrc"

# Install Python deps
COPY warehouse-manager/requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Copy warehouse manager code
COPY warehouse-manager/ /app/
WORKDIR /app
