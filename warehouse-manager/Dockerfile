FROM ros:humble

# System dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

# Python dependencies
COPY warehouse-manager/requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# Build warehouse_msgs
WORKDIR /ros2_ws/src
COPY warehouse_msgs/ warehouse_msgs/

WORKDIR /ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --packages-select warehouse_msgs && \
    echo 'source /ros2_ws/install/setup.bash' >> /etc/bash.bashrc"

# Copy warehouse manager code
COPY warehouse-manager/ /app/
WORKDIR /app

EXPOSE 8000

# NOTE: use_sim_time=true is set in the WarehouseManagerNode constructor
# (parameter_overrides), NOT via CLI --ros-args. This means uvicorn can
# launch normally without passing ROS arguments.
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && uvicorn main:app --host 0.0.0.0 --port 8000"]
