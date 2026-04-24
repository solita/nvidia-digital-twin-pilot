#!/usr/bin/env bash
# ros2_sidecar.sh — Start a ROS2 Humble sidecar container on the instance.
# Shares the network with Isaac Sim via --net=host so DDS topics are visible.
# Run on instance: bash ~/ros2_sidecar.sh
set -euo pipefail

CONTAINER_NAME="ros2-sidecar"
ROS_IMAGE="osrf/ros:humble-desktop"

# Check if already running
if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "ROS2 sidecar already running. Exec into it:"
    echo "  docker exec -it ${CONTAINER_NAME} bash"
    exit 0
fi

# Remove stopped container if exists
docker rm -f "${CONTAINER_NAME}" 2>/dev/null || true

echo "Pulling ${ROS_IMAGE} (first time only)..."
docker pull "${ROS_IMAGE}"

echo "Starting ROS2 sidecar container..."
# Write FastDDS profile to disable SHM transport (Isaac Sim container uses ipc=private)
FASTDDS_XML="/tmp/fastdds_no_shm.xml"
cat > "${FASTDDS_XML}" << 'XMLEOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_only</transport_id>
      <type>UDPv4</type>
      <sendBufferSize>1048576</sendBufferSize>
      <receiveBufferSize>1048576</receiveBufferSize>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="participant_profile" is_default_profile="true">
    <rtps>
      <useBuiltinTransports>false</useBuiltinTransports>
      <userTransports>
        <transport_id>udp_only</transport_id>
      </userTransports>
    </rtps>
  </participant>
</profiles>
XMLEOF

docker run -d \
    --name "${CONTAINER_NAME}" \
    --net=host \
    --ipc=host \
    -e ROS_DOMAIN_ID=0 \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_no_shm.xml \
    -v "${FASTDDS_XML}:/tmp/fastdds_no_shm.xml:ro" \
    "${ROS_IMAGE}" \
    bash -c "sleep infinity"

echo "Waiting for container to start..."
sleep 2

# Copy the obstacle avoidance node into the sidecar
if [[ -f "$HOME/obstacle_avoidance_node.py" ]]; then
    docker exec "${CONTAINER_NAME}" mkdir -p /ros2_ws
    docker cp "$HOME/obstacle_avoidance_node.py" "${CONTAINER_NAME}:/ros2_ws/obstacle_avoidance_node.py"
    echo "Copied obstacle_avoidance_node.py -> container:/ros2_ws/"
fi

echo ""
echo "ROS2 sidecar running. Usage:"
echo ""
echo "  # Interactive shell:"
echo "  docker exec -it ${CONTAINER_NAME} bash"
echo ""
echo "  # Quick commands:"
echo "  docker exec ${CONTAINER_NAME} bash -c 'source /opt/ros/humble/setup.bash && ros2 topic list'"
echo "  docker exec -it ${CONTAINER_NAME} bash -c 'source /opt/ros/humble/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard'"
echo ""
echo "  # Run obstacle avoidance:"
echo "  docker exec ${CONTAINER_NAME} bash -c 'source /opt/ros/humble/setup.bash && python3 /ros2_ws/obstacle_avoidance_node.py'"
echo ""
echo "  # Stop:"
echo "  docker rm -f ${CONTAINER_NAME}"
