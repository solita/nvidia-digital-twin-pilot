FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        docker.io \
        ros-humble-nav-msgs \
        ros-humble-sensor-msgs \
        ros-humble-geometry-msgs \
        ros-humble-rosgraph-msgs \
    && pip3 install --no-cache-dir aiohttp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY dashboard_server.py index.html ./

ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Disable FastDDS SHM transport — Isaac Sim container uses ipc=private
RUN printf '<?xml version="1.0" encoding="UTF-8" ?>\n\
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">\n\
  <transport_descriptors>\n\
    <transport_descriptor>\n\
      <transport_id>udp_only</transport_id>\n\
      <type>UDPv4</type>\n\
      <sendBufferSize>1048576</sendBufferSize>\n\
      <receiveBufferSize>1048576</receiveBufferSize>\n\
    </transport_descriptor>\n\
  </transport_descriptors>\n\
  <participant profile_name="participant_profile" is_default_profile="true">\n\
    <rtps>\n\
      <useBuiltinTransports>false</useBuiltinTransports>\n\
      <userTransports>\n\
        <transport_id>udp_only</transport_id>\n\
      </userTransports>\n\
    </rtps>\n\
  </participant>\n\
</profiles>\n' > /tmp/fastdds_no_shm.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_no_shm.xml

EXPOSE 8080

CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && python3 dashboard_server.py"]
