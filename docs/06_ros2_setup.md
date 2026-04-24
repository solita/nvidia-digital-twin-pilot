## Instructions on how to setup Nvidia ROS

### Prerequisites 
* Ubuntu 24.04
* Nvidia driver 580+
Prerequisites for Nvidia ROS can be found in the official installation guide: https://nvidia-isaac-ros.github.io/getting_started/

### Installing Nvidia ROS 
Below are the steps to install Nvidia ROS on your system and can be also found at https://nvidia-isaac-ros.github.io/getting_started/
1. Configure Isaac ROS Apt Repository
```bash
# Set the locale on your host system to UTF-8
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Install the dependencies
sudo apt update && sudo apt install curl gnupg
sudo apt install software-properties-common
sudo add-apt-repository universe

# Source the Isaac ROS Apt Repository
k="/usr/share/keyrings/nvidia-isaac-ros.gpg"
curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo gpg --dearmor \
    | sudo tee -a $k > /dev/null
f="/etc/apt/sources.list.d/nvidia-isaac-ros.list"
sudo touch $f
s="deb [signed-by=$k] https://isaac.download.nvidia.com/isaac-ros/release-4.3 noble main"
grep -qxF "$s" $f || echo "$s" | sudo tee -a $f

sudo apt-get update
```

2. Install and configure nvidia-container-toolkit for Docker using the official instructions:
[Install NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#with-apt-ubuntu-debian)
[Configure Docker runtime](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#configuring-docker)

3. Initialize Isaac ROS CLI
```bash
# Install the Isaac ROS CLI
sudo apt-get install isaac-ros-cli

# Add your user to the docker group
sudo usermod -aG docker $USER
newgrp docker

# Restart docker
sudo systemctl daemon-reload && sudo systemctl restart docker

# Run Docker pre-flight checks
docker info | grep -E "Runtimes|Default Runtime"
docker run --rm hello-world
docker run --rm --gpus all ubuntu:24.04 bash -lc 'echo "NVIDIA runtime OK"'

# Initialize the Isaac ROS CLI:
sudo isaac-ros init docker
```

4. Running Isaac ROS
```bash
# Run the Isaac ROS CLI:
isaac-ros activate  
```
This command will start a Docker container with the Isaac ROS environment. You can then run any Isaac ROS packages or examples within this container.
From inside the container run:
```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```
This will set the Fast DDS transport to UDPv4, which is necessary for ROS data communication between the Isaac ROS container and other the the Isaac Sim container.

## Working with Nvidia ROS
A good example of how to work with NVIDIA ROS is the [Isaac ROS AprilTag](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_apriltag/isaac_ros_apriltag/index.html#quickstart), which also has a tutorial to [run it with Isaac Sim](https://nvidia-isaac-ros.github.io/concepts/fiducials/apriltag/tutorial_isaac_sim.html)

### Visualizing Rviz2
I an X server + VNC + noVNC inside the container, then port-forwarded the noVNC HTTP port. 
You can install and run this using the following commands, from **inside the Isaac ROS container**:
```bash
# Install the necessary packages for X server, VNC, and noVNC
sudo apt update
sudo apt install -y xvfb x11vnc novnc websockify openbox mesa-utils

# Start the stack
Xvfb :1 -screen 0 1440x900x24 +extension GLX +render -noreset &
export DISPLAY=:1

# 2. Minimal WM (so RViz2 has a window frame)
openbox &

# 3. VNC server on :1
x11vnc -display :1 -forever -shared -nopw -rfbport 5901 &

# 4. noVNC web bridge (HTTP 6080 -> VNC 5901)
websockify --web=/usr/share/novnc 6080 localhost:5901 &

# 5. run RViz2
rviz2
```

You'll have to allow access in Brev to port 6080. You can then access the noVNC interface by navigating to `http://localhost:6080` in your web browser. This will allow you to visualize RViz2 running inside the Isaac ROS container. 
Any subsequent runs of rviz2 from the isaac-ros container will also be visible in the noVNC interface.