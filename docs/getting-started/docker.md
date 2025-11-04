# Docker Setup

Docker provides an isolated, pre-configured environment for running the SCARA ROS2 EtherCAT Workshop without modifying your host system.

## Prerequisites

!!! info "Requirements"
    - [Docker](https://docs.docker.com/get-docker/) installed on your system
    - At least 8 GB of disk space
    - (Optional) [rocker](https://github.com/osrf/rocker) for GUI support

## Quick Start

### Basic Docker Setup

1. **Clone the repository** (if you haven't already):

```bash
git clone https://github.com/ICube-Robotics/ecat_ros2_workshop.git
cd ecat_ros2_workshop
```

2. **Build the Docker image**:

```bash
docker build --tag ecat_ros2_workshop:jazzy --file .docker/Dockerfile .
```

!!! note "Build Time"
    The first build may take 10-20 minutes depending on your internet connection and system performance.

3. **Run the container**:

```bash
docker run ecat_ros2_workshop:jazzy ros2 launch scara_bringup scara.launch.py
```

## Running with GUI

### Option 1: Using Rocker (Recommended)

[Rocker](https://github.com/osrf/rocker) provides seamless X11 forwarding and device access.

1. **Install rocker**:

```bash
sudo apt update
sudo apt install python3-rocker
```

2. **Run with GUI support**:

```bash
rocker --net=host --x11 --devices /dev/dri --user ecat_ros2_workshop:jazzy ros2 launch scara_bringup scara.launch.py
```

!!! info "Rocker Options"
    - `--net=host`: Uses host networking for ROS2 communication
    - `--x11`: Enables X11 forwarding for GUI applications
    - `--devices /dev/dri`: Mounts Direct Rendering Infrastructure for hardware-accelerated graphics (Intel GPUs)
    - `--user`: Runs as non-root user for better security

### Option 2: Using noVNC (Web-Based)

noVNC provides a web-based VNC client accessible through your browser - no X11 configuration needed!

1. **Build the noVNC image**:

```bash
# First ensure the base image is built
docker build --tag ecat_ros2_workshop:jazzy --file .docker/Dockerfile .

# Build the noVNC image
docker build --tag ecat_ros2_workshop:jazzy_novnc --file .docker/Dockerfile_novnc .
```

2. **Run the noVNC container**:

```bash
docker run --rm -p 6080:6080 -it ecat_ros2_workshop:jazzy_novnc
```

3. **Access the desktop**:

Open your web browser and navigate to:
```
http://localhost:6080/vnc.html
```

4. **Inside the noVNC session**, open a terminal and run:

```bash
cd ros2_dev/ecat_ros2_workshop/
source install/setup.bash
ros2 launch scara_bringup scara.launch.py
```

!!! tip "Browser Compatibility"
    noVNC works best with modern browsers (Chrome, Firefox, Edge). For optimal performance, use a desktop browser rather than mobile.

## Interactive Shell

To explore and interact with the environment:

```bash
docker run -it ecat_ros2_workshop:jazzy
```

Inside the container:

```bash
cd ros2_dev/ecat_ros2_workshop/
source install/setup.bash

# Now you can run any ROS2 commands
ros2 launch scara_bringup scara.launch.py
ros2 topic list
ros2 control list_controllers
```

## Development Workflow

### Mounting Local Code

To develop code on your host and run it in the container:

```bash
docker run -it \
  -v $(pwd):/workspace \
  -w /workspace \
  ecat_ros2_workshop:jazzy \
  bash
```

Inside the container:

```bash
# Rebuild after changes
colcon build
source install/setup.bash
ros2 launch scara_bringup scara.launch.py
```

### Persistent Container

Create a named container to preserve your changes:

```bash
# Create and start the container
docker run -it --name scara_dev ecat_ros2_workshop:jazzy bash

# Later, restart and attach to the same container
docker start scara_dev
docker attach scara_dev
```

## Hardware Acceleration

### NVIDIA GPU Support

For NVIDIA GPU support, use the NVIDIA Container Toolkit:

```bash
# Install NVIDIA Container Toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo systemctl restart docker

# Run with GPU support
docker run --gpus all -it ecat_ros2_workshop:jazzy
```

### Intel Integrated Graphics

For Intel integrated graphics (already included in rocker command):

```bash
rocker --devices /dev/dri ecat_ros2_workshop:jazzy
```

## Docker Compose (Advanced)

For more complex setups, create a `docker-compose.yml`:

```yaml
version: '3.8'

services:
  scara_workshop:
    image: ecat_ros2_workshop:jazzy
    container_name: scara_workshop
    stdin_open: true
    tty: true
    network_mode: host
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ./:/workspace
    devices:
      - /dev/dri:/dev/dri
    command: bash
```

Run with:

```bash
xhost +local:docker
docker-compose up
```

## Troubleshooting

### Display Issues

If you encounter display errors with rocker:

```bash
# Allow Docker to connect to X server
xhost +local:docker

# Run rocker again
rocker --x11 ecat_ros2_workshop:jazzy
```

### Network Issues

If ROS2 nodes can't communicate:

```bash
# Use host networking
docker run --net=host -it ecat_ros2_workshop:jazzy
```

### Permission Denied

If you get permission errors:

```bash
# Add your user to docker group
sudo usermod -aG docker $USER

# Log out and back in for changes to take effect
```

### Container Size

To check Docker disk usage:

```bash
docker system df

# Clean up unused containers and images
docker system prune -a
```

## Comparison: Docker Methods

| Method | Pros | Cons | Best For |
|--------|------|------|----------|
| **Basic** | Simple, fast | No GUI | Headless testing |
| **Rocker** | Native performance, hardware acceleration | Requires X11 setup | Local development |
| **noVNC** | Works anywhere, no X11 needed | Slight latency | Remote access, web-based |

## Next Steps

Now that you have Docker set up:

- Continue with the [ros2_control Overview](../tutorials/r2c_overview.md)
- Try running the tutorials inside Docker
- Explore the [URDF Tutorial](../tutorials/urdf_tutorial.md)

---

**Tip**: Docker is great for trying things out, but for serious development, consider a [native installation](installation.md) for better performance.
