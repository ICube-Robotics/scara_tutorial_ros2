# Installation Guide

This guide will walk you through setting up the SCARA ROS2 EtherCAT Workshop on your system.

## System Requirements

!!! info "Required Setup"
    - **Operating System**: Ubuntu 24.04 LTS
    - **ROS Distribution**: ROS2 Jazzy
    - **Disk Space**: ~5 GB free space
    - **Memory**: 4 GB RAM minimum (8 GB recommended)

## Step-by-Step Installation

### 1. Install ROS2 Jazzy

Follow the official ROS2 installation instructions for Ubuntu 24.04:

```bash
# Add ROS2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy Desktop
sudo apt update
sudo apt install ros-jazzy-desktop -y
```

For detailed instructions, visit the [official ROS2 documentation](https://docs.ros.org/en/jazzy/Installation.html).

### 2. Source ROS2 Environment

After installation, source your ROS2 environment:

```bash
source /opt/ros/jazzy/setup.bash
```

!!! tip "Auto-sourcing"
    If you're only using ROS2 Jazzy, add the source command to your `~/.bashrc` file:
    ```bash
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    ```

### 3. Install Colcon

Colcon is the build tool for ROS2 workspaces:

```bash
sudo apt install python3-colcon-common-extensions
```

### 4. Create a ROS2 Workspace

Create a workspace directory for the tutorial:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### 5. Clone the Repository

Clone the workshop repository into your workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/ICube-Robotics/ecat_ros2_workshop.git
```

### 6. Install Dependencies

Use `rosdep` to install all required dependencies:

```bash
cd ~/ros2_ws
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -y -r
```

!!! note
    If this is your first time using `rosdep`, you may need to initialize it:
    ```bash
    sudo rosdep init
    rosdep update
    ```

### 7. Build the Workspace

Build the workspace using colcon:

```bash
cd ~/ros2_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
```

!!! tip "Build Options"
    - `--cmake-args -DCMAKE_BUILD_TYPE=Release`: Optimizes the build for performance
    - `--symlink-install`: Creates symbolic links instead of copying files (useful during development)

### 8. Source the Workspace

After building, source the workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

!!! tip "Auto-sourcing Workspace"
    Add this to your `~/.bashrc` for automatic sourcing:
    ```bash
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
    ```

## Verify Installation

Test that everything is installed correctly:

```bash
# Check if packages are available
ros2 pkg list | grep scara

# Launch the SCARA robot (should open RViz2)
ros2 launch scara_bringup scara.launch.py
```

If RViz2 opens with the SCARA robot displayed, your installation is successful! 🎉

## Troubleshooting

### Common Issues

#### Missing Dependencies

If you encounter missing dependencies during build:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y -r --rosdistro jazzy
```

#### Build Errors

If the build fails, try cleaning the workspace and rebuilding:

```bash
cd ~/ros2_ws
rm -rf build install log
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
```

#### RViz2 Not Opening

If RViz2 doesn't open or displays errors:

```bash
# Install RViz2 if missing
sudo apt install ros-jazzy-rviz2

# Check OpenGL support
glxinfo | grep "OpenGL"
```

### Getting Help

If you're still experiencing issues:

1. Check the [GitHub Issues](https://github.com/ICube-Robotics/ecat_ros2_workshop/issues)
2. Review the [ROS2 troubleshooting guide](https://docs.ros.org/en/jazzy/Troubleshooting.html)
3. Contact the maintainers (see [Contacts](../about/contacts.md))

## Next Steps

Now that your environment is set up, you can:

- Try the [Docker setup](docker.md) as an alternative installation method
- Start with the [ros2_control Overview](../tutorials/r2c_overview.md)
- Jump into the [URDF Tutorial](../tutorials/urdf_tutorial.md)

---

Ready to start? [Begin with the ros2_control Overview :material-arrow-right:](../tutorials/r2c_overview.md){ .md-button .md-button--primary }
