# SCARA ROS2 EtherCAT Workshop

[![Documentation](https://img.shields.io/badge/docs-mkdocs-blue)](https://icube-robotics.github.io/ecat_ros2_workshop/)
[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![License](https://img.shields.io/github/license/ICube-Robotics/ecat_ros2_workshop)](LICENSE)

The [ros2_control](https://control.ros.org) framework is a real-time control framework designed for general robotics applications that is gaining increasing attention for both research and industrial purposes.

This comprehensive tutorial teaches the basic concepts of controlling a robot using ros2_control and EtherCAT.

## 📚 Documentation

**Full documentation is available at: [https://icube-robotics.github.io/ecat_ros2_workshop/](https://icube-robotics.github.io/ecat_ros2_workshop/)**

## 📖 Tutorial Contents

- [ros2_control Overview](https://icube-robotics.github.io/ecat_ros2_workshop/tutorials/r2c_overview/)
- [Write a URDF description of a SCARA manipulator](https://icube-robotics.github.io/ecat_ros2_workshop/tutorials/urdf_tutorial/)
- [Launch and interact with the SCARA robot](https://icube-robotics.github.io/ecat_ros2_workshop/tutorials/launch_tutorial/)
- [Write a custom hardware interface](https://icube-robotics.github.io/ecat_ros2_workshop/tutorials/hardware_tutorial/)
- [Write a custom controller](https://icube-robotics.github.io/ecat_ros2_workshop/tutorials/controller_tutorial/)
- [Set up Gazebo simulation](https://icube-robotics.github.io/ecat_ros2_workshop/tutorials/gazebo_tutorial/)
- [Control EtherCAT motor drives](https://icube-robotics.github.io/ecat_ros2_workshop/tutorials/ethercat_tutorial/)

![SCARA Model](resources/scara_model.png)


## 🚀 Quick Start

**_Required setup: Ubuntu 24.04 LTS and ROS2 Jazzy_**

### Installation

1. Install ROS2 Jazzy ([installation guide](https://icube-robotics.github.io/ecat_ros2_workshop/getting-started/installation/))

2. Source your ROS2 environment:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

3. Install colcon:
   ```bash
   sudo apt install python3-colcon-common-extensions
   ```

4. Create and build workspace:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   git clone https://github.com/ICube-Robotics/ecat_ros2_workshop.git src/ecat_ros2_workshop
   rosdep install --ignore-src --from-paths . -y -r
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
   source install/setup.bash
   ```

### Docker Setup (Alternative)

For a containerized environment, see the [Docker setup guide](https://icube-robotics.github.io/ecat_ros2_workshop/getting-started/docker/).

```bash
docker build --tag ecat_ros2_workshop:jazzy --file .docker/Dockerfile .
docker run ecat_ros2_workshop:jazzy ros2 launch scara_bringup scara.launch.py
```

## 📦 Repository Structure

```
ecat_ros2_workshop/
├── scara_description/      # URDF and robot description files
├── scara_hardware/         # Custom hardware interface implementation
├── scara_controllers/      # Custom controller implementation
├── scara_bringup/         # Launch files and configurations
├── scara_nodes/           # Additional ROS2 nodes
├── docs/                  # MkDocs documentation source
└── resources/             # Tutorial resources and images
```

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

For bugs and feature requests, please open an issue on [GitHub](https://github.com/ICube-Robotics/ecat_ros2_workshop/issues).

## 📄 License

This project is open source. See the [LICENSE](LICENSE) file for details.


## 👥 Contacts & Maintainers

### [ICube Laboratory](https://icube.unistra.fr)
[University of Strasbourg](https://www.unistra.fr/), France

**Manuel Yguel** - [yguel@unistra.fr](mailto:yguel@unistra.fr) | [@yguel](https://github.com/yguel)

### [Asterion Robotics](https://asterion-robotics.com)

**Maciej Bednarczyk** - [m.bednarczyk@asterion-robotics.com](mailto:m.bednarczyk@asterion-robotics.com) | [@mcbed](https://github.com/mcbed)

For more information, see the [full contacts page](https://icube-robotics.github.io/ecat_ros2_workshop/about/contacts/).

---

<div align="center">

[![ICube](https://icube.unistra.fr/fileadmin/templates/DUN/icube/images/logo.png)](https://icube.unistra.fr)
&nbsp;&nbsp;&nbsp;&nbsp;
[![Asterion](https://raw.githubusercontent.com/Asterion-Robotics/assets/refs/heads/main/asterion-logo.png)](https://asterion-robotics.com)

**Made with ❤️ by ICube Laboratory and Asterion Robotics**

</div>
