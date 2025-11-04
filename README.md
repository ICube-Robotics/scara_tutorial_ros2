# Scara tutorial ROS2
The [ros2_control](https://control.ros.org) framework is a realtime control framework designed for general robotics applications that gains more and more attention both for research and industrial purposes. An overview of the ros2_control framework can be found [here](resources/r2c_overview.md).

This tutorial is made to understand the basic concepts of controlling a robot using ros2_control. In particular, it describes how to :
- [Write a URDF description of a simple SCARA manipulator](resources/urdf_tutorial.md)
- [Launch and interact with the SCARA robot](resources/launch_tutorial.md) 
- [Write a custom hardware interface for the SCARA robot](resources/hardware_tutorial.md)
- [Write a custom controller for the SCARA robot](resources/controller_tutorial.md)
- [Set up the SCARA manipulator to run with ros2_control and Gazebo](resources/gazebo_tutorial.md)

![scara model](resources/scara_model.png)

## Getting Started
***Required setup : Ubuntu 24.04 LTS and ROS2 Jazzy***

1.  Install `ros2` packages. The current development is based on `ros2 jazzy`. Installation steps are described [here](https://docs.ros.org/en/jazzy/Installation.html).
2. Source your `ros2` environment:
    ```shell
    source /opt/ros/jazzy/setup.bash
    ```
    **NOTE**: The ros2 environment needs to be sourced in every used terminal. If only one distribution of ros2 is used, it can be added to the `~/.bashrc` file.
3. Install `colcon` and its extensions :
    ```shell
    sudo apt install python3-colcon-common-extensions
     ```
3. Create a new ros2 workspace:
    ```shell
    mkdir ~/ros2_ws/src
    ```
4. Pull relevant packages, install dependencies, compile, and source the workspace by using:
    ```shell
    cd ~/ros2_ws
    git clone https://github.com/ICube-Robotics/scara_tutorial_ros2.git src/scara_tutorial_ros2
    rosdep install --ignore-src --from-paths . -y -r
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
    source install/setup.bash
    ```

## Docker Setup (Alternative)
For a containerized setup, see the [Docker README](.docker/README.md) for instructions on building and running the tutorial in a Docker container with ROS2 Jazzy.

## Acknowledgments 
This tutorial is partially inspired from [pac48](https://github.com/pac48/ros2_control_demos/tree/full-example-tutorial)'s tutorial and the official [ros2_control website](https://control.ros.org).

## Contacts ##
![icube](https://icube.unistra.fr/fileadmin/templates/DUN/icube/images/logo.png)

[ICube Laboratory](https://icube.unistra.fr), [University of Strasbourg](https://www.unistra.fr/), France

__Maciej Bednarczyk:__ [mcbed.robotics@gmail.com](mailto:mcbed.robotics@gmail.com), @github: [mcbed](https://github.com/mcbed)