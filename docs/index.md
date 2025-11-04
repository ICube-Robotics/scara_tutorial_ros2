# SCARA ROS2 EtherCAT Workshop

Welcome to the **SCARA ROS2 EtherCAT Workshop**! This comprehensive tutorial will guide you through the fundamentals of controlling robotic systems using the [ros2_control](https://control.ros.org) framework and EtherCAT communication.

![SCARA Model](images/scara_model.png)

## Overview

The [ros2_control](https://control.ros.org) framework is a real-time control framework designed for general robotics applications that is gaining increasing attention in both research and industrial contexts. This tutorial is designed to help you understand the basic concepts of controlling a robot using ros2_control and EtherCAT.

## What You'll Learn

This workshop covers the following topics:

<div class="grid cards" markdown>

-   :material-cube-outline: **URDF Description**

    ---

    Learn how to write a complete URDF description for a simple SCARA manipulator, including geometry, dynamics, and ros2_control interfaces.

    [:octicons-arrow-right-24: URDF Tutorial](tutorials/urdf_tutorial.md)

-   :material-rocket-launch: **Launch & Interact**

    ---

    Discover how to launch your robot system and interact with controllers through the Controller Manager.

    [:octicons-arrow-right-24: Launch Tutorial](tutorials/launch_tutorial.md)

-   :material-chip: **Hardware Interface**

    ---

    Develop custom hardware interfaces to communicate with your robot's physical hardware or simulation.

    [:octicons-arrow-right-24: Hardware Tutorial](tutorials/hardware_tutorial.md)

-   :material-cog: **Controller Development**

    ---

    Write custom controllers tailored to your specific application requirements.

    [:octicons-arrow-right-24: Controller Tutorial](tutorials/controller_tutorial.md)

-   :material-cube-scan: **Gazebo Simulation**

    ---

    Set up your SCARA manipulator to run with ros2_control and Gazebo for realistic physics simulation.

    [:octicons-arrow-right-24: Gazebo Tutorial](tutorials/gazebo_tutorial.md)

-   :material-ethernet: **EtherCAT Integration**

    ---

    Configure and control CIA 402 compliant EtherCAT motor drives using the ethercat_driver_ros2 stack.

    [:octicons-arrow-right-24: EtherCAT Tutorial](tutorials/ethercat_tutorial.md)

</div>

## Prerequisites

!!! info "System Requirements"
    - **Operating System**: Ubuntu 24.04 LTS
    - **ROS Distribution**: ROS2 Jazzy
    - **Basic Knowledge**: Familiarity with ROS2, C++, and Python

## Quick Start

Ready to get started? Follow our installation guide:

[Get Started :material-arrow-right:](getting-started/installation.md){ .md-button .md-button--primary }

## The SCARA Robot

Throughout this tutorial, we'll work with a SCARA (Selective Compliance Assembly Robot Arm) manipulator. This robot type is commonly used in industrial applications for pick-and-place operations, assembly tasks, and more.

### Robot Structure

The SCARA robot consists of:

- **3 Degrees of Freedom**: Two revolute joints for horizontal movement and one prismatic joint for vertical movement
- **Joint 1 & 2**: Revolute joints controlling the arm position in the XY plane
- **Joint 3**: Prismatic joint controlling the vertical (Z-axis) position

## ros2_control Framework

Before diving into the tutorials, it's helpful to understand the ros2_control framework architecture:

- **Controller Manager**: Orchestrates the control loop and manages controller lifecycle
- **Hardware Interface**: Abstracts physical hardware or simulation
- **Controllers**: Implement control algorithms for your robot
- **Resource Manager**: Manages hardware resources and interfaces

For a detailed overview, see the [ros2_control Overview](tutorials/r2c_overview.md).

## Support & Contribution

!!! question "Need Help?"
    If you encounter any issues or have questions:
    
    - Check the tutorial documentation
    - Open an issue on [GitHub](https://github.com/ICube-Robotics/ecat_ros2_workshop/issues)
    - Contact the maintainers (see [Contacts](about/contacts.md))

## License

This project is open source. Check the repository for license information.

---

<div class="grid" markdown>

<div markdown>
### ICube Laboratory
[![ICube](https://icube.unistra.fr/fileadmin/templates/DUN/icube/images/logo.png){ width="200" }](https://icube.unistra.fr)

[University of Strasbourg](https://www.unistra.fr/), France
</div>

<div markdown>
### Asterion Robotics
[![Asterion](https://raw.githubusercontent.com/Asterion-Robotics/assets/refs/heads/main/asterion-logo.png){ width="200" }](https://asterion-robotics.com)

Industrial robotics solutions
</div>

</div>
