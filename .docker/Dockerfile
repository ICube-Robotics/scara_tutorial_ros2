ARG ROS_DISTRO="jazzy"
FROM ros:${ROS_DISTRO}

COPY . /ros2_dev/scara_tutorial_ros2
RUN apt update && apt upgrade -y
RUN cd ros2_dev && \
    apt update && \
    cd scara_tutorial_ros2 && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --ignore-src --from-paths . -y -r && \
    colcon build --symlink-install
COPY .docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]