ARG ROS_DISTRO="jazzy"
FROM ros:${ROS_DISTRO}

COPY . /ros2_dev/ecat_ros2_workshop
RUN apt update && apt upgrade -y
RUN cd ros2_dev && \
    apt update && \
    cd ecat_ros2_workshop && \
    . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --ignore-src --from-paths . -y -r && \
    colcon build --symlink-install
COPY .docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]