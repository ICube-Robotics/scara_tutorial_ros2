#!/bin/bash

rm -rf /tmp/.X*lock
rm -rf /tmp/.X11-unix

# Start D-Bus
mkdir -p /var/run/dbus
dbus-daemon --system --fork

tigervncserver -SecurityTypes None
websockify -D --web=/usr/share/novnc/ 6080 localhost:5901

set -e

# setup ros environment
# export ROS_LOCALHOST_ONLY=1
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/ros2_dev/ecat_ros2_workshop/install/setup.bash"
exec "$@"