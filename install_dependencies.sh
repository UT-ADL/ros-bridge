#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_SUFFIX=""
if [ "$ROS_PYTHON_VERSION" = "3" ]; then
    PYTHON_SUFFIX=3
fi

if [ "$ROS_VERSION" = "2" ]; then
    ADDITIONAL_PACKAGES="ros-$ROS_DISTRO-rviz2"
else
    ADDITIONAL_PACKAGES="ros-$ROS_DISTRO-rviz
                         ros-$ROS_DISTRO-opencv-apps
                         ros-$ROS_DISTRO-rospy
                         ros-$ROS_DISTRO-rospy-message-converter
                         ros-$ROS_DISTRO-pcl-ros"
fi

if [ "$(lsb_release -sc)" = "focal" ]; then
    ADDITIONAL_PACKAGES="$ADDITIONAL_PACKAGES
                         python-is-python3"
fi

echo ADDITIONAL PACKAGES $ADDITIONAL_PACKAGES

sudo apt update
# NOTE: ackermann_msgs and derived_object_msgs are built from source in this
# workspace (no ros-$ROS_DISTRO-* debs exist for them), so they are intentionally
# not listed here. qt5-default was removed in Ubuntu 22.04; qtbase5-dev provides
# the Qt5 development files it used to pull in.
sudo apt-get install --no-install-recommends -y \
    python$PYTHON_SUFFIX-pip \
    python$PYTHON_SUFFIX-osrf-pycommon \
    python$PYTHON_SUFFIX-catkin-tools \
    python$PYTHON_SUFFIX-catkin-pkg \
    python$PYTHON_SUFFIX-catkin-pkg-modules \
    python$PYTHON_SUFFIX-rosdep \
    python$PYTHON_SUFFIX-wstool \
    python$PYTHON_SUFFIX-opencv \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-vision-opencv \
    ros-$ROS_DISTRO-rqt-image-view \
    ros-$ROS_DISTRO-rqt-gui-py \
    wget \
    qtbase5-dev \
    ros-$ROS_DISTRO-pcl-conversions \
    $ADDITIONAL_PACKAGES

pip$PYTHON_SUFFIX install --upgrade pip
pip$PYTHON_SUFFIX install -r $SCRIPT_DIR/requirements.txt
