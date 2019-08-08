#!/bin/bash

if [ -z "$ROS_DISTRO" ]; then # -z: [-z "$var"] : returns "true = 0" if "var" is empty
  echo "ROS not installed. Please install ROS Melodic (full): http://wiki.ros.org/melodic/Installation/Ubuntu#Ubuntu_install_of_ROS_Melodic"
fi

program="gazebo"
condition=$(which $program 2>/dev/null | grep -v "not found" | wc -l)
if [ $condition -eq 0 ] ; then
    echo "Gazebo is not installed. Please install Gazebo 9"
fi

## Ubuntu Config
sudo apt-get remove modemmanager -y

## Common dependencies

## Required python packages

## Simulatior dependencies

# ROS/Gazebo dependencies
sudo apt-get install \
python-pip python3-vcstool python3-pyqt4 \
pyqt5-dev-tools \
libbluetooth-dev libspnav-dev \
pyqt4-dev-tools libcwiid-dev \
cmake gcc g++ qt4-qmake libqt4-dev \
libusb-dev libftdi-dev \
python3-defusedxml python3-vcstool \
ros-melodic-octomap-msgs        \
ros-melodic-joy                 \
ros-melodic-geodesy             \
ros-melodic-octomap-ros         \
ros-melodic-control-toolbox     \
ros-melodic-pluginlib	       \
ros-melodic-trajectory-msgs     \
ros-melodic-control-msgs	       \
ros-melodic-std-srvs 	       \
ros-melodic-nodelet	       \
ros-melodic-urdf		       \
ros-melodic-rviz		       \
ros-melodic-kdl-conversions     \
ros-melodic-eigen-conversions   \
ros-melodic-tf2-sensor-msgs     \
ros-melodic-pcl-ros \
ros-melodic-navigation \
ros-melodic-sophus

pip install gym
sudo apt-get install python-skimage
pip install h5py
pip install tensorflow-gpu==1.13.1  # you might want to change the version depending on which gpu-driver you have. I used nvidia-driver-410 with CUDA 10.0 and cuDNN 7.4.2.24
pip install keras

# MAVProxy
sudo apt-get install python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-yaml python-pygame
pip install MAVProxy
echo "export PATH=$PATH:$HOME/.local/bin" >> ~/.bashrc

# mavros
sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras ros-melodic-rqt ros-melodic-rqt-common-plugins ros-melodic-rqt-robot-plugins
sudo apt-get install python-catkin-tools
# Dronekit-python

# GYM-WS
#cd gym-ws/gym-ws sudo pip install -e .



# Reload bash
exec bash
