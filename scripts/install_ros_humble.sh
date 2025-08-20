#!/bin/bash

echo step1 >> install_log

locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

echo step2 >> install_log

sudo apt install software-properties-common
sudo add-apt-repository universe

echo step3 >> install_log

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb

echo step4 >> install_log

sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools

echo step5 >> install_log

source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
echo complete >> install_log

echo -e '\n# Auto-set ROS namespace from hostname\nexport ROS_NAMESPACE=$(hostname)\necho -e "\033[1;32m[ROS 2]\033[0m Namespace set to: \033[1;34m$ROS_NAMESPACE\033[0m"' >> ~/.bashrc
echo -e '\n# Auto-set ROS namespace and domain from hostname\nexport ROS_NAMESPACE=$(hostname)\nexport ROS_DOMAIN_ID=$(( $(hostname | cksum | cut -d " " -f1) % 100 ))\necho -e "\033[1;32m[ROS 2]\033[0m Namespace: \033[1;34m$ROS_NAMESPACE\033[0m | Domain ID: \033[1;36m$ROS_DOMAIN_ID\033[0m"' >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc

echo "env setup complete" >> install_log
sudo apt update
sudo apt install ros-humble-turtlesim
sudo apt update
sudo apt install '~nros-humble-rqt*'

echo "exiting for now" >> install_log


