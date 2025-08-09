# Ubuntu (deb packages) — ROS 2 Documentation: Humble  documentation

Table of Contents

*   [Resources](#resources)
    
*   [Set locale](#set-locale)
    
*   [Setup Sources](#setup-sources)
    
*   [Install ROS 2 packages](#install-ros-2-packages)
    
*   [Environment setup](#environment-setup)
    
    *   [Sourcing the setup script](#sourcing-the-setup-script)
        
*   [Try some examples](#try-some-examples)
    
    *   [Talker-listener](#talker-listener)
        
*   [Next steps after installing](#next-steps-after-installing)
    
*   [Using the ROS 1 bridge](#using-the-ros-1-bridge)
    
*   [Additional RMW implementations (optional)](#additional-rmw-implementations-optional)
    
*   [Troubleshooting](#troubleshooting)
    
*   [Uninstall](#uninstall)
    

Deb packages for ROS 2 Humble Hawksbill are currently available for Ubuntu Jammy (22.04). The target platforms are defined in [REP 2000](https://ros.org/reps/rep-2000.html).

[Resources](#id1)
[](#resources "Link to this heading")
-------------------------------------------------------

*   Status Page:
    
    *   ROS 2 Humble (Ubuntu Jammy): [amd64](http://repo.ros2.org/status_page/ros_humble_default.html), [arm64](http://repo.ros2.org/status_page/ros_humble_ujv8.html)
        
*   [Jenkins Instance](http://build.ros2.org/)
    
*   [Repositories](http://repo.ros2.org/)
    

[Set locale](#id2)
[](#set-locale "Link to this heading")
---------------------------------------------------------

Make sure you have a locale which supports `UTF-8`. If you are in a minimal environment (such as a docker container), the locale may be something minimal like `POSIX`. We test with the following settings. However, it should be fine if you’re using a different UTF-8 supported locale.

``` bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

```


[Setup Sources](#id3)
[](#setup-sources "Link to this heading")
---------------------------------------------------------------

You will need to add the ROS 2 apt repository to your system.

First ensure that the [Ubuntu Universe repository](https://help.ubuntu.com/community/Repositories/Ubuntu) is enabled.

``` bash
sudo apt install software-properties-common
sudo add-apt-repository universe

```


The [ros-apt-source](https://github.com/ros-infrastructure/ros-apt-source/) packages provide keys and apt source configuration for the various ROS repositories.

Installing the ros2-apt-source package will configure ROS 2 repositories for your system. Updates to repository configuration will occur automatically when new versions of this package are released to the ROS repositories.

``` bash
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb

```


[Install ROS 2 packages](#id4)
[](#install-ros-2-packages "Link to this heading")
---------------------------------------------------------------------------------

Update your apt repository caches after setting up the repositories.

ROS 2 packages are built on frequently updated Ubuntu systems. It is always recommended that you ensure your system is up to date before installing new packages.

Warning

Due to early updates in Ubuntu 22.04 it is important that `systemd` and `udev`\-related packages are updated before installing ROS 2. The installation of ROS 2’s dependencies on a freshly installed system without upgrading can trigger the **removal of critical system packages**.

Please refer to [ros2/ros2#1272](https://github.com/ros2/ros2/issues/1272) and [Launchpad #1974196](https://bugs.launchpad.net/ubuntu/+source/systemd/+bug/1974196) for more information.

Desktop Install (Recommended): ROS, RViz, demos, tutorials.

``` bash
sudo apt install ros-humble-desktop

```


ROS-Base Install (Bare Bones): Communication libraries, message packages, command line tools. No GUI tools.

``` bash
sudo apt install ros-humble-ros-base

```


Development tools: Compilers and other tools to build ROS packages

``` bash
sudo apt install ros-dev-tools

```


[Environment setup](#id5)
[](#environment-setup "Link to this heading")
-----------------------------------------------------------------------

### [Sourcing the setup script](#id6)
[](#sourcing-the-setup-script "Link to this heading")

Set up your environment by sourcing the following file.

``` bash
source /opt/ros/humble/setup.bash

```


Note

Replace `.bash` with your shell if you’re not using bash. Possible values are: `setup.bash`, `setup.sh`, `setup.zsh`.
### Finally to make the future labs easier add the following:
``` bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
-----------------------------------------------------------------------

## Submissions


By the end of this assignment we require you to submit the following screenshots:
1. type `ros2` in terminal.
2. type `rviz2` in terminal.
3. type `tail -n 5 ~/.bashrc` in terminal.
<img width="1044" height="1742" alt="image" src="https://github.com/user-attachments/assets/2ca30921-1287-4c63-898d-97b6d5be9a19" />
- remember while submitting the images, we only want the images, no word, doc, pdf etc
- all the images should be similar to whatever sample is given
