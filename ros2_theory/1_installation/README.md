# ROS2 Installation Guide

This guide walks through the installation process for ROS2 Humble (recommended for Ubuntu 22.04).

## System Requirements
- Ubuntu 22.04 LTS (Recommended)
- At least 4GB RAM
- At least 20GB free disk space

## Install ROS2 Humble

### Set locale

```bash
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Setup Sources

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Add the ROS2 GPG key:

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Add the repository to your sources list:

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Install ROS2 packages

Update and install:

```bash
sudo apt update
sudo apt install ros-humble-desktop -y
```

### Setup Environment

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Install additional dependencies

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-pip -y
sudo rosdep init
rosdep update
```

## Verification

To verify that your installation is working correctly, open a new terminal and run:

```bash
ros2 --help
```

You should see the help message for the ROS2 command line tools.

## Additional Tools

Install additional development tools:

```bash
sudo apt install ros-humble-rqt ros-humble-rqt-graph ros-humble-turtlesim -y
```

## Troubleshooting

If you encounter issues with your installation, please refer to the official documentation:
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

Or ask for help on the ROS community forums:
https://answers.ros.org/ 