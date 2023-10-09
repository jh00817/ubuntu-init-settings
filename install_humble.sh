#!/bin/bash

# email : dreambait011@naver.com
# 안녕하세요, echo는 터미널에 문자를 출력하는 명령어 입니다. 주석 대신 echo 안에 쓰여진 내용들을 읽으시면 이해하기 쉬울 겁니다 !



echo ""
echo "[Note] Target OS version  >>> Ubuntu 22.04.x (Focal Fossa)"
echo "[Note] Target ROS version >>> ROS Humble"
echo "[Note] Catkin workspace   >>> $HOME/ros2_ws"
echo ""
# echo "PRESS [ENTER] TO CONTINUE THE INSTALLATION"
# echo "IF YOU WANT TO CANCEL, PRESS [CTRL] + [C]"
# read


# mirror를 kakao로 변경 & 필요 없는 패키지 정리 & 업그레이드

echo -e "\n\e[93mRemove APT lock and block autoupdate\e[0m"
sudo rm -rf /var/lib/dpkg/lock* /var/cache/apt/archives/lock*
echo 'APT::Periodic::Update-Package-Lists "0";' | sudo tee /etc/apt/apt.conf.d/20auto-upgrades
echo 'APT::Periodic::Unattended-Upgrade "0";' | sudo tee -a /etc/apt/apt.conf.d/20auto-upgrades
sudo dpkg-reconfigure unattended-upgrades

echo -e "\n\e[93mPurge useless packages\e[0m"
sudo apt purge -y -qq thunderbird aisleriot gnome-mahjongg gnome-mines gnome-sudoku thunderbird transmission-common totem
sudo apt autoremove -y -qq

echo -e "\n\e[93mAPT repository update\e[0m"
sudo sed -i -E 's/.?.?.?archive.ubuntu.com/mirror.kakao.com/g' /etc/apt/sources.list
sudo apt update -qq
sudo apt upgrade -y -qq

# VSCode 설치
echo -e "\n\e[93mInstalling VSCode\e[0m"
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
sudo rm -f packages.microsoft.gpg
sudo apt update -qq
sudo apt install -y -qq code


### ROBOTICS e-manual ros 설치

# Apache License 2.0
# Copyright (c) 2020, ROBOTIS CO., LTD.

echo "[Set the target OS, ROS version and name of catkin workspace]"
name_os_version=${name_os_version:="focal"}
name_ros_version=${name_ros_version:="humble"}
name_catkin_workspace=${name_catkin_workspace:="ros2_ws"}

echo "[Update the package lists]"
sudo apt update -y

echo "[Install build environment, the chrony, ntpdate and set the ntpdate]"
sudo apt install -y chrony ntpdate curl build-essential
sudo ntpdate ntp.ubuntu.com

echo "[Add the ROS repository]"
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
  sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu ${name_os_version} main\" > /etc/apt/sources.list.d/ros-latest.list"
fi

echo "[Download the ROS keys]"
roskey=`apt-key list | grep "Open Robotics"`
if [ -z "$roskey" ]; then
  curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
fi

echo "[Check the ROS keys]"
roskey=`apt-key list | grep "Open Robotics"`
if [ -n "$roskey" ]; then
  echo "[ROS key exists in the list]"
else
  echo "[Failed to receive the ROS key, aborts the installation]"
  exit 0
fi

echo "[Update the package lists]"
sudo apt update -y

echo "[Install ros-desktop-full version of Noetic"
sudo apt install -y ros-$name_ros_version-desktop-full

echo "[Install RQT & Gazebo]"
sudo apt install -y ros-$name_ros_version-rqt-* ros-$name_ros_version-gazebo-*

echo "[Environment setup and getting rosinstall]"
source /opt/ros/$name_ros_version/setup.sh
sudo apt install -y python3-rosinstall python3-rosinstall-generator python3-wstool build-essential git

echo "[Install rosdep and Update]"
sudo apt install python3-rosdep

echo "[Initialize rosdep and Update]"
sudo sh -c "rosdep init"
rosdep update

echo "[Make the catkin workspace and test the catkin_make]"
mkdir -p $HOME/$name_catkin_workspace/src
cd $HOME/$name_catkin_workspace/src
catkin_init_workspace
cd $HOME/$name_catkin_workspace
catkin_make

echo "[Set the ROS evironment]"
sh -c "echo \"alias eb='nano ~/.bashrc'\" >> ~/.bashrc"
sh -c "echo \"alias sb='source ~/.bashrc'\" >> ~/.bashrc"
sh -c "echo \"alias gs='git status'\" >> ~/.bashrc"
sh -c "echo \"alias gp='git pull'\" >> ~/.bashrc"
sh -c "echo \"alias cw='cd ~/$name_catkin_workspace'\" >> ~/.bashrc"
sh -c "echo \"alias cs='cd ~/$name_catkin_workspace/src'\" >> ~/.bashrc"
sh -c "echo \"alias cm='cd ~/$name_catkin_workspace && catkin_make'\" >> ~/.bashrc"

sh -c "echo \"source /opt/ros/$name_ros_version/setup.bash\" >> ~/.bashrc"
sh -c "echo \"source ~/$name_catkin_workspace/devel/setup.bash\" >> ~/.bashrc"

sh -c "echo \"export ROS_MASTER_URI=http://localhost:11311\" >> ~/.bashrc"
sh -c "echo \"export ROS_HOSTNAME=localhost\" >> ~/.bashrc"

source $HOME/.bashrc

###  ROS e-manual 설치 끝

# 아두이노 설치
ARDUINO_VER=arduino-1.8.15
echo -e "\n\e[93mInstalling Arduino IDE version=$ARDUINO_VER\e[0m"
wget https://downloads.arduino.cc/$ARDUINO_VER-linux64.tar.xz
sudo tar -xf $ARDUINO_VER-linux64.tar.xz -C /opt/
rm $ARDUINO_VER-linux64.tar.xz
sudo /opt/$ARDUINO_VER/install.sh
/opt/$ARDUINO_VER/arduino-linux-setup.sh $USER
rm ~/Desktop/arduino-arduinoide.desktop
sudo apt autoremove -y -qq

echo "[Complete!!!]"
exit 0