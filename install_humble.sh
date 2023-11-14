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

locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt -y install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop

sh -c "echo \"source /opt/ros/$name_ros_version/setup.bash\" >> ~/.bashrc"

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