#!/bin/bash

SCRIPT_LOCATION="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd)"
cd $SCRIPT_LOCATION

echo -e "\e[103m\e[30m  [ UBUNTU INIT SETTINGS HELPER ]  \e[0m"
mkdir -p ~/applications



echo -e "\e[93mRemove APT lock and block autoupdate\e[0m"
sudo rm -rf /var/lib/dpkg/lock* /var/cache/apt/archives/lock*
echo 'APT::Periodic::Update-Package-Lists "0";' | sudo tee /etc/apt/apt.conf.d/20auto-upgrades
echo 'APT::Periodic::Unattended-Upgrade "0";' | sudo tee -a /etc/apt/apt.conf.d/20auto-upgrades
sudo dpkg-reconfigure unattended-upgrades



echo -e "\e[93mPurge useless packages\e[0m"
sudo apt purge -y thunderbird aisleriot gnome-mahjongg gnome-mines gnome-sudoku thunderbird transmission-common totem
sudo apt autoremove -y



echo -e "\e[93mAPT repository update\e[0m"
sudo sed -i -E 's/.?.?.?archive.ubuntu.com/mirror.kakao.com/g' /etc/apt/sources.list
sudo apt update
sudo apt upgrade -y



echo -e "\e[93mInstalling CLI packages\e[0m"
sudo apt install -y ncdu htop iputils-ping iputils-tracepath openssh-server byobu apt-transport-https ca-certificates curl gnupg lsb-release ffmpeg git perl whiptail lm-sensors bash dbus



echo -e "\e[93mInstalling GUI packages\e[0m"
sudo apt install -y baobab putty terminator filezilla fonts-noto-cjk* fonts-noto-mono fonts-noto-color-emoji gparted blueman fcitx-hangul



echo -e "\e[93mSetup language/input\e[0m"
sudo apt install $(check-language-support)
gnome-language-selector



echo -e "\e[93mInstalling grub-customizer\e[0m"
sudo add-apt-repository ppa:danielrichter2007/grub-customizer -y
sudo apt update
sudo apt install -y grub-customizer
sudo grub-customizer &



echo -e "\e[93mInstalling GNOME extensions requisities\e[0m"
sudo apt install -y chrome-gnome-shell gir1.2-gtop-2.0 gir1.2-nm-1.0 gir1.2-clutter-1.0
git clone https://github.com/brunelli/gnome-shell-extension-installer
sudo chmod +x gnome-shell-extension-installer/gnome-shell-extension-installer
sudo mv gnome-shell-extension-installer/gnome-shell-extension-installer /usr/bin/
rm -rf  gnome-shell-extension-installer
gnome-shell-extension-installer 15
gnome-shell-extension-installer 615
gnome-shell-extension-installer 1071
gnome-shell-extension-installer 1401
gnome-shell-extension-installer 841
gnome-shell-extension-installer 104
gnome-shell-extension-installer 7
gnome-shell-extension-installer 2741
gnome-shell-extension-installer 906
gnome-shell-extension-installer 355
gnome-shell-extension-installer 1031
gnome-shell-extension-installer 1007 --restart-shell
cd ..



echo -e "\e[93mInstalling VSCode\e[0m"
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
sudo rm -f packages.microsoft.gpg
sudo apt update
sudo apt install -y code



echo -e "\e[93mInstalling Google Chrome\e[0m"
wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | sudo apt-key add - 
sudo sh -c 'echo "deb [arch=amd64] http://dl.google.com/linux/chrome/deb/ stable main" > /etc/apt/sources.list.d/google-chrome.list'
sudo apt update
sudo apt install -y google-chrome-stable



echo -e "\e[93mInstalling GNOME customizations\e[0m"
sudo add-apt-repository ppa:papirus/papirus -y
sudo apt update
sudo apt install -y gnome-tweaks papirus-icon-theme papirus-folders
papirus-folders -C orange --theme Papirus-Dark
#TODO: GNOME  gdm3.css lockscreen color tweak
#perl -0e 's/#lockDialogGroup \{\n  background: #2c001e url\(resource:\/\/\/org\/gnome\/shell\/theme\/noise\-texture\.png\);\n  background\-repeat: repeat; \}/#lockDialogGroup \{\n  background: black; \}/' ubuntu.css
echo -e "\e[92mMANUAL SETTINGS REQUIRED:\e[0m"
echo "fonts selection. theme selection."



echo -e "\e[93mInstalling media packages\e[0m"
sudo apt install -y vlc inkscape



ROS_VER=melodic
ROS_WS=$HOME/catkin_ws
echo -e "\e[93mInstalling ROS version=$ROS_VER, workspace=$ROS_WS\e[0m"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential ros-$ROS_VER-desktop-full ros-$ROS_VER-turtlebot3* ros-$ROS_VER-gmapping
sudo rosdep init
rosdep update



echo -e "\e[93mRestoring .bashrc with ROS configurations\e[0m"
if [ -f ~/.bashrc ]; then
	mv ~/.bashrc ~/.bashrc.bak
fi
wget https://ros.kasimov.synology.me/.bashrc -O ~/.bashrc
source /opt/ros/$ROS_VER/setup.bash
mkdir -p $ROS_WS/src
cd $ROS_WS/src
catkin_init_workspace
cd $ROS_WS
catkin_make
source $ROS_WS/devel/setup.bash



ARDUINO_VER=arduino-1.8.15
echo -e "\e[93mInstalling Arduino IDE version=$ARDUINO_VER\e[0m"
wget https://downloads.arduino.cc/$ARDUINO_VER-linux64.tar.xz
sudo tar -xvf $ARDUINO_VER-linux64.tar.xz -C /opt/
rm $ARDUINO_VER-linux64.tar.xz
sudo /opt/$ARDUINO_VER/install.sh
/opt/$ARDUINO_VER/arduino-linux-setup.sh $USER
rm ~/Desktop/arduino-arduinoide.desktop
sudo apt autoremove -y



echo -e "\e[93mInstalling Virtualbox 6.1 repository\e[0m"
echo "deb [arch=amd64] https://download.virtualbox.org/virtualbox/debian $(lsb_release -sc) contrib" | sudo tee /etc/apt/sources.list.d/virtualbox.list
wget -q https://www.virtualbox.org/download/oracle_vbox_2016.asc -O- | sudo apt-key add -
sudo apt update
sudo apt install -y virtualbox-6.1



echo -e "\e[92mInitlalization finished!"
