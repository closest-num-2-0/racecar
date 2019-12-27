#!/bin/bash

# insatallation process for ROS melodic --> http://wiki.ros.org/melodic/Installation/Ubuntu
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential

# Installing ROS_SERIAL for interfacing to arduino Nano
sudo apt-get install ros-melodic-rosserial-arduino
sudo apt-get install ros-melodic-rosserial

sudo chmod 666 /dev/ttyUSB0

#arduino git-libraries
mkdir ~/Arduino_test/
cd ~/Arduino_test/
mkdir ~/Arduino_test/libraries
cd ~/Arduino_test/libraries

# EnableInterrupt installation --> https://github.com/GreyGnome/EnableInterrupt
cd ~/Arduino/libraries
git clone https://github.com/GreyGnome/EnableInterrupt.git

# Rosserial_Arduino_Library installation --> https://github.com/frankjoshua/rosserial_arduino_lib
cd ~/Arduino/libraries
git clone https://github.com/frankjoshua/rosserial_arduino_lib.git

#New BARC+ low level
cd ~
git clone https://github.com/closest-num-2-0/racecar.git

cd ~/racecar

catkin_make
source ./devel/setup.bash

#making scripts runnable
chmod +x jetson_startup.sh
chmod +x jetson_startup.sh
