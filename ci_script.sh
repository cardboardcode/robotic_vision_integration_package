#!/usr/bin/env bash

mkdir -p /home/catkin_ws/src
cp -r rvip /home/catkin_ws/src
cd /home
git clone https://github.com/STORM-IRIT/OpenGR.git
cd OpenGR
git reset --hard 0967cd880950b35786b8fd098837c9eb1fe2aca4
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make install
cd /home/catkin_ws/
apt-get update
apt-get install wget -y
sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | apt-key add -
apt-get install ros-melodic-catkin python-catkin-tools -y
source /opt/ros/$ROS_DISTRO/setup.bash
cd /home/catkin_ws && catkin init
catkin build --verbose -DCMAKE_BUILD_TYPE=Release rvip
apt-get install lcov -y
cd /home/catkin_ws/src
git clone https://github.com/mikeferguson/code_coverage.git
cd ..
catkin build code_coverage && source devel/setup.bash
catkin config --cmake-args -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug
catkin build rvip -v --no-deps --catkin-make-args rvip_coverage_report
export CODECOV_TOKEN="a8394d4e-99b4-4408-92e3-ed1fc51e9324"
bash <(curl -s https://codecov.io/bash)
