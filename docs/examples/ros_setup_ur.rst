1. ur_modern driveer

sudo apt-get install ros-kinetic-ur-msgs
sudo apt-get install ros-kinetic-hardware-interface
sudo apt-get install ros-kinetic-controller-manager

https://github.com/ThomasTimm/ur_modern_driver/issues/135
cd ~/catkin_ws/src/ur_modern_driver/src
rm ur_hardware_interface.cpp
wget https://raw.githubusercontent.com/iron-ox/ur_modern_driver/883070d0b6c0c32b78bb1ca7155b8f3a1ead416c/src/ur_hardware_interface.cpp
cd ~/catkin_ws/
catkin_make

cd ~/catkin_ws/src
git clone 
catkin_make

roslaunch ur_modern_driver ur5_bringup.launch robot_ip:=192.168.10.10

1. rosbridge, ur_movit, ur5_bringup modern driver