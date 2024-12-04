# Week-1-8-Cognitive-robotics
Machine learning based line following using TurtleBot3 with ROS2 Jazzy and Gazebo Harmonic



Dependencies

https://github.com/MOGI-ROS/DynamixelSDK/tree/humble-devel
https://github.com/MOGI-ROS/turtlebot3_msgs/tree/ros2
https://github.com/MOGI-ROS/turtlebot3/tree/mogi-ros2
https://github.com/MOGI-ROS/turtlebot3_simulations/tree/new_gazebo



Dynamixel SDK if problem with module em uninstall existing em and install this version
pip install empy==3.3.4
https://github.com/ros2/rosidl/issues/779
pip install lark
sudo apt install ros-jazzy-hardware-interface
sudo apt install ros-jazzy-nav2-msgs
sudo apt install ros-jazzy-nav2-costmap-2d
sudo apt install ros-jazzy-nav2-map-server
sudo apt install ros-jazzy-nav2-bt-navigator
sudo apt install ros-jazzy-nav2-bringup
sudo apt install ros-jazzy-interactive-marker-twist-server
sudo apt install ros-jazzy-cartographer-ros


export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_gazebo empty_world.launch.py

ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py


sudo apt install ros-jazzy-cartographer-ros

ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim:=true

ros2 run teleop_twist_keyboard teleop_twist_keyboard 

Navigation:
ros2 launch turtlebot3_navigation2 navigation2_use_sim_time.launch.py map_yaml_file:=/home/david/ros2_ws/src/Week-1-8-Cognitive-robotics/turtlebot3_mogi/maps/map.yaml


start on real robot:
ros2 launch turtlebot3_bringup hardware.launch.py