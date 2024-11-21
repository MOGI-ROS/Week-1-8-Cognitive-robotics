# Week-1-8-Cognitive-robotics
Machine learning based line following using TurtleBot3 with ROS2 Jazzy and Gazebo Harmonic


export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_gazebo empty_world.launch.py

ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py


sudo apt install ros-jazzy-cartographer-ros

ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim:=true

ros2 run teleop_twist_keyboard teleop_twist_keyboard 