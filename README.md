# Kognitív robotika

# Tartalomjegyzék
1. [1. hét](#1-hét)  
2. [2. hét](#2-hét)  

# 1. hét

aaaa


##Extra csomagok:
sudo apt install ros-noetic-hector-trajectory-server
sudo apt install ros-noetic-teleop-twist-keyboard
sudo apt install ros-noetic-map-server

#nav:
sudo apt install ros-noetic-amcl
sudo apt install ros-noetic-move-base
sudo apt install ros-noetic-dwa-local-planner


git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations
git clone https://github.com/ROBOTIS-GIT/turtlebot3
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs

https://github.com/MOGI-ROS/turtlebot3


export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/turtlebot3/turtlebot3_mogi/gazebo_models/

roslaunch turtlebot3_gazebo turtlebot3_house.launch
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

rosrun map_server map_saver -f ~/map

Kogrob:
1. óra
- bemutatkozás
- telepítés
- ROS basic alapok, pub-sub

2. óra
- Gazebo alapok
- turtlebot alap csomag kipróbálása

3. óra
- turtlebot szimuláció módosítása, kamera, saját launch fájlok
- esetleg nav demo

4. óra
- vonalkövetés pálya blender
- turtlebot vonalkövetés szimuláció

5. óra
- turtlebot képfeldolgozással vonalkövetés

6. óra
- neurális háló vonalkövetés szimuláció

7. óra
- turtlebot neurális háló vonalkövetés



ROS alapok, gazebo alapok
Átnézni a launchokat közösen
Kéne saját launch file, odom vizualizáció
Kamera plugin
Kavirnyálás a house pályán
Vonalkövető pálya blenderben, poz és negatív színnel
Vonalkövetés hagyományos algoritmussal poz és neg
Vonalkövetés tanulóminták készítése
Tanítás tensorflowban
Kipróbálás gazeboban és valódi roboton

Robotrdsz:
Átnézni a launchokat közösen
Kéne saját launch file, odom vizualizáció
Kamera plugin
Kavirnyálás a house pályán
Navigációs stack felélesztése
Minden a valódi roboton is






ssh ubuntu@192.168.1.184
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_bringup turtlebot3_robot.launch


SD image:
edit sudo nano /etc/hostname
ubuntu@ubuntu


ROBOT .bashrc:
```bash
# ROS Stuff
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Automatic ROS IP config
IP_ADDRESSES=$(hostname -I | sed 's/ *$//g')
IP_ARRAY=($IP_ADDRESSES)
FIRST_IP=${IP_ARRAY[0]}

if [ "$FIRST_IP" != "" ];
then
    true
    #echo "There are IP addresses!"
else
    echo "Warning FIRST_IP var was empty:" $FIRST_IP
    echo "Maybe client is not connected to any network?"
    FIRST_IP=127.0.0.1
fi

export ROS_MASTER_URI=http://$FIRST_IP:11311
export ROS_IP=$FIRST_IP

echo "=============== NETWORK DETAILS ================="
echo ACTIVE IP ADDRESSES: $IP_ADDRESSES
echo SELECTED IP ADDRESS: $FIRST_IP

echo "============== ROS NETWORK CONFIG ==============="
echo export ROS_MASTER_URI=$ROS_MASTER_URI
echo export ROS_IP=$ROS_IP
echo "================================================="

# Turtlebot 3 config
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-01
```

roboton:
https://github.com/UbiquityRobotics/raspicam_node/ - noetic-devel
