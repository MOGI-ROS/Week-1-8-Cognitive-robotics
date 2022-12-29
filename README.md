# Kognitív robotika

## Tartalomjegyzék
1. [ROS alapok](#ROS-alapok)  
2. [Gazebo alapok](#Gazebo-alapok)  
3. [Turtlebot teszt](#Turtlebot-teszt)  
4. [Vonalkövetés](#Vonalkövetés)
5. [Hagyományos képfeldolgozás](#Hagyományos-képfeldolgozás)
6. [Neurális háló](#Neurális-háló)
7. [Teszt a valódi roboton](#Teszt-a-valódi-roboton)

# ROS alapok
A Kognitív robotika (BMEGEMINMKR) labor anyaga jelentős részben támaszkodik a [Robotrendszerek (BMEGEMINMRL) tárgy anyagára](https://github.com/MOGI-ROS/Week-1-2-Introduction-to-ROS#mi-is-az-a-ros). Az itt található tananyag sokszor rövidített/egyszerűsített kivonata a Robotrendszerek tárgy anyagának, de ettől függetlenül, önálló tananyag és nem szükséges hozzá a Robotrendszerek tárgy ismerete. 

## Mi a ROS?
A ROS = Robot Operating System, de valójában ez nem egy operációs rendszer, hanem egy olyan middleware, melyet a robotikában széles körben alkalmaznak. Nyíltforráskódú és könyvtárai segítségével lehetővé teszi a robot alkalmazások gyors fejlesztését. Sok előre beépített funkciót tartalmaz, amiket meg fogunk ismerni a félév során, például kamerák és más szenzorok kezelése, térképezés és útvonaltervezés, telemanipuláció, stb. Fejlesztését 2007-ben kezdte a Stanford egyetem, 2008-ban csatlakozott a fejlesztéshez a Willow Garage és 2013 óta az OSRF gondozásában, ami 2017-ben Open Robotics-ra változtatta a nevét. 2018 óta a Microsoft és az Amazon részt vesz a ROS fejlesztésében.

Ugyan a Microsoft 2018 óta érdeklődik a ROS iránt, és most már telepíthető Windows-ra is, továbbra is a Linux operációs rendszer a legelterjedtebb, ezt fogjuk használni mi is a WSL (Windows Subsystem Linux) segítségével. Bár egyre több programnyelv támogatott a C++ és Python programozási nyelvek a legelterjedtebbek ROS esetén, mi is ezeket fogjuk használni. A ROS-hoz készített alkalmazásokat/komponenseket node-oknak nevezzük, melyek közötti kommunikációt a ROS valósítja meg, mivel a kommunikáció TCP/IP alapú könnyen fejleszthetünk több, hálózatba kötött számítógépen elosztott alkalmazásokat. A robotot vezérlő ROS alkalmazás tehát sok, egymással kommunikáló node-ból épül fel, ezek a `publisher`-ek és a `subscriber`-ek. A ROS `service`-ekkel és `action`-ökkel ebben a tárgyban nem foglalkozunk részletesen.

## ROS telepítése, indítása
A tárgy során Ubuntu 20.04-et és [ROS Noetic](http://wiki.ros.org/noetic/Installation)-et fogunk használni akárcsak a Robotrendszerek tárgy során. A tárgy és a házifeladat projekt teljesítéséhez érdemes tehát egy Ubuntu 20.04-et futtató számítógép használata. Ha esetleg nem Ubuntu 20.04-et használnánk a mindennapokban, akkor is számos lehetőség van a használatára, a teljesség igénye nélkül felsorolok pár ötletet:
- Egy USB-s SSD-re vagy pendrive-ra telepítve, majd arról bootolva
- Windows 11 esetén a WSL2 Linux subsystemet használva
- Virtuális gépet használva

Telepítés során a `full desktop` csomagot ajánlott telepíteni, ebben a legtöbb dolog alapból benne van, amit használni fogunk.
```bash
sudo apt install ros-noetic-desktop-full
```

>A ROS Noetic 2025-ig támogatott.

A ROS telepítése után minden terminálban be kell tölteni a ROS környezetét, ha használni akarjuk, ezt a következő paranccsal tudjuk megtenni:
```bash
source /opt/ros/noetic/setup.bash
```
Ezután ki is tudjuk próbálni a ROS Master elindításával. A ROS Master felel az egyes node-ok regisztrációjáért, összeköti a publishereket és a subscriberek, ez írja le a teljes rendszerünk gráfját. Emellett ez tárolja a paramétereket is a `parameter server`-en. Miután a ROS Master összekötötte az egyes node-okat, a node-ok peer-to-peer kommunikálnak, nem a ROS Masteren keresztül, ezzel csökkentve a kommuniációs overheadet. További részletek a [ROS wiki](http://wiki.ros.org/Master)n.

A ROS Mastert a `roscore` paranccsal indítjuk el.

Ha a következő hibát kapjuk, akkor nem source-oltuk a ROS-t az adott terminálban a fenti paranccsal.
```bash
david@DavidsLenovoX1:~/catkin_ws$ roscore
Command 'roscore' not found, but can be installed with:

sudo apt install python3-roslaunch
```

Ha minden rendben van, akkor a követlezőt kell lássuk a terminál ablakunkban:
```console
david@david-precision-7520:~/bme_catkin_ws$ roscore
... logging to /home/david/.ros/log/7539c3ba-86a3-11ed-8b98-dd714a583910/roslaunch-david-precision-7520-84787.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://192.168.1.221:41289/
ros_comm version 1.15.15


SUMMARY
========

PARAMETERS
 * /rosdistro: noetic
 * /rosversion: 1.15.15

NODES

auto-starting new master
process[master]: started with pid [84802]
ROS_MASTER_URI=http://192.168.1.221:11311/

setting /run_id to 7539c3ba-86a3-11ed-8b98-dd714a583910
process[rosout-1]: started with pid [84819]
started core service [/rosout]
```

## Catkin workspace

A ROS használata során 3 különböző típusú szoftver csomagot érdemes megkülönböztetnünk:
1. A ROS által alapértelmezetten telepített csomagok, ezekhez bármikor hozzáférünk a `source /opt/ros/noetic/setup.bash` parancs után az adott terminálban.
2. Az `apt` csomagkezelővel, tárolóból feltett csomagok. Bár nem a ROS telepítéssel érkeztek, telepítés után ugyanúgy az alap rendszer részét képezik, mint az 1. kategória.
3. A saját csomagjaink, vagy mások pl. GitHub-ról letöltött csomagjai, amiket magunknak kell lefordítani. Ezeket mindig egy workspace-ben tároljuk, itt fejlesztjük és fordítjuk őket, csak akkor hozzáférhetők, ha a workspace-t is source-oljuk. Természetesen lehet több workspace-ünk is, és ezek között könnyen válthatunk a `source` bash paranccsal. A workspace-t `catkin workspace`-nek nevezzük a ROS fordítója, a catkin után.

A catkin workspace-t a következő paranccsal tudjuk (újra)fordítani:
```bash
catkin_make
```

Először tehát hozzuk létre a workspace-ünket és fordítsuk le üresen!
```bash
cd ~
mkdir -p catkin_ws/src
cd catkin_ws
catkin_make
```

Ezután létrejönnek a szükséges mappák és fájlok, hogy a workspace-ünket is tudjuk source-olni:
```bash
source ~/catkin_ws/devel/setup.bash
```

## .bashrc

A tárgy során - és általánosságban a ROS használata során - sok terminált fogunk használni, és nem praktikus ezeket a source parancsokat minden alkalommal leírni, ezért használhatjuk a Linux és bash egy nagyszerű funkcióját, a `.bashrc`-t. Ez egy olyan fájl, ami minden terminál indjtásakor automatikusan végrehajtódik, tehát nem kell többé maunálisan futtatgatnunk ezeket a parancsokat.

A `.bashrc` mindig a user home mappájában található, a home mappába bármikot visszatérhetünk a `cd ~` paranccsal. A .bashrc-t pedig megnyithatjuk a kedven szövegszerkesztőnkkel, esetemben például a `nano`-val:
```bash
david@david-precision-7520:~$ cd ~
david@david-precision-7520:~$ nano .bashrc
```

 {% gist 06ba536d7b45a658c70ecb39c2465d04 %}
<script src="https://gist.github.com/dudasdavid/06ba536d7b45a658c70ecb39c2465d04.js"></script>


## publisher node

## subscriber node

## turtlesim és hasznos ROS eszközök

# Gazebo alapok

## Mi a Gazebo? Gazebo indítása, modellek

## Turtlebot3 alap csomag szimulációja

# Turtlebot teszt

## Kamera hozzáadása

## Turtlebot MOGI csomag

# Vonalkövetés

## Világ készítése Blenderben és Gazeboban

## Launch fájlok, szimuláció előkészítése

# Hagyományos képfeldolgozás

## Képfeldolgozó algoritmus és ROS node

## Sötét és világos háttér

# Neurális háló

## Neurális háló készítése

## Tanítási minták felvétele

## Neurális háló tanítása

## Teszt sötét és világos háttéren

## Teszt piros-zöld környezetben

# Teszt a valódi roboton

## SD kártya image, beállítások a roboton, GIT repo

## Hagyományos képfeldolgozás

## Neurális háló




roslaunch turtlebot3_mogi simulation_line_follow.launch
rosrun turtlebot3_mogi line_follower_cnn.py




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


/bin/bash -lixc exit 2>&1 | sed -n 's/^+* \(source\|\.\) //p' | grep ws




házi ötletek:
- olyan neurális háló ami a vonal színét is megmondja
- gyorsabb neurális háló, lehető legkevesebb paraméterrel
- képfeldolgozás/neurális háló a roboton
