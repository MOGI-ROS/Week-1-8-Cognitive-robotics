[//]: # (Image References)

[image1]: ./assets/new_node.png "new node"
[image2]: ./assets/rqt.png "rqt"
[image3]: ./assets/turtlesim.png "turtlesim"
[image4]: ./assets/turtlesim_topics.png "turtlesim"
[image5]: ./assets/gazebo.png "gazebo"
[image6]: ./assets/gazebo_2.png "gazebo"
[image7]: ./assets/gazebo_3.png "gazebo"
[image8]: ./assets/rviz_1.png "rviz"
[image9]: ./assets/real_robot_1.png "real robot"
[image10]: ./assets/real_robot_2.png "real robot"
[image11]: ./assets/camera_1.png "camera"
[image12]: ./assets/camera_2.png "camera"

# Kognitív robotika

## Korábbi Házi feladatok - Robotrendszerek

<a href="https://www.youtube.com/watch?v=uLRQJh-y9AU"><img height="400" src="./assets/projects.png"></a>

## Tartalomjegyzék
1. [ROS alapok](#ROS-alapok)  
2. [Gazebo alapok](#Gazebo-alapok)  
3. [Teszt a valódi roboton](#Teszt-a-valódi-roboton)
4. [Turtlebot MOGI](#Turtlebot-MOGI)  
5. [Vonalkövetés](#Vonalkövetés)
6. [Hagyományos képfeldolgozás](#Hagyományos-képfeldolgozás)
7. [Képfeldolgozás a valódi roboton](#Képfeldolgozás-a-valódi-roboton)
8. [Neurális háló](#Neurális-háló)
9. [Neurális háló a valódi roboton](#Neurális-háló-a-valódi-roboton)

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

A `.bashrc`-be egyszerűen tegyük bele a source parancsokat, én például gyakran váltok workspace-ek között a következő módszerrel:
```bash
# ROS workspaces
source /opt/ros/noetic/setup.bash
#WORKSPACE=~/catkin_ws/devel/setup.bash
WORKSPACE=~/bme_catkin_ws/devel/setup.bash
source $WORKSPACE
```

A `.bashrc`-t mindenki a saját kedvére szabhatja, itt egy példa arról, ahogy én használom:

>[https://gist.github.com/dudasdavid/06ba536d7b45a658c70ecb39c2465d04](https://gist.github.com/dudasdavid/06ba536d7b45a658c70ecb39c2465d04)

## publisher node
Az egyszerűség kedvéért a tárgy során csak publisherekkel és subscriberekkel fogunk foglalkozni, és ezekkel is csak Pythonban, ha valaki szeretne megismerkedni a ROS service-ekkel is, illetve a fenti funkciókkal C++-ban, annak ajánlom a Robotrendszerek tárgy anyagát.

Kezdésképpen hozzuk létre az első saját csomagunkat a catkin workspace-ünk `src` mappájában a `catkin_create_pkg` parancs segítségével:
```bash
cd ~/catkin_ws/src
catkin_create_pkg kogrob_tutorial rospy
```

Futtassuk le a catkin_make parancsot a workspace-ünkben, és utána már használhatjuk a roscd parancsot, hogy azonnal a csomagunk mappájába léphessünk:
```bash
roscd kogrob_tutorial
```
A Python scripteket érdemes a csomagunkon belül egy `scripts` mappában tárolni, hozzuk tehát létre a mappát és az első Python ROS node-unkat `publisher.py` néven. Végül tegyük futtathatóvá!
```bash
mkdir scripts
cd scripts/
touch publisher.py
chmod +x publisher.py 
```
![alt text][image1] 

Nyissuk meg a `publisher.py` fájlt a kedvenc szövegszerkesztőnkkel, én a VS Code-ot fogom használni a tárgy során, de a Visual Studio, a PyCharm vagy bármi egyéb is egyformán jó megoldás!

A `publisher.py` kódja:
```python
#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32  # Message type used in the node

rospy.init_node('publisher')    # Init the node with name "publisher"

pub = rospy.Publisher('publisher_topic', Int32, queue_size=10)

rospy.loginfo("Publisher Python node has started and publishing data on publisher_topic")

rate = rospy.Rate(1)            # 1Hz

count = Int32()                 # Count is now a ROS Int32 type variable that is ready to be published

count.data = 0                  # Initializing count

while not rospy.is_shutdown():  # Run the node until Ctrl-C is pressed

    pub.publish(count)          # Publishing data on topic "publisher_topic"
    
    count.data += 1
        
    rate.sleep()                # The loop runs at 1Hz
```

Ha elindítjuk, akkor a következőt kell látnunk:

>_Ne felejtsünk roscore-t indítani egy másik terminálban!_
>```console
>david@david-precision-7520:~$ rosrun kogrob_tutorial publisher.py 
>Unable to register with master node [http://192.168.1.221:11311]: master may not be running yet. Will keep trying.
>```

```console
david@david-precision-7520:~$ rosrun kogrob_tutorial publisher.py 
[INFO] [1672313359.529865]: Publisher Python node has started and publishing data on publisher_topic
```
De hogyan láthatjuk az adatot, amit a `publsiher.py` küld?
1. A `rostopic echo` paranccsal közvetlenül a terminálban:
```console
david@david-precision-7520:~$ rostopic echo /publisher_topic
data: 35
---
data: 36
---
data: 37
```
2. Az `rqt` topic monitorával:
![alt text][image2] 

3. Saját ROS node-dal...

## subscriber node

Készítsük el a saját subscriber-ünket, először hozzuk létre - és tegyük futtathatóvá - a fájlt az előző mintájára:
```bash
roscd kogrob_tutorial
cd scripts/
touch subscriber.py
chmod +x subscriber.py
```

A `subscriber.py` kódja:
```python
#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32 # Message type used in the node

'''
"sub_callback" is the callback method of the subscriber. Argument "msg" contains the received data.
'''
def sub_callback(msg):
    rospy.loginfo("Received data from publisher_topic: %d", msg.data)

rospy.init_node('subscriber') # Init the node with name "subscriber_py"

rospy.Subscriber("publisher_topic", Int32, sub_callback, queue_size=10) 

rospy.loginfo("Subscriber_py node has started and subscribed to publisher_topic")

'''
rospy.spin() simply keeps your node from exiting until the node has been shutdown.
Unlike roscpp::spin(), rospy.spin() does not affect the subscriber callback functions,
as those have their own threads.
'''
rospy.spin() 
```

És próbáljuk is ki!
```console
david@david-precision-7520:~$ rosrun kogrob_tutorial subscriber.py 
[INFO] [1672313947.858942]: Subscriber_py node has started and subscribed to publisher_topic
[INFO] [1672313948.526696]: Received data from publisher_topic: 542
[INFO] [1672313949.526614]: Received data from publisher_topic: 543
[INFO] [1672313950.526684]: Received data from publisher_topic: 544
[INFO] [1672313951.526633]: Received data from publisher_topic: 545
[INFO] [1672313952.526628]: Received data from publisher_topic: 546
```

Ez azt jelenti, hogy ettől a ponttól képesek vagyunk megbízhatóan adatot küldeni és fogadni a ROS rendszerén belül. A ROS absztrakciójának köszönhetően ez történhet egy számítógépen, vagy akár a hálózaton elosztva különböző gépek között. Kis túlzással ilyen egyszerű a robot kerekeinek a sebességét, egy lidar vagy egy kamera képét is továbbítani.

## turtlesim és hasznos ROS eszközök

Végezetül, tegyünk egy lépést a szimuláció és a robotok irányába a `turtlesim` segítségével.
Indítsuk el a turtlesim node-ot, ami egy egyszerű kis 2D-s teknőcös-rajzolós világ:
```console
rosrun turtlesim turtlesim_node
```
![alt text][image3] 

És egy új terminálban indítsuk el a `turtle_teleop_key`-t is, amivel a billentyűzet nyilai segítségével tudjuk irányítani a teknőst:
```console
rosrun turtlesim turtle_teleop_key
```
Ha megnyitjuk az `rqt`-t akkor két érdekes topic-ot is látunk a topic monitorban:
![alt text][image4]

Figyeljük meg a `/turtle1/cmd_vel`-t miközben irányítjuk a teknőst, vagy indítsuk el a következő node-ot:
```console
david@david-precision-7520:~$ rosrun turtlesim draw_square 
```

Próbáljuk ki a teknős mozgatását egy saját node-dal:
```python
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist      # We'll use Twist message in the node

rospy.init_node('turtlesim_draw_circle') # Init the node with name "turtlesim_draw_circle"

pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)

rospy.loginfo("Turtlesim draw circle node has started!")

rate = rospy.Rate(20) # 20Hz

msg = Twist()
msg.linear.x  = 1
msg.angular.z = 1

while not rospy.is_shutdown(): # Run the node until Ctrl-C is pressed

    pub.publish(msg)           # Publishing twist message on topic "/turtle1/cmd_vel"
    #msg.linear.x += 0.005     # Uncomment this line to draw a spiral instead of circle
    rate.sleep()               # The loop runs at 20Hz
```

# Gazebo alapok

A Gazebo egy önálló fizikai szimulációs környezet, nem a ROS része, azonban rengeteg csomag segíti az integrációját a ROS-hoz. Jelenleg a 11-es verziónál tart, ez az utolsó Gazebo Classic kiadás, ami 2025-ig támogatott.

Ha a `ros-noetic-desktop-full` csomagot telepítettétek, akkor a Gazebo már telepítve van.

Ha valamiért még sincs telepítve akkor a `sudo apt install gazebo11` paranccsal tudjátok telepíteni.

A hivatalos Gazebo tutorialokat [ezen a linken](http://gazebosim.org/tutorials) éritek el.

## Gazebo indítása, modellek
A Gazebo-t a `gazebo` paranccsal tudjuk elindítani, és indulás után a következő képernyő fogad bennünket:

![alt text][image6] 

* A bal oldali panel (**zöld**) mutatja a szimulációban lévő objektumok hierarchiáját.

* Fent (**piros**) az eszköztár, mozgatás, forgatás, kamera beállítások, stb.

* Alul (**barna**) a szimuláció megállítása, elindítása, real-time faktor és időbélyegek.

* Középen (**kék**) a szimuláció 3D-s világa.

Az insert fül alatt találjuk a saját modelljeinket illetve az online elérhető modelleket. A Gazebo ezen felül rendelkezik egy model editorral és egy building editorral, amivel különböző épületeket tervezhetünk a szimulációnk számára, de ezekkel a tárgy során nem foglalkozunk részletesen. Aki szeretne ezekről többet megtudni keresse fel a [Robotrendszerek tárgy anyagát](https://github.com/MOGI-ROS/Week-3-4-Gazebo-basics#building-editor).
![alt text][image5] 

## Turtlebot3 alap csomag szimulációja

Töltsük le a catkin workspace-ünkbe az alábbi GIT repokat:
```console
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs
git clone https://github.com/MOGI-ROS/turtlebot3
```

Az utolsó csomagból a saját forkunkat töljük le, mert később módosítunk benne egy pár dolgot. A master branch amúgy megegyezik a ROBOTIS-os repoban találhatóval.

>Az eredeti repo:
>```console
>git clone https://github.com/ROBOTIS-GIT/turtlebot3
>```

A Turtlebot szimulációjához még be kell állítanunk egy környezeti változót az alapján, hogy a `burger` vagy a `waffle` verziót használjuk. A tárgy során csak a `burger`-rel foglalkozunk, így a következő parancsra van szükség:
```bash
export TURTLEBOT3_MODEL=burger
```

Ezt is minden terminálban be kell írjuk, így egyszerűbb, ha ezt is betesszük a `.bashrc`-be.

A Turtlebot szimulációját a következő paranccsal tudjuk elindítani:
```console
roslaunch turtlebot3_gazebo turtlebot3_house.launch
```
![alt text][image7]

Ha ezek után egy másik terminálban elindítjuk a távirányító node-ot, akkor vezethetjük is a robotot a szimulációban a `W,A,S,D` billentyűk segítségével.
```console
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

Egyszerűen kipróbálhatjuk a robot szimulált lidarját is, ha elindítjuk a következő parancsot egy újabb terminálban:
```console
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```
> Ehhez szükség lesz a `gmapping` csomagra, amit a `sudo apt install ros-noetic-gmapping` paranccsal tudunk feltenni.

Ez automatikusan elindít minden szükséges node-ot és nyit egy RViz-t, ahol láthatjuk az elkészített térképet és a valósidejű szenzoradatokat is.

![alt text][image8]

# Teszt a valódi roboton

## Első indítás
Ahhoz, hogy egyszerűen ki tudjuk próbálni az itt tanultakat a valódi roboton, összeraktam egy SD kártya image-et, amit egy tetszőleges Turtlebot SD kártyájára kiírva már lehet is használni pár egyszerű beállítást követően. Az image tartalmazza a következőket:
* beállított `.bashrc`
* beállított és lefordított catkin workspace, ami a Raspberry kamerát is támogatja
* beállított SMB server, hogy könnyen hozzá tudjunk férni a fájlokhoz a roboton hálózaton keresztül

>SD kártya image: [Link a Google drive-ra](https://drive.google.com/file/d/19eiNYBjvwrmzaS9EkkhA9jJw_dw-v1pL/view?usp=sharing)

Miután az image-et kiírtuk a kártyára keressünk egy HDMI monitort és egy USB-s billentyűzetet, hogy beállítsuk a Wifi-t és a robot host nevét a hálózaton, utána nem lesz többet szükségünk sem monitorra sem billentyűzetre.

A roboton a wifi beállításokat a legegyszerűbben a `/etc/netplan/50-cloud-init.yaml` fájl módosításával tudjátok megcsinálni. Itt módosítsátok a Wifi nevét és jelszavát a sajátotokra.
```console
sudo nano /etc/netplan/50-cloud-init.yaml
```

A robot host nevét a következő paranccsal tudjuk beállítani, ez akkor fontos, ha több roboton is használjuk ezt az SD kártya image-et ugyanazon a hálózaton. Az én robotomat `turtlebot3-B4`-nek hívják, ezt minden roboton módosítsátok a megfelelő névre.
```console
sudo nano /etc/hostname
```

Ezek után indítsuk újra a robotot és utána csatlakozhatunk hozzá hálózaton, nem kell többet monitor és billentyűzet.

>Újraindításhoz használhatjuk a `reboot` parancsot:
>```console
>sudo reboot
>```
---

## A Beállítások után

A hostnév alapján derítsétek ki a robot IP címét például a ping paranccsal, esetmben ez `192.168.1.45`.
```console
david@david-precision-7520:~$ ping turtlebot3-B4
PING turtlebot3-B4 (192.168.1.45) 56(84) bytes of data.
64 bytes from turtlebot3-B4.lan (192.168.1.45): icmp_seq=1 ttl=64 time=1651 ms
64 bytes from turtlebot3-B4.lan (192.168.1.45): icmp_seq=2 ttl=64 time=621 ms
64 bytes from turtlebot3-B4.lan (192.168.1.45): icmp_seq=3 ttl=64 time=1.25 ms
```

Ezek után már bejelentkezhetünk SSH-n keresztül a robotra:
```console
ssh ubuntu@192.168.1.45
```
>A jelszó ugyancsak `ubuntu`.

Miután sikeresen beléptünk a terminál kírja a `ROS_MASTER_URI`-t, ezt kell használjuk majd a saját Ubuntus gépünkön, mert azt szeretnénk, ha a robot lenne a ROS master.
```console
============== ROS NETWORK CONFIG ===============
export ROS_MASTER_URI=http://192.168.1.45:11311
export ROS_IP=192.168.1.45
=================================================
```

A roboton indítsuk el a következő parancsot:
```console
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

A saját gépünk termináljaiban pedig állítsuk be a ROS master címét a roboton:
```console
export ROS_MASTER_URI=http://192.168.1.45:11311
```
>Ez csak ideiglenes beállítás, jobb ezt nem a `.bashrc`-be tenni.

Ezután már indíthatjuk a távirányítót, ahogy az előbb a saját gépünkön:
```console
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

Vagy megnézhetjük a topic-okat és a kamera képét (ha van kamera a robotunkon) `rqt`-ben:
![alt text][image9]

De akár elindíthajuk a tértképezést is a roboton lévő lidar szenzor alapján:
```console
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```
![alt text][image10]

# Turtlebot MOGI

A tárgy további részében a saját `turtlebot3-mogi` csomagunkkal fogunk dolgozni, de előtte még egy pár módosítást végre kell hajtanunk a gyári `turtlebot3` csomagon is. Emlékezzünk rá, hogy épp emiatt nem a hivatalos `turtlebot3` csomagot töltötük le GIT-ből, hanem a MOGI-s verziót!

```
git clone https://github.com/MOGI-ROS/turtlebot3
```
>Ehelyett:
>```console
>git clone https://github.com/ROBOTIS-GIT/turtlebot3

A következő lépés a `mogi-ros` branchre váltás a `master` helyett. Ha nem definiáljuk melyik branchet szeretnénk használni letöltéskor, akkor az alapértelmezett branchen leszünk, ami általában a `master` vagy a `main`.

A `git status` paranccsal bármikor megnézhetjük milyen branchen vagyunk:
```console
david@david-precision-7520:~/bme_catkin_ws/src/turtlebot3$ git status
On branch master
Your branch is up to date with 'MOGI-ROS/master'.

nothing to commit, working tree clean
```

Az elérhető brancheket is meg tudjuk nézni a `git branch -a` paranccsal:
```console
david@david-precision-7520:~/bme_catkin_ws/src/turtlebot3$ git branch -a
  camera-plugin
* master
  mogi-robot
  mogi-ros
  remotes/MOGI-ROS/camera-plugin
  remotes/MOGI-ROS/master
  remotes/MOGI-ROS/mogi-robot
  remotes/MOGI-ROS/mogi-ros
```

A `git checkout` paranccsal pedig egyszerűen válthatunk branchet - természetesen akkor, ha minden módosításunkat commitoltuk az aktuális branchen!
```console
david@david-precision-7520:~/bme_catkin_ws/src/turtlebot3$ git checkout mogi-ros
Switched to branch 'mogi-ros'
Your branch is up to date with 'MOGI-ROS/mogi-ros'.
```
>És még ennél is sokkal egyszerűbb egy grafikus felületű git klienst használni, mint mondjuk a GitKraken.

Ha átváltottunk a `mogi-ros` branchre, nézzük is meg a módosításokat, amikre szükségünk lesz!

1. Változtattunk pár launch fájlon a roboton lévő kamera miatt
2. Hozzáadtuk a kamerát a robot modelljéhez

## Kamera hozzáadása
A kamera hozzáadása 2 fájlt érint, a `/turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.urdf.xacro` és a `/turtlebot3/turtlebot3_description/urdf/turtlebot3_burger.gazebo.xacro` fájlokat. Az előbbi a robot 3D modelljéhez adja hozzá a kameránk modelljét - egy 45 fokban megdöntött piros kockát:
```xml
...
  <!-- Camera -->
  <joint type="fixed" name="camera_joint">
    <origin xyz="0.03 0 0.11" rpy="0 0.79 0"/>
    <child link="camera_link"/>
    <parent link="base_link"/>
    <axis xyz="0 1 0" />
  </joint>

  <link name='camera_link'>
    <pose>0 0 0 0 0 0</pose>
    <inertial>
      <mass value="0.001"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia
          ixx="1e-6" ixy="0" ixz="0"
          iyy="1e-6" iyz="0"
          izz="1e-6"
      />
    </inertial>

    <collision name='collision'>
      <origin xyz="0 0 0" rpy="0 0 0"/> 
      <geometry>
        <box size=".01 .01 .01"/>
      </geometry>
    </collision>

    <visual name='camera_link_visual'>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size=".02 .02 .02"/>
      </geometry>
    </visual>

  </link>

  <gazebo reference="camera_link">
    <material>Gazebo/Red</material>
  </gazebo>

  <joint type="fixed" name="camera_optical_joint">
    <origin xyz="0 0 0" rpy="-1.5707 0 -1.5707"/>
    <child link="camera_link_optical"/>
    <parent link="camera_link"/>
  </joint>

  <link name="camera_link_optical">
  </link>
...
```

A másik fájl a kamera modelljéhez tartozó szimulált kamerát hozza létre:
```xml
...
  <!-- Camera -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="camera">
      <update_rate>30.0</update_rate>
      <visualize>false</visualize>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10.0</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <!-- Noise is sampled independently per pixel on each frame.
               That pixel's noise value is added to each of its color
               channels, which at that point lie in the range [0,1]. -->
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>0.0</updateRate>
        <cameraName>camera</cameraName>
        <imageTopicName>image</imageTopicName>
        <cameraInfoTopicName>camera_info</cameraInfoTopicName>
        <frameName>camera_link_optical</frameName>
        <hackBaseline>0.0</hackBaseline>
        <distortionK1>0.0</distortionK1>
        <distortionK2>0.0</distortionK2>
        <distortionK3>0.0</distortionK3>
        <distortionT1>0.0</distortionT1>
        <distortionT2>0.0</distortionT2>
      </plugin>
    </sensor>
  </gazebo>
...
```

Próbáljuk is ki a szokásos módon:
```
roslaunch turtlebot3_gazebo turtlebot3_house.launch
```
![alt text][image12] 
és egy másik terminálban:
```
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```
![alt text][image11] 

## Turtlebot MOGI csomag

Ha eddig még nem tettük volna, akkor töltsük le ennek a tananyagnak a git repoját a catkin workspace-ünkbe és fordítsuk újra.

Ez az anyag tartalmazza a `turtlebot3_mogi` csomagot, amiben a saját modelljeink és launch fájljaink vannak.

```console
$ tree
.
├── CMakeLists.txt
├── package.xml
├── gazebo_models    --> Gazebo models of line following tracks
├── launch           --> New launch files that we'll use from now
├── maps             --> Saved map to use with the navigation stack
├── meshes           --> Blender files of the line following track
├── network_model    --> Trained neural networkto follow line
├── rviz             --> Pre-configured RViz configurations
├── saved_images     --> New images saved for neural network training
├── scripts
│   ├── line_follower_cnn.py     --> Line follower with neural network
│   ├── line_follower.py         --> Line follower with computer vision processing
│   ├── save_training_images.py  --> Save training images to the /saved_images folder
│   └── train_network.py         --> Train the neural network on /training_images folder
├── training_images  --> Training images for the neural network
└── worlds           --> Gazebo worlds for line following using the models from /gazebo_models
```

Az új gazebo modellek használatához hozzá kell adjuk az útvonalukat a `GAZEBO_MODEL_PATH` környezeti változóhoz. Ezt praktikusan a `.bashrc` fájlban tegyük meg, de ugyanezzel a paranccsal minden terminálban kézzel is megtehetjük.
```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/bme_catkin_ws/src/Week-1-8-Cognitive-robotics/turtlebot3_mogi/gazebo_models/
```

## Saját launch fájlok

4 új launch fájlt készítettem a csomagban, ebből 3 segít nekünk egyszerűen megnyitni a szimulációnkat:
- `/turtlebot3_mogi/launch/simulation_bringup.launch` - Egyszerű szimuláció térképezéssel
- `/turtlebot3_mogi/launch/simulation_navigation.launch` - Szimuláció a navigációs stack-kel
- `/turtlebot3_mogi/launch/simulation_line_follow.launch` - Szimuláció, amit a vonalkövetéshez fogunk használni
- `/turtlebot3_mogi/launch/robot_visualization.launch` - Ezt a valódi robot adatainak a számítógépünkön való megjelenítéséhez használjuk, együtt a robot futó `turtlebot3_robot.launch` fájllal

A launch fájlok használatához - különösen a navigáció stack használatához - szükségünk lesz pár extra csomagra is, amiket a következőképpen telepíthetünk:
```
sudo apt install ros-noetic-hector-trajectory-server
sudo apt install ros-noetic-map-server
sudo apt install ros-noetic-amcl
sudo apt install ros-noetic-move-base
sudo apt install ros-noetic-dwa-local-planner
```
> Illetve használhatjuk ezt a csomagot a távirányításhoz: `sudo apt install ros-noetic-teleop-twist-keyboard`

# Vonalkövetés

A labor további részében vonalkövetést fogunk a turtlebot3-on megvalósítai, először hagyományos képfeldolgozás segítségével később pedig neurális hálóval. Megvizsgáljuk mindkét módszer előnyeit és hátrányait.

## Világ készítése Blenderben és Gazeboban

A saját `turtlebot3_mogi` csomag már alapból tartalmazza a különböző pályákat, amiket vonalkövetésre fogunk használni. Ezeket a pályákat blenderben készítettem, amiről egy rövid tutorialt itt találtok:

<a href="https://www.youtube.com/watch?v=i9JbusxTcOg"><img height="400" src="./assets/blender.png"></a>

Ezen kívül a tavalyi évben egy konzultációt ezzel töltöttünk, ennek az anyagát itt éritek el:

<a href="https://www.youtube.com/watch?v=K5v3cWsks8w"><img height="400" src="./assets/blender_2.png"></a>

A két alap pálya van, amit használni fogunk, az egyik sötét alapon világos vonallal, a másik világos alapon sötét vonallal:


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




roslaunch turtlebot3_gazebo turtlebot3_house.launch
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

rosrun map_server map_saver -f ~/map





4. óra
- vonalkövetés pálya blender
- turtlebot vonalkövetés szimuláció

5. óra
- turtlebot képfeldolgozással vonalkövetés

6. óra
- neurális háló vonalkövetés szimuláció

7. óra
- turtlebot neurális háló vonalkövetés



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



ssh ubuntu@192.168.1.45
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
