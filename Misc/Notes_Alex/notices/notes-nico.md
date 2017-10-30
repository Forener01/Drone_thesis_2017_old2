---
title: Notes Nicolas Rowier
date: 07-09-2015
author: Nicolas Rowier, Arnaud Jacques
taxonomy:
  category: [docs]
  tag: [ardrone]
---

# Notes Nicolas Rowier

## Préparation ubuntu
* désactivation de la demande de mot de passe avec sudo :
```
echo "laboinmastudent ALL = NOPASSWD:ALL" > /etc/sudoers.d/laboinmastudent
```

* instalation d'un gnome standart :
```
apt-get update && apt-get install gnome-session-flashback
```

* choisir gnome metacity dans le gestionnaire de connection

* ajouter les dépot ros
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
sudo apt-get update
```

* installation de la version compatible avec les examples (indigo) : (doc : http://wiki.ros.org/indigo/Installation/Ubuntu)
```
sudo apt-get install ros-indigo-desktop-full ros-indigo-joy ros-indigo-ardrone-autonomy git
```

* initialisation
```
sudo rosdep init
rosdep update
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc
source /opt/ros/indigo/setup.bash
```

* à exécuter pour chaque utilisateur qui doit pouvoir utiliser ros:
```
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc
source /opt/ros/indigo/setup.bash
```

* installe ardrone_autonomy le driver du drone (http://robohub.org/up-and-flying-with-the-ar-drone-and-ros-getting-started/)

* installer les examples
```
roscd
cd share
git clone https://github.com/mikehamer/ardrone_tutorials.git
cd ardrone_tutorials
make
```

## Utilisation

* mise ne place des ip sur les drone (voir doc multi drone)
désactivation du gestionnaire réseaux
passer en root
```
sudo su
ifconfig eth0 192.168.1.23
dhclient eth0
trouver les drone : nmap -Pn 192.168.1.1-10
```

* lancement
  * dans une premiére fenêtre `roscore`
  * dans une seconde
```
rostopic pub /ardrone/takeoff std_msgs/Empty
rostopic pub /ardrone/land std_msgs/Empty
rostopic pub /ardrone/reset std_msgs/Empty
```
http://www.peterasaro.org/courses/Studio/ardronelessonplan.html

* api rospy
  * http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers
  * http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
  * https://github.com/eschnou/ardrone-autonomy




* plusieurs drones :
```
<group ns="sim1">
        <node name="ardrone_driver" pkg="ardrone_autonomy" type="ardrone_driver" output="screen" clear_params="true" args="-ip 192.168.1.4">
            <param name="outdoor" value="0" /> <!-- If we are flying outdoors, will select more aggressive default settings -->
            <param name="flight_without_shell" value="0" /> <!-- Changes internal controller gains if we are flying without the propeller guard -->

            <param name="altitude_max" value="3000" /> <!-- in millimeters = 3 meters = 9' -->
        <param name="altitude_min" value="50" /> <!-- in millimeters = 5cm = 2" -->
        <param name="euler_angle_max" value="0.1" /> <!-- maximum allowable body angle in radians = 5 degrees -->
        <param name="control_vz_max" value="200" /> <!-- maximum z velocity in mm/sec = 0.2m/sec -->
            <param name="control_yaw" value="0.7" /> <!-- maximum rotation rate in radians/sec = 40 degrees per second (1/9 rev/sec) -->
        </node>
</group>
<group ns="sim2">
        <node name="ardrone_driver" pkg="ardrone_autonomy" type="ardrone_driver" output="screen" clear_params="true" args="-ip 192.168.1.5">
            <param name="outdoor" value="0" /> <!-- If we are flying outdoors, will select more aggressive default settings -->
            <param name="flight_without_shell" value="0" /> <!-- Changes internal controller gains if we are flying without the propeller guard -->

            <param name="altitude_max" value="3000" /> <!-- in millimeters = 3 meters = 9' -->
        <param name="altitude_min" value="50" /> <!-- in millimeters = 5cm = 2" -->
        <param name="euler_angle_max" value="0.1" /> <!-- maximum allowable body angle in radians = 5 degrees -->
        <param name="control_vz_max" value="200" /> <!-- maximum z velocity in mm/sec = 0.2m/sec -->
            <param name="control_yaw" value="0.7" /> <!-- maximum rotation rate in radians/sec = 40 degrees per second (1/9 rev/sec) -->
        </node>
</group>
```
veiller à ce que aucun driver ardrone ne soit lancé !!!
