#!/bin/bash

# version 1.0 (28/09/17)
# automatic executable for "Operation Seduction"

# changelog
# version 1.0
# author: Aurian d'Avernas


echo "Operation Seduction is starting now!"

gnome-terminal --working-directory=/home/laboinmastudent/Bureau/Drone_thesis_2017/Operation_Seduction/Appareillage -e "bash autoconfarparrot"

sleep 10

gnome-terminal -x bash -c "roscore"

sleep 2

gnome-terminal -x bash -c "roslaunch tum_ardrone ardrone_driver.launch"

sleep 8.5

gnome-terminal -x bash -c "roslaunch tum_ardrone tum_ardrone.launch"

sleep 4.5

gnome-terminal -x bash -c "source ~/rosbuild_ws/setup.bash; rosrun lsd_slam_viewer viewer"

gnome-terminal -x bash -c "source ~/rosbuild_ws/setup.bash; rosrun lsd_slam_core live_slam /image:=/ardrone/front/image_raw _calib:=/home/laboinmastudent/ardrone_front.cfg"
