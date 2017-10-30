# UCL_DRONE_2016

This repository contains the code related to the two master theses conducted at INMA (UCL) in 2016 using Parrot AR.Drone 2.0.

Please contact `drones-inma (AT) uclouvain (DOT) be` for more information.

Please read `CONTRIBUTING.md` to respect code conventions.

## Workspace initialisation

Follow these instructions to set up a development ROS workspace containing this repository.

1.	(Install ROS)
2.	Go into the folder where you want to copy this repository.
3.	Clone (`git clone`).
4.	Follow official ROS workspace initialisation (see ROS Wiki)
5.	Add the folder path into your `.bashrc`

Open
```
$ nano ~/.bashrc
```

Add this line at the end of the file
```
source <path to your ROS workspace>/devel/setup.bash
```

#### Required packages and libraries

 * `ardrone_autonomy`, if not present in your ROS installation, clone it into `<path to your ROS workspace>/src`
 * OpenCV 2.4.* is required with non-free modules
 * PCL (PointCloudLibrary)


#### Build

This package use catkin, type the following command in your terminal at your ROS workspace root:

```
catkin_make
```

Edit `CMakeLists.txt` in packages folder if you want to create new ROS nodes or add some dependencies.

## Folders structure

In `ucl_drone` package:

- `drone_preparation/Appareillage` Contains `autoconfarparrot` to reconfigure AR.Drone for multi-drone missions. (see `README` in this folder)

- `doc/` Documentation `rosdoc` (see below).

-	`include/ucl_drone/` Contains C++ header files.

- `launch/` Contains all `.launch` files (see `README` in this folder).

- `msg/` Contains ROS messages definitions.

-	`src/` Contains ROS nodes.

	-	`controller/`

	    This node sends Takeoff/Land and velocities commands. This is a simple pose (position+orientation) controller (in the world coordinates).

	-	`computer_vision/`

	    This node performs keypoints extraction from the video stream and target detection.

	-	`map/`

	    This node builds a map containing observed keypoints and performs pose estimation based on visual keypoints. (see SLAM)

	-	`path_planning/`

	    This node sends successive pose to the controller according to the instructions from the `strategy` node.

	-	`pose_estimation/`

	    This node performs a simple sensor fusion for pose estimation.

	-   `strategy/`

	    This node determines successive operation and drives the drone to the mission objectives.

	-   `multi_strategy/`

	    This node gives a role to each drone before the beginning of the mission.

- `srv/` Contains ROS services defintions.

- `target/` Contains target pictures.

-	`CmakeLists.txt` Contains instructions to build nodes with catkin.

-	`package.xml` Package information.

### Quick start

##### With only one drone and without router

This procedure do not reconfigure the drone and directly use the WiFi hotspot provided by the drone.

0. In a terminal, launch ROS master:   
    ```
    $ roscore
    ```
1. Plug a fully charged battery in the drone and wait until four green LED's are on
2. On the Ubuntu desktop, turn on the network applet, wait until the drone WiFi discovery and connect
3. Once you see the following output in the terminal:
	```
	started core service [/rosout]
	```
	Open another terminal and launch
  ```
  $ roslaunch ucl_drone <FILENAME>.launch
  ```
	according to the mission (see `README` in the `launch/` directory).

	In the `.launch` file, the drone ip have to be `192.168.0.1`


##### With several drones


###### Configuration

The `essid` of each drone has to be known and listed. An `ip` address has to be attributed as described in `README` in `drone_preparation/Appareillage/` folder.

###### Run

0. Verify the computer has a wired connection to the WiFi router.
1.  Plug fully charged batteries in each drone and wait until four green LED's are on for each drone
2.	On the Ubuntu desktop, turn on the network applet, wait until the drone WiFi discovery of each drone
3.	In a console, run

    ```
    $ cd src/ucl_drone/drone_preparation/Appareillage
    $ bash autoconfarparrot
    ```

    Wait until success messages (in green) for each done.
    If a red message (error) is diplayed, follow instructions on the screen.

4. In another console, run:
    ```
    $ roscore
    ```
    wait until
    ```
    started core service [/rosout]
    ```

5. In a last console,
    ```
    $ roslaunch ucl_drone <FILENAME>.launch
    ```
	 according to the mission (see `README` in the `launch/` directory).

6. Press `Ctrl-C` to kill nodes

    **Caution**: If nodes are killed before the landing command is sent, the drone maintain flight ! Either you launch nodes again or you catch the drone carefully in your hands and you turn it over (it turns off the motors and sets the emergency mode).

    You can also use `ucl_drone_gui`: run `rqt` in a console (the plugin is in the menu). Caution! `rqt` has to be launched after `roscore` and cannot be used after `roscore` is killed.


## Camera calibration

Read the
[ardrone_autonomy tutorial](http://ardrone-autonomy.readthedocs.io/en/latest/FAQ.html#how-can-i-calibrate-the-ardrone-front-bottom-camera)

<!-- [ROS Wiki](http://wiki.ros.org/camera_calibration)
and  [Tutorial](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration) -->

#### ucl_drone
To run properly the launch files available with the ucl_drone nodes, the following configuration is needed:

The two following files are required:
 * `~/.ros/camera_info/ardrone_bottom.yaml`
 * `~/.ros/camera_info/ardrone_front.yaml`

These files are generated by the [camera_calibration](http://wiki.ros.org/camera_calibration) node.

Examples of these files are available in `src/ucl_drone/camera_info`.


## Documentation
This folder `src/ucl_drone/doc` contains the documentation generated in html with `rosdoc-lite` which uses doxygen comments in the codes.

The main page of the generated html documentation is here:
`src/ucl_drone/doc/html/index-msg.html`

### Generate documentation
Go into the `src/ucl_drone` folder and use:
```
$ rosdoc_lite .
```
(do not forget the dot `.` which means "the current folder" in bash)
