## Launch files README

This folder contains files with the parameters fed to `ucl_drone` at launch time.

To use one of these files use the following command:
```
$ roslaunch ucl_drone <FILENAME>.launch
```
#### Files description

 * `drone_8.launch` mission of team 3D
 * `play_with_rosbag.launch` example of launch file to play a [rosbag](http://wiki.ros.org/rosbag) rather than launching a real drone
 * `simple_and_modified.launch` launches one stock drone and one modified (bottom facing camera)
 * `two_simples_new.launch` mission of team tracking
 * `tum_ardrone_single.launch` adapted from tum_ardrone for use with the router
 * `tum_ardrone_double.launch` idem with two drones



#### Step by step description of one example

Each file is structured as the following example (`drone_8.launch`).

```xml
<?xml version="1.0"?>
<launch>
  <!-- IPv4 address of your drones -->
  <arg name="drone_name" default="drone_5" />
  <arg name="mb1_ip" default="192.168.1.1" />

  <arg name="freq" default="8" /> <!-- Ultrasound frequency (7 or 8). -->
```

Header: specify the drone name and ip address as well as other parameters.

```xml
<node name="ucl_drone_multi_strategy" pkg="ucl_drone" type="multi_strategy" output="screen">
</node>
```

command to launch a first node: the `swarm initialisation` node. This node is not nested in a drone group. It requires no parameter.

* `name` is chosen arbitrarily,
* `pkg` the package name
* `type` as defined in the `CMakeList.txt` (`add_executable` line)

```xml
  <group ns="ucl_$(arg drone_name)">

    <arg name="main_motherboard" default="motherboard1" />

```

Declares a new group which corresponds to a physical drone.

```xml
    <!-- [begin] RELAY TOPICS BETWEEN UCL_DRONE_GUI AND ARDRONE_AUTONOMY -->
    <!-- if you need something more sophisticated, create your node and replace the corresponding relay -->
    <node name="relay_navdata" pkg="topic_tools" type="relay"  args="$(arg main_motherboard)/ardrone/navdata navdata">
    </node>
    <node name="relay_emergency" pkg="topic_tools" type="relay"  args="emergency_toggle $(arg main_motherboard)/ardrone/reset">
    </node>
    <node name="relay_land" pkg="topic_tools" type="relay"  args="land $(arg main_motherboard)/ardrone/land">
    </node>
    <node name="relay_takeoff" pkg="topic_tools" type="relay"  args="takeoff $(arg main_motherboard)/ardrone/takeoff">
    </node>
    <!-- [end] RELAY TOPICS BETWEEN UCL_DRONE_GUI AND ARDRONE_AUTONOMY -->

```

These nodes link the `ucl_drone_gui` topics to the `ardrone_autonomy` ones.

```xml
    <group ns="motherboard1">
```

This group embeds the `ardrone_autonomy` node. When several motherboards are used for one single physical drone, (this is the case with the modified bottom facing camera drones) multiple instances of `ardrone_autonomy` are needed, each nested in one separate group to avoid topics names conflicts.

```xml
      <param name="tf_prefix" value="$(arg drone_name)/ucl_mb1" />
      <node name="ardrone_driver" pkg="ardrone_autonomy" type="ardrone_driver"
            output="screen" clear_params="true" args="-ip $(arg mb1_ip)">

        <param name="tf_prefix" value="$(arg drone_name)/autonomy_mb1" />
        <param name="outdoor" value="0" />
        <param name="looprate" value="30" />
        <param name="navdata_demo" value="0" />
        <param name="flight_without_shell" value="0" />
        <param name="altitude_max" value="4000" />
        <param name="altitude_min" value="50" />
        <param name="euler_angle_max" value="0.21" />
        <param name="control_vz_max" value="700" />
        <param name="ultrasound_freq" value="$(arg freq)" />
        <param name="realtime_navdata" value="true" />
        <param name="realtime_video" value="true" />
        <!-- Covariance Values (3x3 matrices reshaped to 1x9)-->
        <rosparam param="cov/imu_la">[0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]</rosparam>
        <rosparam param="cov/imu_av">[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]</rosparam>
        <rosparam param="cov/imu_or">[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 100000.0]</rosparam>
      </node>

```
For a full list of `ardrone_autonomy` parameters see the [documentation](
http://ardrone-autonomy.readthedocs.io/en/latest/parameters.html)


```xml

      <node name="image_proc_bottom" pkg="image_proc" type="image_proc" ns="ardrone/bottom"/>
      <!-- <node name="image_proc_front" pkg="image_proc" type="image_proc" ns="ardrone/front"/> -->

```

These nodes use the camera calibration files (see README) to undistort the images by feeding them into the `image_proc` node.
See [documentation](http://wiki.ros.org/image_proc) for more information.

```xml

    </group>

    <node name="vision_gui" pkg="ucl_drone" type="vision_gui" output="screen">
      <param name="draw_keypoints" value="true" />
      <param name="draw_target" value="true" />
    </node>
```

Vision GUI: open a window with the camera view.

Parameters:  
 * `draw_keypoints`: draw a red dot on each detect keypoint
 * `draw_target`: draw a green rectangle around the target

```xml

    <node name="ucl_drone_pose_estimation" pkg="ucl_drone" type="pose_estimation" output="screen">
      <param name="drone_prefix" value="motherboard1/"/>
        <param name="use_visual_pose" value="true" />
    </node>
```

Pose estimation and fusion

 * `use_visual_pose`: if false, the camera information is not used


```xml
    <!-- bottom camera -->
    <rosparam param="cam_matrix">[705.0, 0.0, 319.16, 0.0, 705.0, 219.6, 0.0, 0.0, 1.0]</rosparam>
    <rosparam param="img_size">[640.0, 360.0]</rosparam>
    <!-- bottom camera -->

    <!-- front camera -->
    <!-- <rosparam param="cam_matrix">[529.1, 0.0, 350.6, 0.0, 529.1, 182.2, 0.0, 0.0, 1.0]</rosparam>
    <rosparam param="img_size">[735.0, 360.0]</rosparam> -->
    <!-- front camera -->
```

Camera parameters: Camera matrix and image size.

```xml
    <node name="ucl_drone_computer_vision" pkg="ucl_drone" type="computer_vision" output="screen">
      <param name="drone_prefix" value="motherboard1/"/>
      <param name="video_channel" value="ardrone/bottom/image_rect_color"/>
      <param name="cam_type" value="bottom"/>
      <param name="use_OpticalFlowPyrLK" value="true"/>
    </node>

    <!-- <param name="video_channel" value="ardrone/front/image_rect_color"/> -->
    <!-- <param name="video_channel" value="ardrone/front/image_raw"/> -->
    <!-- <param name="cam_type" value="front"/> -->
```

Computer vision node:

 * `drone_prefix` motherboard name
 * `video_channel` values:
   * "ardrone/bottom/image_rect_color",
   * "ardrone/front/image_rect_color"
 * `cam_type` values
   * "bottom",
   * "front"
 * `use_OpticalFlowPyrLK` values:
    * "true",
    * "false"

```xml
    <node name="ucl_drone_map_keyframe_based" pkg="ucl_drone" type="map_keyframe_based" output="screen">
        <param name="do_search" value="true" />
        <param name="stop_if_lost" value="true" />
    </node>

```

Mapping Node:

 * `do_search`: activate the search amongst older keyframes (see master thesis text 2016)
 * `stop_if_lost`: if true the mapping is stopped whenever the detected keypoints are insufficient to keep track of movement

```xml
    <node name="ucl_drone_path_planning" pkg="ucl_drone" type="path_planning_custom" output="screen">
      <param name="drone_prefix" value="motherboard1/"/>
    </node>
    <node name="ucl_drone_controller" pkg="ucl_drone" type="controller_custom" output="screen">
      <param name="drone_prefix" value="motherboard1/"/>
    </node>
    <node name="ucl_drone_strategy" pkg="ucl_drone" type="strategy" output="screen">
        <param name="drone_name" value="ucl_$(arg drone_name)" />
    </node>
```
Nodes of the team "tracking".

```xml
</group>

</launch>
```
This is the end :)
