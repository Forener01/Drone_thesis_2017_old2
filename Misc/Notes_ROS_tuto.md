- The build folder is the default location of the build space and is where cmake
and make are called to configure and build your packages.

- The devel folder is the default location of the devel space, which is where
your executables and libraries go before you install your packages.

- rosparam allows you to store and manipulate data on the ROS Parameter Server.
The Parameter Server can store integers, floats, boolean, dictionaries, and
lists. rosparam uses the YAML markup language for syntax.

- The advertise() function is how you tell ROS that you want to publish on a given topic name. advertise() returns a Publisher object which allows you to publish messages on that topic through a call to publish().

# Steps to initialize and create a package
1.  $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws/src
    $ catkin_init_workspace
2.  $ cd ~/catkin_ws/
    $ catkin_make
3.  $ source devel/setup.bash
4.  $ catkin_create_pkg [package_name]

# Little tricks
- Timing loop
  - ros::Rate Class to help run loops at a desired frequency.
  - Always write $ros::Rate rate() with $rate.sleep() after.
- ros::Duration(0.5).sleep(); Sleep for the amount of time specified by the duration
- ros::ok() checks whether the program is in good standing as a ROS node.
- killing a node
  - ctrl + c: generates an interrupt signal.
  - ctrl + z: suspend the process, allowing it to be resumed at a later point.
  - ctrl + d: sends an end-of-file signal.
- If ros cannot start due to an invisible already running ros session, use the following commands:
  - killall -9 roscore
  - killall -9 rosmaster

# Rospack et log
- Rospack gives info about a package => # rospack find [package_name]
- Roscd log brings you into the folder where ROS stores his log files.
rospack = ros + pack (age)
roscd = ros + cd
rosls = ros + ls

# Balises de dépendance (package.xml)
build_depend, (dépendances pour la construction)
buildtool_depend, (dépendances pour les outils de construction)
run_depend, (dépendances pour l'exécution)
test_depend. (dépendances pour le test)

# Rosnode
ROS tool to get information about a node.
$ rosnode list
$ rosnode info /rosout
$ rosrun [package_name] [node_name] // runs a node from a given package.

# Rostopic
$ rostopic -h
rostopic bw     display bandwidth used by topic
rostopic echo   print messages to screen
rostopic hz     display publishing rate of topic    
rostopic list   print information about active topics
rostopic pub    publish data to topic
rostopic type   print topic type

# Ros Services
Services are another way that nodes can communicate with each other. Services allow nodes to send a request and receive a response.

rosservice list         print information about active services
rosservice call         call the service with the provided args
rosservice type         print service type
rosservice find         find services by service type
rosservice uri          print service ROSRPC uri

# Rosparam
rosparam allows you to store and manipulate data on the ROS Parameter Server. The Parameter Server can store integers, floats, boolean, dictionaries, and lists. rosparam uses the YAML markup language for syntax.

rosparam set            set parameter
rosparam get            get parameter
rosparam load           load parameters from file
rosparam dump           dump parameters to file
rosparam delete         delete parameter
rosparam list           list parameter names

# Logging levels
Fatal / Highest
Error
Warn
Info
Debug / Lowest

- By default, ROS C++ programs only generate log messages at the INFO level and
higher.

- To avoid useless repetitive messages: ROS_DEBUG_STREAM_ONCE(message)
For a specific rate: ROS_DEBUG_STREAM_THROTTLE( 0 . 1 ,
" This ␣ a p p e a r s ␣ e v e r y ␣ 0 . 1 ␣ s e c o n d s . " ) ;

- Output: console, rosout topic or log file.

- Formatting: ROSCONSOLE_FORMAT with [${severity}] [${time}]: ${message}
-> options: ${file} , ${line} , and ${function} fields or ${node}

- To see /rosout messages: rqt_console

- Size taken by ROS logs: rosclean check
Deleting them with: rosclean purge

rosservice call /node-name/set_logger_level ros.package-name level

# roslaunch
Starts nodes as defined in a launch file.

# MSG and SRV
- msg files are simple text files that describe the fields of a ROS message. They are used to generate source code for messages in different languages.
msgs are just simple text files with a field type and field name per line. The field types you can use are:

    int8, int16, int32, int64 (plus uint*)
    float32, float64
    string
    time, duration
    other msg files
    variable-length array[] and fixed-length array[C]

- an srv file describes a service. It is composed of two parts: a request and a response.
srv files are just like msg files, except they contain two parts: a request and a response. The two parts are separated by a '---' line.

- There's one more step, though. We need to make sure that the msg files are turned into source code for C++, Python, and other languages:
Open package.xml, and make sure these two lines are in it and uncommented:

  <build_depend>message_generation</build_depend>
  <run_depend>message_runtime</run_depend>

-$ rosmsg -h
Commands:
  rosmsg show     Show message description
  rosmsg list     List all messages
  rosmsg md5      Display message md5sum
  rosmsg package  List messages in a package
  rosmsg packages List packages that contain messages

# Useful example
http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29

# Process data
- mkdir ~/bagfiles
cd ~/bagfiles
rosbag record -a

- rosbag info <your bagfile>
- rosbag play <your bagfile>

- To filter content of bags, use rosbag filter.

- rqt_logger_level provides a GUI plugin for configuring the logger level of 
ROS nodes.
- rqt_publisher provides a GUI plugin for publishing arbitrary messages with 
fixed or computed field values. 
- rqt_plot provides a GUI plugin visualizing numeric values in a 2D plot using 
different plotting backends.
- rqt_topic provides a GUI plugin for displaying debug information about ROS 
topics including publishers, subscribers, publishing rate, and ROS Messages.
- rqt_bag provides a GUI plugin for displaying and replaying ROS bag files.

save topic's data to a .txt file: rostopic echo -b file.bag -p /topic > data.txt

# Coding
- include
include <ros/ros.h>
ros::init(argc, argv, "hello_ros");
ros::NodeHandle nh;

in every ROS program

# Compiling
Four steps:
1. Declaring dependencies (CMakeLists.txt and package.xml)
2. Declaring an executable (CMakeLists.txt)
3. Building with $catkin_make
4. Sourcing with $source devel/setup.bash (only once per terminal, even if there are modifications and recompiling with catkin_make)


# Publisher
Be mindful of the lifetime of your ros::Publisher objects.
Creating the publisher is an expensive operation, so it’s a usually bad idea
to create a new ros::Publisher object each time you want to publish a message.
Instead, create one publisher for each topic, and use that publisher throughout
the execution of your program. In pubvel , we accomplish this by declaring the
publisher outside of the while loop.

# Subscriber
- A callback function is a function, used for a subscriber, responding to incoming
messages.
void callback_function(const package_name::type_name &msg) {
...
}

- Always add "ros::spinOnce" or "ros::spin()" in subscriber nodes.

# Graph resource names
- global names
  - leading slash /
  - namespaces: to group related graph resources together
  - base name: describes the resource itself
- relative names
  - without leading slash /
  - default ns + relative name => global name
- namespace (ns)
  - tracked individually for each node
  - ns:=default-namespace
- private names
  - begins with a tilde ~
  - ns = name of the node
  - often used for param/serv governing the operation of a node.
- anonymous names
  - to obey the rule having a unique name assigned automatically


# Launch
- Must be comprised between <launch> and </launch>.
- output="screen" allows to display on screen instead of in the log files.
- respawn="true" to automatically restart a crashed node.
- required="true" to kill other nodes if this one crashes.
- launch-prefix="xterm -e" to start a node in a separate console window.
- possible to add an argument. A command line argument can override a default, but not a value .
<arg name="arg-name" default="arg-value" />
<arg name="arg-name" value="arg-value" />
- creating groups to push several nodes into the same namespace and also to conditionally enable or disable nodes.
- <node
pkg="package-name"
type="executable-name"
name="node-name"
/>
- rosp.init_node() defines a default name which is used if you don't overwrite it (e.g. rosrun rf_trilateration rf_sim_node.py). The name tag in the launch file on the other hand overwrites this name and therefore you will always find the node named the same as the name tag in the launch file.

# Parameters
- rosparam list
- all parameters are owned by the parameter server
- rosparam get parameter_name: asks for the associated value
- rosparam get namespace: get every parameter within a namespace
- rosparam set parameter_name parameter_value: assigning a value
- rosparam set namespace values: assigning several paramets at once with YAML syntax
- rosparam dump filename namespace: storing all parameters in a file
- rosparam load filename namespace: load all parameters on the parameter server
- C++ interface:
  - void ros::param::set(parameter_name, input_value);
  - bool ros::param::get(parameter_name, output_value);
- ROSlaunch:
  - <param name="param-name" value="param-value" />
parameter under a group ns or a node
  - <rosparam command="load" file="path-to-param-file" />

# Services
- differ from messages due to:
  - bidirectionnal: concept of response
  - one-to-one communication: service call initiated by a certain node, which also receives the response.
- client/server-request/response
- service data type contains 2 named fields: request + response

- rosservice list
- rosservice node service-name: finding the node offering the service
- rosservice info service-name: finding the data type of a service
- rossrv show service-data-type-name
- rostopic, rosservice >< rosmsg, rossrv

- from command line: rosservice call service-name request-content

# Rosbag
Record and replay messages/topics.

- rosbag record -O filename.bag topic-names
- rosbag play filename.bag
- rosbag info filename.bag

# Going further
- Visualizing data with "rviz"
- Managing coordinate frames with "tf"
- High-fidelity robot simulator Gazebo

# C pointer
valeur d'un pointeur contient l'adresse d'une autre variable.
var = valeur de var
&var = adresse de var

créer variable de type pointeur: int *ptr 
*** initialisation d'un pointeur: int *ptr = NULL
*** obtenir valeur d'un pointeur: value = *ptr
*** obtenir l'adresse de la valeur: address = ptr