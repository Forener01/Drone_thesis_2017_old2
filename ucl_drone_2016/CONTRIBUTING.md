# UCL_DRONE CONTRIBUTING

Please read the two reports of master students graduated in 2016.

Please follow the ROS guidelines:

 * http://wiki.ros.org/QAProcess
 * http://wiki.ros.org/CppStyleGuide
 * http://wiki.ros.org/ROSNodeTutorialC%2B%2B
 * https://github.com/davetcoleman/roscpp_code_format

Please document your code and respect the Doxygen syntax:

 * http://wiki.ros.org/rosdoc_lite
 * http://www.stack.nl/~dimitri/doxygen/manual/docblocks.html
 * https://www.stack.nl/~dimitri/doxygen/manual/commands.html

The code contains some TODO tags.
These highlight some ideas of new features that can help the future contributors.
Here is a summary:

  * Parameters:
    * More parameters need to be tunable from the launch file to avoid compilation delay during the test phases
    * Some of these parameters would be tunable during the runtime using the ROS server to ease development
    * Here some examples:
      * target's path (also add a selection menu in the `ucl_drone_gui`)
      * keypoint detection and description
      * thresholds to detect the target
      * thresholds in the mapping node
