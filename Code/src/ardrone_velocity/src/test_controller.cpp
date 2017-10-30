#include <ardrone_velocity/test_controller.hpp>

TestController::TestController() {
  ros::NodeHandle nh;

  ros::param::get("~test_type", test_type);
  ros::param::get("~path_type", path_type);
  // // Subscribers
  // battery_sub =
  //     nh.subscribe("ardrone/navdata", 100, &TestController::batteryCb, this);

  // Publishers
  poseref_pub = nh.advertise<geometry_msgs::Pose>("pose_ref_topic", 1000);

  takeoff_pub = nh.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
  land_pub = nh.advertise<std_msgs::Empty>("ardrone/land", 1);
  reset_pub = nh.advertise<std_msgs::Empty>("ardrone/reset", 1);

  if (test_type == WITHOUT_CONTROL) {
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",
                                                 1000); // without controller
    // vel_pub2 = nh.advertise<geometry_msgs::Twist>("cmd_PID_topic", 1000);
    ROS_DEBUG("Vel_pub connected to topic cmd_vel");
  }

  else if (test_type == VEL_CONTROL || test_type == POSE_CONTROL) {
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_PID_topic",
                                                 1000); // with TUD Controller
    ROS_DEBUG("Vel_pub connected to topic cmd_PID");
  }
}

// void TestController::batteryCb(const ardrone_autonomy::Navdata &mynavdata) {
//   battery = mynavdata.batteryPercent;
// }

// void TestController::print_battery(void) {
//   ROS_INFO("The battery percentage is %f", battery);
// }

void TestController::land(void) { land_pub.publish(std_msgs::Empty()); }

void TestController::takeoff(void) { takeoff_pub.publish(std_msgs::Empty()); }

// This function sets new velocities values and publishes them to navdata.
void TestController::load_vel(double linX, double linY, double linZ,
                              double angZ) {
  geometry_msgs::Twist cmd;

  cmd.linear.x = linX;
  cmd.linear.y = linY;
  cmd.linear.z = linZ;
  cmd.angular.z = angZ;

  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;

  vel_pub.publish(cmd);
}

// This function sets the hover mode.
void TestController::hover(void) {
  geometry_msgs::Twist cmd;

  cmd.linear.x = 0.0;
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;
  cmd.angular.z = 0.0;

  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;

  vel_pub.publish(cmd);
}

void TestController::load_pose(double X, double Y, double Z) {
  geometry_msgs::Pose targetpose;

  targetpose.position.x = X;
  targetpose.position.y = Y;
  targetpose.position.z = Z;

  poseref_pub.publish(targetpose);
}

void TestController::test(double sleeptime, double speed, double hovertime) {
  if (test_type == WITHOUT_CONTROL) {
    // Moving to the left #1
    ROS_INFO_STREAM_ONCE(
        "The drone starts the path-planning without control !");
    load_vel(0.0, speed, 0.0, 0.0);
    ros::Duration(sleeptime).sleep();
    ROS_INFO_STREAM_ONCE("Corner #1");
    // Stabilizing #1
    hover();
    ros::Duration(hovertime).sleep();

    // Moving backward #2
    load_vel(-speed, 0.0, 0.0, 0.0);
    ros::Duration(sleeptime).sleep();
    ROS_INFO_STREAM_ONCE("Corner #2");
    // Stabilizing #2
    hover();
    ros::Duration(hovertime).sleep();

    // Moving to the right #3
    load_vel(0.0, -speed, 0.0, 0.0);
    ros::Duration(sleeptime).sleep();
    ROS_INFO_STREAM_ONCE("Corner #3");
    // Stabilizing #3
    hover();
    ros::Duration(hovertime).sleep();

    // Moving forward #4
    load_vel(speed, 0.0, 0.0, 0.0);
    ros::Duration(sleeptime).sleep();
    ROS_INFO_STREAM_ONCE("Corner #4");
    // Stabilizing #4
    hover();
    ros::Duration(hovertime).sleep();
  }

  else if (test_type == VEL_CONTROL) {
    if (path_type == STRAIGHTLINE) {
      ROS_INFO_STREAM_ONCE("The drone starts the path-planning with velocity "
                           "control following a straight line!");
      load_vel(speed, 0.0, 0.0, 0.0);
      ros::Duration(sleeptime).sleep();
      ROS_INFO_STREAM_ONCE("End of the line");
    }

    else if (path_type == SQUARE) {
      // Moving to the left #1
      ROS_INFO_STREAM_ONCE("The drone starts the path-planning with velocity "
                           "control following a square !");
      load_vel(0.0, speed, 0.0, 0.0);
      ros::Duration(sleeptime).sleep();
      ROS_INFO_STREAM_ONCE("Corner #1");
      // Stabilizing #1
      hover();
      ros::Duration(hovertime).sleep();

      // Moving backward #2
      load_vel(-speed, 0.0, 0.0, 0.0);
      ros::Duration(sleeptime).sleep();
      ROS_INFO_STREAM_ONCE("Corner #2");
      // Stabilizing #2
      hover();
      ros::Duration(hovertime).sleep();

      // Moving to the right #3
      load_vel(0.0, -speed, 0.0, 0.0);
      ros::Duration(sleeptime).sleep();
      ROS_INFO_STREAM_ONCE("Corner #3");
      // Stabilizing #3
      hover();
      ros::Duration(hovertime).sleep();

      // Moving forward #4
      load_vel(speed, 0.0, 0.0, 0.0);
      ros::Duration(sleeptime).sleep();
      ROS_INFO_STREAM_ONCE("Corner #4");
      // Stabilizing #4
      hover();
      ros::Duration(hovertime).sleep();
    }
  }

  else if (test_type == POSE_CONTROL) {
    // Going to Corner #1
    ROS_INFO_STREAM_ONCE(
        "The drone starts the path-planning with pose control !");
    load_pose(20.0, 0.0, 0.0);
    ros::Duration(1.5).sleep();
    ROS_INFO_STREAM_ONCE("Corner #1");
    // Stabilizing #1
    hover();
    ros::Duration(sleeptime).sleep();

    // // Going to Corner #2
    // load_pose(-1.2, 1.2, 0.0);
    // ros::Duration(sleeptime).sleep();
    // ROS_INFO_STREAM_ONCE("Corner #2");
    // // Stabilizing #2
    // hover();
    // ros::Duration(sleeptime).sleep();
    //
    // // Going to Corner #3
    // load_pose(-1.2, 0.0, 0.0);
    // ros::Duration(sleeptime).sleep();
    // ROS_INFO_STREAM_ONCE("Corner #3");
    // // Stabilizing #3
    // hover();
    // ros::Duration(sleeptime).sleep();
    //
    // // Going to Corner #4
    // load_pose(0.0, 0.0, 0.0);
    // ros::Duration(sleeptime).sleep();
    // ROS_INFO_STREAM_ONCE("Corner #4");
    // // Stabilizing #4
    // hover();
    // ros::Duration(sleeptime).sleep();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_controller");
  ros::NodeHandle nh;
  TestController mytest;

  ros::Rate loop_rate(50);

  double hovertime = 2.0;  // unit of s
  double sleeptime = 20.0; // unit of s
  double speed = 0.1;      // unit of m/s

  ros::Duration(10.0).sleep();

  // ROS_INFO("The battery percentage is %f", mytest.battery);
  mytest.takeoff();
  ROS_INFO_STREAM_ONCE("The drone is taking off !");
  ros::Duration(3.0).sleep();

  ROS_INFO_STREAM_ONCE("The drone is in hover mode !");
  mytest.hover();
  ros::Duration(15.0).sleep();

  // Launching the test
  // ROS_INFO_STREAM_ONCE(
  //     "The drone starts the path-planning with pose control !");
  // mytest.load_pose(10.0, 0.0, 0.0);
  // ros::spinOnce();
  // ros::Duration(20.0).sleep();
  mytest.test(sleeptime, speed, hovertime);
  // if (mytest.test_type == VEL_CONTROL) {
  //   // Moving to the left #1
  //   ROS_INFO_STREAM_ONCE(
  //       "The drone starts the path-planning with velocity control !");
  //   mytest.load_vel(0.0, speed, 0.0, 0.0);
  //   ros::Duration(sleeptime).sleep();
  //   ROS_INFO_STREAM_ONCE("Corner #1");
  //   // Stabilizing #1
  //   mytest.hover();
  //   ros::Duration(sleeptime).sleep();
  //
  //   // Moving backward #2
  //   mytest.load_vel(-speed, 0.0, 0.0, 0.0);
  //   ros::Duration(sleeptime).sleep();
  //   ROS_INFO_STREAM_ONCE("Corner #2");
  //   // Stabilizing #2
  //   mytest.hover();
  //   ros::Duration(sleeptime).sleep();
  //
  //   // Moving to the right #3
  //   mytest.load_vel(0.0, -speed, 0.0, 0.0);
  //   ros::Duration(sleeptime).sleep();
  //   ROS_INFO_STREAM_ONCE("Corner #3");
  //   // Stabilizing #3
  //   mytest.hover();
  //   ros::Duration(sleeptime).sleep();
  //
  //   // Moving forward #4
  //   mytest.load_vel(speed, 0.0, 0.0, 0.0);
  //   ros::Duration(sleeptime).sleep();
  //   ROS_INFO_STREAM_ONCE("Corner #4");
  //   // Stabilizing #4
  //   mytest.hover();
  //   ros::Duration(sleeptime).sleep();
  // }
  // Final step
  ROS_INFO_STREAM_ONCE("The drone just finished !");
  mytest.hover();
  ros::Duration(3.0).sleep();
  mytest.land();
  ROS_INFO_STREAM_ONCE("The drone just landed !");

  // ROS_INFO("The battery percentage is %f", mytest.battery);

  return 0;
}
