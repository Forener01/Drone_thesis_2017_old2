#include <ardrone_velocity/test_controller.hpp>

TestController::TestController() {
  ros::NodeHandle nh;

  ros::param::get("~test_type", test_type);
  ros::param::get("~path_type", path_type);
  ros::param::get("~the_speed", speed);
  ros::param::get("~the_hovertime", hovertime);
  ros::param::get("~the_sleeptime", sleeptime);

  // // Subscribers
  // battery_sub =
  //     nh.subscribe("ardrone/navdata", 100, &TestController::batteryCb, this);
  raw_odom_sub =
      nh.subscribe("ardrone/odometry", 100, &TestController::raw_odomCb, this,
                   ros::TransportHints().tcpNoDelay());
  // Publishers
  poseref_pub = nh.advertise<geometry_msgs::Pose>("pose_ref_topic", 1);

  takeoff_pub = nh.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
  land_pub = nh.advertise<std_msgs::Empty>("ardrone/land", 1);
  reset_pub = nh.advertise<std_msgs::Empty>("ardrone/reset", 1);

  raw_error_pub = nh.advertise<nav_msgs::Odometry>("raw/vel_error_topic", 100);
  raw_percent_error_pub =
      nh.advertise<nav_msgs::Odometry>("raw/vel_percent_error_topic", 100);

  if (test_type == WITHOUT_CONTROL) {
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",
                                                 1); // without controller
    // vel_pub2 = nh.advertise<geometry_msgs::Twist>("cmd_PID_topic", 1000);
    ROS_INFO("Vel_pub connected to topic cmd_vel");
  }

  else {
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_PID_topic",
                                                 1); // with TUD Controller
    ROS_INFO("Vel_pub connected to topic cmd_PID");
  }
}

void TestController::raw_odomCb(const nav_msgs::Odometry &odo_msg) {
  m_raw_odo_msg = odo_msg;

  double current_speed = 0.0;

  error_x = m_raw_odo_msg.twist.twist.linear.x;
  error_y = m_raw_odo_msg.twist.twist.linear.y;
  error_z = m_raw_odo_msg.twist.twist.linear.z;

  m_raw_error_msg.twist.twist.linear.x = error_x;
  m_raw_error_msg.twist.twist.linear.y = error_y;
  m_raw_error_msg.twist.twist.linear.z = error_z;

  m_raw_percent_error_msg.twist.twist.linear.x = error_x / current_speed;
  m_raw_percent_error_msg.twist.twist.linear.y = error_y / current_speed;
  m_raw_percent_error_msg.twist.twist.linear.z = error_z / current_speed;

  raw_error_pub.publish(m_raw_error_msg);
  raw_percent_error_pub.publish(m_raw_percent_error_msg);
}

void TestController::land(void) { land_pub.publish(std_msgs::Empty()); }

void TestController::takeoff(void) { takeoff_pub.publish(std_msgs::Empty()); }

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

void TestController::load_pose(double Xpos, double Ypos, double Zpos) {
  targetpose.position.x = Xpos;
  targetpose.position.y = Ypos;
  targetpose.position.z = Zpos;

  poseref_pub.publish(targetpose);
  // ros::spinOnce();
}

void TestController::test() {
  if (test_type == WITHOUT_CONTROL) {
    if (path_type == STRAIGHTLINE) {
      ROS_INFO("The drone starts the path-planning without "
               "control following a straight line during %f sec at %f m/s",
               sleeptime, speed);
      load_vel(speed, 0.0, 0.0, 0.0);
      ros::Duration(sleeptime).sleep();
      ROS_INFO_STREAM_ONCE("End of the line");
    }

    else if (path_type == LEFTLINE) {
      ROS_INFO("The drone starts the path-planning without "
               "control following a left line during %f sec at %f m/s",
               sleeptime, speed);
      load_vel(0.0, speed, 0.0, 0.0);
      ros::Duration(sleeptime).sleep();
      ROS_INFO_STREAM_ONCE("End of the line");
    }

    else if (path_type == SQUARE) {
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
        ROS_INFO("The drone starts the path-planning with velocity "
                 "control following a straight line during %f sec at %f m/s",
                 sleeptime, speed);
        load_vel(speed, 0.0, 0.0, 0.0);
        ros::Duration(sleeptime).sleep();
        ROS_INFO_STREAM_ONCE("End of the line");
      }

      else if (path_type == LEFTLINE) {
        ROS_INFO("The drone starts the path-planning with velocity "
                 "control following a left line during %f sec at %f m/s",
                 sleeptime, speed);
        load_vel(0.0, speed, 0.0, 0.0);
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

      else if (path_type == STATIC) {
        ROS_INFO_STREAM_ONCE(
            "The drone starts hovering with velocity control !");
        hover();
        ros::Duration(sleeptime).sleep();
      }
    }

    else if (test_type == POSE_CONTROL) {

      if (path_type == STRAIGHTLINE) {
        // Going straightforward during 3 meters
        ROS_INFO_STREAM_ONCE(
            "The drone starts the path-planning with pose control !");
        load_pose(3.0, 0.0, 0.0);
        ros::Duration(35.0).sleep();
        ROS_INFO_STREAM_ONCE("Corner #1");
        // Stabilizing #1
        hover();
        ros::Duration(sleeptime).sleep();
      }

      else if (path_type == SQUARE) {
        // Going to Corner #1
        ROS_INFO_STREAM_ONCE(
            "The drone starts the path-planning with pose control !");
        load_pose(0.0, 1.2, 0.0);
        ros::Duration(sleeptime).sleep();
        ROS_INFO_STREAM_ONCE("Corner #1");
        // Stabilizing #1
        hover();
        ros::Duration(hovertime).sleep();
        // Going to Corner #2
        load_pose(-1.2, 1.2, 0.0);
        ros::Duration(sleeptime).sleep();
        ROS_INFO_STREAM_ONCE("Corner #2");
        // Stabilizing #2
        hover();
        ros::Duration(hovertime).sleep();

        // Going to Corner #3
        load_pose(-1.2, 0.0, 0.0);
        ros::Duration(sleeptime).sleep();
        ROS_INFO_STREAM_ONCE("Corner #3");
        // Stabilizing #3
        hover();
        ros::Duration(hovertime).sleep();

        // Going to Corner #4
        load_pose(0.0, 0.0, 0.0);
        ros::Duration(sleeptime).sleep();
        ROS_INFO_STREAM_ONCE("Corner #4");
        // Stabilizing #4
        hover();
        ros::Duration(hovertime).sleep();
      }

      else if (path_type == STATIC) {
        ROS_INFO_STREAM_ONCE("The drone starts hovering with pose control !");
        hover();
        ros::Duration(sleeptime).sleep();
      }
    }
  }
}

// This function makes the drone taking off and stabilizing during 2 sec.
void TestController::init(void) {
  takeoff();
  ROS_INFO_STREAM_ONCE("The drone is taking off !");
  ros::Duration(7.0).sleep();

  ROS_INFO_STREAM_ONCE("The drone is in hover mode !");
  hover();
  ros::Duration(2.5).sleep();
}

// This function stabilizes the drone at the end of the test and do the drone
// landing.
void TestController::finish(void) {
  ROS_INFO_STREAM_ONCE("The drone just finished !");
  hover();
  ros::Duration(3.0).sleep();
  land();
  ROS_INFO_STREAM_ONCE("The drone just landed !");
  ros::Duration(4.0).sleep();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_controller");
  ros::NodeHandle nh;
  TestController mytest;

  ros::Rate loop_rate(50);

  /*** Waiting time to record .bag file on rqt ***/
  ros::Duration(20.0).sleep();
  ROS_INFO_STREAM_ONCE("TUD_PID test!");

  /*** INITIALIZATION ***/
  mytest.init();

  /*** LAUNCHING THE TEST ***/
  mytest.test();

  /*** FINISHING ***/
  mytest.finish();

  return 0;
}
