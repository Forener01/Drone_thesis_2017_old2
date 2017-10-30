/*
 *  This file is part of ucl_drone 2016.
 *  For more information, refer to the corresponding header file.
 *
 *  TODO
 *
 *  \authors
 *  \date 2016
 *
 */

#include "ucl_drone/controller.h"

static bool urgency_signal = false; // true when Ctrl-C (Emergency stop)

// Constructor
BasicController::BasicController() {
  std::string drone_prefix;
  ros::param::get("~drone_prefix", drone_prefix);
  /*
  List of subscribers and publishers. This node subscribes to pose_estimation to
  get the real-time
  position of the drone. It also subsribes to path_planning to know the new pose
  that the drone must
  reach.

  In order to do that, this node publish in the topic cmd_vel in order to give
  the computed speed
  commands to the drone. It also can send message in order to make the drone
  taking off or landing
  and to change its state (emergency or ok).

  The message reset pose is used to reset the pose when the drone finishes to
  take off. It allows
  the pose_estimation to reset and to erase the drift that happenned during
  launching.

  */

  // Subscribers
  pose_channel = nh.resolveName("pose_estimation");
  pose_sub = nh.subscribe(pose_channel, 10, &BasicController::poseCb, this);
  poseref_channel = nh.resolveName("path_planning");
  poseref_sub =
      nh.subscribe(poseref_channel, 10, &BasicController::poseRefCb, this);

  // Publishers
  control_channel = nh.resolveName(drone_prefix + "cmd_vel");
  vel_pub = nh.advertise<geometry_msgs::Twist>(control_channel, 1);

  takeoff_channel = nh.resolveName(drone_prefix + "ardrone/takeoff");
  takeoff_pub = nh.advertise<std_msgs::Empty>(takeoff_channel, 1, true);

  land_channel = nh.resolveName(drone_prefix + "ardrone/land");
  land_pub = nh.advertise<std_msgs::Empty>(land_channel, 1);

  toggleState_channel = nh.resolveName(drone_prefix + "ardrone/reset");
  toggleState_pub = nh.advertise<std_msgs::Empty>(toggleState_channel, 1, true);

  reset_channel = nh.resolveName("reset_pose");
  reset_pub = nh.advertise<std_msgs::Empty>(reset_channel, 1, true);

  // Services To launch and to quit control.
  startControl_ = nh.advertiseService("drone_ucl/start_control",
                                      &BasicController::startControl, this);

  stopControl_ = nh.advertiseService("drone_ucl/stop_control",
                                     &BasicController::stopControl, this);

  // Here are the parameters used before we got the first pose_ref from the
  // path_planning

  // Parameters
  alt_ref = 1.0; // default reference altitude (unit: m)
  x_ref = 0.0;
  y_ref = 0.0;
  yaw_ref = 0.0;
  isControlling = false;

  /*Here are the gains used for the PID controller. Those are tuned for an
   * AR.Drone 2.0 with the
   * indoor skin.
   */
  // Initializing the variables we need

  // altitude
  Kp_alt = 1.8;
  Ki_alt = 0;
  Kd_alt = 2;
  /*working values inside : Kp_alt=7 & Kd_alt=2
  working values dronelab : Kp_alt=1.5 & Kd_alt =2*/

  // yaw
  Kp_yaw = 3;
  Ki_yaw = 0;
  Kd_yaw = 0.3;
  /*working values : Kp_yaw=3 */

  // XY
  Kp_plan = 0.06;
  Ki_plan = 0.002;
  Kd_plan = 0.2;
  /* working values : Kp_plan=0.06 ki_plan = 0.002 kd_plan = 0.2*/

  // integral_alt_error = 0;
  // integral_yaw_error = 0;
  // integral_xy_error = 0;
  // integral_f_error = 0;
  // integral_l_error = 0;

  // Limitation of the rotZ speed in order to stay stable.
  anti_windup_yaw = 0.1;

  // Initialization of some variables useful for regulation (see below).
  alt_desired_old = 0;
  yaw_desired_old = 0;
  x_desired_old = 0;
  y_desired_old = 0;
  old_yaw_desired = 0;
}

BasicController::~BasicController() {}

// From services
bool BasicController::startControl(std_srvs::Empty::Request &req,
                                   std_srvs::Empty::Response &res) {
  ROS_INFO("calling service start");
  takeoff_pub.publish(std_msgs::Empty());
  sendVelToDrone(0, 0, 0, 0,
                 true); // very important: ensure the state at next start is
                        // hover mode (see
                        // ardrone_autonomy doc)
  isControlling = true;
  ros::Duration(8)
      .sleep(); // Wait for 8s before to reset the odometry (in order to erase
                // drift).
  std_msgs::Empty msg;
  reset_pub.publish(msg);
  sendVelToDrone(0, 0, 0, 0, true);
  ros::Duration(5).sleep(); // Wait for 4s
  return true;
}

bool BasicController::stopControl(std_srvs::Empty::Request &req,
                                  std_srvs::Empty::Response &res) {
  // ROS_INFO("calling service stop");
  sendVelToDrone(
      0, 0, 0,
      0); // very important: ensure the state at next start is hover mode (see
          // ardrone_autonomy doc)
  isControlling = false;
  land_pub.publish(std_msgs::Empty());
  return true;
}

// Regulation in the XY plane. The X and Y regulator are exactly the same. It is
// important to know
// that the drone is controlled by Forward-Backward, Left-Right motion in its
// own repere. So we need
// to use a rotation matrix to match drone command and good drone movement.

void BasicController::reguXY(double *xvel_cmd, double *yvel_cmd, double x_mes,
                             double y_mes, double x_desired, double y_desired,
                             double yaw, double regu_new_time_xy) {
  // If navdata has the same timestamp, send the last command (in order to avoid
  // null division)

  double dist = sqrt(pow((x_mes - x_desired), 2) + pow((y_mes - y_desired), 2));
  double delta_x = x_desired - x_mes;
  double delta_y = y_desired - y_mes;

  // axis transformation (from absolute axis to drone axis in order to give it
  // velocities
  // commands)
  double c_theta = cos(yaw);
  double s_theta = sin(yaw);

  double term_f = (delta_x * c_theta - delta_y * s_theta) / dist * 0.05; // 0.05
  double term_l = (delta_x * s_theta + delta_y * c_theta) / dist * 0.05;

  // Velocities command in the drone repere.
  *xvel_cmd = term_f;
  *yvel_cmd = term_l;
}

// Regulation in altitude, according to the Z axis.
void BasicController::reguAltitude(double *zvel_cmd, double alt_mes,
                                   double alt_desired, double regu_new_time_z) {
  double p_term;
  double d_term = 0; // In case time_diff = 0
  double i_term;

  double time_difference = (regu_new_time_z - regu_old_time_z);
  // If navdata has the same timestamp, send the last command
  if (time_difference != 0) {
    if (alt_desired_old !=
        alt_desired) // Reset of the integral term if target has changed
    {
      integral_alt_error = 0;
    }

    alt_desired_old = alt_desired;
    regu_old_time_z = regu_new_time_z;

    double delta_alt = -alt_mes + alt_desired;

    // Proportional term

    p_term = delta_alt;

    // Differential term
    if (time_difference != 0) {
      d_term = (delta_alt - old_delta_alt) / time_difference;
      old_delta_alt = delta_alt;
    }

    integral_alt_error += delta_alt * time_difference;

    // Integral term

    i_term = 0; // integral_alt_error;

    // Z velocity command sent to the drone

    last_vel_z_command = (Kp_alt * p_term + i_term * Ki_alt + Kd_alt * d_term);

    if (last_vel_z_command > anti_windup_yaw) {
      last_vel_z_command = anti_windup_yaw;
    } else if (last_vel_z_command < -anti_windup_yaw) {
      last_vel_z_command = -anti_windup_yaw;
    }

    *zvel_cmd = last_vel_z_command;
    // printf("zvel_cmd: %lf \n", last_vel_z_command);
    /*printf("p_term: %lf\n",p_term);
    printf("d_term: %lf\n",d_term);
    printf("i_term: %lf\n",i_term);*/
  } else {
    *zvel_cmd = last_vel_z_command;
  }
}

// Regulation of theta, the yaw angle.
void BasicController::reguYaw(double *yawvel_cmd, double yaw_mes,
                              double yaw_desired, double regu_new_time) {
  // ROS_INFO_STREAM("Regulating yaw");
  double p_term;
  double d_term = 0; // In case time_diff = 0
  double i_term;

  double time_difference = (regu_new_time - regu_old_time_yaw);
  double new_vel_yaw_cmd;

  // If navdata has the same timestamp, send the last command
  if (time_difference != 0) {
    if (yaw_desired_old !=
        yaw_desired) // Reset of yaw integral error if the target has changed.
    {
      integral_yaw_error = 0;
    }
    yaw_desired_old = yaw_desired;
    regu_old_time_yaw = regu_new_time;
    double delta_yaw = -yaw_mes + yaw_desired;

    // The yaw angle is included in [-pi;pi] interval. In order to make it go
    // from -179° to 179°
    // without taking the large side (passing through 0) we implemented a way to
    // make it take the
    // shorter way to go to a position. If its proportional term is greater than
    // a half turn, we
    // make it take the other side.

    if (delta_yaw > 3.14159) {
      delta_yaw -= 2 * 3.14159;
    } else if (delta_yaw < -3.14159) {
      delta_yaw += 2 * 3.14159;
    }
    p_term = delta_yaw;
    if (time_difference != 0) {
      d_term = (delta_yaw - old_delta_yaw) / time_difference;
      old_delta_yaw = delta_yaw;
    }
    integral_yaw_error += delta_yaw * time_difference;
    i_term = 0; // integral_yaw_error;
    new_vel_yaw_cmd = (Kp_yaw * p_term + i_term * Ki_yaw + Kd_yaw * d_term);

    // rotational speed limitation (wrongly called anti_windup).
    if (new_vel_yaw_cmd > anti_windup_yaw) {
      new_vel_yaw_cmd = anti_windup_yaw;
    } else if (new_vel_yaw_cmd < -anti_windup_yaw) {
      new_vel_yaw_cmd = -anti_windup_yaw;
    }

    // Velocity sent to the drone.
    last_vel_yaw_command = new_vel_yaw_cmd;
    *yawvel_cmd = last_vel_yaw_command;
  } else {
    *yawvel_cmd = last_vel_yaw_command;
  }
}

// This function publishes the computed velocities in the topic to the drone.
void BasicController::sendVelToDrone(double pitch, double roll, double yaw_vel,
                                     double zvel_cmd, bool force /*=false*/) {
  geometry_msgs::Twist cmdT;
  cmdT.angular.z = yaw_vel;
  cmdT.linear.z = zvel_cmd;
  cmdT.linear.x = pitch;
  cmdT.linear.y = roll;

  // printf("cmdT.angular.z : %lf \n", cmdT.angular.z);
  // assume that while actively controlling,
  // the above for will never be equal to zero, so i will never hover.
  cmdT.angular.x = cmdT.angular.y = 0; // TODO: good idea ?

  if (isControlling || force) {
    vel_pub.publish(cmdT);
  }
}

// This function is called when this node receives a message from the topic
// "pose_estimation". So it
// takes this message and put it in a variable where it will be used in the
// other functions.
void BasicController::poseCb(const ucl_drone::Pose3D::ConstPtr posePtr) {
  lastPoseReceived =
      *posePtr; // TODO : put a rate of 1/5 in the pose estimation node
  // lastPoseReceivedAvailable = true;
}

void BasicController::poseRefCb(const ucl_drone::PoseRef::ConstPtr poseRefPtr) {
  lastPoseRefReceived = *poseRefPtr;
}

// This function is really important. It is here that all the regulation
// functions are called and
// their results are sent to the drone.
void BasicController::controlLoop() {
  double xvel_cmd;
  double yvel_cmd;
  double yawvel_cmd;
  double zvel_cmd;

  // Compute a new reference setpoint if a visual pose estimation has arrived
  // if (lastPoseReceivedAvailable)
  // {
  //   lastPoseReceivedAvailable = false;
  // }

  // Do here what is inside the python script

  reguXY(&xvel_cmd, &yvel_cmd, lastPoseReceived.x, lastPoseReceived.y,
         lastPoseRefReceived.x, lastPoseRefReceived.y, lastPoseReceived.rotZ,
         lastPoseReceived.header.stamp.sec +
             lastPoseReceived.header.stamp.nsec / pow(10, 9));
  reguYaw(&yawvel_cmd, lastPoseReceived.rotZ, lastPoseRefReceived.rotZ,
          lastPoseReceived.header.stamp.sec +
              lastPoseReceived.header.stamp.nsec / pow(10, 9));
  reguAltitude(&zvel_cmd, lastPoseReceived.z, lastPoseRefReceived.z,
               lastPoseReceived.header.stamp.sec +
                   lastPoseReceived.header.stamp.nsec / pow(10, 9));

  zvel_cmd = 0;
  sendVelToDrone(xvel_cmd, yvel_cmd, yawvel_cmd, zvel_cmd, false); // ALL

  // ros::Duration(0.7).sleep();
  //
  // sendVelToDrone(0, 0, 0, 0, true);
  //
  // ros::Duration(1).sleep();

  // used if pathplanning asks to stop the control of the drone.
  if (lastPoseRefReceived.landAndStop) {
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response res;
    stopControl(req, res);
  }
  // used if pathplanning asks to takeoff and start control.
  if (lastPoseRefReceived.takeoffAndStart && !isControlling) {
    // isControlling = true;
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response res;
    startControl(req, res);
  }

  // used when user presses the Ctrl-c keys in order to force landing.
  if (urgency_signal) // force landing when Ctrl-C
  {
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response res;
    stopControl(req, res);
    ros::shutdown();
  }
}

//! Override the default behavior when node is killed (ctrl+C)
static void basicSigintHandler(int sig) { urgency_signal = true; }

//! Main function, launching the controlLoop function.
int main(int argc, char **argv) {
  ros::init(argc, argv, "controller_custom",
            ros::init_options::NoSigintHandler);
  signal(SIGINT, basicSigintHandler);
  BasicController bc;
  ros::Duration(10).sleep();
  ROS_INFO_STREAM("controller_custom node started!");
  ros::Rate r(100);
  while (ros::ok()) {
    bc.controlLoop();
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
