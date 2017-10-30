/*!
 *  \file controller.h
 *  \brief Controller for the ardrone: performs position control
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 *  Part of ucl_drone. Controller ROS node for the
 *  ardrone. Contains:
 *              - a simple altitude Controller
 *  \todo PID
 */

#ifndef UCL_DRONE_CONTROLLER_H
#define UCL_DRONE_CONTROLLER_H

// Header files
#include <ros/ros.h>
#include <signal.h>

// ucl_drone
#include <ucl_drone/ucl_drone.h>

// #define USE_PROFILING
#include <ucl_drone/profiling.h>

// messages
// #include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <ucl_drone/Pose3D.h>
#include <ucl_drone/PoseRef.h>

// services
#include <std_srvs/Empty.h>
//#include <ucl_drone/SetPoseRef.h>

//! \class BasicController
//! \brief A basic position controller
class BasicController
{
private:
  ros::NodeHandle nh;

  ros::Subscriber pose_sub;
  ros::Subscriber poseref_sub;
  ros::Publisher land_pub;
  ros::Publisher takeoff_pub;
  ros::Publisher vel_pub;
  ros::Publisher toggleState_pub;
  ros::Publisher reset_pub;

  std::string pose_channel;
  std::string poseref_channel;
  std::string control_channel;
  std::string takeoff_channel;
  std::string land_channel;
  std::string toggleState_channel;
  std::string reset_channel;

  //! is true if the controller is running
  bool isControlling;

  //! Reference
  float alt_ref;
  double x_ref;
  double y_ref;
  double yaw_ref;

  double Kp_alt;
  double Ki_alt;
  double Kd_alt;
  double Kp_yaw;
  double Kp_plan;
  double Ki_plan;
  // double Kd_plan=0.0015; //This Kd is good for y
  double Kd_plan;
  // double Ki_yaw=0.0001;
  double Ki_yaw;
  double Kd_yaw;
  double integral_alt_error;
  double integral_yaw_error;
  double integral_xy_error;
  double integral_f_error;
  double integral_l_error;
  double anti_windup_yaw;

  double regu_old_time_z;  // How to initialize it?
  double regu_old_time_yaw;
  double regu_old_time_xy;
  double old_delta_alt;
  double old_delta_yaw;
  double dist_old;
  double p_term_f_old;
  double p_term_l_old;
  double alt_desired_old;
  double yaw_desired_old;
  double x_desired_old;
  double y_desired_old;
  double last_vel_z_command;
  double last_vel_yaw_command;
  double last_vel_x_command;
  double last_vel_y_command;
  double old_yaw_desired;

  //! Measure
  ucl_drone::Pose3D lastPoseReceived;

  //! Pose desired
  ucl_drone::PoseRef lastPoseRefReceived;

  //! Service to set a new Pose
  // ros::ServiceServer setPoseRef_;  //! \todo better if it was a ROS action ? (provide feedback)

  //! Service to takeoff and start regulation
  ros::ServiceServer startControl_;

  //! Service to stop regulation and land
  ros::ServiceServer stopControl_;

  void reguXY(double *xvel_cmd, double *yvel_cmd, double x_mes, double y_mes, double x_desired,
              double y_desired, double yaw, double regu_new_time_xy);
  void reguAltitude(double *zvel_cmd, double alt_mes, double alt_desired, double regu_new_time_z);
  void reguYaw(double *yawvel_cmd, double yaw_mes, double yaw_desired, double regu_new_time);

  void sendVelToDrone(double pitch, double roll, double yaw_vel, double zvel_cmd,
                      bool force = false);

  //! Callback when pose is received
  void poseCb(const ucl_drone::Pose3D::ConstPtr posePtr);

  //! Callback when new pose ref is received
  void poseRefCb(const ucl_drone::PoseRef::ConstPtr poseRefPtr);

  //! Service method
  // bool setPoseRef(ucl_drone::SetPoseRef::Request &, ucl_drone::SetPoseRef::Response &);

public:
  //! Contructor.
  BasicController();

  //! Destructor.
  ~BasicController();

  void init();

  void controlLoop();

  //! Service method: (set hover mode and) take off and start the controller
  bool startControl(std_srvs::Empty::Request &, std_srvs::Empty::Response &);

  //! Service method: stop the controller and land (and set hover mode)
  bool stopControl(std_srvs::Empty::Request &, std_srvs::Empty::Response &);
};

#endif /* UCL_DRONE_CONTROLLER_H */
