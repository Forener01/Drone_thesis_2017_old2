/*!
 *  \file strategy.h
 *  \place to select the global strategy
 *  \authors Julien Gérardy & Félicien Schiltz
 *  \date 2016
 *
 *  Part of ucl_drone. Controller ROS node for the
 *  ardrone. Contains:
 *              - the strategy node
 */

#ifndef UCL_DRONE_STRATEGY_H
#define UCL_DRONE_STRATEGY_H

// Header files
#include <ros/ros.h>
#include <signal.h>

// #define USE_PROFILING
#include <ucl_drone/profiling.h>

// Messages
#include <ucl_drone/DroneRole.h>
#include <ucl_drone/DroneRoles.h>
#include <ucl_drone/StrategyMsg.h>
#include <ucl_drone/TargetDetected.h>

class Strategy
{
private:
  std::string drone_name;

  ros::NodeHandle nh;

  ros::Publisher strategy_pub;
  ros::Subscriber target_sub;
  ros::Subscriber target_sub_from_master;
  ros::Subscriber target_sub_from_slave;
  ros::Subscriber multi_strategy_sub;
  ros::Subscriber navdata_sub;
  ros::Subscriber pose_from_slave_sub;

  std::string strategy_channel;
  std::string target_channel;
  std::string multi_strategy_channel;
  std::string target_channel_from_master;  //! Measure
  std::string target_channel_from_slave;
  std::string navdata_channel;
  std::string pose_from_slave_channel;

  ucl_drone::TargetDetected lastTargetDetected;
  ucl_drone::TargetDetected lastTargetDetectedFromMaster;
  ucl_drone::TargetDetected lastTargetDetectedFromSlave;

  //! Callback when TargetDetected is received

  void targetDetectedCb(const ucl_drone::TargetDetected::ConstPtr targetDetectedPtr);
  void targetDetectedFromMasterCb(const ucl_drone::TargetDetected::ConstPtr targetDetectedPtr);
  void targetDetectedFromSlaveCb(const ucl_drone::TargetDetected::ConstPtr targetDetectedPtr);
  void multi_strategyCb(const ucl_drone::DroneRoles::ConstPtr drones_rolesPtr);
  void navdataCb(const ardrone_autonomy::Navdata::ConstPtr navdataPtr);
  void poseFromSlaveCb(const ucl_drone::Pose3D::ConstPtr posePtr);

  float xDetectedBySlave;
  float yDetectedBySlave;
  int i;

public:
  //! Constructor
  Strategy();
  //! Destructor
  ~Strategy();

  ucl_drone::DroneRoles lastDronesrolesreceived;

  void init();
  void publish_strategy();
  void reset();
  void Takeoff();
  void Seek();
  void Goto();
  void Follow();
  void BackToBase();
  void Land();
  void SetXYChosen(double xchosen, double ychosen);
  int FindRole();
  float strategychosen;
  float oldstrategychosen;
  bool Drone1;
  bool Intheair;
  bool StrategyCbreceived;
  bool TargetDetectedFromMaster;
  bool TargetDetectedFromSlave;
  float batteryLeft;
  float oldbatteryLeft;
  bool backupCalled;
  float xchosen;
  float ychosen;
  ardrone_autonomy::Navdata lastNavdataReceived;
  ucl_drone::Pose3D lastPoseReceivedFromSlave;
};

#endif /*UCL_DRONE_STRATEGY_H */
