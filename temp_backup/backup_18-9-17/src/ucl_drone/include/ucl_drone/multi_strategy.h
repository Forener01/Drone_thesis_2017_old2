/*!
 *  \file multi_strategy.h
 *  \brief Basic strategy for multi-agent flight. Specify the mission and the
 * drone roles.
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 *  Part of ucl_drone. Multi-Agent strategy ROS node for the specific mission.
 * Contains:
 *        -
 *        -
 *        -
 *        -
 */

#ifndef UCL_DRONE_MULTI_STRATEGY_H
#define UCL_DRONE_MULTI_STRATEGY_H

// ROS Header files
#include <ros/package.h>
#include <ros/ros.h>

// #define USE_PROFILING  // Uncomment this line to display timing print in the standard output
#include <ucl_drone/profiling.h>

// messages
#include <ucl_drone/DroneRole.h>
#include <ucl_drone/DroneRoles.h>
#include <ucl_drone/Pose3D.h>

// ucl_drone
#include <ucl_drone/drone_role.h>
#include <ucl_drone/ucl_drone.h>

/*!
 *  \class MultiStrategy
 *  \brief Provide tools to let drones communicate a common strategy
 */
class MultiStrategy
{
private:
  ros::NodeHandle nh_;

  // Subscriber
  ros::Subscriber ready_sub;
  std::string ready_channel;

  // Publishers
  ros::Publisher drones_roles_pub;
  std::string drones_roles_channel;

  //! \brief Callback: check which drone are ready to take part to the mission
  void readyCb(const ucl_drone::DroneRole::ConstPtr readyPtr);

  //! Frequently updated list of drone roles
  std::vector< DroneRole > role_list;

public:
  //! \brief Contructor.
  MultiStrategy();

  //! \brief Destructor.
  ~MultiStrategy();

  void init();

  void PublishDroneRole();
};

#endif /* UCL_DRONE_MULTI_STRATEGY_H */
