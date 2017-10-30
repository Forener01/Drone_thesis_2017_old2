/*
 *  This file is part of ucl_drone 2016.
 *  For more information, refer
 *  to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 */

#include <ucl_drone/multi_strategy.h>

MultiStrategy::MultiStrategy()
{
  // Subscriber
  ready_channel = nh_.resolveName("ready");  // check which drones are ready
  ready_sub = nh_.subscribe(ready_channel, 1, &MultiStrategy::readyCb, this);

  // Publishers
  drones_roles_channel = nh_.resolveName("drones_roles");  // broadcast a list of roles
  drones_roles_pub = nh_.advertise< ucl_drone::DroneRoles >(drones_roles_channel, 1);
}

MultiStrategy::~MultiStrategy()
{
}

void MultiStrategy::init()
{
  // TODO: use the vector filled in readyCb
  // TODO: add a launch parameter to choose between several multi drone missions

  // Hardcoded:
  DroneRole role1("ucl_drone_5");
  role1.SetDroneRole(SUPER_8);
  role_list.push_back(role1);
  DroneRole role2("ucl_drone_4");
  role2.SetDroneRole(GO_TO, "/ucl_drone_5/target_detected/");
  role_list.push_back(role2);
}

void MultiStrategy::readyCb(const ucl_drone::DroneRole::ConstPtr readyPtr)
{
  // TODO: Add to a std::vector the drone name contained in readyPtr
}

void MultiStrategy::PublishDroneRole()
{
  ucl_drone::DroneRoles msg = DroneRole::DroneRolesToMsg(role_list);
  drones_roles_pub.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "multi_strategy");

  /*if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }*/

  MultiStrategy my_custom_strategy;
  ROS_DEBUG("end of MultiStrategy initialization");

  ros::Rate r1(3);        // 3Hz
  ros::Rate r2(1 / 5.0);  // 1/5Hz

  my_custom_strategy.init();
  // ros::spin();
  ROS_DEBUG("entering loop 2");
  while (ros::ok())
  {
    TIC(multi);
    my_custom_strategy.PublishDroneRole();
    TOC(multi, "multi strategy");
    ros::spinOnce();
    r2.sleep();
    ROS_DEBUG("multistrategy has published drone role");
  }

  return 0;
}
