/*
 *  This file is part of ucl_drone 2016.
 *  For more information, refer
 *  to the corresponding header file.
 *
 * This file receives information from Strategy and the Pose_estimation and publishes to the
 * Controller.
 * It tells the controller where the drone must go as function of the strategy and the position of
 * the drone.
 *
 *  \authors Julien Gérardy & Félicien Schiltz
 *  \date 2016
 *
 */

#include "ucl_drone/strategy.h"

// Constructor
Strategy::Strategy()
{
  // drone prefix and name from the launch file.
  std::string drone_prefix;
  ros::param::get("~drone_prefix", drone_prefix);

  ros::param::get("~drone_name", drone_name);

  // List of subscribers and publishers. This node subscribes to target detection from drone 4,
  // drone 5 and from the drone to which it belongs. This allows this node to always know wich drone
  // sees the target. It is also subscribed to the Multi_drone in order to know which drone is the
  // master and which one is the slave. This gives information of what strategy to chose. The
  // subscription to the master drone navdata gives the information about its battery. This is the
  // only way for the secondary drone to know when it has to start. The subscription to the pose
  // estimation to the secondary drone tells to the main drone when it may go back to the base
  // because the secondary drone is close enough to replace it.
  //
  // This node only publish in the topic strategy that is read from the path_planning.

  // Subscribers

  target_channel = nh.resolveName("ucl_drone/target_detected/");
  target_sub = nh.subscribe(target_channel, 10, &Strategy::targetDetectedCb, this);

  /* ** Other group's comments: ** */
  // TODO: subscribe to channels in funtion of the role ...
  // TODO: do not use absolute path to get the target_detected channel of the current drone
  // TODO: get the other drone name from the DroneRoles message sent by swarm_initialization

  target_channel_from_master = nh.resolveName("/ucl_drone_5/ucl_drone/target_detected/");
  target_sub_from_master =
      nh.subscribe(target_channel_from_master, 10, &Strategy::targetDetectedFromMasterCb, this);

  target_channel_from_slave = nh.resolveName("/ucl_drone_4/ucl_drone/target_detected/");
  target_sub_from_slave =
      nh.subscribe(target_channel_from_slave, 10, &Strategy::targetDetectedFromSlaveCb, this);

  multi_strategy_channel = nh.resolveName("/drones_roles");
  multi_strategy_sub = nh.subscribe(multi_strategy_channel, 10, &Strategy::multi_strategyCb, this);

  navdata_channel = nh.resolveName("/ucl_drone_5/motherboard1/ardrone/navdata");
  navdata_sub = nh.subscribe(navdata_channel, 10, &Strategy::navdataCb, this);

  pose_from_slave_channel = nh.resolveName("ucl_drone_4/pose_estimation");
  pose_from_slave_sub = nh.subscribe(pose_from_slave_channel, 10, &Strategy::poseFromSlaveCb, this);

  // Publishers

  strategy_channel = nh.resolveName("strategy");
  strategy_pub = nh.advertise< ucl_drone::StrategyMsg >(strategy_channel, 1);

  /* ** Other group's comments: ** */
  // TODO: Publisher to send a "ready state" message with the drone_name
  // to the swarm_initialization (multistrategy)
  // the topic name is "ready"
  // the message type is ucl_drone::DroneRole
  // fill the drone_name only

  // Initialization of some parameters.

  Intheair = false;
  StrategyCbreceived = false;
  TargetDetectedFromMaster = false;
  TargetDetectedFromSlave = false;
  backupCalled = false;
  batteryLeft = 100;
  oldbatteryLeft = 101;
}

// Destructor
Strategy::~Strategy()
{
}

// each strategy is corresponding to a number. This number will be sent (with a pose if needed) to
// the pathplanning.

void Strategy::reset()
{
  strategychosen = 0.0;
  oldstrategychosen = 0.0;
}

void Strategy::Takeoff()
{
  oldstrategychosen = strategychosen;
  strategychosen = 1.0;
}

void Strategy::Seek()
{
  oldstrategychosen = strategychosen;
  strategychosen = 2.0;
}

void Strategy::Goto()
{
  oldstrategychosen = strategychosen;
  strategychosen = 3.0;
}

void Strategy::Land()
{
  oldstrategychosen = strategychosen;
  strategychosen = 4.0;
}

void Strategy::Follow()
{
  oldstrategychosen = strategychosen;
  strategychosen = 5.0;
}

void Strategy::BackToBase()
{
  oldstrategychosen = strategychosen;
  strategychosen = 6.0;
}

// This function is used to find the role of the drone, i.e. if it the main or the secondary drone.
int Strategy::FindRole()
{
  i = 0;
  while (ros::ok() && i < 2)  // 2 car 2 drones pour le moment
  {
    if (lastDronesrolesreceived.roles[i].name == drone_name)
    {
      return lastDronesrolesreceived.roles[i].role;
    }
    i++;
  }
  /* ** Other group's comments: ** */
  // TODO: the drone should wait until a role is attributed ...
  printf("I did not find my role");
  return 0.0;
}

// This function give the position chosen to the object of this function.
void Strategy::SetXYChosen(double xchosen, double ychosen)
{
  this->xchosen = xchosen;
  this->ychosen = ychosen;
}

// This function sent the strategy number and the position chosen to the path_planning.
void Strategy::publish_strategy()
{
  // instantiate the strategy message
  ucl_drone::StrategyMsg strategy_msg;

  strategy_msg.type = strategychosen;
  strategy_msg.x = xchosen;
  strategy_msg.y = ychosen;

  // publish
  strategy_pub.publish(strategy_msg);
}

// This function is called when the topic of the target_detected of this drone publishes something.
void Strategy::targetDetectedCb(const ucl_drone::TargetDetected::ConstPtr targetDetectedPtr)
{
  lastTargetDetected = *targetDetectedPtr;
  oldstrategychosen = strategychosen;
  // xchosen = lastTargetDetectedFromSlave.world_point.x;
  // ychosen = lastTargetDetectedFromMaster.world_point.y;
}

// This function is called when the topic of the target_detected of the drone master publishes
// something. The place of the target become the x and y chosen that will be sent to the
// pathplanning.
void Strategy::targetDetectedFromMasterCb(
    const ucl_drone::TargetDetected::ConstPtr lastTargetDetectedPtr)
{
  lastTargetDetectedFromMaster = *lastTargetDetectedPtr;
  xchosen = lastTargetDetectedFromMaster.world_point.x;
  ychosen = lastTargetDetectedFromMaster.world_point.y;
  TargetDetectedFromMaster = true;

  printf("target detected from master \n");
}

// This function is called when the topic of the target_detected of the secondary drone publishes
// something. The position of the target on the real playground will be set in the x and y chosen
// and in the x and y dectectedbyslave variables.

void Strategy::targetDetectedFromSlaveCb(
    const ucl_drone::TargetDetected::ConstPtr lastTargetDetectedPtr)
{
  lastTargetDetectedFromSlave = *lastTargetDetectedPtr;
  xDetectedBySlave = lastTargetDetectedFromSlave.world_point.x;
  yDetectedBySlave = lastTargetDetectedFromSlave.world_point.y;
  xchosen = lastTargetDetectedFromSlave.world_point.x;
  ychosen = lastTargetDetectedFromSlave.world_point.y;
  TargetDetectedFromSlave = true;
  ROS_INFO("Slave detected the target");
}

// This function is executed when the multi strategy publishes something. This message contains the
// roles (main/master, secondary/slave) of the drones.
void Strategy::multi_strategyCb(const ucl_drone::DroneRoles::ConstPtr drones_rolesPtr)
{
  lastDronesrolesreceived = *drones_rolesPtr;
  StrategyCbreceived = true;
  ROS_DEBUG("strategy received multistrategy message");
}

// This function is called when a message is published in the navdata channel. Navdata channel
// contains a lot of information but, in this case, the intersting one is the percentage of the
// master drone. This will allow to call the secondary drone if the latter is to low.
void Strategy::navdataCb(const ardrone_autonomy::Navdata::ConstPtr navdataPtr)
{
  lastNavdataReceived = *navdataPtr;
  batteryLeft = lastNavdataReceived.batteryPercent;

  if (drone_name == "ucl_drone_4" && batteryLeft != oldbatteryLeft)
  {
    ROS_INFO("BatteryLeft read from drone 2 only %lf:", batteryLeft);
    // ROS_INFO("BackupCalled is %d", backupCalled);
    oldbatteryLeft = batteryLeft;
  }
  if (batteryLeft < 75)
  {
    backupCalled = true;
  }
}

// This function is called when the drone slave published its pose.
void Strategy::poseFromSlaveCb(const ucl_drone::Pose3D::ConstPtr posePtr)
{
  lastPoseReceivedFromSlave = *posePtr;
}

// This is the main function where the strategy to sent to the path_planning will be chosen as a
// function of all above data.
int main(int argc, char** argv)
{
  ROS_INFO_STREAM("strategy started");
  ros::init(argc, argv, "strategy");
  printf("Main de la strategy");

  /*  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
      ros::console::notifyLoggerLevelsChanged();
    }*/

  Strategy myStrategy;
  ros::Rate r(20);  // This function refreshes every 1/20 second.
  ROS_DEBUG("Strategy started");
  myStrategy.reset();
  myStrategy.publish_strategy();
  ros::spinOnce();
  r.sleep();

  // while strategy hasn't receive the roles from the multi_strategy, it does nothing and keep
  // listening.
  while (!myStrategy.StrategyCbreceived)
  {
    // printf("Waiting for strategy to send me something to do");
    ros::spinOnce();
    r.sleep();
  }

  // If the role received is 4, that means this node belongs to the master drone. So, strategy 1
  // (takeoff) is sent to the pathplanning.
  // Then, the drone waits 10 seconds (time to take off and get stabilized). After that, the
  // strategy 2 (Seek/explore) is sent to the path_planning.
  //
  // ros::spinOnce(); means that ros can see if it got a message from a topic.
  if (myStrategy.FindRole() == 4)
  {
    myStrategy.Takeoff();
    myStrategy.publish_strategy();
    ros::spinOnce();
    r.sleep();
    ros::Duration(10).sleep();  // Wait for 10s
    myStrategy.Seek();
    myStrategy.publish_strategy();
    ros::spinOnce();
    r.sleep();

    while (ros::ok())
    {
      TIC(stratego);
      // The strategy stays in Seek/exploration mode until the drones find the target. Then the
      // strategy becomes "Follow" the target.
      if (myStrategy.TargetDetectedFromMaster && !myStrategy.TargetDetectedFromSlave)
      {
        myStrategy.strategychosen = 5.0;
        // ROS_INFO("Strategy: Follow");
      }
      // If the backup is called and that the secondary drone is near from the main drone, so the
      // strategy back to base is sent to the pathplanning node.
      if ((myStrategy.lastPoseReceivedFromSlave.x - myStrategy.xchosen) *
                      (myStrategy.lastPoseReceivedFromSlave.x - myStrategy.xchosen) +
                  ((myStrategy.lastPoseReceivedFromSlave.y - 0.5) - myStrategy.ychosen) *
                      ((myStrategy.lastPoseReceivedFromSlave.y - 0.5) - myStrategy.ychosen) <
              16 &&
          myStrategy.strategychosen == 5.0 && myStrategy.backupCalled)
      {
        myStrategy.BackToBase();
      }
      myStrategy.publish_strategy();
      //  }
      ros::spinOnce();

      TOC(stratego, "stratego");
      r.sleep();
    }
  }

  // if the messages got from the multi_drone node contains role = 2, so this node belongs to the
  // drone slave. So the strategy GOTO is selected but not yet sent to the pathplanning node.
  else if (myStrategy.FindRole() == 2)
  {
    myStrategy.strategychosen = 3.0;

    // While the backup, i.e. this drone, is not called, Ros does nothing and read its topics. So,
    // the strategy is still not sent and nothing happen to the secondary drone.If
    // backup is called, we go to the next loop.
    while (ros::ok() && !myStrategy.backupCalled)
    {
      // Waiting for master to reach its critical battery.
      ros::spinOnce();
      r.sleep();
    }

    while (ros::ok())
    {
      // if the backup is called (previous loop) and the drone has not yest took off, the strategy
      // take off is sent to the pathplanning. Then the drone waits 10 seconds to stabilized.
      // Then,
      // the strategy goto is sent to the pathplanning and the drone is considred as "intheair" so
      // this loop will not be called anymore.
      if (myStrategy.strategychosen == 3.0 && !myStrategy.Intheair)
      {
        myStrategy.Takeoff();
        myStrategy.publish_strategy();
        ros::spinOnce();
        r.sleep();
        ros::Duration(10).sleep();  // Wait for 10s
        myStrategy.Goto();  // Goto is called and x,y from the TargetDetectedFromMaster are the
                            // pose_ref
        myStrategy.publish_strategy();
        ros::spinOnce();
        r.sleep();
        myStrategy.Intheair = true;
      }

      // When the slave detects the target, the strategy follow is sent to the pathplanning.The
      // coordinates from the strategy message are the position of the target read from the
      // targetDetectedFromSlaveCb message.
      if (myStrategy.TargetDetectedFromSlave)
      {
        myStrategy.strategychosen = 5.0;
        myStrategy.publish_strategy();
        // ROS_INFO("Slave must follow the target!");
        ros::spinOnce();
        r.sleep();
      }

      // If the drone must land and is still in the air, this strategy is sent to the pathplanning
      // and the variable Intheair is reset to false. Land is never called by the IA for the
      // secondary drone. So, this function is only true when we ask for an emergency stop.
      if (myStrategy.strategychosen == 4.0 && myStrategy.Intheair)
      {
        myStrategy.publish_strategy();
        ros::spinOnce();
        r.sleep();
        myStrategy.Intheair = false;
      }
      ros::spinOnce();
      r.sleep();
    }
  }
  return 0;
}
