/*!
 *  \file path_planning.h
 *  \Give the reference position to the drone
 *  \authors Julien Gérardy & Félicien Schiltz
 *  \date 2016
 *
 *  Part of ucl_drone. Controller ROS node for the
 *  ardrone. Contains:
 *              - a simple pathplanning method
 */

#ifndef UCL_DRONE_PATH_PLANNING_H
#define UCL_DRONE_PATH_PLANNING_H

// Header files
#include <ros/ros.h>
#include <signal.h>

// #define USE_PROFILING
#include <ucl_drone/profiling.h>

// messages
// #include <ardrone_autonomy/Navdata.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <ucl_drone/Pose3D.h>
#include <ucl_drone/PoseRef.h>
#include <ucl_drone/StrategyMsg.h>
#include <ucl_drone/ucl_drone.h>
#include <ucl_drone/cellUpdate.h>
class PathPlanning
{
private:
  ros::NodeHandle nh;

  ros::Publisher poseref_pub;
  ros::Subscriber pose_sub;
  ros::Subscriber strategy_sub;

  std::string poseref_channel;
  std::string pose_channel;
  std::string strategy_channel;

  //Just for some tests
  ros::Publisher mapcell_pub;
  std::string mapcell_channel;

  int i;
  int j;
  //! Callback when pose is received
  void poseCb(const ucl_drone::Pose3D::ConstPtr posePtr);
  void strategyCb(const ucl_drone::StrategyMsg::ConstPtr strategyPtr);
  // double distance(int i, int j, int k, int l);

public:
  //! Constructor
  PathPlanning();
  //! Destructor
  ~PathPlanning();


  ucl_drone::cellUpdate cellUpdateMsg;
  void init();
  double next_x;
  double next_y;
  double next_z;
  double next_rotZ;
  bool instruction_publishing;
  ucl_drone::StrategyMsg lastStrategyReceived;
  ucl_drone::Pose3D lastPoseReceived;
  bool landing;
  bool takeoff;
  void publish_poseref();
  void yaw_desired();
  bool xy_desired();
  void reset();
  void SetRef(double x_ref, double y_ref, double z_ref, double rotZ_ref);
  void InitializeGrid();
  bool gridInitialized;
  int myGrid[SIDE * 10][SIDE * 10];
  int bordersList[SIDE * 100];  // What is the maximum number of frontier cells?
  // bool advanced_xy_desired();
  bool ThereIsAWallCell(int i, int j);
  void UpdateMap(double x, double y);
  void AbsOrdMinMax(double x, double y, int* absMin, int* absMax, int* ordMin, int* ordMax);
  void advanced_xy_desired(double x, double y, double* k, double* l);
  void CellToXY(int i, int j, double* xfromcell, double* yfromcell);
  double distance(int i, int j, int k, int l);
  double XMax;
  double YMax;
  int CellUp;
  int CellDown;
  int CellRight;
  int CellLeft;
  int myAbsMin;
  int myAbsMax;
  int myOrdMin;
  int myOrdMax;
  double xfromcell;
  double yfromcell;
  double xfromcell2;
  double yfromcell2;
  int closestJ;
  int closestI;
  double poseRefX;
  double poseRefY;
  double bestDist;
  double alt;
};

#endif /*UCL_DRONE_PATH_PLANNING_H */
