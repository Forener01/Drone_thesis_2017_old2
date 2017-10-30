/*!
 *  \file pose_estimation.h
 *  \brief Pose estimation node of the drone (x,y,z,theta,phi,psi) in an absolute coordinate frame.
 * At the present: on the basis of the Odometry received from ardrone_autonomy and Visual pose
 * estimation from the mapping node In future developpment: Kalman filters, visual odometry, raw
 * sensors, etc.
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 *  Part of ucl_drone. Pose estimation ROS node for the
 *  ardrone.
 */

#ifndef UCL_POSE_ESTIMATION_H
#define UCL_POSE_ESTIMATION_H

#include <ucl_drone/ucl_drone.h>

// #define USE_PROFILING  // Uncomment this line to display timing print in the standard output
#include <ucl_drone/profiling.h>

// Header files
#include <ardrone_autonomy/Navdata.h>
#include <ros/ros.h>
#include <signal.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <queue>

// messages
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <ucl_drone/Pose3D.h>

/** \class  PoseEstimator
 *  This class defines an object which wraps all method to perform the pose estimation based on
 * fusion between sensors and visual information
 */
class PoseEstimator
{
private:
  ros::NodeHandle nh;

  // Subscribers:
  ros::Subscriber navdata_sub;      //!< Subscriber to Navdata messages from ardrone_autonomy
  ros::Subscriber odometry_sub;     //!< Subscriber to Odometry messages from ardrone_autonomy
  ros::Subscriber pose_visual_sub;  //!< Subscriber to visual pose messages from mapping node
  ros::Subscriber reset_sub;        //!< Subscriber to Reset messages (e.g. from gui)

  // Publishers:
  ros::Publisher pose_pub;            //!< Publisher of fused pose estimation
  ros::Publisher end_reset_pose_pub;  //!< Publisher of signal for the reset ending

  // Paths to topics:
  std::string navdata_channel;
  std::string odometry_channel;
  std::string pose_channel;
  std::string end_reset_pose_channel;
  std::string pose_visual_channel;
  std::string pose_visual_correction_channel;
  std::string imu_channel;
  std::string reset_channel;

  //! Copy of the last Navdata message received from ardrone_autonomy
  ardrone_autonomy::Navdata lastNavdataReceived;

  //! Copy of the last Odometry message received from ardrone_autonomy
  nav_msgs::Odometry lastOdometryReceived;

  //! Copy of the last visual pose computed in map node
  ucl_drone::Pose3D lastposeVisualReceived;

  ros::Time odom_time;
  double odometry_x;  //!< Accumulator for the x direction
  std::queue< std::vector< double > > queue_dx;
  double odometry_y;  //!< Accumulator for the y direction
  std::queue< std::vector< double > > queue_dy;
  double odometry_rotX;  //!< Accumulator for the rotX angle
  double odometry_rotY;
  double odometry_rotZ;
  std::queue< std::vector< double > > queue_drotZ;
  double rot_Z_offset;   //!< Offset of rotZ from ardrone_autonomy at reset
  double lastRotX;       //!< Copy of the last rotX received in Odometry
  double lastRotY;       //!< Copy of the last rotY received in Odometry
  double lastRotZ;       //!< Copy of the last rotZ received in Odometry
  double previous_rotX;  //!< Copy of the previous rotX received in Odometry
  double previous_rotY;  //!< Copy of the previous rotY received in Odometry
  double previous_rotZ;  //!< Copy of the previous rotZ received in Odometry

  //! Callback: do a copy of last Navdata message
  void navdataCb(const ardrone_autonomy::Navdata::ConstPtr navdataPtr);

  //! Callback: do a copy of last Odometry message
  void odometryCb(const nav_msgs::Odometry::ConstPtr odometryPtr);

  //! Callback: do a copy of last visual pose message
  void poseVisualCb(const ucl_drone::Pose3D::ConstPtr poseVisualPtr);

  //! Copy of Odometry behalve for (x,y) position, the integration of velocities is recomputed
  bool poseFusion(ucl_drone::Pose3D& pose_msg);

  //! Simple fusion between Odometry and visual pose estimation
  bool queuePoseFusion(ucl_drone::Pose3D& pose_msg);

  //! Sum all the values contained in myqueue (used in queuePoseFusion)
  void processQueue(std::queue< std::vector< double > >& myqueue, double& result);

  //! Add item to myqueue (used in queuePoseFusion)
  void pushQueue(std::queue< std::vector< double > >& myqueue, double item);

  //! Simple copy of Odometry
  bool poseCopy(ucl_drone::Pose3D& pose_msg);

  //! Callback: Listen to reset message and switch reset on
  void resetCb(const std_msgs::Empty msg);

  bool use_visual_pose;  //!< if false only information from ardrone_autonomy is used

public:
  //! Constructor
  PoseEstimator();

  //! Destructor
  ~PoseEstimator();

  bool odometry_publishing;       //!< true after the first Odometry message is received
  bool visual_pose_available;     //!< true when the last visual pose message is not yet processed
  bool pending_reset;             //!< true during the reset phase
  void publish_pose();            //!< sends message with the fused pose
  void publish_end_reset_pose();  //!< sends message at the end of the reset time
  void doReset();                 //!< performs the reset
  void doFlatTrim();              //!< calls flattrim from ardrone_autonomy
};

#endif /* UCL_POSE_ESTIMATION_H */
