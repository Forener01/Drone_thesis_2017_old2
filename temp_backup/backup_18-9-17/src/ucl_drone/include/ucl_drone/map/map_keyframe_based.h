/*!
 *  \file map_keyframe_based.h
 *  \brief This header file defines classes for the mapping node and visual pose estimation
 * (keyframe-based)
 *  \authors Arnaud Jacques & Alexandre Leclere
 */

#ifndef UCL_DRONE_SIMPLE_MAP_H
#define UCL_DRONE_SIMPLE_MAP_H
#define PCL_NO_PRECOMPILE

#define USE_PROFILING
#include <ucl_drone/profiling.h>

// #define DEBUG_PROJECTION  // if defined print relative errors of projection for the target

/* Header files */
#include <ucl_drone/ucl_drone.h>

#include <ros/package.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

// vision
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

/* Point Cloud library */
#include <pcl/visualization/point_cloud_geometry_handlers.h>

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>  // pcl::PointXYZRGB
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>  // pcl::fromROSMsg
#include <pcl_ros/point_cloud.h>              // pcl::PointCloud

/* Boost */
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>

/* Messages */
//#include <sensor_msgs/PointCloud2.h>  // sensor_msgs::PointCloud2
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Empty.h>

/* ucl_drone */
#include <ucl_drone/PointXYZRGBSIFT.h>  // pcl::PointXYZRGBSIFT
#include <ucl_drone/Pose3D.h>
#include <ucl_drone/ProcessedImageMsg.h>
#include <ucl_drone/TargetDetected.h>
#include <ucl_drone/map/projection_2D.h>
#include <ucl_drone/opencv_utils.h>
#include <ucl_drone/read_from_launch.h>

#include <ucl_drone/map/frame.h>
#include <ucl_drone/map/keyframe.h>

// to print the pointcloud at the console
void cloud_debug(pcl::PointCloud< pcl::PointXYZRGBSIFT >::ConstPtr cloud);

/** \class Map
 *  This object wraps functions to execute the mapping node
 */
class Map
{
private:
  /* Attributes */

  ros::NodeHandle nh;

  bool processedImgReceived;   //!< true after receiving the first ProcessedImage message
  ucl_drone::Pose3D PnP_pose;  //!< last visual pose estimation
  bool tracking_lost;  //!< true if the last visual information does not permit to estimate the
                       //!< drone pose
  bool do_search;      //!< parameter: if true the previous keyframe are used to perform searches
  bool stop_if_lost;   //!< parameter: if true the mapping stops when tracking_lost is true

  /* Subscribers */
  ros::Subscriber processed_image_sub;  //!< ProcessedImage subscriber
  std::string processed_image_channel_in;
  ros::Subscriber reset_pose_sub;  //!< PoseReset subscriber
  std::string reset_pose_channel;
  ros::Subscriber end_reset_pose_sub;  //!< EndPoseReset subscriber
  std::string end_reset_pose_channel;

  /* Publishers */
  ros::Publisher pose_PnP_pub;  //!< visual pose estimation publisher
  std::string pose_PnP_channel;
  ros::Publisher pose_correction_pub;  //!< correction between sensors and visual estimation
                                       //!< publisher
  std::string pose_correction_channel;
  ros::Publisher target_pub;  //!< target publisher
  std::string target_channel_out;

  /* Tresholds for PnP */
  int threshold_lost;          //!< min number of matching keypoints in the keyframe to compute pose
                               //!< estimation
  int threshold_new_keyframe;  //!< min number of matching keypoints with the keyframe to build a
                               //!< new keyframe
  double threshold_new_keyframe_percentage;  //!< percentage of matching keypoints with the keyframe
                                             //!< to build a new keyframe

  //! Callback when image is received
  void processedImageCb(const ucl_drone::ProcessedImageMsg::ConstPtr processed_image_in);

  //! This method computes the PnP estimation
  //! \param[in] current_frame The frame containing keypoint of the last camera observation
  //! \param[out] PnP_pose The visual pose estimation
  //! \param[out] inliers The indexes of keypoints with a correct matching
  //! \param[in] ref_keyframe The KeyFrame used to perform the estimation
  bool doPnP(Frame current_frame, ucl_drone::Pose3D& PnP_pose, std::vector< int >& inliers,
             KeyFrame* ref_keyframe);

  //! This method searches among previously mapped KeyFrame the closest one with the given Frame
  //! \param[in] pose The estimated pose
  //! \param[out] keyframe_ID The identification number of the closest KeyFrame
  //! \param[in] current_frame The frame object
  bool closestKeyFrame(const ucl_drone::Pose3D& pose, int& keyframe_ID, Frame current_frame);

  // Measure
  ucl_drone::ProcessedImageMsg::ConstPtr lastProcessedImgReceived;  //!< s last message received

  /* Services Definition */

  cv::Mat camera_matrix_K;

  cv::Mat tvec;  //!< last translation vector (PnP estimation)
  cv::Mat rvec;  //!< last rotational vector (PnP estimation)

  Frame previous_frame;          //!< Frame built with the previous ImageProcessed message
  KeyFrame* reference_keyframe;  //!< The reference KeyFrame is the last matching keyframe

  cv::Mat cam_plane_top;
  cv::Mat cam_plane_bottom;
  cv::Mat cam_plane_left;
  cv::Mat cam_plane_right;

  //! This method initializes planes defining the visible area from the camera (according to camera
  //! parameters)
  void init_planes();

  //! Callback
  void resetPoseCb(const std_msgs::Empty& msg);

  //! Callback
  void endResetPoseCb(const std_msgs::Empty& msg);

  //! This method determines if a new KeyFrame is needed
  //! \param[in] number_of_common_keypoints The number of commin keypoint between the reference
  //! KeyFrame and the last Frame
  //! \return true if a  new KeyFrame is needed, false otherwise
  bool newKeyFrameNeeded(int number_of_common_keypoints);
  bool newKeyFrameNeeded(int number_of_common_keypoints, KeyFrame* reference_keyframe_candidate);

public:
  bool pending_reset;  //!< true during a reset

  //! The cloud object containing 3D points
  pcl::PointCloud< pcl::PointXYZRGBSIFT >::Ptr cloud;

  //! The visualizer object to perform projection
  boost::shared_ptr< pcl::visualization::PCLVisualizer > visualizer;

  //! Contructor. Initialize an empty map
  Map();

  //! Destructor.
  ~Map();

  //! old implementation of searches in the map, do not use keyframe, not used anymore
  void getDescriptors(const ucl_drone::Pose3D& pose, cv::Mat& descriptors, std::vector< int >& idx,
                      bool only_visible = true);

  //! old implementation of searches in the map, do not use keyframe, not used anymore
  void getDescriptors(const std::vector< int >& idx, cv::Mat& descriptors);

  //! This method searches all keyframes containing keypoints visible frome the pose given
  //! \param[in] pose The pose of the drone
  //! \param[out] keyframes_ID Identification number of all keyframes in the map visible from the
  //! given pose
  void getVisibleKeyFrames(const ucl_drone::Pose3D& pose,
                           std::vector< std::vector< int > >& keyframes_ID);

  //! This method searches all keypoints visible frome the pose given
  //! \param[in] pose The pose of the drone
  //! \param[out] idx Indexes of all keypoints in the map visible from the given pose
  void getVisiblePoints(const ucl_drone::Pose3D& pose, std::vector< int >& idx);

  //! This method publishes a message if the last ImageProcessed contains information about the
  //! target detection and add the estimated pose of the detected target in world coordinates
  void targetDetectedPublisher();

  //! This method publishes a message containing the visual pose estimation
  //! \param[in] poseFrame Pose published by sensors when the camera picture was received
  //! \param[in] poseFrame Pose estimated with visual information and the contents of the map
  void publishPoseVisual(ucl_drone::Pose3D poseFrame, ucl_drone::Pose3D posePnP);
};

#endif /* UCL_DRONE_SIMPLE_MAP_H */
