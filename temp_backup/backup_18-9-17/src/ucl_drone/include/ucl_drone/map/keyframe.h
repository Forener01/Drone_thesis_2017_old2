/*!
 *  \file keyframe.h
 *  \brief This header file contains KeyFrame class definition
 *         In this file, 3D points refer to points in world coordinates,
 *         and 2D points refer to points in the calibrated image coordinates.
 *  \authors Arnaud Jacques & Alexandre Leclere
 */

#ifndef UCL_DRONE_KEYFRAME_H
#define UCL_DRONE_KEYFRAME_H
#define PCL_NO_PRECOMPILE

#include <ucl_drone/ucl_drone.h>

// vision
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

/* Point Cloud library */
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>      // pcl::PointXYZRGB
#include <pcl_ros/point_cloud.h>  // pcl::PointCloud

#include <boost/shared_ptr.hpp>

/* ucl_drone */
#include <ucl_drone/PointXYZRGBSIFT.h>
#include <ucl_drone/Pose3D.h>

/** \class KeyFrame
 *  A KeyFrame object stores 3D points seen on a same picture
 *  In the future, with triangulation enabled for 3D reconstruction
 *  a KeyFrame object will store 3D points seen from some close poses
 *  (close to be defined according to the camera observation model)
 *  A list of Frames (containing 2D keypoint) will be necessary
 */
class KeyFrame
{
private:
  static std::vector< KeyFrame* > instances_list;

  int ID;                         //!< Identification number of the keyframe
  std::vector< int > points;      //!< indexes of points in the pointcloud (the map)
  ucl_drone::Pose3D pose;         //!< pose of the drone from which the keypoints were observed
  cv::FlannBasedMatcher matcher;  //!< object for descriptiors comparison
  cv::Mat descriptors;            //!< descriptors in opencv format

public:
  pcl::PointCloud< pcl::PointXYZRGBSIFT >::Ptr cloud;  // all keypoints of this keyframe

  //! Empty Contructor.
  KeyFrame();

  //! Constructor
  //! \param[in] pose Pose of the drone from which the keypoints were observed
  KeyFrame(ucl_drone::Pose3D& pose);

  //! Destructor.
  ~KeyFrame();

  //! Function to get a Keyframe by its ID
  static KeyFrame* getKeyFrame(const int ID);

  //! Function to remove all previous KeyFrames taken
  static void resetList();

  //! Method to add a list of points to the current Keyframe
  //! \param[in] pointcloud points to add in PCL format
  //! \parma[in] map_idx_points indexes of thes points in the map pointcloud
  void addPoints(pcl::PointCloud< pcl::PointXYZRGBSIFT >::Ptr& pointcloud,
                 std::vector< int >& map_idx_points);

  //! Method to add a list of points to the current Keyframe
  //! \param[in] pointcloud points to add in PCL format
  void addPoints(pcl::PointCloud< pcl::PointXYZRGBSIFT >::Ptr& pointcloud);

  //! Method to get current KeyFrame ID number
  int getID();

  //! Method to get the number of keypoints in the current Keyframe
  int size();

  //! Method to get the pose of the drone from which the keypoints were observed
  ucl_drone::Pose3D getPose();

  //! Method to get the keypoints descriptors
  //! \param[out] descriptors Descriptors in the OpenCV format
  void getDescriptors(cv::Mat& descriptors);

  //! Method to get convert keypoints descriptors in the OpenCV format
  void convertDescriptors();

  //! Method to compare the current KeyFrame to a 2D Frame
  //! \param[in] frame Frame to compare
  //! \param[out] idx_matching_points Vector of pairs of indexes: (i,j) with i the map index,
  //! j the frame index
  //! \param[out] keyframe_matching_points Vector of matching points (3D) in the Keyframe
  //! \param[out] frame_matching_points Vector of matching points (2D) in the Frame
  void matchWithFrame(Frame& frame, std::vector< std::vector< int > >& idx_matching_points,
                      std::vector< cv::Point3f >& keyframe_matching_points,
                      std::vector< cv::Point2f >& frame_matching_points);
};

#endif /* UCL_DRONE_KEYFRAME_H */
