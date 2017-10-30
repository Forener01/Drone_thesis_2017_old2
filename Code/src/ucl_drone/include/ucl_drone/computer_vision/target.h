/*!
 *  \file target.h
 *  \brief Header file for the Target class which wraps all procedures to detect a predfined target
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 *  Part of ucl_drone.
 */

#ifndef UCL_DRONE_TARGET_H
#define UCL_DRONE_TARGET_H

// #define DEBUG_TARGET // if defined a window with target matches is displayed

// #define DEBUG_PROJECTION  // if defined print relative errors of projection for the target

#include <ucl_drone/computer_vision/computer_vision.h>

//! Filename to the target from within the package
static const std::string TARGET_RELPATH = "/target/target_bottom.png";

#ifdef DEBUG_TARGET
static const std::string OPENCV_WINDOW = "Object matches";
#endif

/*!
 *  \class Target
 *  \brief Provide tools to track the presence of a target
 */
class Target
{
private:
  cv::Mat image;                          //! Picture of the target as read at $TARGET_RELPATH$
  std::vector< cv::KeyPoint > keypoints;  //! keypoints detected on the target picture
  cv::Mat descriptors;                    //! target keypoints descriptors
  std::vector< cv::Point2f > centerAndCorners;  //! position of the center and the corners of the
                                                //! target
  cv::FlannBasedMatcher matcher;  //! wrapper object to the FLANN library to perform matching with
                                  //! the video pictures

public:
  //! Constructor
  Target();

  //! Destructor
  ~Target();

  //! initializer
  bool init(const std::string relative_path);

  //! This method detects the target in a given picture
  //! \param[in] cam_descriptors The desciptors of keypoints in camera picture
  //! \param[in] cam_keypoints The coordinates of keypoints in camera picture
  //! \param[out] good_matches The good matches between the target picture and the camera picture,
  //! in OpenCV format
  //! \param[out] idxs_to_remove A list of indexes of keypoints on the target
  //! \param[out] target_coord The coordinates if the target is detected
  //! \param[in] pose (#ifdef DEBUG_PROJECTION) Pose of the drone estimated with
  //! \param[in] image_cam (#ifdef DEBUG_TARGET) image matrix (OpenCV format)
  //! \return true if the target is detected
  bool detect(cv::Mat cam_descriptors, std::vector< cv::KeyPoint >& cam_keypoints,
              std::vector< cv::DMatch >& good_matches, std::vector< int >& idxs_to_remove,
              std::vector< cv::Point2f >& target_coord
#ifdef DEBUG_PROJECTION
              ,
              ucl_drone::Pose3D pose
#endif
#ifdef DEBUG_TARGET
              ,
              cv::Mat& image_cam
#endif
              );

  //! This method draws a green frame to indicate the detected target
  //! \param[in] cam_img image matrix (OpenCV format)
  void draw(cv::Mat cam_img, std::vector< cv::KeyPoint > cam_keypoints,
            std::vector< cv::DMatch > good_matches, cv::Mat& img_matches);

  //! This method computes the position of the target on the camera image
  void position(std::vector< cv::KeyPoint > cam_keypoints, std::vector< cv::DMatch > good_matches,
                std::vector< cv::Point2f >& coord);
};

bool customLess(cv::DMatch a, cv::DMatch b);

#endif /* UCL_DRONE_TARGET_DETECTION_H */
