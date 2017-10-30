/*!
 *  \file image_processor.h
 *  \brief Hheader file for the main class in the computer vision node.
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 *  Part of ucl_drone. Computer vision ROS node. Contains:
 *        - image analysis
 *        - keypoints detection & description
 */

#ifndef UCL_DRONE_PROCESSED_IMAGE_H
#define UCL_DRONE_PROCESSED_IMAGE_H

#include <ucl_drone/computer_vision/computer_vision.h>

/*!
 *  \class ProcessedImage
 *  \brief Provide tools to process keypoint extraction
 */
class ProcessedImage
{
private:
  static int last_number_of_keypoints;    //! the number of keypoints detected in the last image
  ucl_drone::ProcessedImageMsg::Ptr msg;  //! the message to be sent

public:
  cv_bridge::CvImagePtr cv_ptr;           //! conversion from ROS to OpenCV Image format
  std::vector< cv::KeyPoint > keypoints;  //! vector of keypoints detected
  cv::Mat descriptors;                    //! the keypoints descripors in opencv format
  sensor_msgs::Image image;               //! video image in the ROS format after rescaling
  ucl_drone::Pose3D pose;  //! estimated pose of the drone before the visual estimation

  ProcessedImage();  //! Constructor
  ProcessedImage(const sensor_msgs::Image image, const ucl_drone::Pose3D pose, ProcessedImage& prev,
                 bool use_OpticalFlowPyrLK);  //! Constructor
  ~ProcessedImage();                          //! Destructor

  //! build a message that can be sent to other nodes
  void convertToMsg(ucl_drone::ProcessedImageMsg::Ptr& msg, Target target);
};

#endif /*UCL_DRONE_PROCESSED_IMAGE_H*/
