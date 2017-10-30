/*!
 *  \file vision_gui.h
 *  \brief Node for displaying images super-imposed to 2D features and stuff
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 */

#ifndef UCL_DRONE_VISION_GUI_H
#define UCL_DRONE_VISION_GUI_H

// ROS debug messages
#include <ros/package.h>
#include <ros/ros.h>

// #define USE_PROFILING
#include <ucl_drone/profiling.h>

// messages
#include <sensor_msgs/image_encodings.h>
#include <ucl_drone/ProcessedImageMsg.h>

// vision
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

//! Name of the window where images are displayed
static const std::string OPENCV_WINDOW1 = "Keypoints window";

/*!
 *  \class VisionGui
 *  \brief Class having one attribute per OpenCV 2D Viewer
 */
class VisionGui
{
private:
  //! ros node
  ros::NodeHandle nh_;

  //! Launch parameters
  bool draw_keypoints;
  bool draw_target;

  //! Subscribers
  ros::Subscriber processed_img_sub;
  std::string processed_img_channel;

  //! Callbacks
  void processedImageCb(const ucl_drone::ProcessedImageMsg::ConstPtr processed_image);

  //! Translation to OpenCV format
  void convertMsgToAttributes(ucl_drone::ProcessedImageMsg::ConstPtr msg);

  //! Measure
  ucl_drone::ProcessedImageMsg::ConstPtr lastProcessedImgReceived;

  //! Measure after translation to OpenCV format
  cv_bridge::CvImagePtr cv_ptr;

  std::vector< cv::Point2f > keypoints;
  std::vector< cv::Point2f > target_cornersAndCenter;

  bool target_detected;

public:
  //! Constructor
  VisionGui();

  //! Destructor
  ~VisionGui();

  void guiDrawKeypoints();

  // bool processed_image_publishing;
  bool new_processed_img_available;
};

#endif /* UCL_DRONE_VISION_GUI_H */
