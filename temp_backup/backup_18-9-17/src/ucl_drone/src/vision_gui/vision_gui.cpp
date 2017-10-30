/*
 *  This file is part of ucl_drone 2016.
 *  For more information, refer
 *  to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 */

#include <ucl_drone/vision_gui/vision_gui.h>

VisionGui::VisionGui()
{
  // Instantiate the viewer widow
  cv::namedWindow(OPENCV_WINDOW1);

  // Subscribe to processed_image (contains image + keypoints + ...)
  processed_img_channel = nh_.resolveName("processed_image");
  processed_img_sub = nh_.subscribe(processed_img_channel, 1, &VisionGui::processedImageCb, this);

  // Get parameters in launch file
  ros::param::get("~draw_keypoints", this->draw_keypoints);
  ros::param::get("~draw_target", this->draw_target);

  new_processed_img_available = false;
  target_detected = false;
}

VisionGui::~VisionGui()
{
  cv::destroyWindow(OPENCV_WINDOW1);
}

/* This function is called every time a new processed_image is published */
void VisionGui::processedImageCb(const ucl_drone::ProcessedImageMsg::ConstPtr processed_image)
{
  ROS_DEBUG("VisionGui::processedImageCb start");
  this->lastProcessedImgReceived = processed_image;
  convertMsgToAttributes(processed_image);
  new_processed_img_available = true;
}

void VisionGui::guiDrawKeypoints()
{
  ROS_DEBUG("signal sent VisionGui::guiDrawKeypoints");

  cv::Mat output = cv_ptr->image;

  // Show all keypoints in image
  if (this->draw_keypoints)
  {
    for (unsigned i = 0; i < keypoints.size(); ++i)
    {
      cv::circle(output, keypoints[i], 3, cv::Scalar(0, 0, 255), -1, 8);
    }
  }

  // Show the target
  if (this->draw_target && target_detected)
  // if (target_detected)
  {
    cv::line(output, this->target_cornersAndCenter[0], this->target_cornersAndCenter[1],
             cv::Scalar(0, 255, 0), 4);
    cv::line(output, this->target_cornersAndCenter[1], this->target_cornersAndCenter[2],
             cv::Scalar(0, 255, 0), 4);
    cv::line(output, this->target_cornersAndCenter[2], this->target_cornersAndCenter[3],
             cv::Scalar(0, 255, 0), 4);
    cv::line(output, this->target_cornersAndCenter[3], this->target_cornersAndCenter[0],
             cv::Scalar(0, 255, 0), 4);
  }
  // Update GUI Window
  cv::imshow(OPENCV_WINDOW1, output);
  cv::waitKey(3);
}

void VisionGui::convertMsgToAttributes(ucl_drone::ProcessedImageMsg::ConstPtr msg)
{
  // convert points in the msg to opencv format
  // and store these as object attributes
  this->keypoints.resize(msg->keypoints.size());
  for (unsigned i = 0; i < msg->keypoints.size(); ++i)
  {
    this->keypoints[i].x = (double)msg->keypoints[i].point.x;
    this->keypoints[i].y = (double)msg->keypoints[i].point.y;
  }
  target_detected = msg->target_detected;
  if (target_detected)
  {
    this->target_cornersAndCenter.resize(msg->target_points.size());
    for (unsigned i = 0; i < msg->target_points.size(); ++i)
    {
      this->target_cornersAndCenter[i].x = msg->target_points[i].x;
      this->target_cornersAndCenter[i].y = msg->target_points[i].y;
    }
  }

  // convert ROS image to OpenCV image
  try
  {
    this->cv_ptr = cv_bridge::toCvCopy(msg->image, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("ucl_drone::VisionGui::cv_bridge exception: %s", e.what());
    return;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_gui");

  // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  // {
  //   ros::console::notifyLoggerLevelsChanged();
  // }

  VisionGui my_gui;
  ROS_DEBUG("end of computer vision viewer initialization");

  ros::Rate r(12);

  while (ros::ok())
  {
    ros::spinOnce();
    if (my_gui.new_processed_img_available)
    {
      TIC(gui);
      my_gui.guiDrawKeypoints();
      my_gui.new_processed_img_available = false;
      TOC(gui, "vision_gui")
    }
    r.sleep();
  }

  return 0;
}
