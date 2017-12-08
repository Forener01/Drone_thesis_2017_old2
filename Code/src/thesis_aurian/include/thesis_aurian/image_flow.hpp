#ifndef IMAGE_FLOW_HPP
#define IMAGE_FLOW_HPP

#include <ardrone_autonomy/Navdata.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <vector>

class ImageFlow {
public:
  ImageFlow();

  // void imageCb(const sensor_msgs::Image &img_msg);
  void calib_imageCb(const sensor_msgs::ImageConstPtr &msg);
  void image_processor(const cv::Mat my_img);

private:
  ros::Subscriber calib_image_sub;

  image_transport::Publisher processed_image_pub;

  sensor_msgs::Image current_img, processed_img;
  sensor_msgs::ImagePtr ros_img;

  cv::Mat converted_img, redfilt_upp_img, redfilt_low_img, redfilt_dark_img1,
      redfilt_dark_img2, redfilt_lu_img, redfilt_du_img, redfilt_ldu_img,
      redfilt_img, hsv_img, gray_img, redfilt_sub, redfilt_final;
};

#endif /* IMAGE_FLOW_HPP */