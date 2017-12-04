#ifndef IMAGE_FLOW_HPP
#define IMAGE_FLOW_HPP

#include <ardrone_autonomy/Navdata.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

class ImageFlow {
public:
  ImageFlow();
  void imageCb(const sensor_msgs::Image &img_msg);

private:
  ros::Subscriber image_sub;
  ros::Publisher processed_image_pub;

  sensor_msgs::Image current_img;
};

#endif /* IMAGE_FLOW_HPP */