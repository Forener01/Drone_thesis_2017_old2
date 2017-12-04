#include <thesis_aurian/image_flow.hpp>

ImageFlow::ImageFlow() {
  ros::NodeHandle nh;
  // Subscribers
  image_sub =
      nh.subscribe("ardrone/front/image_raw", 1, &ImageFlow::imageCb, this);

  // Publishers
  processed_image_pub =
      nh.advertise<sensor_msgs::Image>("processed_image_topic", 1);
}

void ImageFlow::imageCb(const sensor_msgs::Image &img_msg) {
  ROS_DEBUG("ImageFlow::imageCb");
  current_img = img_msg;
  processed_image_pub.publish(current_img);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_flow");
  ImageFlow imageflownode;
  ros::Rate loop_rate(1);

  ROS_INFO_STREAM("image_flow node started!");

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}