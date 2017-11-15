#include <ardrone_velocity/raw_odom.hpp>

RawOdom::RawOdom() {
  raw_odom_sub = nh.subscribe("ardrone/odometry", 100, &RawOdom::raw_odomCb,
                              this, ros::TransportHints().tcpNoDelay());

  raw_error_pub =
      nh.advertise<geometry_msgs::Twist>("raw/vel_error_topic", 100);
  raw_percent_error_pub =
      nh.advertise<geometry_msgs::Twist>("raw/vel_percent_error_topic", 100);
}

void RawOdom::raw_odomCb(const nav_msgs::Odometry &odo_msg) {
  m_raw_odo_msg = odo_msg;

  double current_speed_x = 0.0;
  double current_speed_y = 0.6;

  error_x = current_speed_x - m_raw_odo_msg.twist.twist.linear.x;
  error_y = current_speed_y - m_raw_odo_msg.twist.twist.linear.y;
  error_z = m_raw_odo_msg.twist.twist.linear.z;

  m_raw_error_msg.linear.x = error_x;
  m_raw_error_msg.linear.y = error_y;
  m_raw_error_msg.linear.z = error_z;

  m_raw_percent_error_msg.linear.x = error_x / current_speed_x;
  m_raw_percent_error_msg.linear.y = error_y / current_speed_y;
  m_raw_percent_error_msg.linear.z = error_z;

  raw_error_pub.publish(m_raw_error_msg);
  raw_percent_error_pub.publish(m_raw_percent_error_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "raw_odom");
  RawOdom rawodomnode;
  ros::Rate loop_rate(2000);

  ROS_INFO_STREAM("raw_odom node started!");

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}