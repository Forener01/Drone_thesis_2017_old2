#include <ardrone_velocity/path_planning.hpp>

Path_Planning::Path_Planning() {
  ros::NodeHandle nh;

  // Subscribers
  odom_sub =
      nh.subscribe("ardrone/odometry", 1000, &Path_Planning::odomCb, this);
  poseref_sub = nh.subscribe("pose_ref", 1000, &Path_Planning::poserefCb, this);
  // poseout_sub = nh.subscribe("pose_out", 1, &Path_Planning::pose_to_velCb,
  // this);
  // Publishers
  // pose_pub = nh.advertise<geometry_msgs::Pose>("pose_out", 1);
  // poseref_pub = nh.advertise<geometry_msgs::Pose>("pose_ref", 1);
  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_PID", 1000);
}

void Path_Planning::poserefCb(const geometry_msgs::Pose &pose) {
  pose_ref = pose;
}

void Path_Planning::odomCb(const nav_msgs::Odometry &odo) {
  odo_msg = odo;

  position_control();
}

void Path_Planning::position_control(void) {
  double distX, distY, distZ;

  // float yaw   = atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0
  // * q.q0 + q.q1 * q.q1));

  distX =
      sqrt(pow(pose_ref.position.x, 2) - pow(odo_msg.pose.pose.position.x, 2));
  distY =
      sqrt(pow(pose_ref.position.y, 2) - pow(odo_msg.pose.pose.position.y, 2));
  distZ =
      sqrt(pow(pose_ref.position.z, 2) - pow(odo_msg.pose.pose.position.z, 2));

  if (distX > 0.1) {
    pose_out.position.x = pose_ref.position.x - odo_msg.pose.pose.position.x;
  }

  if (distY > 0.1) {
    pose_out.position.y = pose_ref.position.y - odo_msg.pose.pose.position.y;
  }

  if (distZ > 0.1) {
    pose_out.position.z = pose_ref.position.z - odo_msg.pose.pose.position.z;
  }

  // pose_pub.publish(pose_out);

  ROS_DEBUG_THROTTLE(1, "Odo x: %f", odo_msg.pose.pose.position.x);
  ROS_DEBUG_THROTTLE(1, "Odo y: %f", odo_msg.pose.pose.position.y);
  ROS_DEBUG_THROTTLE(1, "Odo z: %f", odo_msg.pose.pose.position.z);

  ROS_DEBUG_THROTTLE(1, "Pose out x: %f", pose_out.position.x);
  ROS_DEBUG_THROTTLE(1, "Pose out y: %f", pose_out.position.y);
  ROS_DEBUG_THROTTLE(1, "Pose out z: %f", pose_out.position.z);

  K = 1 / 10;

  velIn.linear.x = K * (pose_out.position.x);
  velIn.linear.y = K * (pose_out.position.y);
  velIn.linear.z = K * (pose_out.position.z);

  vel_pub.publish(velIn);
}

// void Path_Planning::pose_to_velCb(const nav_msgs::Pose &pose_out){
//   K = 1/10;
//
//   velIn.linear.x = K*(pose_out.position.x);
//   velIn.linear.y = K*(pose_out.position.y);
//   velIn.linear.z = K*(pose_out.position.z);
//
//   vel_pub.publish(velIn);
// }

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "path_planning"); // Name of the node
  Path_Planning mypath;

  int32_t looprate = 500; // hz
  ros::Rate loop_rate(looprate);

  // ros::spin();
  while (mypath.nh.ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
