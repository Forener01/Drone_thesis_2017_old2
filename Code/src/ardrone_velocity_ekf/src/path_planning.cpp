#include <ardrone_velocity/path_planning.hpp>
#include <ardrone_velocity/test_controller.hpp>

Path_Planning::Path_Planning() {
  ros::NodeHandle nh;

  ros::param::get("~test_type", test_type);
  // Subscribers
  odom_sub =
      nh.subscribe("ardrone/odometry", 1000, &Path_Planning::odomCb, this);
  poseref_sub =
      nh.subscribe("tud/pose_ref_topic", 1000, &Path_Planning::poserefCb, this);
  // poseout_sub = nh.subscribe("pose_out", 1, &Path_Planning::pose_to_velCb,
  // this);
  // Publishers
  // pose_pub = nh.advertise<geometry_msgs::Pose>("pose_out", 1);
  // poseref_pub = nh.advertise<geometry_msgs::Pose>("pose_ref", 1);
  veltoPID_pub = nh.advertise<geometry_msgs::Twist>("tud/cmd_PID_topic", 1000);
}

void Path_Planning::poserefCb(const geometry_msgs::Pose &pose_in) {
  current_pose_ref = pose_in;
  ROS_DEBUG("Poseref msg has been received: %f", current_pose_ref.position.x);
  // position_control();
}

void Path_Planning::odomCb(const nav_msgs::Odometry &odo) {
  odo_msg = odo;
  ROS_DEBUG("Odo msg x_pos has been received: %f",
            odo_msg.pose.pose.position.x);
  position_control();
}

void Path_Planning::position_control(void) {

  // float yaw   = atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0
  // * q.q0 + q.q1 * q.q1));

  distX = sqrt(pow(current_pose_ref.position.x, 2) -
               pow(odo_msg.pose.pose.position.x, 2));
  distY = sqrt(pow(current_pose_ref.position.y, 2) -
               pow(odo_msg.pose.pose.position.y, 2));
  distZ = sqrt(pow(current_pose_ref.position.z, 2) -
               pow(odo_msg.pose.pose.position.z, 2));

  tol = 0.3;

  if (distX > tol) {
    pose_out.position.x =
        current_pose_ref.position.x - odo_msg.pose.pose.position.x;
  } else {
    pose_out.position.x = 0.0;
    // ROS_DEBUG("Pos_X reached !");
  }

  if (distY > tol) {
    pose_out.position.y =
        current_pose_ref.position.y - odo_msg.pose.pose.position.y;
  } else {
    pose_out.position.y = 0.0;
    // ROS_DEBUG("Pos_Y reached !");
  }

  if (distZ > tol) {
    pose_out.position.z =
        current_pose_ref.position.z - odo_msg.pose.pose.position.z;
  } else {
    pose_out.position.z = 0.0;
  }

  if (distX < tol || distY < tol) {
    ROS_DEBUG("Position reached !");
  }

  // pose_pub.publish(pose_out);

  // ROS_DEBUG_THROTTLE(1, "Odo x: %f", odo_msg.pose.pose.position.x);
  // ROS_DEBUG_THROTTLE(1, "Odo y: %f", odo_msg.pose.pose.position.y);
  // ROS_DEBUG_THROTTLE(1, "Odo z: %f", odo_msg.pose.pose.position.z);

  // ROS_DEBUG_THROTTLE(1, "Pose out x: %f", pose_out.position.x);
  // ROS_DEBUG_THROTTLE(1, "Pose out y: %f", pose_out.position.y);
  // ROS_DEBUG_THROTTLE(1, "Pose out z: %f", pose_out.position.z);

  K = 0.3;
  // double xpos = current_pose_ref.position.x;
  velInPID.linear.x = K * pose_out.position.x;
  velInPID.linear.y = K * pose_out.position.y;
  velInPID.linear.z = K * pose_out.position.z;
  velInPID.angular.x = 0.0;
  velInPID.angular.y = 0.0;
  velInPID.angular.z = 0.0;

  veltoPID_pub.publish(velInPID);
  // ros::spinOnce();
  // ROS_DEBUG("Poseref msg has sent the following x_vel_ref: %f",
  //           velInPID.linear.x);
  // ROS_DEBUG("Poseref msg has sent the following y_vel_ref: %f",
  //           velInPID.linear.y);
  // ROS_DEBUG("Poseref msg has sent the following x_pos: %f", xpos);
}

// void Path_Planning::pose_to_velCb(const nav_msgs::Pose &pose_out){
//   K = 1/10;
//
//   velIn.linear.x = K*(pose_out.position.x);
//   velIn.linear.y = K*(pose_out.position.y);
//   velIn.linear.z = K*(pose_out.position.z);
//
//   velpath_pub.publish(velIn);
// }

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_planning"); // Name of the node
  Path_Planning mypath;
  ros::Rate loop_rate(2000);

  ROS_INFO_STREAM("path_planning node started!");

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
