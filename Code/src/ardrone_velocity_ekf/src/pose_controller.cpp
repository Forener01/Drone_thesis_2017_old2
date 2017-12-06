#include <ardrone_velocity_ekf/pose_controller.hpp>

Pose_Control::Pose_Control() {
  ros::NodeHandle nh;

  // Subscribers
  odom_sub =
      nh.subscribe("ardrone/odometry", 1000, &Pose_Control::odomCb, this);
  poseref_sub =
      nh.subscribe("cmd_pose_ref", 1000, &Pose_Control::poserefCb, this);
  // poseout_sub = nh.subscribe("pose_out", 1, &Pose_Control::pose_to_velCb,
  // this);
  // Publishers
  // pose_pub = nh.advertise<geometry_msgs::Pose>("pose_out", 1);
  // poseref_pub = nh.advertise<geometry_msgs::Pose>("pose_ref", 1);

  veltoPID_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_ref", 1000);
  ROS_INFO("pose_controller node publishing to cmd_vel_ref topic !");
}

void Pose_Control::poserefCb(const geometry_msgs::Pose &pose_in) {
  current_pose_ref = pose_in;
  ROS_DEBUG("Poseref msg has been received: %f", current_pose_ref.position.x);
  // position_control();
}

void Pose_Control::odomCb(const nav_msgs::Odometry &odo) {
  odo_msg = odo;
  ROS_DEBUG("Odo msg x_pos has been received: %f",
            odo_msg.pose.pose.position.x);
  position_control();
}

void Pose_Control::position_control(void) {

  // float yaw   = atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0
  // * q.q0 + q.q1 * q.q1));

  distX = sqrt(pow(current_pose_ref.position.x, 2) -
               pow(odo_msg.pose.pose.position.x, 2));
  distY = sqrt(pow(current_pose_ref.position.y, 2) -
               pow(odo_msg.pose.pose.position.y, 2));
  distZ = sqrt(pow(current_pose_ref.position.z, 2) -
               pow(odo_msg.pose.pose.position.z, 2));

  tol = 0.3;
  tolz = 0.25;

  // Method #1
  if (distX > tol) {
    // pose_out.position.x =
    //     current_pose_ref.position.x - odo_msg.pose.pose.position.x;
    velInPID.linear.x = 0.25;
  } else {
    velInPID.linear.x = 0.0;
    // pose_out.position.x = 0.0;
    // ROS_DEBUG("Pos_X reached !");
  }

  if (distY > tol) {
    // pose_out.position.y =
    //     current_pose_ref.position.y - odo_msg.pose.pose.position.y;
    velInPID.linear.y = 0.10;
  } else {
    velInPID.linear.y = 0.0;
    // pose_out.position.y = 0.0;
    // ROS_DEBUG("Pos_Y reached !");
  }
  //
  // if (distZ > tol) {
  //   pose_out.position.z =
  //       current_pose_ref.position.z - odo_msg.pose.pose.position.z;
  // } else {
  //   pose_out.position.z = 0.0;
  // }
  //
  if (distX < tol || distY < tol) {
    ROS_INFO_THROTTLE(1, "Position reached !");
  }

  // Method #2

  // error_dist = sqrt(pow(distX, 2) - pow(distY, 2));

  // if (distX > tolxy || distY > tolxy) {
  //   // pose_out.position.x =
  //   //     current_pose_ref.position.x - odo_msg.pose.pose.position.x;
  //   // pose_out.position.y =
  //   //     current_pose_ref.position.y - odo_msg.pose.pose.position.y;
  //   // pose_out.position.z = 0.0;
  //   velInPID.linear.x = 0.15;
  //   velInPID.linear.y = 0.15;
  //   velInPID.linear.z = 0.0;
  // } else {
  //   // pose_out.position.x = 0.0;
  //   // pose_out.position.y = 0.0;
  //   // pose_out.position.z = 0.0;
  //   velInPID.angular.x = 0.0;
  //   velInPID.angular.y = 0.0;
  //   velInPID.angular.z = 0.0;
  //   ROS_INFO_THROTTLE(0.5, "Position reached !");
  // }
  velInPID.angular.z = 0.0;
  // pose_pub.publish(pose_out);

  // ROS_DEBUG_THROTTLE(1, "Odo x: %f", odo_msg.pose.pose.position.x);
  // ROS_DEBUG_THROTTLE(1, "Odo y: %f", odo_msg.pose.pose.position.y);
  // ROS_DEBUG_THROTTLE(1, "Odo z: %f", odo_msg.pose.pose.position.z);

  // ROS_DEBUG_THROTTLE(1, "Pose out x: %f", pose_out.position.x);
  // ROS_DEBUG_THROTTLE(1, "Pose out y: %f", pose_out.position.y);
  // ROS_DEBUG_THROTTLE(1, "Pose out z: %f", pose_out.position.z);

  K = 0.25;
  // double xpos = current_pose_ref.position.x;
  // velInPID.linear.x = K * pose_out.position.x;
  // velInPID.linear.y = K * pose_out.position.y;
  // velInPID.linear.z = K * pose_out.position.z;
  // velInPID.linear.x = 0.15;
  // velInPID.linear.y = 0.15;
  // velInPID.linear.z = 0.15;
  // velInPID.angular.x = 0.0;
  // velInPID.angular.y = 0.0;
  // velInPID.angular.z = 0.0;

  veltoPID_pub.publish(velInPID);
  // ros::spinOnce();
  ROS_DEBUG_THROTTLE(1, "Poseref msg has sent the following x_vel_ref: %f",
                     velInPID.linear.x);
  ROS_DEBUG_THROTTLE(1, "Poseref msg has sent the following y_vel_ref: %f",
                     velInPID.linear.y);
  // ROS_DEBUG("Poseref msg has sent the following x_pos: %f", xpos);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pose_control"); // Name of the node
  Pose_Control mypath;
  ros::Rate loop_rate(2000);

  ROS_INFO_STREAM("pose_control node started !");

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
