/*
Parker Conroy
Algorithmic Robotics Lab @ University of Utah


This code actuates the AR Drone back and forth.
It is intended as a simple example for those starting with the AR Drone platform.
*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

	geometry_msgs::Twist twist_msg;
	geometry_msgs::Twist twist_msg_neg;
	geometry_msgs::Twist twist_msg_hover;
	geometry_msgs::Twist twist_msg_up;
	std_msgs::Empty emp_msg;
	

int main(int argc, char** argv)
{

	ROS_INFO("ARdrone Test Back and Forth Starting");
	ros::init(argc, argv,"ARDrone_test");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);

	ros::Publisher pub_empty_land;
	ros::Publisher pub_twist;
	ros::Publisher pub_empty_takeoff;
	ros::Publisher pub_empty_reset;
	double start_time;

//hover message
			twist_msg_hover.linear.x=0.0; 
			twist_msg_hover.linear.y=0.0;
			twist_msg_hover.linear.z=0.0;
			twist_msg_hover.angular.x=0.0; 
			twist_msg_hover.angular.y=0.0;
			twist_msg_hover.angular.z=0.0;  
//up message
			twist_msg_up.linear.x=0.0; 
			twist_msg_up.linear.y=0.0;
			twist_msg_up.linear.z=0.5;
			twist_msg_up.angular.x=0.0; 
			twist_msg_up.angular.y=0.0;
			twist_msg_up.angular.z=0.0;
//command message
			float takeoff_time=5.0;
			float fly_time=7.0;
			float land_time=3.0;
			float kill_time =2.0;	
			
			
			twist_msg.linear.x=0.0; 
			twist_msg.linear.y=0.25;
			twist_msg.linear.z=0.0;
			twist_msg.angular.x=0.0; 
			twist_msg.angular.y=0.0;
			twist_msg.angular.z=0.0;

			twist_msg_neg.linear.x=-twist_msg.linear.x; 
			twist_msg_neg.linear.y=-twist_msg.linear.y;
			twist_msg_neg.linear.z=-twist_msg.linear.z;
			twist_msg_neg.angular.x=-twist_msg.angular.x; 
			twist_msg_neg.angular.y=-twist_msg.angular.y;
			twist_msg_neg.angular.z=-twist_msg.angular.z;


	
    pub_twist = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1); /* Message queue length is just 1 */
	pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); /* Message queue length is just 1 */
	pub_empty_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1); /* Message queue length is just 1 */
pub_empty_reset = node.advertise<std_msgs::Empty>("/ardrone/reset", 1); /* Message queue length is just 1 */

	
	start_time =(double)ros::Time::now().toSec();	
	ROS_INFO("Starting ARdrone_test loop");


while (ros::ok()) {
		while ((double)ros::Time::now().toSec()< start_time+takeoff_time){ //takeoff
		
			pub_empty_takeoff.publish(emp_msg); //launches the drone
				pub_twist.publish(twist_msg_hover); //drone is flat
			ROS_INFO("Taking off");
			ros::spinOnce();
			loop_rate.sleep();
			}//while takeoff

		while  ((double)ros::Time::now().toSec()> start_time+takeoff_time+fly_time){
		
			pub_twist.publish(twist_msg_hover); //drone is flat
			pub_empty_land.publish(emp_msg); //lands the drone
			ROS_INFO("Landing");
			
					
			if ((double)ros::Time::now().toSec()> takeoff_time+start_time+fly_time+land_time+kill_time){
		
				ROS_INFO("Closing Node");
				//pub_empty_reset.publish(emp_msg); //kills the drone		
				exit(0); 	}//kill node
			ros::spinOnce();
			loop_rate.sleep();			
}//while land

		while ( (double)ros::Time::now().toSec()> start_time+takeoff_time && 						(double)ros::Time::now().toSec()< start_time+takeoff_time+fly_time){	
		

			if((double)ros::Time::now().toSec()< start_time+takeoff_time+fly_time/2){
			pub_twist.publish(twist_msg);
			ROS_INFO("Flying +ve");

			}//fly according to desired twist
			
			if((double)ros::Time::now().toSec()> start_time+takeoff_time+fly_time/2){
			pub_twist.publish(twist_msg_neg);
			ROS_INFO("Flying -ve");

			}//fly according to desired twist
			
			ros::spinOnce();
		loop_rate.sleep();
			}

	ros::spinOnce();
	loop_rate.sleep();

}//ros::ok

}//main
