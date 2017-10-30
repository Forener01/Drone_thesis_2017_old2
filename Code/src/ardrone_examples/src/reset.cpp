/*
Parker Conroy
Algorithmic Robotics Lab @ University of Utah

This program resets the ardrone, generally used after a crash.
It is intended as a simple example for those starting with the AR Drone platform.
*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <cstdlib>
#include <ardrone_autonomy/Navdata.h>

std_msgs::Empty emp_msg;
ardrone_autonomy::Navdata msg_in;
int drone_state;

void nav_callback(const ardrone_autonomy::Navdata& msg_in)
{
		//Take in state of ardrone	
	drone_state=msg_in.state;	
}

int main(int argc, char** argv)
{
	using namespace std;
	ROS_INFO("Reseting ARdrone");
	ros::init(argc, argv,"ARDrone_test");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);
	ros::Publisher pub_empty;
	ros::Subscriber nav_sub;

	nav_sub = node.subscribe("/ardrone/navdata", 1, nav_callback);
	pub_empty = node.advertise<std_msgs::Empty>("/ardrone/reset", 1); /* Message queue length is just 1 */
	
 	while (ros::ok()) {
		double time_start=(double)ros::Time::now().toSec();
				
		while ((double)ros::Time::now().toSec()< time_start+1.0 || drone_state == 0)
				{				
				pub_empty.publish(emp_msg); //launches the droe
				ROS_INFO("Sending Reset Signal");
				ros::spinOnce();
				loop_rate.sleep();
					if((double)ros::Time::now().toSec()> time_start+3.0){ 					
					
						ROS_ERROR("Time limit reached, unable to set state of ardrone");
						exit(0);
					}//end if
				
				}//while time or state
		system("rosservice  call /ardrone/setledanimation 1 5 2");
		ROS_INFO("ARdrone reset");
		exit(0);
		}//ros::ok
}//main
