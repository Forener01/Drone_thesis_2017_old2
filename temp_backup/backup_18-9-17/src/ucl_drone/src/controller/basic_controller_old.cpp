// This is a ROS version of the standard "hello ardrone" program.

// This header defines the standard ROS classes.
#include <ardrone_autonomy/Navdata.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <signal.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"

double Kp_alt=0.3;
double Ki_alt=0;
double Kd_alt=75;
double Kp_yaw=25;
double Kp_plan=0.6;
double Ki_plan=0;
//double Kd_plan=0.0015; //This Kd is good for y
double Kd_plan=0;
//double Ki_yaw=0.0001;
double Ki_yaw=40;
double Kd_yaw=0;
double integral_alt_error=0;
double integral_yaw_error=0;
double integral_xy_error=0;
double integral_f_error=0;
double integral_l_error=0;
double anti_windup_yaw=0.5;

double regu_old_time_z;  //How to initialize it?
double regu_old_time_yaw;
double regu_old_time_xy;
double old_delta_alt;
double old_delta_yaw;
double dist_old;
double p_term_f_old;
double p_term_l_old;
double alt_desired_old=0;
double yaw_desired_old=0;
double x_desired_old=0;
double y_desired_old=0;
double last_vel_z_command;
double last_vel_yaw_command;
double last_vel_x_command;
double last_vel_y_command;
double old_yaw_desired=0;

struct ControlCommand
{
  inline ControlCommand()
  {
    roll = pitch = yaw = gaz = 0;
  }
  inline ControlCommand(double pitch, double roll, double yaw, double gaz)
  {
    this->roll = roll;
    this->pitch = pitch;
    this->yaw = yaw;
    this->gaz = gaz;
  }
  double yaw, roll, pitch, gaz;
};

ros::Publisher land_pub;
ros::Publisher vel_pub;
// double desired_alt;
bool isControlling = false;
ardrone_autonomy::Navdata lastNavdataReceived;
nav_msgs::Odometry lastOdometryReceived;

void navdataCb(const ardrone_autonomy::Navdata::ConstPtr navdataPtr)
{
  lastNavdataReceived = *navdataPtr;
  // ROS_INFO("altd: %i", lastNavdataReceived.altd);
}

void odometryCb(const nav_msgs::Odometry::ConstPtr odometryPtr)
{
  lastOdometryReceived = *odometryPtr;
}

void basic_sigint_handler(int sig)
{
  geometry_msgs::Twist cmdT;
  cmdT.angular.z = 0;
  cmdT.linear.z = 0;
  cmdT.linear.x = 0;
  cmdT.linear.y = 0;

  // assume that while actively controlling, the above for will never be equal to zero, so i will never hover.
  cmdT.angular.x = cmdT.angular.y = 0;


    vel_pub.publish(cmdT);
    // lastSentControl = cmd;

  land_pub.publish(std_msgs::Empty());
  ros::shutdown();
}

// Altitude controller
void path_regu_altitude(double *zvel_cmd, double alt_mes, double alt_desired, double regu_new_time_z)
{
  double p_term;
  double d_term=0; //In case time_diff = 0
  double i_term;

  double time_difference=(regu_new_time_z-regu_old_time_z)/1000;

  //If navdata has the same timestamp, send the last command
  if(time_difference!=0){
  //printf("time_difference: %lf, regu_old_time: %lf, regu_new_time/ %lf \n", time_difference, regu_old_time, regu_new_time);
    if(alt_desired_old!=alt_desired){
        integral_alt_error=0;
    }
    alt_desired_old=alt_desired;
    regu_old_time_z=regu_new_time_z;
    double delta_alt=-alt_mes+alt_desired;
    p_term=delta_alt;  //Checker le signe!
    if(time_difference!=0){
        //printf("Entering if. delta_alt: %lf, old_delta_alt: %lf:", delta_alt, old_delta_alt);
        d_term=(delta_alt-old_delta_alt)/time_difference;
        old_delta_alt=delta_alt;
    }
    integral_alt_error+=delta_alt*time_difference;
    i_term=integral_alt_error;

    last_vel_z_command=(Kp_alt*p_term+i_term*Ki_alt+Kd_alt*d_term)/10;
    *zvel_cmd=last_vel_z_command;
    //printf("zvel_cmd: %lf \n", last_vel_z_command);
  /*printf("p_term: %lf\n",p_term);
  printf("d_term: %lf\n",d_term);
  printf("i_term: %lf\n",i_term);*/
    }
  else{
    *zvel_cmd=last_vel_z_command;
  }
}

  void path_regu_yaw(double *yawvel_cmd, double yaw_mes, double yaw_desired, double regu_new_time)
 {

    //ROS_INFO_STREAM("Regulating");
  double p_term;
  double d_term=0; //In case time_diff = 0
  double i_term;

  double time_difference=(regu_new_time-regu_old_time_yaw)/1000; //On obtient un nombre de millier de secondes... A changer!
  double new_vel_yaw_cmd;

  //If navdata has the same timestamp, send the last command
  if(time_difference!=0){
  //printf("time_difference: %lf, regu_old_time_yaw: %lf, regu_new_time/ %lf \n", time_difference, regu_old_time_yaw, regu_new_time);
  if(yaw_desired_old!=yaw_desired)
  {
    integral_yaw_error=0;
  }
  yaw_desired_old=yaw_desired;
  regu_old_time_yaw=regu_new_time;
  double delta_yaw=-yaw_mes+yaw_desired;
  p_term=delta_yaw;  //Checker le signe!
  if(time_difference!=0)
  {
    //printf("Entering if. delta_alt: %lf, old_delta_alt: %lf:", delta_yaw, old_delta_yaw);
    d_term=(delta_yaw-old_delta_yaw)/time_difference;
    old_delta_yaw=delta_yaw;
  }
  integral_yaw_error+=delta_yaw*time_difference;
  i_term=integral_yaw_error;
  new_vel_yaw_cmd=(Kp_yaw*p_term+i_term*Ki_yaw+Kd_yaw*d_term)/10;
  //printf("p_term: %lf, i_term: %lf \n",p_term, i_term);

  //Anti windup
  if(new_vel_yaw_cmd>anti_windup_yaw){
    new_vel_yaw_cmd=anti_windup_yaw;
  }
  else if(new_vel_yaw_cmd<-anti_windup_yaw){
    new_vel_yaw_cmd=-anti_windup_yaw;
  }
  last_vel_yaw_command=new_vel_yaw_cmd;
  *yawvel_cmd=last_vel_yaw_command;
  //printf("yaw_vel_cmd: %lf \n", last_vel_yaw_command);
  /*printf("p_term: %lf\n",p_term);
  printf("d_term: %lf\n",d_term);
  printf("i_term: %lf\n",i_term);*/
  }
  else{
  *yawvel_cmd=last_vel_yaw_command;
  }
 }



 void path_regu_xy(double *xvel_cmd, double *yvel_cmd, double x_mes, double y_mes, double x_desired,double y_desired, double yaw,double regu_new_time_xy)
 {
  double p_term_l;
  double d_term_l=0; //In case time_diff = 0
  double i_term_l;
  double p_term_f;
  double d_term_f=0; //In case time_diff = 0
  double i_term_f;

  double time_difference=(regu_new_time_xy-regu_old_time_xy)/1000;

  //If navdata has the same timestamp, send the last command
  if(time_difference!=0){
    if(x_desired_old!=x_desired||y_desired_old!=y_desired){
        integral_xy_error=0;
    }
    //printf("x: %lf   y: %lf   yaw: %lf\n",x_mes,y_mes,yaw);
    double dist=sqrt(pow((x_mes-x_desired),2)+pow((y_mes-y_desired),2));
    dist_old=dist;
    double delta_x=-x_mes+x_desired;
    double delta_y=y_desired-y_mes;
    regu_old_time_xy=regu_new_time_xy;
    //p_term=dist;  //Checker le signe!
    //Could technically be deleted
    double c_theta=cos(yaw*3.14159);
    double s_theta=sin(yaw*3.14159);
    p_term_f=delta_x*c_theta-delta_y*s_theta;
    p_term_l=delta_x*s_theta+delta_y*c_theta;

    if(time_difference!=0){
        //printf("Entering if. delta_alt: %lf, old_delta_alt: %lf:", delta_alt, old_delta_alt);
        d_term_f=(p_term_f-p_term_f_old)/time_difference;
        d_term_l=(p_term_l-p_term_l_old)/time_difference;
        p_term_f_old=p_term_f;
        p_term_l_old=p_term_l;
    }
    integral_f_error+=p_term_f*time_difference;
    integral_l_error+=p_term_l*time_difference;
    last_vel_x_command=(Kp_plan*p_term_f+i_term_f*Ki_plan+Kd_plan*d_term_f)/10;
    last_vel_y_command=(Kp_plan*p_term_l+i_term_l*Ki_plan+Kd_plan*d_term_l)/10;
    printf("p_term_f: %lf p_term_l: %lf\n",p_term_f,p_term_l);
    *xvel_cmd=last_vel_x_command;
    *yvel_cmd=last_vel_y_command;
    }


  else{
    *xvel_cmd=last_vel_x_command;
    *yvel_cmd=last_vel_y_command;
  }
 }

  void calculate_yaw_desired(double *yaw_desired,double x_desired, double y_desired, double x_mes, double y_mes){
    double delta_x=x_desired-x_mes;
    double delta_y=y_desired-y_mes;
    if(sqrt(delta_x*delta_x+delta_y*delta_y)<1.5){
    *yaw_desired=old_yaw_desired;
    }
    else{

    double theta;
    if(delta_x!=0){
    theta=atan(delta_y/delta_x)/3.14159;
    }
    else{
    theta=1;
    }
    if(delta_y<0 && delta_x<0){
    theta+=1;
        if(theta>1){
            theta=theta-2;
        }
    }
    else if(delta_y>0 && delta_x<0){
    theta=abs(theta);
    }
    old_yaw_desired=theta;
    *yaw_desired=theta;
    }

  }


// To send the command to the drone
void sendControlToDrone(ControlCommand cmd)
{
  geometry_msgs::Twist cmdT;
  cmdT.angular.z = cmd.yaw;
  cmdT.linear.z = cmd.gaz;
  cmdT.linear.x = cmd.pitch;
  cmdT.linear.y = cmd.roll;

  // assume that while actively controlling, the above for will never be equal to zero, so i will never hover.
  cmdT.angular.x = cmdT.angular.y = 0;

  if (isControlling)
  {
    vel_pub.publish(cmdT);
    // lastSentControl = cmd;
  }

  // lastControlSentMS = getMS(ros::Time::now());
}

int main(int argc, char **argv)
{
  // Initialise the ROS client library. NoSigintHandler option to be able to manage the ctrl+c key ourselves here under.
  ros::init(argc, argv, "basic_controller", ros::init_options::NoSigintHandler);

  // Establish this program as a ROS node.
  ros::NodeHandle nh;

  // Override the default ros singint handler (must be set after the nodeHandle is created)
  signal(SIGINT, basic_sigint_handler);

  // Send some output as a log message.
  ROS_INFO_STREAM("Hello, ROS!");

  std::string navdata_channel = nh.resolveName("ardrone/navdata");
  std::string odometry_channel = nh.resolveName("ardrone/odometry");
  ros::Subscriber navdata_sub = nh.subscribe(navdata_channel, 10, &navdataCb);
  ros::Subscriber odometry_sub = nh.subscribe(odometry_channel, 10, &odometryCb);

  std::string control_channel = nh.resolveName("cmd_vel");
  std::string takeoff_channel = nh.resolveName("ardrone/takeoff");
  std::string land_channel = nh.resolveName("ardrone/land");
  std::string toggleState_channel = nh.resolveName("ardrone/reset");

  ros::Publisher toggleState_pub = nh.advertise<std_msgs::Empty>(toggleState_channel, 1, true);
  ros::Duration(1).sleep();
  ros::Publisher takeoff_pub = nh.advertise<std_msgs::Empty>(takeoff_channel, 1, true);
  land_pub = nh.advertise<std_msgs::Empty>(land_channel, 1);
  vel_pub = nh.advertise<geometry_msgs::Twist>(control_channel, 1);

  // Mission
  // toggleState_pub.publish(std_msgs::Empty());
  takeoff_pub.publish(std_msgs::Empty());
  ros::Duration(10).sleep();  // Wait for 10s
  isControlling = true;

  ROS_INFO_STREAM("Hello, ROS!");

  float alt_desired;
  double yaw_desired;
  float x_desired;
  float y_desired;
  ControlCommand cmd;
  double xvel_cmd;
  double yvel_cmd;
  double zvel_cmd;
  double yawvel_cmd;


  ros::Time tic;
  ros::Time toc;
  ros::Rate r(200);  // 200Hz

  // while not emergency land {
  alt_desired = 800;
  x_desired=0;
  y_desired=0;
  tic = ros::Time::now();
  toc = tic;
  ros::spinOnce();
  r.sleep();
  regu_old_time_yaw=lastOdometryReceived.header.stamp.sec+lastOdometryReceived.header.stamp.nsec/pow(10,9);
  regu_old_time_xy=lastOdometryReceived.header.stamp.sec+lastOdometryReceived.header.stamp.nsec/pow(10,9);

  while (ros::ok() && (toc - tic < ros::Duration(100)))
  {
   // ROS_INFO_STREAM("Check 3");
    calculate_yaw_desired(&yaw_desired,x_desired,y_desired,lastOdometryReceived.pose.pose.position.x,lastOdometryReceived.pose.pose.position.y);
    path_regu_altitude(&zvel_cmd, lastNavdataReceived.altd, alt_desired,lastNavdataReceived.tm);
    path_regu_yaw(&yawvel_cmd,lastOdometryReceived.pose.pose.orientation.z,yaw_desired,lastOdometryReceived.header.stamp.sec+lastOdometryReceived.header.stamp.nsec/pow(10,9));
    path_regu_xy(&xvel_cmd,&yvel_cmd,lastOdometryReceived.pose.pose.position.x,lastOdometryReceived.pose.pose.position.y,x_desired,y_desired,lastOdometryReceived.pose.pose.orientation.z,lastOdometryReceived.header.stamp.sec+lastOdometryReceived.header.stamp.nsec/pow(10,9));

    cmd=ControlCommand(xvel_cmd, yvel_cmd, yawvel_cmd, 0);
    printf("xvel_cmd: %lf     yvel_cmd: %lf\n", xvel_cmd,yvel_cmd);

    sendControlToDrone(cmd);
    ros::spinOnce();
    r.sleep();
    toc = ros::Time::now();
  }

  //}

  land_pub.publish(std_msgs::Empty());

  // let ROS do some stuff
  //
  ros::spin();

  /*
  while(ros::ok())
  {
    ros::spinOnce();
  }
  */
}
