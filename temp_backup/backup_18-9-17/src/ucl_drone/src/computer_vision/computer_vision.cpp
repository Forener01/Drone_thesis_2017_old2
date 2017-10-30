/*
 *  This file is part of ucl_drone 2016.
 *  For more information, refer
 *  to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 */

#include <ucl_drone/computer_vision/computer_vision.h>
#include <ucl_drone/read_from_launch.h>

ImageProcessor::ImageProcessor()
  : it_(nh_), pose_publishing(false), video_publishing(false), pending_reset(false)
{
  cv::initModule_nonfree();  // initialize the opencv module which contains SIFT and SURF

  // Get all parameters from the launch file

  // use_OpticalFlowPyrLK: determine if keypoints tracking is used
  ros::param::get("~use_OpticalFlowPyrLK", this->use_OpticalFlowPyrLK);

  // autonomy_unavailable: set it to true if the ardrone_autonomy node is not launched
  // useful when a rosbag is played
  bool autonomy_unavailable = false;  // default value
  bool flag_autonomy_unavailable = ros::param::get("~autonomy_unavailable", autonomy_unavailable);

  // drone_prefix: relative path to the ardrone_autonomy node
  std::string drone_prefix = "/";  // default value
  bool flag_drone_prefix = ros::param::get("~drone_prefix", drone_prefix);

  // Prepare the subscription to input video
  std::string cam_type;  // bottom or front
  bool flag_cam_type = ros::param::get("~cam_type", cam_type);
  std::string video_channel;  // path to the undistorted video channel
  bool flag_video_channel = ros::param::get("~video_channel", video_channel);
  if (!flag_video_channel)
  {
    if (cam_type == "front")
    {
      video_channel = drone_prefix + "ardrone/front/image_raw";
    }
    else
    {
      cam_type = "bottom";
      video_channel = drone_prefix + "ardrone/bottom/image_raw";
    }
  }

  // Read camera calibration coefficients in the launch file
  if (!Read::CamMatrixParams("cam_matrix"))
  {
    ROS_ERROR("cam_matrix not properly transmitted");
  }
  if (!Read::ImgSizeParams("img_size"))
  {
    ROS_ERROR("img_size not properly transmitted");
  }

  video_channel_ = nh_.resolveName(drone_prefix + video_channel);

  if (!autonomy_unavailable)  // then set the drone to the selected camera
  {
    ros::ServiceClient client =
        nh_.serviceClient< ardrone_autonomy::CamSelect >(drone_prefix + "ardrone/setcamchannel");
    ardrone_autonomy::CamSelect srv;
    if (cam_type == "bottom")
    {
      srv.request.channel = 1;
    }
    else
    {
      srv.request.channel = 0;
    }
    int counter_call_success = 0;
    ros::Rate r(1 / 2.0);
    while (counter_call_success < 2)
    {
      ros::spinOnce();
      r.sleep();
      if (client.call(srv))
      {
        ROS_INFO("Camera toggled ?");
        if (srv.response.result)
          counter_call_success += 1;
      }
      else
      {
        ROS_INFO("Failed to call service setcamchannel, try it again in 1sec...");
      }
    }
  }

  // Subscribe to input video
  image_sub_ =
      it_.subscribe(video_channel_, 1, &ImageProcessor::imageCb, this);  // second param: queue size

  // Subscribe to pose
  pose_channel = nh_.resolveName("pose_estimation");
  pose_sub = nh_.subscribe(pose_channel, 1, &ImageProcessor::poseCb, this);

  // Subscribe to reset
  reset_pose_channel = nh_.resolveName("reset_pose");
  reset_pose_sub = nh_.subscribe(reset_pose_channel, 1, &ImageProcessor::resetPoseCb, this);

  // Subscribe to end reset
  end_reset_pose_channel = nh_.resolveName("end_reset_pose");
  end_reset_pose_sub =
      nh_.subscribe(end_reset_pose_channel, 1, &ImageProcessor::endResetPoseCb, this);

  // Initialize publisher of processed_image
  processed_image_channel_out = nh_.resolveName("processed_image");
  processed_image_pub =
      nh_.advertise< ucl_drone::ProcessedImageMsg >(processed_image_channel_out, 1);

  // Initialize an empty processed_image
  this->prev_cam_img = new ProcessedImage();

#ifdef DEBUG_TARGET
  cv::namedWindow(OPENCV_WINDOW);
#endif /* DEBUG_TARGET */

  // Load and initialize the target object
  target_loaded = target.init(TARGET_RELPATH);
}

ImageProcessor::~ImageProcessor()
{
#ifdef DEBUG_TARGET
  cv::destroyWindow(OPENCV_WINDOW);
#endif /* DEBUG_TARGET */
  delete this->prev_cam_img;
}

void ImageProcessor::resetPoseCb(const std_msgs::Empty& msg)
{
  pending_reset = true;
  this->prev_cam_img = new ProcessedImage();
}

void ImageProcessor::endResetPoseCb(const std_msgs::Empty& msg)
{
  pending_reset = false;
}

/* This function is called every time a new image is published */
void ImageProcessor::imageCb(const sensor_msgs::Image::ConstPtr& msg)
{
  if (pending_reset)
    return;
  ROS_DEBUG("ImageProcessor::imageCb");
  this->lastImageReceived = msg;
  this->video_publishing = true;
}

/* This function is called every time a new pose is published */
void ImageProcessor::poseCb(const ucl_drone::Pose3D::ConstPtr& posePtr)
{
  if (pending_reset)
    return;
  ROS_DEBUG("ImageProcessor::poseCb");
  lastPoseReceived = posePtr;
  pose_publishing = true;
}

/* This function is called at every loop of the current node */
void ImageProcessor::publishProcessedImg()
{
  if (pending_reset)
    return;
  ROS_DEBUG("ImageProcessor::publishProcessedImg");

  TIC(processed_image);
  // give all data to process the last image received (keypoints an target detection)
  ProcessedImage cam_img(*lastImageReceived, *lastPoseReceived, *prev_cam_img,
                         this->use_OpticalFlowPyrLK);
  // initialize the message to send
  ucl_drone::ProcessedImageMsg::Ptr msg(new ucl_drone::ProcessedImageMsg);
  // build the message to send
  cam_img.convertToMsg(msg, target);
  TOC(processed_image, "processedImage");

  TIC(publish);
  // publish the message
  processed_image_pub.publish(msg);
  TOC(publish, "publisher");

  // replace the previous image processed
  delete this->prev_cam_img;
  this->prev_cam_img = new ProcessedImage(cam_img);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "computer_vision");

  // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  // {
  //   ros::console::notifyLoggerLevelsChanged();
  // }

  // initialize the node object
  ImageProcessor ic;

  // rate of this node
  ros::Rate r(6);  // 12Hz =  average frequency at which we receive images

  // wait until pose and video are available
  while ((!ic.pose_publishing || !ic.video_publishing) && ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  while (ros::ok())  // while the node is not killed by Ctrl-C
  {
    ros::spinOnce();
    ic.publishProcessedImg();
    r.sleep();
  }

  return 0;
}
