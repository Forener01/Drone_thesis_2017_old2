/*
 *  This file is part of ucl_drone 2016.
 *  For more information, refer to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 */

#include <ucl_drone/map/map_keyframe_based.h>

Map::Map()
  : visualizer(new pcl::visualization::PCLVisualizer("3D visualizer"))
  , cloud(new pcl::PointCloud< pcl::PointXYZRGBSIFT >())
{
  cv::initModule_nonfree();  // initialize OpenCV SIFT and SURF

  // initialize default status boolean
  this->processedImgReceived = false;
  this->tracking_lost = false;
  this->pending_reset = false;

  // get launch parameters
  ros::param::get("~do_search", this->do_search);
  ros::param::get("~stop_if_lost", this->stop_if_lost);

  // define some threshold used later
  // better if defined in the launch file
  threshold_lost = 10;
  threshold_new_keyframe = 50;
  threshold_new_keyframe_percentage = 0.25;

  // initialize an Empty reference keyframe
  reference_keyframe = new KeyFrame();

  // Subsribers

  processed_image_channel_in = nh.resolveName("processed_image");
  processed_image_sub = nh.subscribe(processed_image_channel_in, 1, &Map::processedImageCb,
                                     this);  // carefull!!! size of queue is 1

  reset_pose_channel = nh.resolveName("reset_pose");
  reset_pose_sub = nh.subscribe(reset_pose_channel, 1, &Map::resetPoseCb, this);

  end_reset_pose_channel = nh.resolveName("end_reset_pose");
  end_reset_pose_sub = nh.subscribe(end_reset_pose_channel, 1, &Map::endResetPoseCb, this);

  // Publishers

  pose_PnP_channel = nh.resolveName("pose_visual");
  pose_PnP_pub = nh.advertise< ucl_drone::Pose3D >(pose_PnP_channel, 1);

  pose_correction_channel = nh.resolveName("pose_visual_correction");
  pose_correction_pub = nh.advertise< ucl_drone::Pose3D >(pose_correction_channel, 1);

  target_channel_out = nh.resolveName("ucl_drone/target_detected");
  target_pub = nh.advertise< ucl_drone::TargetDetected >(target_channel_out, 1);

  // initialize the map and the visualizer
  pcl::visualization::PointCloudColorHandlerCustom< pcl::PointXYZRGBSIFT > single_color(cloud, 0,
                                                                                        255, 0);
  visualizer->setBackgroundColor(0, 0.1, 0.3);
  visualizer->addPointCloud< pcl::PointXYZRGBSIFT >(cloud, single_color, "SIFT_cloud");
  visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,
                                               "SIFT_cloud");
  visualizer->addCoordinateSystem(1.0);  // red: x, green: y, blue: z

  // get camera parameters in launch file
  if (!Read::CamMatrixParams("cam_matrix"))
  {
    ROS_ERROR("cam_matrix not properly transmitted");
  }
  if (!Read::ImgSizeParams("img_size"))
  {
    ROS_ERROR("img_size not properly transmitted");
  }

  this->camera_matrix_K =
      (cv::Mat_< double >(3, 3) << Read::focal_length_x(), 0, Read::img_center_x(), 0,
       Read::focal_length_y(), Read::img_center_y(), 0, 0, 1);

  // initialize empty opencv vectors
  this->tvec = cv::Mat::zeros(3, 1, CV_64FC1);
  this->rvec = cv::Mat::zeros(3, 1, CV_64FC1);

  this->init_planes();

  ROS_DEBUG("simple_map initialized");
}

Map::~Map()
{
}

void Map::resetPoseCb(const std_msgs::Empty& msg)
{
  pending_reset = true;
  KeyFrame::resetList();  // remove all keyframes
  this->cloud = boost::shared_ptr< pcl::PointCloud< pcl::PointXYZRGBSIFT > >(
      new pcl::PointCloud< pcl::PointXYZRGBSIFT >);  // empty the cloud
  processed_image_sub.shutdown();                    // stop receiving new visual information
  // update visualizer
  this->visualizer->updatePointCloud< pcl::PointXYZRGBSIFT >(this->cloud, "SIFT_cloud");
  // reset
  this->previous_frame = Frame();
  this->tvec = cv::Mat::zeros(3, 1, CV_64FC1);
  this->rvec = cv::Mat::zeros(3, 1, CV_64FC1);
  processed_image_sub = nh.subscribe(processed_image_channel_in, 3, &Map::processedImageCb, this);
}

void Map::endResetPoseCb(const std_msgs::Empty& msg)
{
  pending_reset = false;
}

void Map::processedImageCb(const ucl_drone::ProcessedImageMsg::ConstPtr processed_image_in)
{
  TIC(callback);
  if (pending_reset)
    return;
  this->processedImgReceived = true;
  this->lastProcessedImgReceived = processed_image_in;
  Frame current_frame(processed_image_in);

#ifdef DEBUG_PROJECTION  // the cloud is re-initialized at each new message and alawys centered at
                         // the drone pose
  this->cloud = boost::shared_ptr< pcl::PointCloud< pcl::PointXYZRGBSIFT > >(
      new pcl::PointCloud< pcl::PointXYZRGBSIFT >);
  current_frame.msg.pose.x = 0;
  current_frame.msg.pose.y = 0;
#endif

  if (this->cloud->size() != 0)  // IF THE MAP IS NOT EMPTY
  {
    TIC(doPnP);
    std::vector< int > inliers;
    bool PnP_success = this->doPnP(current_frame, PnP_pose, inliers, reference_keyframe);
    TOC(doPnP, "doPnP");

    // filter out bad PnP estimation not yet treated: case where PnP thinks he has a good pose
    // etimation while he actually has found the bad clone configuration (symmetry in ground plane:
    // like a mirror effect), resulting in negative altitude and bad (uninterpretable?) other
    // meausures
    if (abs(PnP_pose.z - current_frame.msg.pose.z) > 0.8)  // treshold in cm
    {
      ROS_INFO("TRACKING LOST ! (PnP found bad symmetric clone? jump of: %f)",
               fabs(PnP_pose.z - current_frame.msg.pose.z));
      PnP_success = false;
    }

    if (PnP_success)  // if the visual pose is performed
    {
      TIC(publish);
      this->publishPoseVisual(current_frame.msg.pose, PnP_pose);
      TOC(publish, "publish");
      current_frame.pose_visual_msg = PnP_pose;  // correction of the pose associated to the frame

      // ROS_DEBUG("PnP_pose:  x=%+2.6f y=%+2.6f z=%+2.6f rotX=%+2.6f rotY=%+2.6f rotZ=%+2.6f",
      //           PnP_pose.x, PnP_pose.y, PnP_pose.z, PnP_pose.rotX, PnP_pose.rotY,
      //           PnP_pose.rotZ);
      // ROS_DEBUG("trad_pose: x=%+2.6f y=%+2.6f z=%+2.6f rotX=%+2.6f rotY=%+2.6f rotZ=%+2.6f",
      //             current_frame.msg.pose.x, current_frame.msg.pose.y, current_frame.msg.pose.z,
      //             current_frame.msg.pose.rotX, current_frame.msg.pose.rotY,
      //             current_frame.msg.pose.rotZ);
    }

    // if a new keyframe is needed (because the current one does not match enough, search among
    // previous ones, or create a new one)
    if (this->newKeyFrameNeeded(inliers.size()))
    {
      ROS_DEBUG("new keyframe needed");
      int keyframe_ID = -1;

      ucl_drone::Pose3D pose;  // pose used locally to attach to new keyframe
      if (!PnP_success)        // && !this->stop_if_lost)
      {
        pose = current_frame.msg.pose;  // sensor based pose of current frame
      }
      else
      {
        pose = PnP_pose;                    // PnP pose of the current frame
        pose.z = current_frame.msg.pose.z;  // in theory, absolute (because ultrasonic sensor)
        // and more precise than PnP, so use this one to map. In practice: Kalman filtered on board
        // (firmware), so, good meausure (not drifted) but not absolute!!! causes bad errors in map.
        // We want to avoid this, so if PnP available, use it!
        pose.rotX = current_frame.msg.pose.rotX;  // idem
        pose.rotY = current_frame.msg.pose.rotY;  // idem
      }

      bool search_success = false;
      if (do_search)
      {
        TIC(search);
        search_success = closestKeyFrame(pose, keyframe_ID, current_frame);
        TOC(search, "search");
      }

      if (search_success)
      {
        reference_keyframe = KeyFrame::getKeyFrame(keyframe_ID);
      }
      else
      {
        reference_keyframe = new KeyFrame(pose);

        // add new keyframe to the map
        std::vector< cv::Point3f > points_out;  // points after 2D projection
        TIC(projection);
        projection_2D(current_frame.imgPoints, pose, points_out);
        TOC(projection, "projection");
        pcl::PointCloud< pcl::PointXYZRGBSIFT >::Ptr pointcloud(
            new pcl::PointCloud< pcl::PointXYZRGBSIFT >);
        current_frame.convertToPcl(points_out, reference_keyframe->getID(), pointcloud);
        reference_keyframe->addPoints(pointcloud);

        *(this->cloud) += *pointcloud;

        this->visualizer->updatePointCloud< pcl::PointXYZRGBSIFT >(this->cloud, "SIFT_cloud");
      }
      TOC(callback, "callback");
      return;
    }
  }
  else  // IF THE MAP IS EMPTY
  {
    ROS_DEBUG("MAP INITIALIZATION");
    reference_keyframe = new KeyFrame(current_frame.msg.pose);

    std::vector< cv::Point3f > points_out;  // points after 2D projection
    projection_2D(current_frame.imgPoints, current_frame.msg.pose, points_out,
                  false);  // to change for flying
    pcl::PointCloud< pcl::PointXYZRGBSIFT >::Ptr pointcloud(
        new pcl::PointCloud< pcl::PointXYZRGBSIFT >);
    current_frame.convertToPcl(points_out, reference_keyframe->getID(), pointcloud);
    reference_keyframe->addPoints(pointcloud);
    *(this->cloud) = *pointcloud;
    // PnP_pose.z = 0.73;
    PnP_pose.header.stamp =
        processed_image_in->pose.header.stamp;  // needed for rqt_plot // ros::Time::now();  //
    this->publishPoseVisual(current_frame.msg.pose, PnP_pose);
    ROS_DEBUG("MAP INITIALIZATION : rotZ = %f", current_frame.msg.pose.rotZ);
    this->visualizer->updatePointCloud< pcl::PointXYZRGBSIFT >(this->cloud, "SIFT_cloud");
  }

  ROS_DEBUG("Present keyframe id = %d", reference_keyframe->getID());
  previous_frame = current_frame;

  TOC(callback, "callback");
}

void Map::publishPoseVisual(ucl_drone::Pose3D poseFrame, ucl_drone::Pose3D posePnP)
{
  ucl_drone::Pose3D pose_correction;
  pose_correction.header.stamp = ros::Time::now();
  pose_correction.x = poseFrame.x - posePnP.x;
  pose_correction.y = poseFrame.y - posePnP.y;
  pose_correction.z = poseFrame.z - posePnP.z;
  pose_correction.rotX = poseFrame.rotX - posePnP.rotX;
  pose_correction.rotY = poseFrame.rotY - posePnP.rotY;
  pose_correction.rotZ = poseFrame.rotZ - posePnP.rotZ;
  pose_PnP_pub.publish(PnP_pose);
  pose_correction_pub.publish(pose_correction);
}

bool Map::doPnP(Frame current_frame, ucl_drone::Pose3D& PnP_pose, std::vector< int >& inliers,
                KeyFrame* ref_keyframe)
{
  std::vector< std::vector< int > > idx_matching_points;
  std::vector< cv::Point3f > ref_keyframe_matching_points;
  std::vector< cv::Point2f > frame_matching_points;

  TIC(matchWithFrame);
  ref_keyframe->matchWithFrame(current_frame, idx_matching_points, ref_keyframe_matching_points,
                               frame_matching_points);
  if (ref_keyframe == reference_keyframe)
    TOC(matchWithFrame, "matchWithFrame");

  if (ref_keyframe_matching_points.size() < threshold_lost)
  {
    if (ref_keyframe == reference_keyframe)
      ROS_INFO("TRACKING LOST ! (Not enough matching points: %d)",
               ref_keyframe_matching_points.size());
    return false;
  }

  cv::Mat_< double > tcam, cam2world, world2drone, distCoeffs;

  distCoeffs = (cv::Mat_< double >(1, 5) << 0, 0, 0, 0, 0);

  TIC(solvePnPRansac);
  // solve with P3P
  cv::solvePnPRansac(ref_keyframe_matching_points, frame_matching_points, this->camera_matrix_K,
                     distCoeffs, rvec, tvec, true, 2500, 2, 2 * threshold_new_keyframe, inliers,
                     CV_P3P);  // alternatives: CV_EPNP and CV_ITERATIVE
  TOC(solvePnPRansac, "solvePnPRansac");

  // ROS_DEBUG("rvec: %+2.6f, %+2.6f, %+2.6f", rvec.at< double >(0, 0), rvec.at< double >(1, 0),
  //           rvec.at< double >(2, 0));

  if (inliers.size() < threshold_lost)
  {
    if (ref_keyframe == reference_keyframe)
      ROS_INFO("TRACKING LOST ! (Not enough inliers)");
    return false;
  }

  if (ref_keyframe == reference_keyframe)
  {
    // select only inliers
    std::vector< cv::Point3f > inliers_ref_keyframe_matching_points;
    std::vector< cv::Point2f > inliers_frame_matching_points;

    for (int j = 0; j < inliers.size(); j++)
    {
      int i = inliers[j];

      inliers_ref_keyframe_matching_points.push_back(ref_keyframe_matching_points[i]);
      inliers_frame_matching_points.push_back(frame_matching_points[i]);
    }

    // solve with PnP n>3
    cv::solvePnP(inliers_ref_keyframe_matching_points, inliers_frame_matching_points,
                 this->camera_matrix_K, distCoeffs, rvec, tvec, true, CV_EPNP);
  }

  cv::Rodrigues(rvec, cam2world);

  if (fabs(determinant(cam2world)) - 1 > 1e-07)
  {
    ROS_DEBUG("TRACKING LOST ! (Determinant of rotation matrix)");
    return false;
  }

  // equivalent to rollPitchYawToRotationMatrix(PI, 0, -PI / 2);
  cv::Mat_< double > drone2cam =
      (cv::Mat_< double >(3, 3) << 0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0);

  tcam = -cam2world.t() * tvec;
  PnP_pose.x = tcam(0);
  PnP_pose.y = tcam(1);
  PnP_pose.z = tcam(2);

  cv::Mat_< double > world2cam = cam2world.t();
  cv::Mat_< double > cam2drone = drone2cam.t();
  world2drone = world2cam * cam2drone;

  tf::Matrix3x3(world2drone(0, 0), world2drone(0, 1), world2drone(0, 2), world2drone(1, 0),
                world2drone(1, 1), world2drone(1, 2), world2drone(2, 0), world2drone(2, 1),
                world2drone(2, 2))
      .getRPY(PnP_pose.rotX, PnP_pose.rotY, PnP_pose.rotZ);

  PnP_pose.xvel = 0.0;
  PnP_pose.yvel = 0.0;
  PnP_pose.zvel = 0.0;
  PnP_pose.rotXvel = 0.0;
  PnP_pose.rotYvel = 0.0;
  PnP_pose.rotZvel = 0.0;

  PnP_pose.header.stamp = current_frame.msg.pose.header.stamp;  // needed for rqt_plot

  ROS_DEBUG("# img:%4d, keyframe%d:%4d, match:%4d, inliers:%4d", current_frame.imgPoints.size(),
            ref_keyframe->getID(), ref_keyframe->cloud->points.size(),
            ref_keyframe_matching_points.size(), inliers.size());

  return true;
}

//! better if in the future, PnP is used, for the moment, detection with 2D hypothesis.
void Map::targetDetectedPublisher()
{
  if (processedImgReceived && target_pub.getNumSubscribers() > 0)
  {
    if (lastProcessedImgReceived->target_detected)
    {
      ROS_DEBUG("TARGET IS DETECTED");

      ucl_drone::TargetDetected msg;
      msg.pose = lastProcessedImgReceived->pose;
      msg.img_point.x = lastProcessedImgReceived->target_points[4].x;
      msg.img_point.y = lastProcessedImgReceived->target_points[4].y;
      msg.img_point.z = 0;
      std::vector< cv::Point2f > target_center(1);
      target_center[0].x = lastProcessedImgReceived->target_points[4].x;
      target_center[0].y = lastProcessedImgReceived->target_points[4].y;
      std::vector< cv::Point3f > world_coord;
      projection_2D(target_center, msg.pose, world_coord);
      msg.world_point.x = world_coord[0].x;
      msg.world_point.y = world_coord[0].y;
      msg.world_point.z = world_coord[0].z;
      target_pub.publish(msg);
    }
  }
}

void Map::init_planes()
{
  float thres = 0.95;
  cv::Mat top_left = (cv::Mat_< double >(3, 1) << -Read::img_center_x() / Read::focal_length_x(),
                      -Read::img_center_y() / Read::focal_length_y(), thres);
  cv::Mat top_right = (cv::Mat_< double >(3, 1)
                           << (Read::img_width() - Read::img_center_x()) / Read::focal_length_x(),
                       -Read::img_center_y() / Read::focal_length_y(), thres);
  cv::Mat bottom_right =
      (cv::Mat_< double >(3, 1) << (Read::img_width() - Read::img_center_x()) /
                                       Read::focal_length_x(),
       (Read::img_height() - Read::img_center_y()) / Read::focal_length_y(), thres);
  cv::Mat bottom_left =
      (cv::Mat_< double >(3, 1) << -Read::img_center_x() / Read::focal_length_x(),
       (Read::img_height() - Read::img_center_y()) / Read::focal_length_y(), thres);
  cam_plane_top = top_left.cross(top_right);
  cam_plane_right = top_right.cross(bottom_right);
  cam_plane_bottom = bottom_right.cross(bottom_left);
  cam_plane_left = bottom_left.cross(top_left);
  ROS_DEBUG("MAP: planes initilized");
}

bool customLess(std::vector< int > a, std::vector< int > b)
{
  return a[1] > b[1];
}

bool Map::closestKeyFrame(const ucl_drone::Pose3D& pose, int& keyframe_ID, Frame current_frame)
{
  std::vector< std::vector< int > > keyframes_ID;
  TIC(getVisibleKeyFrames)
  this->getVisibleKeyFrames(pose, keyframes_ID);
  TOC(getVisibleKeyFrames, "getVisibleKeyFrames")

  int best_keyframe = -1;
  int best_keyframe_inlier_size = 0;
  std::vector< int > inliers;
  ucl_drone::Pose3D PnP_pose;
  KeyFrame* reference_keyframe_candidate;

  for (int i = 0; i < keyframes_ID.size(); i++)
  {
    // check if enough points for PnP
    if (keyframes_ID[i][1] > threshold_lost)
    {
      // ROS_DEBUG("keyframe #%d: enough points for PnP (i=%d)", keyframes_ID[i][0], i);
      // PnP number of inliers test
      reference_keyframe_candidate = KeyFrame::getKeyFrame(keyframes_ID[i][0]);
      bool PnP_success =
          this->doPnP(current_frame, PnP_pose, inliers, reference_keyframe_candidate);

      // filter out bad PnP estimation not yet treated: case where PnP thinks he has a good pose
      // estimation while he actually has found the bad clone configuration (symmetry in ground
      // plane:
      // like a mirror effect), resulting in negative altitude and bad (uninterpretable?) other
      // meausures
      if (abs(PnP_pose.z - current_frame.msg.pose.z) > 0.5)  // treshold in m
      {
        PnP_success = false;
      }

      if (PnP_success)
      {
        // PnP number of inliers test
        if (inliers.size() > best_keyframe_inlier_size &&
            !this->newKeyFrameNeeded(inliers.size(), reference_keyframe_candidate))
        {
          best_keyframe = keyframes_ID[i][0];
          best_keyframe_inlier_size = inliers.size();
        }
        if (inliers.size() > 1.1 * threshold_new_keyframe)
        {
          break;
        }
      }
    }
    else
    {
      break;  // because keyframes_ID is sorted in decreasing order with respect to
              // number of visible points (keyframes_ID[i][1])
    }
  }

  // ROS_DEBUG("best_keyframe = %d", best_keyframe);
  if (best_keyframe == -1)
  {
    return false;
  }
  keyframe_ID = best_keyframe;
  return true;
}

void Map::getVisibleKeyFrames(const ucl_drone::Pose3D& pose,
                              std::vector< std::vector< int > >& keyframes_ID)
{
  std::vector< int > idx;
  this->getVisiblePoints(pose, idx);
  int j;
  for (unsigned i = 0; i < idx.size(); ++i)
  {
    int id = cloud->points[idx[i]].keyframe_ID;
    for (j = 0; j < keyframes_ID.size(); j++)
    {
      if (id == keyframes_ID[j][0])
      {
        keyframes_ID[j][1]++;
        break;
      }
    }
    if (j == keyframes_ID.size())
    {
      std::vector< int > vec(2);
      vec[0] = id;
      vec[1] = 1;
      keyframes_ID.push_back(vec);
    }
  }

  std::sort(keyframes_ID.begin(), keyframes_ID.end(), customLess);

  // ROS_DEBUG("Visible Keyframes:");
  // for (j = 0; j < keyframes_ID.size(); j++)
  // {
  //   ROS_DEBUG("id %d containing %d points", keyframes_ID[j][0], keyframes_ID[j][1]);
  // }
}

void Map::getVisiblePoints(const ucl_drone::Pose3D& pose, std::vector< int >& idx)
{
  double yaw = -pose.rotZ;
  double pitch = -pose.rotY;
  double roll = -pose.rotX;

  cv::Mat world2drone = rollPitchYawToRotationMatrix(roll, pitch, yaw);
  // cv::Mat drone2cam = rollPitchYawToRotationMatrix(PI, 0, -PI / 2);
  cv::Mat_< double > drone2cam =
      (cv::Mat_< double >(3, 3) << 0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0);
  cv::Mat world2cam = drone2cam * world2drone;

  // ROS_DEBUG("TRANSFO READY");

  cv::Mat world_plane_top = world2cam.t() * cam_plane_top;
  cv::Mat world_plane_right = world2cam.t() * cam_plane_right;
  cv::Mat world_plane_bottom = world2cam.t() * cam_plane_bottom;
  cv::Mat world_plane_left = world2cam.t() * cam_plane_left;
  cv::Mat translation = (cv::Mat_< double >(3, 1) << pose.x, pose.y, pose.z);
  double d_top = -translation.dot(world_plane_top);
  double d_right = -translation.dot(world_plane_right);
  double d_bottom = -translation.dot(world_plane_bottom);
  double d_left = -translation.dot(world_plane_left);

  // ROS_DEBUG("WORLD PLANES READY");

  for (unsigned i = 0; i < cloud->points.size(); ++i)
  {
    cv::Mat point =
        (cv::Mat_< double >(3, 1) << cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);

    double t_top = point.dot(world_plane_top) + d_top;
    double t_right = point.dot(world_plane_right) + d_right;
    double t_bottom = point.dot(world_plane_bottom) + d_bottom;
    double t_left = point.dot(world_plane_left) + d_left;

    if (t_top >= 0 && t_right >= 0 && t_bottom >= 0 && t_left >= 0)
    {
      idx.push_back(i);
    }
  }
}

void Map::getDescriptors(const ucl_drone::Pose3D& pose, cv::Mat& descriptors,
                         std::vector< int >& idx, bool only_visible)
{
  // only_visible = false;
  if (!only_visible)  // get all points descriptors
  {
    descriptors = cv::Mat_< float >(cloud->points.size(), DESCRIPTOR_SIZE);
    for (unsigned i = 0; i < cloud->points.size(); ++i)
    {
      for (unsigned j = 0; j < DESCRIPTOR_SIZE; ++j)
      {
        descriptors.at< float >(i, j) = cloud->points[i].descriptor[j];
      }
      idx.push_back(i);
    }
  }
  if (only_visible)  // get only points visible from pose
                     // It doesn't take into account obstacles
  {
    ROS_DEBUG("GET_DESCRIPTORS, pose:pos(%f, %f, %f) rot(%f, %f, %f)", pose.x, pose.y, pose.z,
              pose.rotX, pose.rotY, pose.rotZ);

    this->getVisiblePoints(pose, idx);
    this->getDescriptors(idx, descriptors);

    ROS_DEBUG(" %d POINTS VISIBLE", idx.size());
  }
}

void Map::getDescriptors(const std::vector< int >& idx, cv::Mat& descriptors)
{
  descriptors = cv::Mat_< float >(idx.size(), DESCRIPTOR_SIZE);
  for (unsigned i = 0; i < idx.size(); ++i)
  {
    for (unsigned j = 0; j < DESCRIPTOR_SIZE; ++j)
    {
      descriptors.at< float >(i, j) = cloud->points[idx[i]].descriptor[j];
    }
  }
}

bool Map::newKeyFrameNeeded(int number_of_common_keypoints)
{
  return newKeyFrameNeeded(number_of_common_keypoints, this->reference_keyframe);
}

bool Map::newKeyFrameNeeded(int number_of_common_keypoints, KeyFrame* reference_keyframe_candidate)
{
  return (number_of_common_keypoints < threshold_new_keyframe ||
          number_of_common_keypoints <
              threshold_new_keyframe_percentage * reference_keyframe_candidate->size());
}

void cloud_debug(pcl::PointCloud< pcl::PointXYZRGBSIFT >::ConstPtr cloud)
{
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    ROS_DEBUG("points[%d] = (%f, %f, %f)", i, cloud->points[i].x, cloud->points[i].y,
              cloud->points[i].z);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_map");
  ROS_INFO_STREAM("simple map started!");

  // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  // {
  //   ros::console::notifyLoggerLevelsChanged();
  // }

  Map map;
  ros::Time t = ros::Time::now() + ros::Duration(13);
  while (ros::Time::now() < t)
  {
    map.visualizer->spinOnce(100);
  }

  ros::Rate r(3);

  int visualizer_count = 0;

  while (ros::ok())
  {
    TIC(cloud);
    map.visualizer->spinOnce(10);
    TOC(cloud, "cloud");

    TIC(target);
    map.targetDetectedPublisher();
    TOC(target, "target");

    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
