/*
 *  This file is part of ucl_drone 2016.
 *  For more information, refer
 *  to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 */

#include <ucl_drone/map/simple_map.h>

#if EXTRACTOR_TYPE == TYPE_ORB
Map::Map()
  : visualizer(new pcl::visualization::PCLVisualizer("3D visualizer"))
  , cloud(new pcl::PointCloud< pcl::PointXYZRGBSIFT >())
  , matcher(new cv::flann::LshIndexParams(20, 10, 2))
#else
Map::Map()
  : visualizer(new pcl::visualization::PCLVisualizer("3D visualizer"))
  , cloud(new pcl::PointCloud< pcl::PointXYZRGBSIFT >())
#endif
{
  cv::initModule_nonfree();
  this->processedImgReceived = false;
  this->tracking_lost = false;
  this->pending_reset = false;
  // Subsribers
  processed_image_channel_in = nh.resolveName("processed_image");
  processed_image_sub =
      nh.subscribe(processed_image_channel_in, 3, &Map::processedImageCb,
                   this);  // carefull!!! size of queue is 10 to permit to manipulate the viewer
  // (which blocks the rest of program execution) without losing too much
  // data. Be carefull to have sufficient debit so it makes sense .

  reset_channel = nh.resolveName("reset_pose");
  reset_sub = nh.subscribe(reset_channel, 1, &Map::resetCb, this);

  // Publishers
  target_channel_out = nh.resolveName("ucl_drone/target_detected");
  target_pub = nh.advertise< ucl_drone::TargetDetected >(target_channel_out, 1);

  // pcl::visualization::PointCloudColorHandlerRGBField< pcl::PointXYZRGBSIFT > rgb(cloud);
  pcl::visualization::PointCloudColorHandlerCustom< pcl::PointXYZRGBSIFT > single_color(cloud, 255,
                                                                                        0, 0);
  visualizer->setBackgroundColor(0, 0.1, 0.3);
  // visualizer->addPointCloud< pcl::PointXYZRGBSIFT >(cloud, rgb, "SIFT_cloud");
  visualizer->addPointCloud< pcl::PointXYZRGBSIFT >(cloud, single_color, "SIFT_cloud");
  visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,
                                               "SIFT_cloud");
  visualizer->addCoordinateSystem(1.0);  // red: x, green: y, blue: z
  visualizer->initCameraParameters();
  // pcl::visualization::Camera cam;
  // visualizer->getCameraParameters(cam);
  // cam.pos[3] = 10;
  // cam.view[0] = 1;
  // cam.view[1] = 0;
  // visualizer->setCameraParameters(cam);
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

  ROS_DEBUG("simple_map initialized");

  pose_PnP_channel = nh.resolveName("pose_visual");
  pose_PnP_pub = nh.advertise< ucl_drone::Pose3D >(pose_PnP_channel, 1);

  this->tvec = cv::Mat::zeros(3, 1, CV_64FC1);
  this->rvec = cv::Mat::zeros(3, 1, CV_64FC1);

  this->init_planes();
}

Map::~Map()
{
}

void Map::resetCb(const std_msgs::Empty msg)
{
  pending_reset = true;
  this->cloud = boost::shared_ptr< pcl::PointCloud< pcl::PointXYZRGBSIFT > >(
      new pcl::PointCloud< pcl::PointXYZRGBSIFT >);
  processed_image_sub.shutdown();
  this->visualizer->updatePointCloud< pcl::PointXYZRGBSIFT >(this->cloud, "SIFT_cloud");
  this->previous_frame = Frame();
  this->tvec = cv::Mat::zeros(3, 1, CV_64FC1);
  this->rvec = cv::Mat::zeros(3, 1, CV_64FC1);
  ros::Rate r(1 / 5.0);
  r.sleep();
  processed_image_sub = nh.subscribe(processed_image_channel_in, 3, &Map::processedImageCb, this);
}

void Map::processedImageCb(const ucl_drone::ProcessedImageMsg::ConstPtr processed_image_in)
{
  if (pending_reset)
    return;
  // ROS_DEBUG("Map::processedImageCb start");
  this->processedImgReceived = true;
  this->lastProcessedImgReceived = processed_image_in;
  Frame current_frame(processed_image_in);

#ifdef DEBUG_PROJECTION
  this->cloud = boost::shared_ptr< pcl::PointCloud< pcl::PointXYZRGBSIFT > >(
      new pcl::PointCloud< pcl::PointXYZRGBSIFT >);
  current_frame.msg.pose.x = 0;
  current_frame.msg.pose.y = 0;
#endif

  if (this->cloud->size() != 0)  /// IF THE MAP IS NOT EMPTY
  {
    std::vector< std::vector< int > > idx_matching_points;
    std::vector< int > idx_unknown_points;
    std::vector< cv::Point3f > map_matching_points;
    std::vector< cv::Point2f > frame_matching_points;
    std::vector< cv::Point2f > frame_unknown_points;

    this->matchWithFrame(current_frame, idx_matching_points, idx_unknown_points,
                         map_matching_points, frame_matching_points, frame_unknown_points);

    tracking_lost = false;

    ucl_drone::Pose3D mixed_pose;
    if (map_matching_points.size() > 7)
    {
      cv::Mat_< double > tcam, cam2world, world2drone, distCoeffs;
      std::vector< int > inliers;

      distCoeffs = (cv::Mat_< double >(1, 5) << 0, 0, 0, 0, 0);

      cv::solvePnPRansac(map_matching_points, frame_matching_points, this->camera_matrix_K,
                         distCoeffs, rvec, tvec, true, 3000, 2, 300, inliers,
                         CV_P3P);  // CV_EPNP);  // CV_ITERATIVE);  //

      ROS_DEBUG("# img:%4d, map:%4d, match:%4d, inliers::%4d", current_frame.imgPoints.size(),
                this->cloud->points.size(), map_matching_points.size(), inliers.size());

      // ROS_DEBUG("rvec: %+2.6f, %+2.6f, %+2.6f", rvec.at< double >(0, 0), rvec.at< double >(1, 0),
      //           rvec.at< double >(2, 0));

      if (inliers.size() < 7 || inliers.size() < map_matching_points.size() / 10.0)
      {
        ROS_DEBUG("TRACKING LOST ! (not enough inliers)");
        tracking_lost = true;
        return;
      }

      //   if (cv::norm(tvec) > 100.0)
      //   {
      //     ROS_DEBUG("TRACKING LOST ! (norm of translation)");
      //     return;
      //   }

      cv::Rodrigues(rvec, cam2world);

      if (fabs(determinant(cam2world)) - 1 > 1e-07)
      {
        ROS_DEBUG("TRACKING LOST ! (determinant of rotation matrix)");
        tracking_lost = true;
        return;
      }

      // equivalent to rollPitchYawToRotationMatrix(PI, 0, -PI / 2);
      cv::Mat_< double > drone2cam =
          (cv::Mat_< double >(3, 3) << 0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0,
           -1.0);  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! bizarre, sur papier different
      // cout << "drone2cambis = " << endl << " " << drone2cambis << endl << endl;

      tcam = -cam2world.t() * tvec;
      PnP_pose.x = tcam(0);
      PnP_pose.y = tcam(1);
      PnP_pose.z = tcam(2);

      cv::Mat_< double > world2cam = cam2world.t();
      cv::Mat_< double > cam2drone = drone2cam.t();
      world2drone =
          world2cam * cam2drone;  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  bizarre, sur
                                  // papier different

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

      PnP_pose.header.stamp = processed_image_in->pose.header.stamp;  // needed for rqt_plot
      pose_PnP_pub.publish(PnP_pose);
      current_frame.pose_visual_msg = PnP_pose;  // correction of the pose associated to the frame

      ROS_DEBUG("PnP_pose:  x=%+2.6f y=%+2.6f z=%+2.6f rotX=%+2.6f rotY=%+2.6f rotZ=%+2.6f",
                PnP_pose.x, PnP_pose.y, PnP_pose.z, PnP_pose.rotX, PnP_pose.rotY, PnP_pose.rotZ);
      ROS_DEBUG("trad_pose: x=%+2.6f y=%+2.6f z=%+2.6f rotX=%+2.6f rotY=%+2.6f rotZ=%+2.6f",
                current_frame.msg.pose.x, current_frame.msg.pose.y, current_frame.msg.pose.z,
                current_frame.msg.pose.rotX, current_frame.msg.pose.rotY,
                current_frame.msg.pose.rotZ);

      mixed_pose = current_frame.msg.pose;
      mixed_pose.x = PnP_pose.x;
      mixed_pose.y = PnP_pose.y;
      mixed_pose.z = PnP_pose.z;
      // mixed_pose.rotX = PnP_pose.rotX;
      // mixed_pose.rotY = PnP_pose.rotY;
      mixed_pose.rotZ = PnP_pose.rotZ;

      std::vector< cv::Point3f > points_out;  // points after 2D projection

      /* remove all outliers */
      //   ROS_DEBUG("remove outliers");

      //   int j = 0;
      //   for (int i = 0; i < frame_matching_points.size(); i++)
      //   {
      //     if (i == inliers[j] && j < inliers.size())
      //     {
      //       j++;
      //     }
      //     else
      //     {
      //       this->cloud->points.erase(this->cloud->points.begin() + idx_matching_points[i][0]);
      //     }
      //   }

      //   std::vector< int > outliers;
      //   int j = 0;
      //   for (int i = 0; i < frame_matching_points.size(); i++)
      //   {
      //     if (i == inliers[j] && j < inliers.size())
      //     {
      //       j++;
      //     }
      //     else
      //     {
      //       outliers.push_back(idx_matching_points[i][0]));
      //     }
      //   }
      //   pcl::ExtractIndices< PointType > eifilter(true);
      //   // Initializing with true will allow us to extract the removed indices
      //   eifilter.setInputCloud(this->cloud);
      //   eifilter.setIndices(outliers);
      //   eifilter.filter(*this->cloud);

      /* update inliers positions */
      ROS_DEBUG("update inliers");
      projection_2D(frame_matching_points, mixed_pose, points_out);
      for (int i = 0; i < inliers.size(); i++)
      {
        int count = this->cloud->points[idx_matching_points[inliers[i]][0]].view_count;
        if (count <= 5)
        {
          double alpha = 1.0 / (count + 1.0);
          double beta = count / (count + 1.0);

          this->cloud->points[idx_matching_points[inliers[i]][0]].x =
              beta * this->cloud->points[idx_matching_points[inliers[i]][0]].x +
              alpha * points_out[inliers[i]].x;
          this->cloud->points[idx_matching_points[inliers[i]][0]].y =
              beta * this->cloud->points[idx_matching_points[inliers[i]][0]].y +
              alpha * points_out[inliers[i]].y;
          this->cloud->points[idx_matching_points[inliers[i]][0]].z =
              beta * this->cloud->points[idx_matching_points[inliers[i]][0]].z +
              alpha * points_out[inliers[i]].z;
        }
        this->cloud->points[idx_matching_points[inliers[i]][0]].view_count += 1;
      }

      if (frame_unknown_points.size() <= 200 &&
          2 * frame_unknown_points.size() <= frame_matching_points.size())
      // Important to ensure coherent shapes
      {
        ROS_DEBUG("NOT ENOUGH TO ADD !");
        return;  // no projection with no points
      }
      else
      {
        projection_2D(frame_unknown_points, mixed_pose, points_out);

        // if (current_frame.imgPoints.size() > 2 * map_matching_points.size())
        // {
        pcl::PointCloud< pcl::PointXYZRGBSIFT >::Ptr pointcloud(
            new pcl::PointCloud< pcl::PointXYZRGBSIFT >);
        current_frame.convertToPcl(idx_unknown_points, points_out, pointcloud);
        //*(this->cloud) = *pointcloud;
        *(this->cloud) += *pointcloud;
      }
      this->visualizer->updatePointCloud< pcl::PointXYZRGBSIFT >(this->cloud, "SIFT_cloud");
      // }
    }
    else
    {
      ROS_DEBUG("TRACKING LOST ! (not enough matching points)");
      tracking_lost = true;
    }
  }
  else  /// IF THE MAP IS EMPTY
  {
    ROS_DEBUG("MAP INITIALIZATION");
    std::vector< cv::Point3f > points_out;  // points after 2D projection
    projection_2D(current_frame.imgPoints, current_frame.msg.pose, points_out,
                  true);  // to change for flying
    pcl::PointCloud< pcl::PointXYZRGBSIFT >::Ptr pointcloud(
        new pcl::PointCloud< pcl::PointXYZRGBSIFT >);
    current_frame.convertToPcl(points_out, pointcloud);
    *(this->cloud) = *pointcloud;
    PnP_pose.z = 0.76;
    PnP_pose.header.stamp = processed_image_in->pose.header.stamp;  // needed for rqt_plot
    pose_PnP_pub.publish(PnP_pose);
    this->visualizer->updatePointCloud< pcl::PointXYZRGBSIFT >(this->cloud, "SIFT_cloud");
  }

  previous_frame = current_frame;
}

//! TODO: for the moment, detection with 2D hypothesis. In the future, use PnP
void Map::targetDetectedPublisher()
{
  if (processedImgReceived && target_pub.getNumSubscribers() > 0)
  {
    ROS_DEBUG("signal sent ImageProcessor::targetDetectedPublisher");

    if (lastProcessedImgReceived->target_detected)
    {
      ROS_DEBUG("TARGET IS DETECTED");

      ucl_drone::TargetDetected msg;
      msg.pose = lastProcessedImgReceived->pose;
      // msg.navdata = lastProcessedImgReceived->navdata;
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
  float thres = 0.9;
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
  ROS_DEBUG("MAp: planes initilized");
}

//! return the entire map
void Map::getDescriptors(ucl_drone::Pose3D pose, cv::Mat& descriptors, std::vector< int >& idx,
                         bool only_visible)
{
  // only_visible = false;
  if (!only_visible)  // get all points descriptors
  {
    descriptors = cv::Mat_< float >(cloud->points.size(), DESCRIPTOR_SIZE);
    for (unsigned i = 0; i < cloud->points.size(); ++i)
    {
      cloud->points[i].x;
      cloud->points[i].y;
      cloud->points[i].z;
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
    double yaw = -pose.rotZ;
    double pitch = pose.rotY;
    double roll = pose.rotX;

    cv::Mat world2drone = rollPitchYawToRotationMatrix(roll, pitch, yaw);
    // cv::Mat drone2cam = rollPitchYawToRotationMatrix(PI, 0, -PI / 2);
    cv::Mat_< double > drone2cam =
        (cv::Mat_< double >(3, 3) << 0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0);
    cv::Mat world2cam = drone2cam * world2drone;

    ROS_DEBUG("TRANSFO READY");

    cv::Mat world_plane_top = world2cam.t() * cam_plane_top;
    cv::Mat world_plane_right = world2cam.t() * cam_plane_right;
    cv::Mat world_plane_bottom = world2cam.t() * cam_plane_bottom;
    cv::Mat world_plane_left = world2cam.t() * cam_plane_left;
    cv::Mat translation = (cv::Mat_< double >(3, 1) << pose.x, pose.y, pose.z);
    double d_top = -translation.dot(world_plane_top);
    double d_right = -translation.dot(world_plane_right);
    double d_bottom = -translation.dot(world_plane_bottom);
    double d_left = -translation.dot(world_plane_left);

    ROS_DEBUG("WORLD PLANES READY");

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
    ROS_DEBUG(" %d POINTS VISIBLE", idx.size());
    descriptors = cv::Mat_< float >(idx.size(), DESCRIPTOR_SIZE);
    for (unsigned i = 0; i < idx.size(); ++i)
    {
      for (unsigned j = 0; j < DESCRIPTOR_SIZE; ++j)
      {
        descriptors.at< float >(i, j) = cloud->points[idx[i]].descriptor[j];
      }
    }
  }
}

//! \param[in] map Map to compare
//! \param[out] idx_matching_points vector of pairs of indexes: (i,j) with i the index in the map, j
//! the index in the frame
//! \param[out] idx_unknown_points vector of indexes in the frame
//! \param[out] map_matching_points
//! \param[out] frame_matching_points
//! \param[out] frame_unknown_points
void Map::matchWithFrame(Frame& frame, std::vector< std::vector< int > >& idx_matching_points,
                         std::vector< int >& idx_unknown_points,
                         std::vector< cv::Point3f >& map_matching_points,
                         std::vector< cv::Point2f >& frame_matching_points,
                         std::vector< cv::Point2f >& frame_unknown_points)
{
  bool use_ratio_test = false;

  if (frame.descriptors.rows == 0)
  {
    return;
  }

  std::vector< int > idx;
  cv::Mat map_descriptors;
  std::vector< int > map_idx;
  bool only_visible = !tracking_lost;
  this->getDescriptors(frame.msg.pose, map_descriptors, map_idx, only_visible);
  if (map_descriptors.rows == 0)
  {
    idx_unknown_points.resize(frame.imgPoints.size());
    frame_unknown_points.resize(frame.imgPoints.size());
    for (unsigned k = 0; k < frame.imgPoints.size(); k++)
    {
      idx_unknown_points[k] = k;
      frame_unknown_points[k] = frame.imgPoints[k];
    }
    return;
  }
  else
  {
    // cv::FlannBasedMatcher matcher;
    if (use_ratio_test)
    {
      std::vector< std::vector< cv::DMatch > > knn_matches;
      matcher.knnMatch(frame.descriptors, map_descriptors, knn_matches, 2);

      // ratio_test + threshold test
      for (unsigned k = 0; k < knn_matches.size(); k++)
      {
        if (knn_matches[k][0].distance / knn_matches[k][1].distance < 0.9)
        {
          if (knn_matches[k][0].distance < DIST_THRESHOLD)
          {
            std::vector< int > v(2);
            v[0] = map_idx[knn_matches[k][0].trainIdx];
            v[1] = knn_matches[k][0].queryIdx;
            idx_matching_points.push_back(v);

            cv::Point3f map_point;
            pcl::PointXYZRGBSIFT pcl_point =
                this->cloud->points[map_idx[knn_matches[k][0].trainIdx]];
            map_point.x = pcl_point.x;
            map_point.y = pcl_point.y;
            map_point.z = pcl_point.z;
            map_matching_points.push_back(map_point);
            frame_matching_points.push_back(frame.imgPoints[knn_matches[k][0].queryIdx]);
          }
          else
          {
            idx_unknown_points.push_back(knn_matches[k][0].queryIdx);
            frame_unknown_points.push_back(frame.imgPoints[knn_matches[k][0].queryIdx]);
          }
        }
      }
    }
    else
    {
      std::vector< cv::DMatch > simple_matches;
      matcher.match(frame.descriptors, map_descriptors, simple_matches);

      // threshold test
      for (unsigned k = 0; k < simple_matches.size(); k++)
      {
        if (simple_matches[k].distance < DIST_THRESHOLD)
        {
          std::vector< int > v(2);
          v[0] = map_idx[simple_matches[k].trainIdx];
          v[1] = simple_matches[k].queryIdx;
          idx_matching_points.push_back(v);

          cv::Point3f map_point;
          pcl::PointXYZRGBSIFT pcl_point = this->cloud->points[map_idx[simple_matches[k].trainIdx]];
          map_point.x = pcl_point.x;
          map_point.y = pcl_point.y;
          map_point.z = pcl_point.z;
          map_matching_points.push_back(map_point);
          frame_matching_points.push_back(frame.imgPoints[simple_matches[k].queryIdx]);
        }
        else
        {
          idx_unknown_points.push_back(simple_matches[k].queryIdx);
          frame_unknown_points.push_back(frame.imgPoints[simple_matches[k].queryIdx]);
        }
      }
    }
  }
}

void cloud_debug(pcl::PointCloud< pcl::PointXYZRGBSIFT >::ConstPtr cloud)
{
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    ROS_DEBUG("points[%d] = (%f, %f, %f)", i, cloud->points[i].x, cloud->points[i].y,
              cloud->points[i].z);
  }
}

// void Map::visu_loop()
// {
//   ROS_DEBUG("Map:  visu_loop");
//   ros::Rate r(8);
//   while (ros::ok())
//   {
//     this->visualizer->updatePointCloud< pcl::PointXYZRGBSIFT >(this->cloud, "SIFT_cloud");
//     ROS_DEBUG("Map:  visualizer->spinOnce");
//     this->visualizer->spinOnce(100);
//     r.sleep();
//     boost::this_thread::interruption_point();
//   }
// }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_map");
  ROS_INFO_STREAM("simple map started!");

  // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  // {
  //   ros::console::notifyLoggerLevelsChanged();
  // }

  Map map;

  ros::Rate r(12);

  int visualizer_count = 0;

  // boost::thread visu_thread(&Map::visu_loop, &map);

  while (ros::ok())
  {
    // To reduce rate of visualizer by 5
    // if (visualizer_count >= 5)
    // {
    //   ROS_DEBUG("Map:  visualizer->spinOnce");
    map.visualizer->spinOnce(100);
    //   visualizer_count = 0;
    // }
    // visualizer_count += 1;

    map.targetDetectedPublisher();

    // pcl::visualization::Camera cam;
    // // Save the position of the camera
    // map.visualizer->getCameraParameterscam);
    // // Print recorded points on the screen:
    // cout << "Cam: " << endl
    //      << " - pos: (" << cam.pos[0] << ", " << cam.pos[1] << ", " << cam.pos[2] << ")" << endl
    //      << " - view: (" << cam.view[0] << ", " << cam.view[1] << ", " << cam.view[2] << ")" <<
    //      endl
    //      << " - focal: (" << cam.focal[0] << ", " << cam.focal[1] << ", " << cam.focal[2] << ")"
    //      << endl;

    if (map.pending_reset)
    {
      ros::Time t = ros::Time::now() + ros::Duration(1);
      while (ros::Time::now() < t)
      {
        ros::spinOnce();
      }
      map.pending_reset = false;
    }

    ros::spinOnce();  // if we dont want this we have to place callback and services in threads
    r.sleep();
  }
  // visu_thread.interrupt();
  return 0;
}
