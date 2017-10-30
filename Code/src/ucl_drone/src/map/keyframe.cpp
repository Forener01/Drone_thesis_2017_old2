/*
 *  This file is part of ucl_drone 2016.
 *  For more information, please refer
 *  to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 */

#include <ucl_drone/map/map_keyframe_based.h>

std::vector< KeyFrame* > KeyFrame::instances_list;

KeyFrame::KeyFrame() : cloud(new pcl::PointCloud< pcl::PointXYZRGBSIFT >())
{
}

KeyFrame::KeyFrame(ucl_drone::Pose3D& pose) : cloud(new pcl::PointCloud< pcl::PointXYZRGBSIFT >())
{
  // add the current keyframe to the list of all existing keyframes
  this->instances_list.push_back(this);
  // fill the identification number
  this->ID = instances_list.size() - 1;
  // fill the pose
  this->pose = pose;
}

KeyFrame::~KeyFrame()
{
}

void KeyFrame::resetList()
{
  for (int i = 0; i < KeyFrame::instances_list.size(); i++)
  {
    delete KeyFrame::instances_list[i];  // this line calls keyframe destructor
  }
  KeyFrame::instances_list.clear();
}

void KeyFrame::addPoints(pcl::PointCloud< pcl::PointXYZRGBSIFT >::Ptr& pointcloud,
                         std::vector< int >& new_points)
{
  *(this->cloud) += *pointcloud;
  this->points.insert(this->points.end(), new_points.begin(), new_points.end());
  this->convertDescriptors();
}

void KeyFrame::addPoints(pcl::PointCloud< pcl::PointXYZRGBSIFT >::Ptr& pointcloud)
{
  *(this->cloud) += *pointcloud;
  this->convertDescriptors();
}

KeyFrame* KeyFrame::getKeyFrame(const int i)
{
  if (i < 0 || i >= instances_list.size())
  {
    ROS_ERROR("You try to access a keyframe with invalid ID");
  }
  // ROS_DEBUG("getKeyFrame %d", i);
  return instances_list[i];
}

int KeyFrame::getID()
{
  return this->ID;
}

int KeyFrame::size()
{
  return this->cloud->size();
}

ucl_drone::Pose3D KeyFrame::getPose()
{
  return this->pose;
}

void KeyFrame::getDescriptors(cv::Mat& descriptors)
{
  if (this->descriptors.rows == 0)
  {
    this->convertDescriptors();
  }
  descriptors = this->descriptors;
}

void KeyFrame::convertDescriptors()
{
  this->descriptors = cv::Mat_< float >(this->cloud->points.size(), DESCRIPTOR_SIZE);
  for (unsigned i = 0; i < this->cloud->points.size(); ++i)
  {
    for (unsigned j = 0; j < DESCRIPTOR_SIZE; ++j)
    {
      this->descriptors.at< float >(i, j) = cloud->points[i].descriptor[j];
    }
  }
}

void KeyFrame::matchWithFrame(Frame& frame, std::vector< std::vector< int > >& idx_matching_points,
                              std::vector< cv::Point3f >& keyframe_matching_points,
                              std::vector< cv::Point2f >& frame_matching_points)
{
  bool use_ratio_test = false;  // if true, the second best match is also computed and the ratio
  // between the descriptors distance of the firt best match and the second best match must be less
  // than a defined treshold, otherwise the best match is discarded
  // but in pracice it doesn't work so well

  if (frame.descriptors.rows == 0)
  {
    // ROS_DEBUG("KeyFrame::matchWithFrame frame.descriptors.rows == 0");
    return;
  }

  if (this->descriptors.rows == 0)
  {
    // ROS_DEBUG("KeyFrame::matchWithFrame this->descriptors.rows == 0");
    return;
  }
  else
  {
    if (use_ratio_test)
    {
      std::vector< std::vector< cv::DMatch > > knn_matches;
      matcher.knnMatch(frame.descriptors, this->descriptors, knn_matches, 2);

      // ratio_test + threshold test
      for (unsigned k = 0; k < knn_matches.size(); k++)
      {
        if (knn_matches[k][0].distance / knn_matches[k][1].distance < 0.9)
        {
          if (knn_matches[k][0].distance < DIST_THRESHOLD)
          {
            std::vector< int > v(2);
            v[0] = knn_matches[k][0].trainIdx;
            v[1] = knn_matches[k][0].queryIdx;
            idx_matching_points.push_back(v);

            cv::Point3f keyframe_point;
            pcl::PointXYZRGBSIFT pcl_point = this->cloud->points[knn_matches[k][0].trainIdx];
            keyframe_point.x = pcl_point.x;
            keyframe_point.y = pcl_point.y;
            keyframe_point.z = pcl_point.z;
            keyframe_matching_points.push_back(keyframe_point);
            frame_matching_points.push_back(frame.imgPoints[knn_matches[k][0].queryIdx]);
          }
        }
      }
    }
    else
    {
      std::vector< cv::DMatch > simple_matches;
      matcher.match(frame.descriptors, this->descriptors, simple_matches);

      // threshold test
      for (unsigned k = 0; k < simple_matches.size(); k++)
      {
        if (simple_matches[k].distance < DIST_THRESHOLD)
        {
          std::vector< int > v(2);
          v[0] = simple_matches[k].trainIdx;
          v[1] = simple_matches[k].queryIdx;
          idx_matching_points.push_back(v);

          cv::Point3f keyframe_point;
          pcl::PointXYZRGBSIFT pcl_point = this->cloud->points[simple_matches[k].trainIdx];
          keyframe_point.x = pcl_point.x;
          keyframe_point.y = pcl_point.y;
          keyframe_point.z = pcl_point.z;
          keyframe_matching_points.push_back(keyframe_point);
          frame_matching_points.push_back(frame.imgPoints[simple_matches[k].queryIdx]);
        }
      }
    }
  }
}
