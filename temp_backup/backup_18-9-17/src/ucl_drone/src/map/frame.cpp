/*
 *  This file is part of ucl_drone 2016.
 *  For more information, refer
 *  to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 */

#include <ucl_drone/map/map_keyframe_based.h>

Frame::Frame()
{
}

Frame::Frame(ucl_drone::ProcessedImageMsg::ConstPtr msg)
{
  this->msg = *msg;

  // convert msg to opencv format
  this->imgPoints.resize(msg->keypoints.size());
  this->descriptors = cv::Mat_< float >(msg->keypoints.size(), DESCRIPTOR_SIZE);

  for (unsigned i = 0; i < msg->keypoints.size(); ++i)
  {
    this->imgPoints[i].x = (double)msg->keypoints[i].point.x;
    this->imgPoints[i].y = (double)msg->keypoints[i].point.y;

    // ROS_DEBUG("POINT[%d] (%f;%f) vs (%f;%f)", i, this->imgPoints[i].x, this->imgPoints[i].y,
    //           msg->keypoints[i].point.x, msg->keypoints[i].point.y);

    for (unsigned j = 0; j < DESCRIPTOR_SIZE; ++j)
    {
      this->descriptors.at< float >(i, j) = (float)msg->keypoints[i].descriptor[j];
    }
  }
}

Frame::~Frame()
{
}

// OLD CODE, NOW IN KEYFRAME
// //! \param[in] map Map to compare
// //! \param[out] idx_matching_points vector of pairs of indexes: (i,j) with i the index in the
// map, j
// //! the index in the frame
// //! \param[out] idx_unknown_points vector of indexes in the frame
// //! \param[out] map_matching_points
// //! \param[out] frame_matching_points
// //! \param[out] frame_unknown_points
// void Frame::matchWithMap(Map& map, std::vector< std::pair< int, int > >& idx_matching_points,
//                          std::vector< int >& idx_unknown_points,
//                          std::vector< cv::Point3f >& map_matching_points,
//                          std::vector< cv::Point2f >& frame_matching_points,
//                          std::vector< cv::Point2f >& frame_unknown_points)
// {
//   bool use_ratio_test = false;
//
//   std::vector< int > idx;
//   if (this->descriptors.rows == 0)
//   {
//     return;
//   }
//
//   cv::Mat map_descriptors;
//   std::vector< int > map_idx;
//   map.getDescriptors(this->msg.pose, map_descriptors, map_idx);
//   if (map_descriptors.rows == 0)
//   {
//     idx_unknown_points.resize(this->imgPoints.size());
//     frame_unknown_points.resize(this->imgPoints.size());
//     for (unsigned k = 0; k < this->imgPoints.size(); k++)
//     {
//       idx_unknown_points[k] = k;
//       frame_unknown_points[k] = this->imgPoints[k];
//     }
//     return;
//   }
//   else
//   {
//     cv::FlannBasedMatcher matcher;
//     if (use_ratio_test)
//     {
//       std::vector< std::vector< cv::DMatch > > knn_matches;
//       matcher.knnMatch(this->descriptors, map_descriptors, knn_matches, 2);
//
//       // ratio_test + threshold test
//       for (unsigned k = 0; k < knn_matches.size(); k++)
//       {
//         if (knn_matches[k][0].distance / knn_matches[k][1].distance < 0.9)
//         {
//           if (knn_matches[k][0].distance < DIST_THRESHOLD)
//           {
//             std::pair< int, int > p(map_idx[knn_matches[k][0].trainIdx],
//                                     knn_matches[k][0].queryIdx);
//             idx_matching_points.push_back(p);
//
//             cv::Point3f map_point;
//             pcl::PointXYZRGBSIFT pcl_point =
//             map.cloud->points[map_idx[knn_matches[k][0].trainIdx]];
//             map_point.x = pcl_point.x;
//             map_point.y = pcl_point.y;
//             map_point.z = pcl_point.z;
//             map_matching_points.push_back(map_point);
//             frame_matching_points.push_back(this->imgPoints[knn_matches[k][0].queryIdx]);
//           }
//           else
//           {
//             idx_unknown_points.push_back(knn_matches[k][0].queryIdx);
//             frame_unknown_points.push_back(this->imgPoints[knn_matches[k][0].queryIdx]);
//           }
//         }
//       }
//     }
//     else
//     {
//       std::vector< cv::DMatch > simple_matches;
//       matcher.match(this->descriptors, map_descriptors, simple_matches);
//
//       // threshold test
//       for (unsigned k = 0; k < simple_matches.size(); k++)
//       {
//         if (simple_matches[k].distance < DIST_THRESHOLD)
//         {
//           std::pair< int, int > p(map_idx[simple_matches[k].trainIdx],
//           simple_matches[k].queryIdx);
//           idx_matching_points.push_back(p);
//
//           cv::Point3f map_point;
//           pcl::PointXYZRGBSIFT pcl_point =
//           map.cloud->points[map_idx[simple_matches[k].trainIdx]];
//           map_point.x = pcl_point.x;
//           map_point.y = pcl_point.y;
//           map_point.z = pcl_point.z;
//           map_matching_points.push_back(map_point);
//           frame_matching_points.push_back(this->imgPoints[simple_matches[k].queryIdx]);
//         }
//         else
//         {
//           idx_unknown_points.push_back(simple_matches[k].queryIdx);
//           frame_unknown_points.push_back(this->imgPoints[simple_matches[k].queryIdx]);
//         }
//       }
//     }
//   }
// }

// \param[in] idx_points index of points in this->imgPoints to convert
// \param[out] pointcloud points converted to PointXYZRGBSIFT
void Frame::convertToPcl(std::vector< int >& idx_points, std::vector< cv::Point3f > points_out,
                         int keyframe_ID, pcl::PointCloud< pcl::PointXYZRGBSIFT >::Ptr& pointcloud)
{
  pointcloud->points.resize(idx_points.size());  // prepares output structure
  // for each points selected by idx_points
  for (unsigned i = 0; i < idx_points.size(); i++)
  {
    int j = idx_points[i];

    pcl::PointXYZRGBSIFT point;  // initialize a new PCL point

    // Fill coordinates
    point.x = points_out[i].x;
    point.y = points_out[i].y;
    point.z = points_out[i].z;

    // Other fields
    point.view_count = 1;
    point.keyframe_ID = keyframe_ID;

    int rgb_ = 255 << 16 | 0 << 8 | 0;  // red by default
    point.rgb = *reinterpret_cast< float* >(&rgb_);

    // Fill the corresponding description
    for (int k = 0; k < DESCRIPTOR_SIZE; k++)
    {
      point.descriptor[k] = this->descriptors.at< float >(j, k);
    }

    // add the PCL point to the output
    pointcloud->points[i] = point;
  }
}

void Frame::convertToPcl(std::vector< cv::Point3f > points_out, int keyframe_id,
                         pcl::PointCloud< pcl::PointXYZRGBSIFT >::Ptr& pointcloud)
{
  // call the other definition of Frame::convertToPcl
  std::vector< int > idx_points;
  idx_points.resize(this->imgPoints.size());
  for (unsigned k = 0; k < this->imgPoints.size(); k++)
  {
    idx_points[k] = k;
  }
  convertToPcl(idx_points, points_out, keyframe_id, pointcloud);
}
