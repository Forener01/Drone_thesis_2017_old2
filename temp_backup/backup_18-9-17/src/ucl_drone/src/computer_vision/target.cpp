/*
 *  This file is part of ucl_drone 2016.
 *  For more information, refer
 *  to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 *  tuto:
 * http://docs.opencv.org/2.4/doc/tutorials/features2d/feature_homography/feature_homography.html
 */

#include <ucl_drone/computer_vision/computer_vision.h>

//! Absolute path to the package
static const std::string PKG_DIR = ros::package::getPath("ucl_drone");

#if EXTRACTOR_TYPE == TYPE_ORB
Target::Target() : matcher(new cv::flann::LshIndexParams(20, 10, 2))
#else
Target::Target()
#endif /* EXTRACTOR_TYPE == TYPE_ORB */
{
}

Target::~Target()
{
}

bool Target::init(const std::string relative_path)
{
  // Read and import the picture file
  std::string str = PKG_DIR + relative_path;
  ROS_INFO("%s", str.c_str());
  cv::Mat target_img = cv::imread(str.c_str(), CV_LOAD_IMAGE_COLOR);

  if (!target_img.data)  // Check for invalid input
  {
    ROS_INFO("Could not open or find the image");
    return false;
  };

  image = target_img;

  // Determine position of corners and center on the image picture
  // This is used to draw green rectangle in the viewer and to estimate target world coordinates
  centerAndCorners.resize(5);
  centerAndCorners[0] = cv::Point(0, 0);
  centerAndCorners[1] = cv::Point(image.cols, 0);
  centerAndCorners[2] = cv::Point(image.cols, image.rows);
  centerAndCorners[3] = cv::Point(0, image.rows);
  centerAndCorners[4] = cv::Point(image.cols / 2.0, image.rows / 2.0);

  ROS_DEBUG("TARGET IMAGE DIMENSIONS: %d,%d", image.cols, image.rows);

  detector.detect(image, keypoints);
  extractor.compute(image, keypoints, descriptors);

  /*
  // We didn't know the exact effect of these lines, maybe it will improve matching performances
  // Read opencv API for more informations
  std::vector<cv::Mat> descriptor_vector;
  descriptor_vector.push_back(descriptors);
  matcher.add(descriptor_vector);
  matcher.train();
  */

  ROS_DEBUG("DESCRIPTORS DIMENSIONS: %d,%d", descriptors.cols, descriptors.rows);
  ROS_DEBUG("end Target::init");
  ROS_DEBUG("Target: %d keypoints", keypoints.size());
  return true;
}

// This function is called when target detection has to be performed on a picture
bool Target::detect(cv::Mat cam_descriptors, std::vector< cv::KeyPoint >& cam_keypoints,
                    std::vector< cv::DMatch >& good_matches, std::vector< int >& idxs_to_remove,
                    std::vector< cv::Point2f >& target_coord
#ifdef DEBUG_PROJECTION
                    ,
                    ucl_drone::Pose3D pose
#endif
#ifdef DEBUG_TARGET
                    ,
                    cv::Mat& image_cam
#endif
                    )
{
  ROS_DEBUG("begin Target::detect");
  // step 3: matching descriptors

  if (cam_descriptors.rows == 0)
  {
    return false;
  }

  bool target_detected = false;

  std::vector< cv::DMatch > matches;
  matcher.match(descriptors, cam_descriptors, matches);

  // step 5: Draw only "good" matches
  for (int i = 0; i < descriptors.rows; i++)
  {
    // ROS_DEBUG("Target::detect DISTANCE %d: %f", i, matches[i].distance);
    if (matches[i].distance < DIST_THRESHOLD * 0.75)  // 1.2)
    {
      good_matches.push_back(matches[i]);
    }
  }
  // std::vector< std::vector< cv::DMatch > > matches2;
  // matcher.radiusMatch(descriptors, cam_descriptors, matches2, DIST_THRESHOLD * 1.1);
  // for (int i = 0; i < matches2.size(); i++)
  // {
  //   for (int k = 0; k < matches2[i].size(); k++)
  //   {
  //     idxs_to_remove.push_back(matches2[i][k].trainIdx);
  //   }
  // }

  ROS_DEBUG("Target::detect matches:%d", good_matches.size());
  if (good_matches.size() > this->descriptors.rows / 6.0 && good_matches.size() > 8)
  {
    target_detected = true;
    this->position(cam_keypoints, good_matches, target_coord);

    std::vector< cv::Point2f >::const_iterator first = target_coord.begin();
    std::vector< cv::Point2f >::const_iterator last = target_coord.begin() + 4;
    std::vector< cv::Point2f > contour(first, last);

    for (unsigned i = 0; i < cam_keypoints.size(); i++)
    {
      double result = cv::pointPolygonTest(contour, cam_keypoints[i].pt, false);
      if (result >= 0)
      {
        idxs_to_remove.push_back(i);
      }
    }
  }

  std::sort(idxs_to_remove.begin(), idxs_to_remove.end());  //, customLess);

  // remove duplicates
  int k = 1;
  for (int i = 1; i < idxs_to_remove.size(); i++)
  {
    if (idxs_to_remove[i] != idxs_to_remove[i - 1])
    {
      idxs_to_remove[k] = idxs_to_remove[i];
      k++;
    }
  }
  idxs_to_remove.resize(k);

#ifdef DEBUG_TARGET
  cv::Mat img_matches;
  cv::drawMatches(image, keypoints, image_cam, cam_keypoints, good_matches, img_matches,
                  cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector< char >(),
                  cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  cv::imshow(OPENCV_WINDOW, img_matches);
  cv::waitKey(3);
#endif /*DEBUG_TARGET*/

  if (!target_detected)
    return false;

#ifdef DEBUG_PROJECTION

  ROS_INFO("DEBUG_PROJECTION");
  std::vector< cv::Point2f > obj;
  std::vector< cv::Point2f > scene;

  for (int i = 0; i < good_matches.size(); i++)
  {
    // Get the keypoints from the good matches
    obj.push_back(keypoints[good_matches[i].queryIdx].pt);
    scene.push_back(cam_keypoints[good_matches[i].trainIdx].pt);
  }

  std::vector< uchar > mask;
  cv::Mat H =
      cv::findHomography(obj, scene, CV_RANSAC, 2, mask);  // arg#4: ransacReprojThreshold [pixel]

  std::string rel_error = "\nrelative_error = [";
  std::string abs_error = "\nabsolute_error = [";
  for (int i = 0; i < good_matches.size(); i++)
  {
    for (int j = i + 1; j < good_matches.size(); j++)
    {
      if (mask[i] == 1 && mask[j] == 1)
      {
        cv::Point2f target_point1 = this->keypoints[good_matches[i].queryIdx].pt;
        cv::Point2f target_point2 = this->keypoints[good_matches[j].queryIdx].pt;
        double dist1 = cv::norm(target_point1 - target_point2) * 0.274 / 240.0;  // 0.205 / 135.0;

        if (dist1 > 0.1)
        {
          std::vector< cv::Point2f > camera_points(2);
          camera_points[0] = cam_keypoints[good_matches[i].trainIdx].pt;
          camera_points[1] = cam_keypoints[good_matches[j].trainIdx].pt;
          std::vector< cv::Point3f > map_points(2);
          projection_2D(camera_points, pose, map_points, true);

          double dist2 = cv::norm(map_points[0] - map_points[1]);

          //   if (dist2 > 0.001)  // if two super-imposed identical descriptors in map
          //   {
          double relative_error = (dist2 - dist1) / dist1;

          double absolute_error = (dist2 - dist1);

          rel_error += to_string(relative_error) + ", ";
          abs_error += to_string(absolute_error) + ", ";
          //   }
        }
      }
    }
  }
  rel_error += "];\n\n";
  abs_error += "];\n\n";

  printf("%s", rel_error.c_str());
  printf("%s", abs_error.c_str());

#endif /*DEBUG_PROJECTION*/

  ROS_DEBUG("end Target::detect");
  return true;
}

void Target::draw(cv::Mat cam_img, std::vector< cv::KeyPoint > cam_keypoints,
                  std::vector< cv::DMatch > good_matches, cv::Mat& img_matches)
{
  cv::drawMatches(image, keypoints, cam_img, cam_keypoints, good_matches, img_matches,
                  cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector< char >(),
                  cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

  // step 6: Localize the object
  std::vector< cv::Point2f > obj;
  std::vector< cv::Point2f > scene;

  for (int i = 0; i < good_matches.size(); i++)
  {
    // Get the keypoints from the good matches
    obj.push_back(keypoints[good_matches[i].queryIdx].pt);
    scene.push_back(cam_keypoints[good_matches[i].trainIdx].pt);
  }

  cv::Mat H = cv::findHomography(obj, scene, CV_RANSAC, 2);

  // step 7: Get the corners from the target
  std::vector< cv::Point2f > scene_corners(4);

  cv::perspectiveTransform(centerAndCorners, scene_corners, H);

  // step 8: Draw lines between the corners (the mapped object in the scene - image_2 )
  cv::line(img_matches, scene_corners[0] + centerAndCorners[1],
           scene_corners[1] + centerAndCorners[1], cv::Scalar(0, 255, 0), 4);
  cv::line(img_matches, scene_corners[1] + centerAndCorners[1],
           scene_corners[2] + centerAndCorners[1], cv::Scalar(0, 255, 0), 4);
  cv::line(img_matches, scene_corners[2] + centerAndCorners[1],
           scene_corners[3] + centerAndCorners[1], cv::Scalar(0, 255, 0), 4);
  cv::line(img_matches, scene_corners[3] + centerAndCorners[1],
           scene_corners[0] + centerAndCorners[1], cv::Scalar(0, 255, 0), 4);

  ROS_DEBUG("corner[0] %f %f", scene_corners[0].x, scene_corners[0].y);
  ROS_DEBUG("corner[1] %f %f", scene_corners[1].x, scene_corners[1].y);
  ROS_DEBUG("corner[2] %f %f", scene_corners[2].x, scene_corners[2].y);
  ROS_DEBUG("corner[3] %f %f", scene_corners[3].x, scene_corners[3].y);

  ROS_DEBUG("end Target::draw");
}

void Target::position(std::vector< cv::KeyPoint > cam_keypoints,
                      std::vector< cv::DMatch > good_matches, std::vector< cv::Point2f >& coord)
{
  // step 6: Localize the object
  std::vector< cv::Point2f > obj;
  std::vector< cv::Point2f > scene;

  for (int i = 0; i < good_matches.size(); i++)
  {
    // Get the keypoints from the good matches
    obj.push_back(keypoints[good_matches[i].queryIdx].pt);
    scene.push_back(cam_keypoints[good_matches[i].trainIdx].pt);
  }

  cv::Mat H = cv::findHomography(obj, scene, CV_RANSAC, 2);

  // step 7: Get the corners from the target
  coord.resize(5);

  cv::perspectiveTransform(centerAndCorners, coord, H);

  ROS_DEBUG("end Target::position");
}

bool customLess(cv::DMatch a, cv::DMatch b)
{
  return a.trainIdx < b.trainIdx;
}
