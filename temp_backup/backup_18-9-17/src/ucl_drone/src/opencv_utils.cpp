/*
 *  This file is part of ucl_drone 2016.
 *  For more information, refer
 *  to the corresponding header file.
 *
 *  http://answers.opencv.org/question/64427/position-and-rotation-of-two-cameras-which-functions-i-need-in-what-order/
 *  https://gist.github.com/royshil/7087bc2560c581d443bc
 */

#include <ucl_drone/opencv_utils.h>

//! Rotation matrix about the X axis
cv::Mat rotationMatrixX(const double angle)
{
  cv::Mat Rx = (cv::Mat_< double >(3, 3) << 1.0, 0.0, 0.0, 0.0, cos(angle), -sin(angle), 0.0,
                sin(angle), cos(angle));
  return Rx;
}

//! Rotation matrix about the Y axis
cv::Mat rotationMatrixY(const double angle)
{
  cv::Mat Ry = (cv::Mat_< double >(3, 3) << cos(angle), 0.0, sin(angle), 0.0, 1.0, 0.0, -sin(angle),
                0.0, cos(angle));
  return Ry;
}

//! Rotation matrix about the Z axis
cv::Mat rotationMatrixZ(const double angle)
{
  cv::Mat Rz = (cv::Mat_< double >(3, 3) << cos(angle), -sin(angle), 0.0, sin(angle), cos(angle),
                0.0, 0.0, 0.0, 1.0);
  return Rz;
}

//! Rotation matrix given RPY angles
cv::Mat rollPitchYawToRotationMatrix(const double roll, const double pitch, const double yaw)
{
  cv::Mat Rx = rotationMatrixX(roll);
  cv::Mat Ry = rotationMatrixY(pitch);
  cv::Mat Rz = rotationMatrixZ(yaw);

  return Rz * Ry * Rx;
}

//! Transofmation (rotation and translation) matrix given RPY angles and translation lengths
cv::Mat rTMatrix(const cv::Mat rot, const double tx, const double ty, const double tz)
{
  cv::Mat Rt = (cv::Mat_< double >(3, 4) << rot.at< double >(0, 0), rot.at< double >(0, 1),
                rot.at< double >(0, 2), tx, rot.at< double >(1, 0), rot.at< double >(1, 1),
                rot.at< double >(1, 2), ty, rot.at< double >(2, 0), rot.at< double >(2, 1),
                rot.at< double >(2, 2), tz);
  return Rt;
}

void debugRTMatrix(cv::Mat Rt)
{
  ROS_DEBUG("Rt = \n[ %f \t %f \t %f \t %f \n  %f \t %f \t %f \t %f \n  %f \t %f \t %f \t %f ]",
            Rt.at< double >(0, 0), Rt.at< double >(0, 1), Rt.at< double >(0, 2),
            Rt.at< double >(0, 3), Rt.at< double >(1, 0), Rt.at< double >(1, 1),
            Rt.at< double >(1, 2), Rt.at< double >(1, 3), Rt.at< double >(2, 0),
            Rt.at< double >(2, 1), Rt.at< double >(2, 2), Rt.at< double >(2, 3));
}

//! Convert vector of opencv keypoints in a vector of opencv 2D points
std::vector< cv::Point2f > Points(const std::vector< cv::KeyPoint >& keypoints)
{
  std::vector< cv::Point2f > result;
  for (unsigned i = 0; i < keypoints.size(); i++)
  {
    result.push_back(cv::Point2f(keypoints[i].pt.x, keypoints[i].pt.y));
  }
  return result;
}
