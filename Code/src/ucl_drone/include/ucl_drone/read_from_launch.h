
/*!
 *  \file read_from_launch.h
 *  \brief This header file defines a class to read some parameters in the launch file
 *  like the camera calibration matrix
 *  \authors Arnaud Jacques & Alexandre Leclere
 */

#ifndef UCL_DRONE_READ_FROM_LAUNCH_H
#define UCL_DRONE_READ_FROM_LAUNCH_H

#include <ucl_drone/ucl_drone.h>
#include <opencv2/core/core.hpp>

/** \class Read
 *  This class gives some function to read some parameters in the launch file
 */
class Read
{
private:
  static double _img_width;
  static double _img_height;
  static double _img_center_x;
  static double _img_center_y;
  static double _focal_length_x;
  static double _focal_length_y;

public:
  static bool CamMatrixParams(const std::string &param_name, cv::Mat &cam_matrix);
  static bool CamMatrixParams(const std::string &param_name);
  static bool ImgSizeParams(const std::string &param_name);

  static double img_width();
  static double img_height();
  static double img_center_x();
  static double img_center_y();
  static double focal_length_x();
  static double focal_length_y();
};

#endif /* UCL_DRONE_READ_FROM_LAUNCH_H */
