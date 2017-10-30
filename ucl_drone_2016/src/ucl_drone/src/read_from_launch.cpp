/*!
 *  This file is part of ucl_drone 2016.
 *  For more information, refer
 *  to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 */

#include <ucl_drone/read_from_launch.h>

// IMPROVEMENT: use NAN as default value rather than 0

double Read::_img_width;
double Read::_img_height;
double Read::_img_center_x;
double Read::_img_center_y;
double Read::_focal_length_x;
double Read::_focal_length_y;

bool Read::CamMatrixParams(const std::string &param_name, cv::Mat &cam_matrix)
{
  XmlRpc::XmlRpcValue cam_list;
  cam_matrix = cv::Mat_< double >(3, 3);
  // std::stringstream str_stream;
  if (ros::param::get(param_name, cam_list))
  {
    if (cam_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_WARN("Camera matrix values for %s is not a list", param_name.c_str());
      return false;
    }

    if (cam_list.size() != 9)
    {
      ROS_WARN("Camera list size for %s is not of size 9 (Size: %d)", param_name.c_str(),
               cam_list.size());
      return false;
    }

    for (int32_t i = 0; i < cam_list.size(); i++)
    {
      ROS_ASSERT(cam_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

      cam_matrix.at< double >(i % 3, i / 3) = static_cast< double >(cam_list[i]);
    }
    return true;
  }
  else
  {
    ROS_INFO("No values found for `%s` in parameters, set all to zero.", param_name.c_str());
    return false;
  }
}

bool Read::CamMatrixParams(const std::string &param_name)
{
  XmlRpc::XmlRpcValue cam_list;

  if (ros::param::get(param_name, cam_list))
  {
    if (cam_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_WARN("Camera matrix values for %s is not a list", param_name.c_str());
      return false;
    }

    if (cam_list.size() != 9)
    {
      ROS_WARN("Camera list size for %s is not of size 9 (Size: %d)", param_name.c_str(),
               cam_list.size());
      return false;
    }

    _img_center_x = static_cast< double >(cam_list[2]);
    _img_center_y = static_cast< double >(cam_list[5]);
    _focal_length_x = static_cast< double >(cam_list[0]);
    _focal_length_y = static_cast< double >(cam_list[4]);

    return true;
  }
  else
  {
    ROS_INFO("No values found for `%s` in parameters, set all to zero.", param_name.c_str());
    return false;
  }
}

bool Read::ImgSizeParams(const std::string &param_name)
{
  XmlRpc::XmlRpcValue size_list;
  // std::stringstream str_stream;
  if (ros::param::get(param_name, size_list))
  {
    if (size_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_WARN("Camera matrix values for %s is not a list", param_name.c_str());
      return false;
    }

    if (size_list.size() != 2)
    {
      ROS_WARN("Camera list size for %s is not of size 9 (Size: %d)", param_name.c_str(),
               size_list.size());
      return false;
    }
    // str_stream << param_name << " set to [";

    _img_width = static_cast< double >(size_list[0]);
    _img_height = static_cast< double >(size_list[1]);

    return true;
  }
  else
  {
    ROS_INFO("No values found for `%s` in parameters, set all to zero.", param_name.c_str());
    return false;
  }
}

double Read::img_width()
{
  if (_img_width == 0)
  {
    ROS_ERROR("Read error (img_width)");
  }
  return _img_width;
}

double Read::img_height()
{
  if (_img_height == 0)
  {
    ROS_ERROR("Read error (img_height)");
  }
  return _img_height;
}

double Read::img_center_x()
{
  if (_img_center_x == 0)
  {
    ROS_ERROR("Read error (img_center_x)");
  }
  return _img_center_x;
}

double Read::img_center_y()
{
  if (_img_center_y == 0)
  {
    ROS_ERROR("Read error (img_center_y)");
  }
  return _img_center_y;
}

double Read::focal_length_x()
{
  if (_focal_length_x == 0)
  {
    ROS_ERROR("Read error (img_length_x)");
  }
  return _focal_length_x;
}

double Read::focal_length_y()
{
  if (_focal_length_y == 0)
  {
    ROS_ERROR("Read error (img_length_y)");
  }
  return _focal_length_y;
}
