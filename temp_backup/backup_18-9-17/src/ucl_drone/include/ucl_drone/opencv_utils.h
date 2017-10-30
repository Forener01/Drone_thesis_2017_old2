/*!
 *  \file opencv_utils.h
 *  \brief opencv_utils contains functions to handle
 *         frame transformation with opencv according
 *         with the Pose3D message definition
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 *  Part of ucl_drone.
 */

#ifndef UCL_DRONE_OPENCV_UTILS_H
#define UCL_DRONE_OPENCV_UTILS_H

#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include <ucl_drone/PointXYZRGBSIFT.h>

//! \return the rotation matrix given the roll angle (around x axis)
cv::Mat rotationMatrixX(const double angle);

//! \return the rotation matrix given the pitch angle (around y axis)
cv::Mat rotationMatrixY(const double angle);

//! \return the rotation matrix given the yaw angle (around z axis)
cv::Mat rotationMatrixZ(const double angle);

//! \return the rotation matrix given the roll,pitch,yaw angles
cv::Mat rollPitchYawToRotationMatrix(const double roll, const double pitch, const double yaw);

//! \return the transformation (rotation and transltation) matrix
cv::Mat rTMatrix(const cv::Mat rot, const double tx, const double ty, const double tz);

//! Debug for the transformation (rotation and transltation) matrix
void debugRTMatrix(cv::Mat Rt);

//! This method converts opencv keypoint coordinates to opencv point format
std::vector< cv::Point2f > Points(const std::vector< cv::KeyPoint >& keypoints);

#endif /* UCL_DRONE_OPENCV_UTILS_H */
