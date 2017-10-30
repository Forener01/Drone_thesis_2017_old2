/*!
 *  \file computer_vision.h
 *  \brief Computer vision header file.
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 *  Part of ucl_drone. Computer vision ROS node. Contains:
 *        - keypoints detector/descriptor
 *        - image transformations
 *        - target detection
 */

#ifndef UCL_DRONE_COMPUTER_VISION_H
#define UCL_DRONE_COMPUTER_VISION_H
#define PCL_NO_PRECOMPILE

// #define USE_PROFILING
#include <ucl_drone/profiling.h>

#include <boost/shared_ptr.hpp>

// ROS Header files
#include <ros/package.h>
#include <ros/ros.h>

#include <ucl_drone/ucl_drone.h>

// vision
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/gpu.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/ocl.hpp>
#include <opencv2/ocl/ocl.hpp>
#include <opencv2/video/tracking.hpp>

// messages
#include <ardrone_autonomy/CamSelect.h>
#include <ardrone_autonomy/Navdata.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <ucl_drone/Pose3D.h>
#include <ucl_drone/ProcessedImageMsg.h>

// Choose one detector in function of what is defined in ucl_drone.h

#if DETECTOR_TYPE == TYPE_SIFT
// static const cv::SIFT detector(0, 3, 0.1, 20, 3);
static const cv::SIFT detector(0, 3, 0.1, 15, 2);

#elif DETECTOR_TYPE == TYPE_SURF
// static const cv::SurfFeatureDetector detector(8000, 8, 3);
// static const cv::SurfFeatureDetector detector(4000, 6, 4, false);
// static const cv::SurfFeatureDetector detector(2000, 8, 3, false);
static const cv::SurfFeatureDetector detector(1800, 6, 3, false);

#elif DETECTOR_TYPE == TYPE_FAST
static const cv::FastFeatureDetector detector(50);

#elif DETECTOR_TYPE == TYPE_BRISK
static const cv::BRISK detector;

#elif DETECTOR_TYPE == TYPE_ORB
static const cv::OrbFeatureDetector detector(200, 1.4f, 5, 60, 2, 2, cv::ORB::HARRIS_SCORE, 60);
#endif

// Choose one descriptor in function of what is defined in ucl_drone.h

#if EXTRACTOR_TYPE == TYPE_SIFT
static const cv::SiftDescriptorExtractor extractor;

#elif EXTRACTOR_TYPE == TYPE_SURF
static const cv::SurfDescriptorExtractor extractor(4000, 6, 4, false);

#elif EXTRACTOR_TYPE == TYPE_SURF_128
static const cv::SurfDescriptorExtractor extractor(4000, 6, 4, true);

#elif EXTRACTOR_TYPE == TYPE_ORB
static const cv::OrbDescriptorExtractor extractor;

#elif EXTRACTOR_TYPE == TYPE_FREAK
static const cv::FREAK extractor;

#endif

// ucl_drone

#include <ucl_drone/opencv_utils.h>

#include <ucl_drone/map/projection_2D.h>

#include <ucl_drone/computer_vision/target.h>

#include <ucl_drone/computer_vision/processed_image.h>

#include <ucl_drone/computer_vision/image_processor.h>

#endif /* UCL_DRONE_COMPUTER_VISION_H */
