/*
 *  This file is part of ucl_drone 2016.
 *  For more information, refer
 *  to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 *
 *  based on tuto:
 *  http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
 */

#include <ucl_drone/computer_vision/computer_vision.h>

int ProcessedImage::last_number_of_keypoints = 0;

// Contructor for the empty object
ProcessedImage::ProcessedImage()
{
  ucl_drone::ProcessedImageMsg::Ptr msg(new ucl_drone::ProcessedImageMsg());
}

// Constructor
// [in] msg: ROS Image message sent by the camera
// [in] pose_: Pose3D message before visual estimation to be attached with the processed image
// [in] prev: previous processed image used for keypoints tracking
// [in] use_OpticalFlowPyrLK: flag to enable keypoints tracking
ProcessedImage::ProcessedImage(const sensor_msgs::Image msg, const ucl_drone::Pose3D pose_,
                               ProcessedImage& prev, bool use_OpticalFlowPyrLK)
{
  ROS_DEBUG("ProcessedImage::init");
  this->pose = pose_;

  // convert ROS image to OpenCV image
  try
  {
    this->cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    // ROS_DEBUG("ProcessedImage::init: the size of the corrected image is: %d, %d",
    // cv_ptr->image.rows, cv_ptr->image.cols);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("ucl_drone::imgproc::cv_bridge exception: %s", e.what());
    return;
  }

  // Resize the image according to the parameters in the launch file
  cv::Size size(Read::img_width(), Read::img_height());
  cv::resize(this->cv_ptr->image, this->cv_ptr->image, size);

  // Convert opencv image to ROS Image message format
  this->cv_ptr->toImageMsg(this->image);

  // Use keypoints tracking between the previous and the current image processed
  if (use_OpticalFlowPyrLK && prev.cv_ptr && prev.keypoints.size() > 0 &&
      ProcessedImage::last_number_of_keypoints != 0)  // performs only if the previous image
                                                      // processed is not empty and if keypoints
                                                      // were detected
  {
    ROS_DEBUG("use_OpticalFlowPyrLK");
    TIC(optical_flow);
    bool OF_success = false;  // flag which indicates the success of keypoints tracking

    // If the picture is in colours, convert it to grayscale
    cv::Mat prevgray, gray;
    if (this->cv_ptr->image.channels() == 3)
    {
      cv::cvtColor(this->cv_ptr->image, gray, CV_RGB2GRAY);
      cv::cvtColor(prev.cv_ptr->image, prevgray, CV_RGB2GRAY);
    }
    else
    {
      prevgray = prev.cv_ptr->image;
      gray = this->cv_ptr->image;
    }

    // Prepare structures to receive keypoints tracking results
    std::vector< uchar > vstatus(prev.keypoints.size());
    std::vector< float > verror(prev.keypoints.size());

    std::vector< cv::Point2f > found;
    std::vector< cv::Point2f > to_find = Points(prev.keypoints);

    // Perform keypoints tracking
    cv::calcOpticalFlowPyrLK(prevgray, gray, to_find, found, vstatus, verror);

    // Copy all keypoints tracked with success
    double thres = 12.0;

    for (int i = 0; i < prev.keypoints.size(); i++)
    {
      if (vstatus[i] && verror[i] < thres)
      {
        cv::KeyPoint newKeyPoint = prev.keypoints[i];
        newKeyPoint.pt.x = found[i].x;
        newKeyPoint.pt.y = found[i].y;
        this->keypoints.push_back(newKeyPoint);
      }
    }

    // Determine if enough keypoints were detected or if new keypoints detection is needed
    OF_success = this->keypoints.size() > ProcessedImage::last_number_of_keypoints * 0.75 &&
                 this->keypoints.size() > 80;
    TOC(optical_flow, "optical flow tracking");

    if (OF_success)  // then simply copy all descriptors previously computed
    {
      this->descriptors = cv::Mat::zeros(this->keypoints.size(), DESCRIPTOR_SIZE, CV_32F);
      for (int i = 0, j = 0; i < prev.keypoints.size(); i++)
      {
        if (vstatus[i] && verror[i] < thres)
        {
          for (unsigned k = 0; k < DESCRIPTOR_SIZE; k++)
          {
            this->descriptors.at< float >(j, k) = prev.descriptors.at< float >(i, k);
          }
          j++;
        }
      }
      // KEYPOINTS TRACKING SUCCESS, no more computation needed
      return;
    }
    else
    {
      this->keypoints.clear();
    }
  }

  ROS_DEBUG("=== OPTICAL FLOW NOT SUCCESS ===");

  // Keypoints detection for current image
  TIC(detect);
  detector.detect(this->cv_ptr->image, this->keypoints);
  TOC(detect, "detect keypoints");
  ROS_DEBUG("ProcessedImage::init this->keypoints.size()=%d", this->keypoints.size());
  ProcessedImage::last_number_of_keypoints = this->keypoints.size();
  if (this->keypoints.size() == 0)
  {
    return;
  }
  TIC(extract);
  // Perform keypoints description
  extractor.compute(this->cv_ptr->image, this->keypoints, this->descriptors);
  TOC(extract, "descriptor extrator");
  ROS_DEBUG("end ProcessedImage::init");
}

ProcessedImage::~ProcessedImage()
{
}

// This function fill the message that will be sent by the computer vision node
// [out] msg: the filled message
// [in] target: the object to perform target detection
void ProcessedImage::convertToMsg(ucl_drone::ProcessedImageMsg::Ptr& msg, Target target)
{
  msg->pose = this->pose;
  msg->image = this->image;

  if (this->keypoints.size() == 0)
  {
    return;
  }

  TIC(target);
  // Prepare structures for target detection
  std::vector< cv::DMatch > good_matches;
  std::vector< int > idxs_to_remove;
  std::vector< cv::Point2f > target_coord;
  // Perform target detection
  bool target_is_detected =
      target.detect(this->descriptors, this->keypoints, good_matches, idxs_to_remove, target_coord
#ifdef DEBUG_PROJECTION
                    ,
                    this->pose
#endif /* DEBUG_PROJECTION */
#ifdef DEBUG_TARGET
                    ,
                    this->cv_ptr->image
#endif /* DEBUG_TARGET */
                    );

  if (target_is_detected)
  {
    ROS_DEBUG("TARGET IS DETECTED");
    msg->target_detected = true;
    // Copy target center and corners position in the picture coordinates
    msg->target_points.resize(5);
    for (int i = 0; i < 5; i++)
    {
      msg->target_points[i].x = target_coord[i].x;
      msg->target_points[i].y = target_coord[i].y;
      msg->target_points[i].z = 0;
    }
  }
  else
  {
    msg->target_detected = false;
  }

  if (this->keypoints.size() - idxs_to_remove.size() <= 0)
  {
    // All keypoints are on the target
    return;
  }

  // Remove keypoints on the target
  msg->keypoints.resize(this->keypoints.size() - idxs_to_remove.size());
  ROS_DEBUG("ProcessedImage::init msg->keypoints.size()=%d", msg->keypoints.size());
  int count = 0;
  int j = 0;
  for (unsigned i = 0; i < this->keypoints.size() && j < msg->keypoints.size() &&
                       (count < idxs_to_remove.size() || idxs_to_remove.size() == 0);
       i++)
  {
    // exclude target points from message to send (to avoid mapping of moving target)
    if (idxs_to_remove.size() == 0 || i != idxs_to_remove[count])
    {
      // Copy the current keypoint position
      ucl_drone::KeyPoint keypoint;
      geometry_msgs::Point point;
      point.x = (double)this->keypoints[i].pt.x;
      point.y = (double)this->keypoints[i].pt.y;

      keypoint.point = point;

      // Copy the current keypoint description
      std::vector< float > descriptor;
      descriptor.resize(DESCRIPTOR_SIZE);
      for (unsigned k = 0; k < DESCRIPTOR_SIZE; k++)
      {
        descriptor[k] = this->descriptors.at< float >(i, k);
      }
      keypoint.descriptor = descriptor;

      msg->keypoints[j] = keypoint;
      j++;
    }
    if (idxs_to_remove.size() != 0 && i == idxs_to_remove[count])
    {
      // this way of doing is possible since idxs_to_remove are sorted in increasing order
      if (count < idxs_to_remove.size() - 1)
      {
        count++;
      }
    }
  }
  TOC(target, "detect and remove target");
  ROS_DEBUG("=========== POINT (%f;%f)", msg->keypoints[msg->keypoints.size() - 1].point.x,
            msg->keypoints[msg->keypoints.size() - 1].point.y);
}
