
#include <thesis_aurian/image_flow.hpp>

ImageFlow::ImageFlow() {
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  // Subscribers
  // image_sub =
  //     nh.subscribe("ardrone/front/image_raw", 1, &ImageFlow::imageCb, this);
  calib_image_sub = nh.subscribe("ardrone/front/image_rect_color", 1,
                                 &ImageFlow::calib_imageCb, this);

  // Publishers
  processed_image_pub = it.advertise("processed_image", 1);
}

// void ImageFlow::imageCb(const sensor_msgs::Image &img_msg) {
//   ROS_DEBUG("ImageFlow::imageCb");
//   current_img = img_msg;
// }

void ImageFlow::calib_imageCb(const sensor_msgs::ImageConstPtr &msg) {
  // Conversion from ROS image message to OpenCV Mat image format
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(
      msg, sensor_msgs::image_encodings::BGR8); // 8UC3 = BGR8 format
  converted_img = cv_ptr->image;
  image_processor(converted_img);
}

void ImageFlow::image_processor(const cv::Mat bgr_img) {

  /**********************************
   ********* COLOR FILTERING ********
   **********************************/
  // Applying red-color filtering
  // door_a115: BGR [17, 21, 104];
  // door_lab: BGR [29, 30, 121];
  // BGR 24 18 49 + 27 21 52 + 26 18 48 + 32 23 57 + 13 16 29
  // HSV 339 62 22 + 351 60 17 + 344 52 25 + 328 54 11 + 346 64 18
  // Shelve
  // BGR 34 45 66 + 13 19 35
  // HSV 20 52 34 + 17 58 12

  cv::cvtColor(bgr_img, hsv_img, cv::COLOR_BGR2HSV);
  cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);
  /* OLD
    cv::inRange(hsv_img, cv::Scalar(155, 95, 35), cv::Scalar(174, 255, 102),
                redfilt_upp_img);
    cv::inRange(hsv_img, cv::Scalar(0, 95, 35), cv::Scalar(5, 255, 102),
                redfilt_low_img);
    cv::inRange(hsv_img, cv::Scalar(0, 0, 0), cv::Scalar(179, 255, 39),
                redfilt_dark_img);
  */

  cv::inRange(hsv_img, cv::Scalar(150, 60, 35), cv::Scalar(174, 255, 102),
              redfilt_upp_img);
  // cv::inRange(hsv_img, cv::Scalar(0, 85, 35), cv::Scalar(5, 255, 102),
  //             redfilt_low_img);
  cv::inRange(hsv_img, cv::Scalar(0, 0, 0), cv::Scalar(25, 255, 45),
              redfilt_dark_img1);
  // cv::inRange(hsv_img, cv::Scalar(8, 135, 0), cv::Scalar(179, 255, 45),
  //             redfilt_dark_img2);

  // Combine the above two images
  // cv::addWeighted(redfilt_low_img, 1.0, redfilt_upp_img, 1.0, 0.0,
  //                 redfilt_lu_img);
  cv::addWeighted(redfilt_dark_img1, 1.0, redfilt_upp_img, 1.0, 0.0,
                  redfilt_du_img);
  // cv::addWeighted(redfilt_du_img, 1.0, redfilt_low_img, 1.0, 0.0,
  //                 redfilt_ldu_img);

  /* to add
    158;84;51
    6.4;56;41
    170;97;46
    172;64;51
    4.48;89;56 */

  /* to withdraw
    6-8;120;43
    83;33;38
    89.5;30.6;17.85
  */

  /* expe total manual:
  H: 0-7 + 170-179
  S: 107-255  ?90
  V: 12-128 ?0-230
  */

  cv::inRange(hsv_img, cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 255), redfilt_sub);
  redfilt_final = redfilt_du_img - redfilt_sub;
  /* to withdraw outside (hall)
  60 11 7 -> 29 28 17
  0 0 3 -> 0 0 7.65
  0 0 4 -> 0 0 10.2
  0 0 5 -> 0 0 12.75
  0 0 9 -> 0 0 22.95
  */
  // ros_img =
  //     cv_bridge::CvImage(std_msgs::Header(), "HSV8",
  //     redfilt_img).toImageMsg();
  //
  // processed_img = *ros_img;
  // processed_img.width = 640;  /// 2;
  // processed_img.height = 360; /// 2;
  // processed_img.step = 640 * 3;
  // cv::namedWindow("Low-Up Red-filtered image", cv::WINDOW_AUTOSIZE);
  // cv::imshow("Low-Up Red-filtered image", redfilt_lu_img);

  // cv::namedWindow("Upper Red-filtered image", cv::WINDOW_AUTOSIZE);
  // cv::imshow("Upper Red-filtered image", redfilt_upp_img);

  // cv::namedWindow("Lower Red-filtered image", cv::WINDOW_AUTOSIZE);
  // cv::imshow("Lower Red-filtered image", redfilt_low_img);

  // cv::namedWindow("Dark1 Red-filtered image", cv::WINDOW_AUTOSIZE);
  // cv::imshow("Dark1 Red-filtered image", redfilt_dark_img1);

  // cv::namedWindow("Dark2 Red-filtered image", cv::WINDOW_AUTOSIZE);
  // cv::imshow("Dark2 Red-filtered image", redfilt_dark_img2);

  cv::namedWindow("Dark-Up Red-filtered image", cv::WINDOW_AUTOSIZE);
  cv::imshow("Dark-Up Red-filtered image", redfilt_du_img);

  // cv::namedWindow("Low-Dark-Up Red-filtered image", cv::WINDOW_AUTOSIZE);
  // cv::imshow("Low-Dark-Up Red-filtered image", redfilt_ldu_img);

  // cv::namedWindow("Sub Red-filtered image", cv::WINDOW_AUTOSIZE);
  // cv::imshow("Sub Red-filtered image", redfilt_sub);

  cv::namedWindow("Final Red-filtered image", cv::WINDOW_AUTOSIZE);
  cv::imshow("Final Red-filtered image", redfilt_final);

  // processed_image_pub.publish(processed_img);

  /*********************************
   ********* SOBEL OPERATOR ********
   *********************************/
  int ksize = 3;
  int scale = 1;
  int delta = 0;
  int ddepth_filt = CV_64F;
  cv::Mat grad, grad_filt;
  cv::Mat grad_x_filt, grad_x, grad_y_filt, grad_y;
  cv::Mat abs_grad_x, abs_grad_x_filt, abs_grad_y, abs_grad_y_filt;

  // With color filter
  cv::Sobel(redfilt_final, grad_x_filt, ddepth_filt, 1, 0, ksize, scale, delta,
            cv::BORDER_DEFAULT);
  cv::Sobel(redfilt_final, grad_y_filt, ddepth_filt, 0, 1, ksize, scale, delta,
            cv::BORDER_DEFAULT);
  // converting back to CV_8U
  cv::convertScaleAbs(grad_x_filt, abs_grad_x_filt);
  cv::convertScaleAbs(grad_y_filt, abs_grad_y_filt);
  cv::addWeighted(abs_grad_x_filt, 0.5, abs_grad_y_filt, 0.5, 0, grad_filt);
  cv::imshow("Sobel - Color-filtered", grad_filt);

  // Without color filter
  int ddepth = CV_8U;
  cv::Sobel(gray_img, grad_x, ddepth, 1, 0, ksize, scale, delta,
            cv::BORDER_DEFAULT);
  cv::Sobel(gray_img, grad_y, ddepth, 0, 1, ksize, scale, delta,
            cv::BORDER_DEFAULT);
  // converting back to CV_8U
  // cv::convertScaleAbs(grad_x, abs_grad_x);
  // cv::convertScaleAbs(grad_y, abs_grad_y);
  // cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
  cv::addWeighted(grad_x, 0.5, grad_y, 0.5, 0, grad);
  cv::imshow("Sobel - Direct", grad);

  using namespace cv;
  using namespace std;
  /**********************************
   ********* HOUGH TRANSFORM ********
   **********************************/

  // Set1: threshold = 101; minLength = 66; maxLineGap = 12; thickness rho deg =
  // 1
  // Set2: threshold = 71; minLength = 32; maxLineGap = 11; thickness = 4; rho
  // deg = 1
  if (keypressed) {
    if (k == 'R' || k == 'r') {
      rho += 1;
      ROS_INFO("rho = %f", rho);
    } else if (k == 'E' || k == 'e') {
      rho -= 1;
      ROS_INFO("rho = %f", rho);
    }

    if (k == 'D' || k == 'd') {
      deg += 5;
      ROS_INFO("deg = %i", deg);
    } else if (k == 'S' || k == 's') {
      deg -= 5;
      ROS_INFO("deg = %i", deg);
    }

    if (k == 'T' || k == 't') {
      threshold += 5;
      ROS_INFO("threshold = %i", threshold);
    } else if (k == 'Y' || k == 'y') {
      threshold -= 5;
      ROS_INFO("threshold = %i", threshold);
    }

    if (k == 'L' || k == 'l') {
      minLength += 2;
      ROS_INFO("minLength = %f", minLength);
    } else if (k == 'K' || k == 'k') {
      minLength -= 2;
      ROS_INFO("minLength = %f", minLength);
    }

    if (k == 'G' || k == 'g') {
      maxLineGap += 1;
      ROS_INFO("maxLineGap = %f", maxLineGap);
    } else if (k == 'F' || k == 'f') {
      maxLineGap -= 1;
      ROS_INFO("maxLineGap = %f", maxLineGap);
    }

    if (k == 'P' || k == 'p') {
      thickness += 1;
      ROS_INFO("thickness = %i", thickness);
    } else if (k == 'O' || k == 'o') {
      thickness -= 1;
      ROS_INFO("thickness = %i", thickness);
    }

    if (k == 'A' || k == 'a') {
      ROS_INFO("rho = %f", rho);
      ROS_INFO("deg = %i", deg);
      ROS_INFO("threshold = %i", threshold);
      ROS_INFO("minLength = %f", minLength);
      ROS_INFO("maxLineGap = %f", maxLineGap);
      ROS_INFO("thickness = %i", thickness);
    }

  } else {
    // Hough Transform parameters
    deg = 1;
    rho = 1;
    theta = CV_PI / 180 * deg;
    threshold = 1;
    minLength = 0;
    maxLineGap = 0;
    thickness = 1;
  }

  // WITHOUT color-filtering
  cvtColor(grad, gradBGR, CV_GRAY2BGR);
  vector<Vec4i> lines;
  HoughLinesP(grad, lines, rho, theta, threshold, minLength, maxLineGap);
  for (size_t i = 0; i < lines.size(); i++) {
    Vec4i l = lines[i];
    line(gradBGR, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255),
         thickness, CV_AA);
  }
  imshow("Hough Transform", gradBGR);

  // WITH color-filtering
  cvtColor(grad_filt, gradBGR_filt, CV_GRAY2BGR);
  Mat Blank(grad_filt.rows, grad_filt.cols, CV_8UC3, Scalar(0, 0, 0));
  vector<Vec4i> lines_filt;
  HoughLinesP(grad_filt, lines_filt, rho, theta, threshold, minLength,
              maxLineGap);
  for (size_t i = 0; i < lines_filt.size(); i++) {
    Vec4i l = lines_filt[i];
    line(gradBGR_filt, Point(l[0], l[1]), Point(l[2], l[3]),
         Scalar(255, 255, 255), thickness, CV_AA);
    line(Blank, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255),
         thickness, CV_AA);
  }

  imshow("Hough Transform with color-filtering", Blank);

  // Canny WITHOUT color-filtering
  Canny(gray_img, grad_canny, 50, 200, 3);
  cvtColor(grad_canny, gradBGR_canny, CV_GRAY2BGR);
  vector<Vec4i> lines_canny;
  HoughLinesP(grad_canny, lines_canny, rho, theta, threshold, minLength,
              maxLineGap);

  for (size_t i = 0; i < lines_canny.size(); i++) {
    Vec4i l = lines_canny[i];
    line(gradBGR_canny, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255),
         thickness, CV_AA);
    line(Blank, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255),
         thickness, CV_AA);
  }

  imshow("Hough Transform with Canny", Blank);

  k = waitKey(0);
  keypressed = true;

  /**********************************
   ********* MATCHING STEP **********
   **********************************/
  // Creation of the reference image
  rectangle(Mat & img, Point pt1, Point pt2, const Scalar &color,
            int thickness = 1, int lineType = 8, int shift = 0)¶
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_flow");
  ImageFlow imageflownode;
  ros::Rate loop_rate(1);

  ROS_INFO_STREAM("image_flow node started!");

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}