/* Pojection 2D
*/

#include <ucl_drone/map/projection_2D.h>

//#define PROJECTION_2D_2015  // to comment if wanna use the 2016 projection

// this function prepares structures to compute all point projections
void projection2D_utils1(double h, const ucl_drone::Pose3D pose, cv::Mat &T, cv::Mat &n_cam)
{
  // Conversion
  double yaw = -pose.rotZ;
  double pitch = -pose.rotY;
  double roll = -pose.rotX;

  // compute the rotation matrix from world  to the drone
  cv::Mat world2drone = rollPitchYawToRotationMatrix(roll, pitch, yaw);
  // compute the rotation matrix from the drone to the camera
  cv::Mat drone2cam = rollPitchYawToRotationMatrix(PI, 0, -PI / 2);
  // compute the rotation matrix from the world to the camera
  T = drone2cam * world2drone;

  // ground normal vector in world coordinates
  cv::Mat n_world = (cv::Mat_< double >(3, 1) << 0.0, 0.0, 1.0);
  // ground normal vector in camera coordinates
  n_cam = T * n_world;
}

// this function compute the ground projection for one point
void projection2D_utils2(double pixelx, double pixely, double h, const cv::Mat &T,
                         const cv::Mat &n_cam, cv::Mat &p_world)
{
  // compute point coordinates in calibrated image coordinates
  cv::Mat d = (cv::Mat_< double >(3, 1) << (pixelx - Read::img_center_x()) / Read::focal_length_x(),
               (pixely - Read::img_center_y()) / Read::focal_length_y(), 1);

  // see report 2016 for details section workspace transformation
  cv::Mat temp = n_cam.t() * d;
  double t = -h / (temp.at< double >(0, 0));

  cv::Mat p_cam = d * t;
  p_world = T.t() * p_cam;
}

#ifndef PROJECTION_2D_2015

//! converts a point in an image in openCV format to a mappoint in OpenCV format.
//! 2015-2016 version
void projection_2D(std::vector< cv::Point2f > &points_in, ucl_drone::Pose3D &pose,
                   std::vector< cv::Point3f > &points_out, bool h_flag /*= false*/)
{
  cv::Mat T;
  cv::Mat n_cam;
  double h = pose.z;
  if (h_flag)  // useful when the drone is not flying: the altitude is not published
  {
    h = 0.73;
  }
  projection2D_utils1(h, pose, T, n_cam);
  points_out.resize(points_in.size());  // prepares output structure
  for (int i = 0; i < points_in.size(); i++)
  {
    cv::Mat p_world;
    projection2D_utils2(points_in[i].x, points_in[i].y, h, T, n_cam, p_world);
    points_out[i] = cv::Point3f((float)(p_world.at< double >(0, 0) + pose.x),
                                (float)(p_world.at< double >(1, 0) + pose.y),
                                (float)(p_world.at< double >(2, 0) + h));

    // ROS_DEBUG("POINT[%d] (%f,%f) ::: (%f,%f,%f)", i, points_in[i].x, points_in[i].y,
    //           points_out[i].x, points_out[i].y, points_out[i].z);
  }
}

#endif

#ifdef PROJECTION_2D_2015

#define MAP_WIDTH 640
#define MAP_HEIGHT 360
// Half of the total view angle for x and y respectively
#define FOV_X 0.2575
#define FOV_Y 0.465
// Measured 93cm along y axis and 51.5cm along x axis for height = 100cm

//! converts a point in an image in openCV format to a mappoint in OpenCV format.
//! 2014-2015 version
void projection_2D(std::vector< cv::Point2f > &points_in, ucl_drone::Pose3D &pose,
                   std::vector< cv::Point3f > &points_out, bool h_flag /*= false*/)

// boost::shared_ptr< ProcessedImage > image, ucl_drone::Pose3D pose,
// pcl::PointCloud< pcl::PointXYZRGB >::Ptr pointcloud)
{
  // see report 2015 for details section workspace transformation

  double altitude = 1;  // image->navdata.altd; // Altitude given by the verticale

  double theta = -(double)pose.rotY / 1000 / 180 * PI;  // Conversion from 10^-3 degrees to radians
  double phi = (double)pose.rotX / 1000 / 180 * PI;
  double theta_z = (double)pose.rotZ / 1000 / 180 * PI;
  double theta_x = cos(theta_z) * theta - sin(theta_z) * phi;
  double theta_y = -sin(theta_z) * theta - cos(theta_z) * phi;

  // Pixel projection on image plane
  // Distance from the drone to the center of the image plane (=center on the
  // ground)
  double plane_dist = altitude / cos(theta_x) / cos(theta_y);

  points_out.resize(points_in.size());

  for (int i = 0; i < points_in.size(); i++)
  {
    double pixelx = points_in[i].x;
    double pixely = points_in[i].y;
    /* (x,y,z) is the relative position between the camera and the point on the
    image
    !! Watch out, x and y axis are switched between the image and the map:
            Looking forwards is along positive y values on the image as opposed
    to
    positive
            x values on the map */
    double x = plane_dist *
               sin(((double)pixelx - (double)MAP_WIDTH / 2) / ((double)MAP_WIDTH / 2) * FOV_Y);
    double y = plane_dist *
               sin(-((double)pixely - (double)MAP_HEIGHT / 2) / ((double)MAP_HEIGHT / 2) * FOV_X);
    double z = 1 / cos(theta_x) / cos(theta_y);
    /* The terms plane_dist get eliminated by division, however if the depth of
    the points weren't the
    same, the results would change. So plane_dist is left in case 3D
    implementation is to be implemented
    at some point. */

    /* (dx,dy,dz) are results of multiplying (x,y,z) by three rotation matrices,
    respectively around axis x,y and z */
    double dx = cos(theta_y) * (sin(theta_z) * y + cos(theta_z) * x) - sin(theta_y) * z;
    double dy =
        sin(theta_x) * (cos(theta_y) * z + sin(theta_y) * (sin(theta_z) * y + cos(theta_z) * x)) +
        cos(theta_x) * (cos(theta_z) * y - sin(theta_z) * x);
    double dz =
        cos(theta_x) * (cos(theta_y) * z + sin(theta_y) * (sin(theta_z) * y + cos(theta_z) * x)) -
        sin(theta_x) * (cos(theta_z) * y - sin(theta_z) * x);

    // printf("dx = %f, dy = %f, dz = %f\n", dx, dy, dz);

    cv::Point3f point;
    point.x = pose.x + dx / dz;
    point.y = pose.y + dy / dz;
    point.z = 0;
    points_out[i] = point;
  }
}

/* Function convert_imgpoint_to_mappoint
*	@pre : Takes as inputs the pixel positions on the image(pixelx,y),
*			and the navdata.
*	@post : Pointers to the output map positions (output_mapx,y)
*	Converts a point from the image workspace to the map workspace.
*	This requires geometric conversions based on rotation matrices as well as
*	calibrated values FOV_X,y, the image dimensions (WIDTH and HEIGHT) all defined
*	in the header (.h)
*/
void convert_imgpoint_to_mappoint(int pixelx, int pixely, double *output_mapx, double *output_mapy,
                                  navdata_struct navdata)
{
  double altitude = 1;  // navdata.altitude; // Altitude given by the verticale distance between the
                        // drône and the Cocard [in m]
  double theta =
      (double)navdata.Theta / 1000 / 180 * PI;  // Conversion from 10^-3 degrees to radians
  double phi = -(double)navdata.Phi / 1000 / 180 * PI;
  double theta_z = (double)navdata.Psi / 1000 / 180 * PI;
  double theta_x = cos(theta_z) * theta - sin(theta_z) * phi;
  double theta_y = -sin(theta_z) * theta - cos(theta_z) * phi;

  // Pixel projection on image plane
  // Distance from the drone to the center of the image plane (=center on the ground)
  double plane_dist = altitude / cos(theta_x) / cos(theta_y);
  /* (x,y,z) is the relative position between the camera and the point on the image
  !! Watch out, x and y axis are switched between the image and the map:
    Looking forwards is along positive y values on the image as opposed to positive
    x values on the map */
  double x =
      plane_dist * sin(((double)pixelx - (double)MAP_WIDTH / 2) / ((double)MAP_WIDTH / 2) * FOV_Y);
  double y = plane_dist *
             sin(-((double)pixely - (double)MAP_HEIGHT / 2) / ((double)MAP_HEIGHT / 2) * FOV_X);
  double z = 1 / cos(theta_x) / cos(theta_y);
  /* The terms plane_dist get eliminated by division, however if the depth of the points weren't the
  same, the results would change. So plane_dist is left in case 3D implementation is to be
  implemented
  at some point. */

  /* (dx,dy,dz) are results of multiplying (x,y,z) by three rotation matrices,
  respectively around axis x,y and z */
  double dx = cos(theta_y) * (sin(theta_z) * y + cos(theta_z) * x) - sin(theta_y) * z;
  double dy =
      sin(theta_x) * (cos(theta_y) * z + sin(theta_y) * (sin(theta_z) * y + cos(theta_z) * x)) +
      cos(theta_x) * (cos(theta_z) * y - sin(theta_z) * x);
  double dz =
      cos(theta_x) * (cos(theta_y) * z + sin(theta_y) * (sin(theta_z) * y + cos(theta_z) * x)) -
      sin(theta_x) * (cos(theta_z) * y - sin(theta_z) * x);

  *output_mapx = navdata.posx + dx / dz;  // Result is in meters
  *output_mapy = navdata.posy + dy / dz;
}

#endif
