/**
 *  \file PointXYZRGBSIFT.h
 *  \brief PointXYZRGBSIFT is a PCL PointXYZ with additionnal fields like a
 *         (SIFT) descriptor of the point (computed by opencv)
 *  \author Arnaud Jacques & Alexandre Leclère
 */

#ifndef UCL_DRONE_POINTXYZRGBSIFT_H
#define UCL_DRONE_POINTXYZRGBSIFT_H
#define PCL_NO_PRECOMPILE

#include <ucl_drone/ucl_drone.h>

#include <pcl/point_types.h>

namespace pcl
{
struct PointXYZRGBSIFT  //: PointXYZRGB
{
  PCL_ADD_POINT4D;  // preferred way of adding a XYZ+padding
  // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  PCL_ADD_RGB;
  float descriptor[DESCRIPTOR_SIZE];
  int view_count;
  int keyframe_ID;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;                   // enforce SSE padding for correct memory alignment
}

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZRGBSIFT,  // here we assume a XYZRGB + "descriptor"
                                                         // (as
                                                         // fields)
                                  (float, x, x)(float, y, y)(float, z, z)(float, rgb, rgb)(
                                      float[DESCRIPTOR_SIZE], descriptor,
                                      descriptor)(int, view_count, view_count)(int, keyframe_ID,
                                                                               keyframe_ID))

namespace pcl
{
struct PointUVSIFT  //: PointXYZRGB
{
  float u;
  float v;
  float descriptor[DESCRIPTOR_SIZE];

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;                   // enforce SSE padding for correct memory alignment
}

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointUVSIFT,  // here we assume a XYZRGB + "descriptor"
                                                     // (as fields)
                                  (float, u, u)(float, v, v)(float[DESCRIPTOR_SIZE], descriptor,
                                                             descriptor))

#endif /* UCL_DRONE_POINTXYZRGBSIFT_H */
