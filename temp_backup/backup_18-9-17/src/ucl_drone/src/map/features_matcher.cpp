/*
 *  This file is part of ucl_drone 2016.
 *  For more information, refer
 *  to the corresponding header file.
 *
 *  \author Arnaud Jacques & Alexandre Leclere
 *  \date 2016
 */

#include <ucl_drone/imgproc2D/imgproc2D.h>

//! works for only 2 images
FeaturesMatcher::FeaturesMatcher(std::vector< boost::shared_ptr< ProcessedImage > >& images_)
  : use_ratio_test(true), images(images_)
{
  // std::vector< std::vector< cv::DMatch > > all_matches;
  for (unsigned i = 0; i < 1; i++)  //(unsigned i = 0; i < images.size(); i++)
  {
    for (unsigned j = i + 1; j < 2; j++)  //(unsigned j = i + 1; j < images.size(); j++)
    {
      cv::FlannBasedMatcher matcher;
      if (use_ratio_test)
      {
        std::vector< std::vector< cv::DMatch > > knn_matches;
        matcher.knnMatch(images[i]->descriptors, images[j]->descriptors, knn_matches, 2);

        // ratio_test + threshold test
        for (unsigned k = 0; k < knn_matches.size(); k++)
        {
          if (knn_matches[k][0].distance < 200.0)
          {
            if (knn_matches[k][0].distance / knn_matches[k][1].distance < 0.7)
            {
              matches.push_back(knn_matches[k][0]);
            }
          }
        }
      }
      else
      {
        std::vector< cv::DMatch > simple_matches;
        matcher.match(images[i]->descriptors, images[j]->descriptors, simple_matches);
        // threashold test
        for (unsigned k = 0; k < simple_matches.size(); k++)
        {
          if (simple_matches[k].distance < 200.0)
          {
            matches.push_back(simple_matches[k]);
          }
        }
      }
      // all_matches.push_back(matches);
    }
  }
}

FeaturesMatcher::~FeaturesMatcher()
{
}

void FeaturesMatcher::setUseRatioTest(bool use_ratio_test_)
{
  use_ratio_test = use_ratio_test_;
}
