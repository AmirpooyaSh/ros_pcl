
/////////////////
// Point cloud utilities
//
// Written by:  Matthew Peavy
//              Kyungki Kim
// (c) 2021
//

#ifndef UNL_ROBOTICS_PCLUTILS_H
#define UNL_ROBOTICS_PCLUTILS_H

#include <pcl/point_cloud.h>

namespace UNL_Robotics {

  //Extracts the points from a point cloud that fall within the frame (pixel) coordinates.
  //Supply a source cloud which is read from and a target cloud which is written to.
  //Also, the frame coordinates (the "min" coordinates are the upper-left corner,
  // the "max" coords are the lower-right corner, in terms of pixels)
  template<typename PointType>
  void extractFrame(typename pcl::PointCloud<PointType>::ConstPtr sourceCloud,
                    typename pcl::PointCloud<PointType>::Ptr targetloud,
                    unsigned xmin, unsigned xmax,
                    unsigned ymin, unsigned ymax);
};

#include "pclUtils.ipp"

#endif

