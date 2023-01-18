
#ifndef UNL_ROBOTICS_CONVEXHULL_H
#define UNL_ROBOTICS_CONVEXHULL_H

#include<vector>

namespace UNL_Robotics {

  struct Point2D {    //define points for 2d plane
     double x, y;
  };

  std::vector<Point2D> findConvexHull(std::vector<Point2D> points);
  
}

#endif

