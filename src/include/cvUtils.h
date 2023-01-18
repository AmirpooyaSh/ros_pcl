
/////////////////
// CV utilities
//
// Written by:  Matthew Peavy
//              Kyungki Kim
// (c) 2021
//

#ifndef UNL_ROBOTICS_CVUTILS_H
#define UNL_ROBOTICS_CVUTILS_H

//ROS
#include <sensor_msgs/Image.h>
//std
#include <string>

namespace UNL_Robotics {

  //Saves only the cropped images as a jpeg file
  void cropAndSaveImage(const sensor_msgs::Image& msg,
                        const std::string& croppedImagePath,
                        unsigned xmin, unsigned ymin,
                        unsigned x_delta, unsigned y_delta);

  //Saves the raw original (full) and cropped images as jpeg files
  void cropAndSaveImages(const sensor_msgs::Image& msg,
                         const std::string& fullImagePath,
                         const std::string& croppedImagePath,
                         unsigned xmin, unsigned ymin,
                         unsigned x_delta, unsigned y_delta);
  
};

#endif

