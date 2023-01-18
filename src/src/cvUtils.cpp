
/////////////////
// A function for cropping and saving (as jpeg files)
// an image from a ROS sensor (RGB) message.
// Saves both the cropped and original images.
//
// Written by:  Matthew Peavy
//              Kyungki Kim
// (c) 2021
//

#include "cvUtils.h"
//OpenCV
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//Saves only the cropped images as a jpeg file
void UNL_Robotics::cropAndSaveImage(const sensor_msgs::Image& msg,
                                    const std::string& croppedImagePath,
                                    unsigned xmin, unsigned ymin,
                                    unsigned x_delta, unsigned y_delta)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    //Covert the image from a ROS image message to a CV image
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    
    //Crop out the sub image, given the coordinates
    cv::Rect2d cropRect(xmin, ymin, x_delta, y_delta);
    cv::Mat croppedImage = cv_ptr->image(cropRect);
    imwrite(croppedImagePath, croppedImage);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR_STREAM("cv_bridge exception. What = " << e.what());
    return;
  }
}

//Saves the raw original (full) and cropped (cup only) images as jpeg files
void UNL_Robotics::cropAndSaveImages(const sensor_msgs::Image& msg,
                                     const std::string& fullImagePath,
                                     const std::string& croppedImagePath,
                                     unsigned xmin, unsigned ymin,
                                     unsigned x_delta, unsigned y_delta)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    //Covert the image from a ROS image message to a CV image
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    //Save the full image
    imwrite(fullImagePath, cv_ptr->image);
    
    //Crop out the sub image, given the coordinates
    cv::Rect2d cropRect(xmin, ymin, x_delta, y_delta);
    cv::Mat croppedImage = cv_ptr->image(cropRect);
    imwrite(croppedImagePath, croppedImage);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR_STREAM("cv_bridge exception. What = " << e.what());
    return;
  }
}
