
//A node to load a CV image, convert it to a ROS sensor image
// and then broadcast it on the camera/rgb/image_raw topic

//CV includes
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
//ROS
#include <ros/ros.h>
//std
#include <cstdlib>
#include <string>


//Define a few constant strings for our node
const std::string IMAGE_PUBLISH_TOPIC("camera/rgb/image_raw");
const std::string IMAGE_FILEPATH("./images/keyboard.jpeg");


int main(int argc, char** argv)
{
  //Standard initialization
  ros::init(argc, argv, "cv_load_and_publish");
  ros::NodeHandle nodeHandle;
  ROS_INFO("Initializing the CVLoadAndPublish node");

  ///////
  //Load the image
  cv::Mat imageMatrix = cv::imread(IMAGE_FILEPATH);        //First load into a CV matrix
  std_msgs::Header header;                           //Create the message header
  header.stamp = ros::Time::now();
  cv_bridge::CvImage bridgeImage = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, imageMatrix);
  ROS_INFO_STREAM("Loaded the image " << IMAGE_FILEPATH);

  
  //////////
  //Convert the image
  sensor_msgs::ImagePtr sensorMsgImage = bridgeImage.toImageMsg();
  ROS_INFO("Converted the image to a sensor_msg image");

  
  //////////
  //Publish the image

  //Define a few variables
  ros::Duration durationToPublish(10.0);
  ros::Rate loop_rate(5);            //The rate in Hz (i.e., publish 5 times / second)
  ROS_INFO_STREAM("Publishing (for " << durationToPublish << " seconds) the image to topic: " << IMAGE_PUBLISH_TOPIC);
  ros::Publisher imagePublisher = nodeHandle.advertise<sensor_msgs::Image>(IMAGE_PUBLISH_TOPIC, 1);

  ros::Time begin = ros::Time::now();  //When we are beginning to publish

  //Loop while the elapsed time is less than the durationToPublish
  while((ros::Time::now() - begin) < durationToPublish) {
    imagePublisher.publish(*sensorMsgImage);    //The * converts from a point to an object
    ros::spinOnce();
    loop_rate.sleep();               //Keeps our timing of 5 Hz
  }
  

  //Done with demo
  ROS_INFO("The CVLoadAndPublish node is exiting successfully");
  return EXIT_SUCCESS;
}
