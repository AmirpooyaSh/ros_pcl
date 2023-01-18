/////////////////
// A node to respond to YOLO / darknet_ros
// messages of recognized objects 
//
// 
// ** Subscribe Topics **
// 
//    /camera/depth/points
//    /darknet_ros/bounding_boxes  <==  Including a possible prepended namesapce passed in on the CL
//   
// ** Broadcast Topics **
//
//    None
//    
//
// Written by:  Matthew Peavy
//              Kyungki Kim
// (c) 2022
//

#include "PeopleLocalizer.h"
//UNIX (for mkdir)
#include <sys/stat.h>
#include <sys/types.h>
//std
#include <cstdlib>
#include <fstream>
#include <ctime>

//Message Topics
const std::string TOPIC_DARKNET_BB("darknet_ros/bounding_boxes");
//const std::string TOPIC_DARKNET_DETECTION_IMAGE("darknet_ros/detection_image");  This is (apparently) not the Yolo image with bounding boxes imparted onto it
const std::string TOPIC_POINT_CLOUD("depth/points");  
const std::string TOPIC_RGB_IMAGE("rgb/image_raw");

ros::Duration MAX_WAIT(1.0); //Wait up to this amount of time for point cloud
const unsigned PUB_FREQ(10);   //What frequency to publish the position messages
const unsigned QUEUE(10);     //How many messages to allow to queue before discarding

namespace {

  std::vector<std::string> parseCL(int argc, char** argv)
  {
    return std::vector<std::string>(argv+1, argv + argc);
  }

  std::string dateTimestamp()
  {
    std::time_t now_time_t = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_time_t), "%Y%m%d_%H%M%S");
    return ss.str();
  }

  bool myMkdir(const std::string& dirName)
  {
    if(mkdir(dirName.c_str(), 0777) == -1)
      return false;
    else
      return true;
  }
  
}

class PeopleObserver {
public:
  PeopleObserver(ros::NodeHandle& nh, const std::string& cameraName, const std::string& workingPath)
  : m_cameraName(cameraName),
    //m_subscriber(nh.subscribe("/" + cameraName + "_ns/" + TOPIC_DARKNET_BB, QUEUE, &PeopleObserver::objectsDetected, this)),
    m_subscriber(nh.subscribe(TOPIC_DARKNET_BB, QUEUE, &PeopleObserver::objectsDetected, this)),
    m_localizer(workingPath, UNL_Robotics::PeopleLocalizer::Pose{0.0, 0.0, 0.0})
//  m_localizer(workingPath, UNL_Robotics::PeopleLocalizer::Pose{-1.6, 1.8, 3.14159})
//  m_localizer(workingPath, UNL_Robotics::PeopleLocalizer::Pose{-2.0, 4.0, 3.14159})
  {
  }

private:
  std::string m_cameraName;
  ros::Subscriber m_subscriber;
  UNL_Robotics::PeopleLocalizer m_localizer;

  void objectsDetected(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg_BB)
  {
    ROS_INFO_STREAM("Object detected");

    /*
    This is apparently not the Yolo image with bounding boxes imparted onto like I thought / wanted

    //Get the current Yolo detection image (the RGB image with bounding box imparted onto it)
    //  std::string yoloImageTopicName = "/" + m_cameraName + "_ns/" + TOPIC_DARKNET_DETECTION_IMAGE;
    std::string yoloImageTopicName = TOPIC_DARKNET_DETECTION_IMAGE;
    ROS_INFO_STREAM("Fetching Yolo detection image on topic " << yoloImageTopicName);    
    boost::shared_ptr<sensor_msgs::Image const> yoloImagePtr =
      ros::topic::waitForMessage<sensor_msgs::Image>(yoloImageTopicName, MAX_WAIT);
    if(yoloImagePtr == NULL) {
      ROS_ERROR_STREAM("No Yolo detection image messages received on " << yoloImageTopicName);
      return;
    }
    ROS_INFO_STREAM("Successfully fetched the Yolo detection image on topic " << yoloImageTopicName);
    */
    
    //Get the current RGB image
    //  std::string rgbImageTopicName = "/" + m_cameraName + "_ns/" + m_cameraName + "/" + TOPIC_RGB_IMAGE;
    std::string rgbImageTopicName = "/" + m_cameraName + "/" + TOPIC_RGB_IMAGE;
    
    ROS_INFO_STREAM("Fetching RGB image on topic " << rgbImageTopicName);    
    boost::shared_ptr<sensor_msgs::Image const> rgbImagePtr =
      ros::topic::waitForMessage<sensor_msgs::Image>(rgbImageTopicName, MAX_WAIT);
    if(rgbImagePtr == NULL) {
      ROS_ERROR_STREAM("No RGB image messages received on " << rgbImageTopicName);
      return;
    }
    ROS_INFO_STREAM("Successfully fetched the RGB image on topic " << rgbImageTopicName);

    //Get the current point cloud
    //  std::string pointCloudTopicName = "/" + m_cameraName + "_ns/" + m_cameraName + "/" + TOPIC_POINT_CLOUD;
    std::string pointCloudTopicName = "/" + m_cameraName + "/" + TOPIC_POINT_CLOUD;
    ROS_INFO_STREAM("Fetching point cloud on topic " << pointCloudTopicName);    
    boost::shared_ptr<sensor_msgs::PointCloud2 const> pointCloudPtr =
      ros::topic::waitForMessage<sensor_msgs::PointCloud2>(pointCloudTopicName, MAX_WAIT);
    if(pointCloudPtr == NULL) {
      ROS_ERROR_STREAM("No point clound messages received on " << pointCloudTopicName);
      return;
    }
    ROS_INFO_STREAM("Successfully fetched point cloud on topic " << pointCloudTopicName);

    std::vector<UNL_Robotics::PeopleLocalizer::Pose> poses = m_localizer.localizePeople(*pointCloudPtr, msg_BB, *rgbImagePtr);
    ROS_INFO_STREAM(poses.size() << " people localized");
  }

};


///// MAIN ////

int main(int argc, char** argv)
{
  ros::init(argc, argv, "people_observer");
  ROS_INFO("Initializing the people_observer node");

  //Check if there was a command-line arg. If so, it's the camera name
  std::vector<std::string> args = parseCL(argc, argv);
  std::string cameraName = "camera";
  if(args.size() > 0) {
    cameraName = args[0];
    ROS_INFO_STREAM("Command line request to set camera name to " << cameraName);
  }
  else
    cameraName = "camera";
  ROS_INFO_STREAM("Using camera name: " << cameraName);

  //Save all data wihtin a newly created directory that is the current date/time stamp
  std::string stampDirName = dateTimestamp();
  if(!myMkdir(stampDirName)) {
    ROS_ERROR_STREAM("Couldn't make a new directory with name: " << stampDirName << " - will continue with CWD");
    stampDirName = "./";
  }

  //Start the spinner
  uint32_t numThreads(4);
  ros::AsyncSpinner spinner(numThreads);
  spinner.start();
  
  //Start the people localzier
  ros::NodeHandle nh;
  PeopleObserver peopleObserver(nh, cameraName, stampDirName);

  //Rather than ros::spin(), use waitForShutdown() with the async spinner 
  ros::waitForShutdown();
  
  ROS_INFO("Finalizing the people_observer node");
  return EXIT_SUCCESS;
}
