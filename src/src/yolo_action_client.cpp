
//An attempt to use the Action Client functionality of Yolo.
// This doesn't work as I intend.


#include "yolo_action_client.h"
//ROS
#include <actionlib/client/terminal_state.h>
//Yolo
#include <darknet_ros_msgs/CheckForObjectsResult.h>
//CV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
//std
#include <cstdlib>
#include <sstream>
#include <iostream>
#include <future>        //These 3 are for the non-blocking cin
#include <thread>
#include <chrono>

//This is the name used to set up the action server, so we need the same for the client
const std::string actionServerName("/darknet_ros/check_for_objects");

//This is the image topic we're publishing the raw image to
const std::string yoloPublisherTopic("/camera/yolo_test");
//const std::string yoloPublisherTopic("/camera/rgb/image_raw");

//The topic we're publishing the detected image on
//const std::string detectedImagePublisherTopic("/camera/yolo_detected");

//This is the cancel topic
const std::string yoloCancelTopic("/darknet_ros/check_for_objects/cancel");

//The path to the images
const std::string imagePath = "inventory/";

//How many messages to allow to queue before discarding
const unsigned QUEUE(10);  



static std::string getKey()
{    
    std::string key;
    std::cin >> key;
    return key;
}


//////////

UNL_Robotics::YoloActionClient::YoloActionClient(ros::NodeHandle& nodeHandle)
  : m_actionClient(actionServerName, true)

{
  ROS_INFO("Waiting for action server to start");
  m_actionClient.waitForServer();
  ROS_INFO("Action server started");

  //Publisher
  m_rawImagePublisher = nodeHandle.advertise<sensor_msgs::Image>(yoloPublisherTopic, 1);
  m_cancelPublisher = nodeHandle.advertise<actionlib_msgs::GoalID>(yoloCancelTopic, 1);
  //m_detectedImagePublisher = nodeHandle.advertise<sensor_msgs::Image>(detectedImagePublisherTopic, 1);
  
  //Subscribers
  std::string bbTopic("/darknet_ros/bounding_boxes");
  m_objectDetectionSubscriber = nodeHandle.subscribe(bbTopic, QUEUE, &YoloActionClient::boundingBoxesCallback, this);

  std::string detectionImageTopic("/darknet_ros/detection_image");  
  m_detectionImageSubscriber = nodeHandle.subscribe(detectionImageTopic, QUEUE, &YoloActionClient::detectionImageCallback, this);
  
  std::string foundObjectTopic("/darknet_ros/found_object");  
  m_foundObjectSubscriber = nodeHandle.subscribe(foundObjectTopic, QUEUE, &YoloActionClient::foundObjectCallback, this);  

  
}

void UNL_Robotics::YoloActionClient::test()
{

  for(unsigned i=0; i<10; ++i) {

    ROS_INFO_STREAM("-----------------");
    ROS_INFO_STREAM("Image #" << i);
    ROS_INFO_STREAM(" ");
      
    //Load the full image into a CV matrix
    std::stringstream ss;
    ss << imagePath << "image_0" << i << ".jpeg";
    cv::Mat imageMatrix = cv::imread(ss.str());
    ROS_INFO_STREAM("Loaded image " << ss.str(););    
    
    //Set up the ROS message
    std_msgs::Header header;
    header.seq = i;
    header.stamp = ros::Time::now();
    
    //Create a proper cv_bridge image that will be used for conversion to a ROS message
    cv_bridge::CvImage bridgeImage = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, imageMatrix);
    sensor_msgs::ImagePtr sensorMsgImage = bridgeImage.toImageMsg();

    //Broadcast the raw image - not used for Yolo, but so that the user can see it
    ros::Duration durationToPublish(3.0);
    ros::Rate loop_rate(5);              // Hz
    ros::Time begin = ros::Time::now();  //When we are beginning to publish
    while((ros::Time::now() - begin) < durationToPublish) {
      m_rawImagePublisher.publish(*sensorMsgImage);
      ros::spinOnce();
      loop_rate.sleep();               //Keeps our timing of 5 Hz
    }
    ROS_INFO_STREAM("Published raw image to topic: " << yoloPublisherTopic);

    //Set up the goal
    darknet_ros_msgs::CheckForObjectsGoal goal;
    goal.id = i;                             //Is this the object ID? in which case 0 = person, 41 = cup
    goal.image = *sensorMsgImage;

/*    
    //Pre-empt whatever Yolo is currently doing
    if(i == 0) {
      ROS_INFO("Sending initial goal to action server");    
      m_actionClient.sendGoalAndWait(goal,ros::Duration(30.0));
      //bool finished_before_timeout = m_actionClient.waitForResult(ros::Duration(30.0));
      //ros::Duration(12).sleep();
      continue;
    }
    else {
      actionlib_msgs::GoalID cancel;
      //m_cancelPublisher.publish(cancel);
      //m_actionClient.cancelGoalsAtAndBeforeTime(ros::Time::now());
      //m_actionClient.cancelAllGoals();
      m_actionClient.cancelGoal();
      ROS_INFO("Canceled all action server goals");
    }
*/
    
    //Send a goal (new image) to the Yolo action server
    ROS_INFO("Sending goal to action server");    
    m_actionClient.sendGoal(goal);
    
    //wait for the action to return
    bool finished_before_timeout = m_actionClient.waitForResult(ros::Duration(30.0));
    
    m_actionClient.sendGoalAndWait(goal,ros::Duration(30.0));
    if(finished_before_timeout) {
      actionlib::SimpleClientGoalState state = m_actionClient.getState();
      ROS_INFO_STREAM("Action finished: " << state.toString());
      darknet_ros_msgs::CheckForObjectsResultConstPtr result = m_actionClient.getResult();
      std::cout << "Bounding box count:  " << result->bounding_boxes.bounding_boxes.size() << std::endl;
      for(int bb=0; bb<result->bounding_boxes.bounding_boxes.size(); ++bb)
        std::cout << "Bounding box #" << bb << std::endl << result->bounding_boxes.bounding_boxes[bb] << std::endl;
    }
    else
      ROS_INFO("Action did not finish before the time out.");

    ros::Duration(120).sleep();  

    
 /*
    //Do a non-blocking cin
    std::chrono::seconds timeout(30);
    std::cout << "Any key for next image: " << std::flush;
    std::string key = "c";
    std::future<std::string> future = std::async(getKey);
    if (future.wait_for(timeout) == std::future_status::ready)
        key = future.get();
 */
    
  }
}

void UNL_Robotics::YoloActionClient::boundingBoxesCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
  ROS_INFO_STREAM("yolo_action_client - bounding boxes");
  unsigned i=1;
  for(auto box : msg->bounding_boxes) {
    std::string objectName = box.Class;
    ROS_INFO_STREAM("  " << i << " - " << objectName);
    ++i;
  }  
}

void UNL_Robotics::YoloActionClient::detectionImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  ROS_INFO_STREAM("yolo_action_client - detection image, in callback");

}

void UNL_Robotics::YoloActionClient::foundObjectCallback(const darknet_ros_msgs::ObjectCount::ConstPtr& msg)
{
  int count =  msg->count;
  ROS_INFO_STREAM("yolo_action_client - found object.  Count = " << count);
}

///////////////////////

int main (int argc, char **argv)
{
  ros::init(argc, argv, "yolo_action_client");
  ros::NodeHandle nodeHandle;
  ROS_INFO("Initializing the yolo action client");

  UNL_Robotics::YoloActionClient client(nodeHandle);
  client.test();
  
  ros::spin();

  ROS_INFO("Shutting down the yolo action client");
  ros::shutdown();
  
  return EXIT_SUCCESS;
}

