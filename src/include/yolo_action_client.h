
#ifndef UNL_ROBOTICS_YOLO_ACTION_CLIENT_H
#define UNL_ROBOTICS_YOLO_ACTION_CLIENT_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
//Yolo
#include <darknet_ros_msgs/CheckForObjectsAction.h>
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/ObjectCount.h"


namespace UNL_Robotics {

  using ActionClient = actionlib::SimpleActionClient<darknet_ros_msgs::CheckForObjectsAction>;
  
  class YoloActionClient {
  public:
    YoloActionClient(ros::NodeHandle& nodeHandle);

    void test();

  private:
    ActionClient m_actionClient;

    //Publisher
    ros::Publisher m_rawImagePublisher;
    //ros::Publisher m_detectedImagePublisher;
    ros::Publisher m_cancelPublisher;

    //Subscribers
    ros::Subscriber m_objectDetectionSubscriber;
    ros::Subscriber m_detectionImageSubscriber;
    ros::Subscriber m_foundObjectSubscriber;

    void boundingBoxesCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
    void detectionImageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void foundObjectCallback(const darknet_ros_msgs::ObjectCount::ConstPtr& msg);
    
  };

}

#endif
