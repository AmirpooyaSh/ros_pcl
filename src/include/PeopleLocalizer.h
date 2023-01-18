/////////////////
// A node to recognize and localize people.
// Will listen for Yolo topics of recognized
// images and keep track of all people.
//
// Written by:  Matthew Peavy
//              Kyungki Kim
// (c) 2022
//

#ifndef UNL_ROBOTICS_PEOPLELOCALIZER_H
#define UNL_ROBOTICS_PEOPLELOCALIZER_H

//ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
//Darknet / Yolo
#include "darknet_ros_msgs/BoundingBoxes.h"
//std
#include <fstream>

namespace UNL_Robotics {
  
  class PeopleLocalizer {
  public:

    struct Pose {
      double x;
      double y;
      double yaw;
    };

    /////////
    
    PeopleLocalizer(const std::string& workingPath,
                    const Pose& cameraPose = Pose{0, 0, 0},
                    bool saveCroppedJpegs = true,
                    bool saveCroppedPCDs = true);

    void resetCameraPose(const Pose& newPose) {m_cameraPose = newPose;}
    void resetSaveCroppedJpegs(bool saveJpegs) {m_saveCroppedJpegs = saveJpegs;}
    void resetSaveCroppedPCDs(bool savePCDs) {m_saveCroppedPCDs = savePCDs;}
    
    //Perform the people localization. Returns a vector of poses of people identified by Yolo.
    std::vector<PeopleLocalizer::Pose> localizePeople(const sensor_msgs::PointCloud2& cloud,
                                                      const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbMsg,
                                                      const sensor_msgs::Image& latestRGBImage);
    
  private:
    std::ofstream m_out;         //Stream for datalog
    std::string m_workingPath;   // Cropped images & PCDs are saved here
    Pose m_cameraPose;
    bool m_saveCroppedJpegs;
    bool m_saveCroppedPCDs;
  };

}

/////////////////////

std::ostream& operator<<(std::ostream& out, const UNL_Robotics::PeopleLocalizer::Pose& pose);

#endif
