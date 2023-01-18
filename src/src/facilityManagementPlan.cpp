
#include "facilityManagementPlan.h"
//gmfXML
#include "gmfXML/gmfXMLDocument.h"
//ROS
#include <ros/ros.h>
//std
#include <fstream>
#include <vector>
#include <regex>
#include <cassert>

namespace {

  
  std::vector<UNL_Robotics::Room> parseFacility(std::istream& in)
  {
    std::vector<UNL_Robotics::Room> facility;

    //Load the room coordinate information
    std::string line;
    unsigned roomID;
    std::string roomName;
    double x,y, yaw(0.0);
    unsigned maxOccupancy;

    //Throw away the first two lines. 1st is the title, second is divider.
    std::getline(in, line);
    std::getline(in, line);
    while(std::getline(in, line)) {

      //Skip any blank lines (which is often the case at the end of the file)
      if(line.empty())
        break;
      
      //Tokenize the string based on the pipe |  character
      const std::regex re{"[\\|]+"};
      std::vector<std::string> tokens{
          std::sregex_token_iterator(line.begin(), line.end(), re, -1),
          std::sregex_token_iterator()
      };

      //The first token is a throw away, since it is up to the first pipe | (or nothing)
      // so we should have 6 real fields.
      if(tokens.size() != 6) {
        ROS_ERROR_STREAM("Malformed facility file. Line = " << line);
        std::cout << "Malformed facility file. Line = " << line << std::endl;
        std::cout << "Tokens.size() = " << tokens.size() << std::endl;
        for(auto token : tokens)
          std::cout << token << std::endl;
        std::cout << std::endl;
        exit(EXIT_FAILURE);
      }

      //Skip the blank 0
      std::stringstream ss1, ss3, ss4, ss5;
      ss1 << tokens[1];
      ss1 >> roomID;
      roomName = tokens[2];  //No need for stringstream conversion here
      ss3 << tokens[3];
      ss3 >> x;
      ss4 << tokens[4];
      ss4 >> y;
      ss5 << tokens[5];
      ss5 >> maxOccupancy;
      
      facility.push_back(UNL_Robotics::Room{roomID,
                                            roomName,
                                            UNL_Robotics::Position_2D{x, y, yaw},
                                            maxOccupancy});
    }
    
    return facility;
  }

}

UNL_Robotics::RotationDirection UNL_Robotics::parseRotationDirection(const std::string& direction)
{
  if(direction == "CCW")
    return RotationDirection::eCCW;
  else if(direction == "CW")
    return RotationDirection::eCW;
  else
    throw(std::runtime_error("Rotation direction string unrecognized: " + direction));
}


///////////


UNL_Robotics::RoomRoute::RoomRoute(const Room& room, unsigned numRotations)
  : m_room(room), m_numRotations(numRotations)
{
}

void UNL_Robotics::RoomRoute::print(std::ostream& out) const
{
  out << "Room" << std::endl;
  out << "  Room ID:  " << m_room.roomID << std::endl;
  out << "  Room Name:  " << m_room.roomName << std::endl;
  out << "  Location:  ("
      << m_room.roomCenter.x << "  "
      << m_room.roomCenter.y << ")" << std::endl;
  out << "  Maximum Occupancy: " << m_room.maxOccupancy << std::endl;
  out << "  Number of Rotations: " << m_numRotations << std::endl;
  out << std::endl;
}

////////////////////

std::tuple<bool, UNL_Robotics::InspectionRoute>
UNL_Robotics::parseInspectionRoute(std::istream& in)
{
  ROS_INFO("Starting parsing of inspection route");
  
  namespace gmfXML = GiveMeFish::XMLlib;

  gmfXML::XMLDocument doc(in);
  gmfXML::XMLElement root = doc.getRootElement();
  gmfXML::XMLElement plan = root.getNode(0);
  gmfXML::XMLElement defaultParamsNode = plan.getNode(0);
  gmfXML::XMLElement facilityDescriptionNode = plan.getNode(1);
  gmfXML::XMLElement inspectionRouteNode = plan.getNode(2);

  /*
  defaultParams params{};
  
  std::stringstream ss1;
  ss1 << paramsNode.getNode(0).getValue();
  ss1 >> std::boolalpha >> params.moveArm_;

  std::stringstream ss2;
  ss2 << paramsNode.getNode(1).getValue();
  ss2 >> std::boolalpha >> params.displayStroke_;

  std::stringstream ss3;
  ss3 << paramsNode.getNode(2).getValue();
  ss3 >> params.strokeWidth_;
  */

  //Facility file information
  std::string facilityFilePath = facilityDescriptionNode.getNode(0).getValue();
  std::string facilityFilename = facilityDescriptionNode.getNode(1).getValue();
  
  //Load the accompanying facility file
  std::string facilityFileFullPath = facilityFilePath + "/" + facilityFilename;
  std::ifstream in_ff(facilityFileFullPath.c_str());
  if(!in_ff) {
    ROS_ERROR_STREAM("Can't open facility file with path: " << facilityFileFullPath << "  -- Consider setting the ROS_HOME environment variable so that the management file can be loaded, or running launch script from the catkin workspace prepended with:   ROS_HOME=`pwd`");
    ROS_FATAL("No facility file available, so must exit");
    exit(EXIT_FAILURE);
  }
  ROS_INFO_STREAM("Opened facility file stream with path: " << facilityFileFullPath);
  
  std::vector<Room> facility = parseFacility(in_ff);
  ROS_INFO_STREAM("Loaded facility file with path: " << facilityFilePath);
  ROS_INFO_STREAM("Parsed a total of " << facility.size() << " facility rooms");
  
  //Add all room routes to this inspection
  InspectionRoute inspectionRoute;
  
  //Iterate over the room routes in the plan
  gmfXML::XMLElement roomRoutesNode = inspectionRouteNode.getNode(0);
  for(gmfXML::XMLElement::const_iterator roomIter = roomRoutesNode.begin();
      roomIter != roomRoutesNode.end(); ++roomIter) {

    //Get out the room ID and coords
    unsigned roomNumber;
    Position_2D location;
    unsigned numRotations;
    
    std::stringstream ss4, ss5, ss6, ss7;
    ss4 << roomIter->getNode(0).getValue();
    ss4 >> roomNumber;

    //We'll need to look up room information in the facility based on the room ID
    auto si = std::find_if(facility.begin(), facility.end(),
                           [roomNumber](Room room)->bool { return room.roomID == roomNumber; }
                           );
    if(si == facility.end()) {
      ROS_ERROR_STREAM("ERROR - can't find room #" << roomNumber << " in facility information.");
      return std::tuple<bool, InspectionRoute>(false, inspectionRoute);
    }
    
    gmfXML::XMLElement locationNode = roomIter->getNode(1);
    if(locationNode.size() == 0) {
      
      //Then this should be a room center. Look it up from the facility info
      location = si->roomCenter;
    }
    else {
      //Otherwise it has a specific location
      ss5 << locationNode.getNode(0).getValue();
      ss5 >> location.x;
      ss6 << locationNode.getNode(1).getValue();
      ss6 >> location.y;
    }

    gmfXML::XMLElement numRotationsNode = roomIter->getNode(2);
    ss7 << numRotationsNode.getValue();
    ss7 >> numRotations;

    //Now add the room route to the inspection route
    Room room{ roomNumber, si->roomName, location, si->maxOccupancy };
    inspectionRoute.push_back(RoomRoute(room, numRotations));
  }

  ROS_INFO_STREAM("Finished parsing. Total room routes parsed = " << inspectionRoute.size());
  
  return std::tuple<bool, InspectionRoute>(true, inspectionRoute);
}

/*
std::ostream& operator<<(std::ostream& out, const UNL_Robotics::PaintSimulationParams& params)
{
  //Simulation Parameters
  out << "Move_arm: " << std::boolalpha << params.moveArm_ << std::endl;
  out << "Display_stroke: " << std::boolalpha << params.displayStroke_ << std::endl;
  out << "Stroke_width: " << std::boolalpha << params.strokeWidth_ << std::endl << std::endl;
  return out;
}
*/

std::ostream& operator<<(std::ostream& out, const UNL_Robotics::InspectionRoute& inspectionRoute)
{
  //Print all the walls
  for(auto roomRoute : inspectionRoute)
    roomRoute.print(out);

  return out;
}
  
