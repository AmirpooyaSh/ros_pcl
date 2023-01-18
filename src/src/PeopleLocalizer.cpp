/////////////////
// A very simple node to respond to YOLO/Darknet
// messages of people and localize them.
//
// Written by:  Matthew Peavy
//              Kyungki Kim
// (c) 2022
//

#include "PeopleLocalizer.h"
#include "gaussKernel.h"
#include "pclUtils.h"
#include "cvUtils.h"
#include "segmentation_pipeline.h"
#include <frustum.h>
//OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
//ROS
#include <ros/assert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>
//std
#include <cstdlib>
#include <string>
#include <sstream>
#include <iomanip>
//math
#include <math.h>

const unsigned QUEUE(100);                        //How many messages to allow to queue before discarding


// Segmentation Pipeline Parameters

//Plane segmentation (for removing floor)
//The higher the theshold, the more exact the plane must be. In Gazebo, this can be 0.99
// but not so high in the real world
const double NORMAL_THRESHOLD = 0.95;
UNL_Robotics::Normal FLOOR_PLANE_NORMAL = UNL_Robotics::Normal::eY;   //$VERIFY: Is there a Gazebo vs. real-world difference

//How must to crop around the bounding box
const double CROP_PERCENTAGE = 0.10;          // 0.00  to  0.20
      
//Statistical outlier removal parameters
const double MEAN_K = 50.0;                   // 50.0  to  100.0
const double STDDEV_MUL_THRESHOLD = 0.5;           // 0.5  to    1.0


namespace {

  std::string addS(unsigned val)
  {
    return (val != 1 ? "s" : "");
  }
  
}

///////////////

UNL_Robotics::PeopleLocalizer::PeopleLocalizer(const std::string& workingPath,
                                               const Pose& cameraPose,
                                               bool saveCroppedJpegs,
                                               bool saveCroppedPCDs)
  : m_out((workingPath + "/datalog.txt").c_str()),
    m_workingPath(workingPath),
    m_cameraPose(cameraPose),
    m_saveCroppedJpegs(saveCroppedJpegs),
    m_saveCroppedPCDs(saveCroppedPCDs)
    
{
  if(!m_out) {
    std::cout << "Output file not opened successfully. Must exit." << std::endl;
    exit(EXIT_FAILURE);
  }

  m_out << "Camera pose: " << m_cameraPose << std::endl << std::endl;
}

std::vector<UNL_Robotics::PeopleLocalizer::Pose>
UNL_Robotics::PeopleLocalizer::localizePeople(const sensor_msgs::PointCloud2& rosSensorCloud,
                                              const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbMsg,
                                              const sensor_msgs::Image& latestRGBImage)
{
  ROS_INFO_STREAM("Localizing people within cloud");

  //First convert the ROS sensor point cloud msg into a pcl::PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(rosSensorCloud, *pclCloud);

  std::vector<Pose> poses;     //Append the poses here
  
  unsigned numBoxes = bbMsg->bounding_boxes.size();
  ROS_INFO_STREAM("Working on " << numBoxes << " detected object" << addS(numBoxes) );

  //Save the full PCD
  pcl::PCDWriter pcdWriter;         //Used for all the PCD writing
  std::stringstream fullPCDPath;
  fullPCDPath << m_workingPath << "/fullPointCloud.pcd";
  pcdWriter.write<pcl::PointXYZ>(fullPCDPath.str(), *pclCloud, false);

  //Save the full jpeg image
  std::stringstream fullJpegPath;
  fullJpegPath << m_workingPath << "/fullImage.jpeg";
  cv_bridge::CvImagePtr fullImagePtr = cv_bridge::toCvCopy(latestRGBImage, sensor_msgs::image_encodings::BGR8);
  cv::imwrite(fullJpegPath.str(), fullImagePtr->image);  


  //Decide if we want to down-sample (reduce data points) for this image
  /*
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud_working(new pcl::PointCloud<pcl::PointXYZ>());
  if(m_useDownSampling) {
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(pclCloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*pclCloud_working);
  }
  else
    pclCloud_working = pclCloud;  //No down-sampling requested
  */
  
  //Iterate over all the objects identified by Yolo

  for(auto box : bbMsg->bounding_boxes) {
    
    std::string objectName = box.Class;
    ROS_INFO_STREAM("  " << objectName  << "  -  Probability " << std::setprecision(4) << (box.probability*100) << "%" );
    m_out << "*) Object type:  " << objectName << std::endl;
    m_out << "   Probability  " << std::setprecision(4) << (box.probability*100.0) << "%" << std::endl;

    //Continus working on object if it is a person
    if(objectName == "person") {

      //Start with the original cloud each time through the loop
      pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::copyPointCloud(*pclCloud, *newCloud);
      
      unsigned xmin = box.xmin;
      unsigned xmax = box.xmax;
      unsigned ymin = box.ymin;
      unsigned ymax = box.ymax;
      unsigned x_delta = xmax - xmin;
      unsigned y_delta = ymax - ymin;
      ROS_INFO_STREAM("    " << "BB Min (x,y) = (" << xmin << ", " << ymin << ")" );
      ROS_INFO_STREAM("    " << "BB Max (x,y) = (" << xmax << ", " << ymax << ")" );
      
      //Set up the segmentation and clustering pipeline
      ROS_INFO_STREAM("Running segmentation and clustering pipeline");

      //**‍*
      std::float_t Camera_Degree = 45;
      std::float_t Theta = (Camera_Degree/180)*M_PI;
      
      //***

      BoundingBox boundingBox{xmin, xmax, ymin, ymax};
      std::stringstream pipelinePath;
      pipelinePath << m_workingPath << "/person_" << std::setfill('0') << std::setw(2) << poses.size() << "_";
      SegmentationPipeline pipeline(pipelinePath.str(), boundingBox, newCloud);

      //Save the cropped RGB image
      cropAndSaveImage(latestRGBImage, pipelinePath.str() + "_croppedImage.jpeg", xmin, ymin, x_delta, y_delta);
      std::cout << "*) Saved cropped image" << std::endl;
      
      std::cout << "*) Raw data" << std::endl;
      pipeline.printMinMax();
      std::cout << std::endl;
      
      std::cout << "*) Extracting floor plane" << std::endl;
      pipeline.doPlaneExtraction(FLOOR_PLANE_NORMAL, NORMAL_THRESHOLD);

      /*
      std::cout << "*) Extracting bounding box, with crop = " << CROP_PERCENTAGE << " ...." << std::endl;
      pipeline.extractObjectInBoundingBox(CROP_PERCENTAGE);
      */

      std::cout << "*) Removing outliers...." << std::endl;
      pipeline.removeOutliers(MEAN_K, STDDEV_MUL_THRESHOLD);
      pipeline.printMinMax();
      std::cout << std::endl;
      
      std::cout << "*) Performing Euclidean extraction...." << std::endl;
      pipeline.performEuclideanExtraction();
      pipeline.printMinMax();
      std::cout << std::endl;
      
      std::cout << "*) Calculating distance average....";
      double dist = pipeline.calculateDepths();
      std::cout << "  Avg Depth = " << dist << std::endl;
      m_out << "   Average point distance from camera = " << dist << std::endl;

      //Using the distance and pose, calculate the position of the object
      std::cout << "*) Calculating distance average....";
      double objYaw = m_cameraPose.yaw + pipeline.calculateObjectAngleOffset();    //The object yaw is adjusted for its position within the image
      Pose personPose = { m_cameraPose.x + dist * cos(objYaw),
                          m_cameraPose.y + dist * sin(objYaw),
                          0.0 };
      m_out << "   Person pose (x,y,yaw) = " << personPose << std::endl << std::endl;
      poses.push_back(personPose);
      std::cout  << std::endl;
      
      std::cout << "=============================================" << std::endl;
      std::cout << "Finished segmentation and clustering pipeline" << std::endl;
    }
  }
  
  ROS_INFO_STREAM("PeopleLocalizer - finished processing cloud");
  return poses;
}

/////////////////////

std::ostream& operator<<(std::ostream& out, const UNL_Robotics::PeopleLocalizer::Pose& pose)
{
  out << "("  << pose.x << ", " << pose.y << ", " << pose.yaw << ")";
  return out;
}
