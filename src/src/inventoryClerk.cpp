/////////////////
// A very simple node to respond to YOLO/Darknet
// messages of recognized objects 
// and keep an inventory of all objects recognized.
//
// 
// ** Subscribe Topics **
// 
//    /camera/depth/points
//    /camera/rgb/image_raw    <==  -OR- whatever is set in  setImageTopic()
//    /camera/depth/image
//    /darknet_ros/bounding_boxes
//    /darknet_ros/detection_image
//    /darknet_ros/found_object
//   
// ** Broadcast Topics **
// 
//    /camera/rgb/yolo
//
//    
//
// Written by:  Matthew Peavy
//              Kyungki Kim
// (c) 2021
//



// Note:  all of the calls to
//  
//    ROS_INFO_STREAM("m_pause = " << m_pause << "  :  m_YOLO_imageReceived = " << m_YOLO_imageReceived);
//
// are used for helping to debug and get the timing correct between Yolo and inventory clerk.
// Especially difficult trying to use Yolo as an action server.



#include "inventoryClerk.h"
#include "gaussKernel.h"
#include "pclUtils.h"
#include "cvUtils.h"
//OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//pcl
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
#include "ros/assert.h"
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
#include <ctime>
#include <thread>


const unsigned QUEUE(100);                        //How many messages to allow to queue before discarding

//The minimum angle to differentiae between people. If a person object is detected at least
// this far from another, it will be added to the m_peopleInRoom vector.
const double MIN_ANGLE_BETWEEN_PEOPLE(0.174);  //radians, roughly 10 degrees

//Topics
const std::string TOPIC_POINT_CLOUD("/camera/depth/points");
const std::string TOPIC_DARKNET_BB("/darknet_ros/bounding_boxes");
const std::string TOPIC_DARKNET_IMAGE("/darknet_ros/detection_image");
const std::string TOPIC_DARKNET_FOUND_OBJECT("/darknet_ros/found_object");
const std::string TOPIC_YOLO_IMAGE("/camera/rgb/yolo");
//The default image topic for archiving and feeding to Yolo, if none is
// passed as a CL-arg. This might change if, for example, you're using the
// "head" mounted camera for images (with turtlebot) while using the
// default camera for fake-laser navigation.
const std::string DEFAULT_IMAGE_TOPIC("/camera/rgb/image_raw");


//Astra camera specs
const unsigned CAMERA_NUM_PIXELS_WIDTH(640);
const unsigned CAMERA_NUM_PIXELS_HEIGHT(480);
const double   CAMERA_HORIZONTAL_VIEW_ANGLE(1.19);   //The angle (in radians) of what the camera views

//Yolo timing
const double   YOLO_IMAGE_REFRESH_LAG(11.3);    //seconds, approximate time between YOLO images on my system
const unsigned NUM_REFRESHES_UNTIL_CURRENT(3);  //Number of image refreshes until we have the current view

//For Gaussian convolution filter
const unsigned kernelRad(5);                    //The Gaussian convolution radius
const unsigned numSamplePts(kernelRad*2 + 1);   //How many sample points for depth sampling

//Plane segmentation
const unsigned DISTANCE_PRECISION(5);
const unsigned MAX_PLANE_COUNT(20);             //A maximum threshold for # of planes. Used only if something goes really wrong

namespace {

  std::string timestamp()
  {
    std::time_t now_time_t = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_time_t), "%Y%m%d_%H%M%S");
    return ss.str();
  }

  UNL_Robotics::InventoryClerk::Pose getCurrentPose()
  {
    //Capture the current robot location
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped currentPose;
    try{
      //Use a duration of a few seconds to give the buffer a chance to fill properly.
      //Otherwise you'll get (0, 0, nan) as your coords
      currentPose = tfBuffer.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(2.0));
    }
    catch(tf2::TransformException& ex)
    {
      ROS_ERROR_STREAM("tf2 tranform excpetion caught. What = " << ex.what());
    }
    
    geometry_msgs::Quaternion quat_msg = currentPose.transform.rotation;
    tf2::Quaternion quat_tf2;
    tf2::fromMsg(quat_msg , quat_tf2);
    tf2::Matrix3x3 mat(quat_tf2);
    double roll, pitch, yaw;
    mat.getEulerYPR(yaw, pitch, roll);
    return(UNL_Robotics::InventoryClerk::Pose{currentPose.transform.translation.x,
                                              currentPose.transform.translation.y,
                                              yaw}
           );
  }
  
}

///////////////

UNL_Robotics::InventoryClerk::InventoryClerk(ros::NodeHandle& nodeHandle,
                                             const std::string& workingPath,
                                             const std::string& inventoryFilename,
                                             bool beginPaused,
                                             bool useDownSampling)
  : //m_nodeHandle(nodeHandle),
    m_out((workingPath + inventoryFilename).c_str()),
    m_workingPath(workingPath),
    m_imageTopic(DEFAULT_IMAGE_TOPIC),
    m_pause(beginPaused),
    m_YOLO_imageReceived(false),
    m_useDownSampling(useDownSampling)
{
  std::cout << "IC 1" << std::endl;
  ROS_INFO_STREAM("IC " << 1);
  
  if(!m_out) {
    std::cout << inventoryFilename << " output file not opened successfully. "
              << "Likely you need to set ROS_HOME or launch this node from the facility_management_robot directory" << std::endl;
    exit(EXIT_FAILURE);
  }
  
  ROS_INFO_STREAM("Sampling [" << numSamplePts << " x " << numSamplePts << "] points on depth point cloud");
  
  m_objectDetectionSubscriber = nodeHandle.subscribe(TOPIC_DARKNET_BB, QUEUE, &InventoryClerk::objDetectionCallback, this);
  m_detectionImageSubscriber = nodeHandle.subscribe(TOPIC_DARKNET_IMAGE, QUEUE, &InventoryClerk::detectionImageCallback, this);
  m_foundObjectSubscriber = nodeHandle.subscribe(TOPIC_DARKNET_FOUND_OBJECT, QUEUE, &InventoryClerk::foundObjectCallback, this);  
  
  m_yoloImagePublisher = nodeHandle.advertise<sensor_msgs::Image>(TOPIC_YOLO_IMAGE, 1);
    
  ROS_INFO_STREAM("m_pause = " << m_pause << "  :  m_YOLO_imageReceived = " << m_YOLO_imageReceived);
}

void UNL_Robotics::InventoryClerk::setImageTopic(const std::string& imageTopic)
{
  m_imageTopic = imageTopic;  //Overrides default
  ROS_INFO_STREAM("Image topic for the inventory clerk reset to " << imageTopic);
}

void UNL_Robotics::InventoryClerk::beginInspection(unsigned roomNumber, const std::string& roomName)
{
  //Indicates in the log that we're starting at a new room
  m_out << "--===  Room " << roomNumber << ", " << roomName << "  ===--" << std::endl << std::endl;
  ROS_INFO_STREAM("Reset numPeople counter to 0 for this room");
  ROS_INFO_STREAM("InventoryClerk beginning inspection");
  ROS_INFO_STREAM("m_pause = " << m_pause << "  :  m_YOLO_imageReceived = " << m_YOLO_imageReceived);
  ROS_ASSERT(m_pause == true);

  //Also, clear the "people in room" vector so we can start a new count
  m_peopleInRoom.clear();
}

void UNL_Robotics::InventoryClerk::inspectPosition(unsigned positionNumber)
{
  ROS_INFO_STREAM("InventoryClerk - inspecting position");
  ROS_INFO_STREAM("m_pause = " << m_pause << "  :  m_YOLO_imageReceived = " << m_YOLO_imageReceived);
  m_out << "-- Observation point # " << positionNumber << " --" << std::endl;
    std::time_t now_time_t = std::time(nullptr);
  m_out << std::put_time(std::localtime(&now_time_t), "%b %d %Y,  %H:%M:%S") << std::endl;

  m_currentPose = getCurrentPose();
  m_out << "Current robot pose (x,y,yaw): " << m_currentPose << std::endl << std::endl;

  //Verify that we are set to pause
  ROS_ASSERT(m_pause == true);

  //Capture and store the latest depth image, RGB image, and point-cloud.
  //This is done here once, and for the remainder of this loop, all 3 of those
  // images will be used together. This guarantees that the images being processed
  // by Yolo match the depth images being used for depth information.
  //The RGB image will be continuously broadcast specially for Yolo consumption

  ros::Duration maxWait(1.0); //Wait up to this amount of time for images/point cloud
  
  boost::shared_ptr<sensor_msgs::Image const> rawImagePtr =
    ros::topic::waitForMessage<sensor_msgs::Image>(m_imageTopic, maxWait);
  if(rawImagePtr == NULL)
    ROS_ERROR_STREAM("No raw image messages received on " << m_imageTopic);
  else
    m_latestRGBImage = *rawImagePtr;

  boost::shared_ptr<sensor_msgs::PointCloud2 const> pointCloudPtr =
    ros::topic::waitForMessage<sensor_msgs::PointCloud2>(TOPIC_POINT_CLOUD, maxWait);
  if(pointCloudPtr == NULL)
    ROS_ERROR_STREAM("No point clound messages received on " << TOPIC_POINT_CLOUD);
  else
    m_latestPointCloud = *pointCloudPtr;

  ROS_INFO_STREAM("Most recent images captured");

  //Begin broadcasting the most recent RGB image for Yolo to pick up
  //Do this on a thread which will be destroyed at the end of this scope and
  // therefore stop broadcasting
  ROS_INFO("Launching broadcast image thread");
  std::thread broadcastThread(&InventoryClerk::broadcastLatestImage, this);
  ROS_INFO("Broadcast image thread launched");
  
  //Wait until Yolo catches up. Pause is currently true, so any object detection
  // callbacks are ignored during this nap time
  ROS_INFO_STREAM("Beginning nap for a time of " << (YOLO_IMAGE_REFRESH_LAG * NUM_REFRESHES_UNTIL_CURRENT) << " seconds");
  ros::Duration(YOLO_IMAGE_REFRESH_LAG * NUM_REFRESHES_UNTIL_CURRENT).sleep();

  //Now un-pause. We're ready to process whatever Yolo finds
  ROS_INFO_STREAM("Awoke from nap. Setting pause to false");
  m_pause = false;

  //Loop until the next object detection and processing has occurred
  do {
    ROS_INFO_STREAM("InventoryClerk in loop waiting for image to be processed. Waiting 1 second");
    ROS_INFO_STREAM("m_pause = " << m_pause << "  :  m_YOLO_imageReceived = " << m_YOLO_imageReceived);
    ros::spinOnce();
    ros::Duration(1.0).sleep();  //Sleep for a second
  }
  while(!m_YOLO_imageReceived);

  //Wait for the thread to finish
  broadcastThread.join();
  
  //Re-set to pause and image received status
  m_pause = true;
  m_YOLO_imageReceived = false;
}

void UNL_Robotics::InventoryClerk::recordMaxOccupancyResult(unsigned maxOccupancy)
{
  //Called when a room inspection is finished
  m_out << "---" << std::endl << std::endl;
  
  ROS_INFO_STREAM("Number of people detected in this room was " << m_peopleInRoom.size());
  m_out << "A total room occupancy of " << m_peopleInRoom.size() << " was recorded" << std::endl;
  
  if(m_peopleInRoom.size() <= maxOccupancy) {
    ROS_INFO_STREAM("Number of people within the room's maximum occupancy of " << maxOccupancy);
    m_out << "The number of people was within the maximum occupancy of " << maxOccupancy << std::endl;
  }
  else {
    ROS_INFO_STREAM("***WARNING*** Number of people EXCEED room's maximum occupancy of " << maxOccupancy);
    m_out << std::endl << "***WARNING*** The maximum occupancy of " << maxOccupancy << " was exceeded" << std::endl;
  }

  m_out << std::endl << "______________________________" << std::endl << std::endl << std::endl;
}

/////////// Private //////////

void UNL_Robotics::InventoryClerk::objDetectionCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
  ROS_INFO_STREAM("InventoryClerk - object detected, in callback");
  ROS_INFO_STREAM("m_pause = " << m_pause << "  :  m_YOLO_imageReceived = " << m_YOLO_imageReceived);
  
  if(m_pause) {
    ROS_INFO_STREAM("InventoryClerk - currently paused, so abandoning callback");
    ROS_INFO_STREAM("m_pause = " << m_pause << "  :  m_YOLO_imageReceived = " << m_YOLO_imageReceived);
    return;
  }

  ROS_INFO_STREAM("InventoryClerk - not paused so continuing with callback");
  ROS_INFO_STREAM("m_pause = " << m_pause << "  :  m_YOLO_imageReceived = " << m_YOLO_imageReceived);
  
  unsigned numBoxes = msg->bounding_boxes.size();
  std::string timeStamp = timestamp();
  ROS_INFO_STREAM( "Detected " << numBoxes << " object" << (numBoxes > 1 ? "s" : "") );
  unsigned itemNum = 0;
  for(auto box : msg->bounding_boxes) {
    std::string objectName = box.Class;
    
    unsigned xmin = box.xmin;
    unsigned xmax = box.xmax;
    unsigned ymin = box.ymin;
    unsigned ymax = box.ymax;
    unsigned x_delta = xmax - xmin;
    unsigned y_delta = ymax - ymin;
    ROS_INFO_STREAM("  " << objectName  << "  -  Probability " << std::setprecision(4) << (box.probability*100) << "%" );
    ROS_INFO_STREAM("    " << "BB Min (x,y) = (" << xmin << ", " << ymin << ")" );
    ROS_INFO_STREAM("    " << "BB Max (x,y) = (" << xmax << ", " << ymax << ")" );
    
    m_out << "*) Object type:  " << objectName << std::endl;
    m_out << "   Probability  " << std::setprecision(4) << (box.probability*100.0) << "%" << std::endl;

    //Calculate the angle offset of the picture relative to the center of the view port
    unsigned x_centerBB = xmin + static_cast<unsigned>(x_delta/2);
    unsigned y_centerBB = ymin + static_cast<unsigned>(y_delta/2);
    int x_offset = static_cast<unsigned>(CAMERA_NUM_PIXELS_WIDTH/2) - x_centerBB;   //Can be negative! This orientation assumes CCW=+
    double objectAngleOffset = CAMERA_HORIZONTAL_VIEW_ANGLE * (static_cast<double>(x_offset) / static_cast<double>(CAMERA_NUM_PIXELS_WIDTH));

    m_out << "   " << "Bounding Box (x,y):"
          << "   Min = (" << xmin << ", " << ymin << ")"
          << "   Max = (" << xmax << ", " << ymax << ")"
          << "   Center = (" << x_centerBB << ", " << y_centerBB << ")" << std::endl;
    m_out << "   In-image object angle offset = " << objectAngleOffset << " (rad)" << std::endl;

    //Add the person to the vector
    if(objectName == "person") {
      double personAngle = m_currentPose.yaw + objectAngleOffset;  //The object yaw is adjusted for its position within the image
      m_peopleInRoom.push_back(personAngle);
      ROS_INFO_STREAM("Person detected. Increasing number of people count to " << m_peopleInRoom.size());
    }

    
    //Now do the distance calculation

    //Convert the most recent ROS point cloud msg into a pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(m_latestPointCloud, *pclCloud);

    //Assert a few things about our point cloud and RGB images. They should match sizes.
    //If not, our bounding box values will not correspond
    ROS_ASSERT(m_latestRGBImage.height ==  pclCloud->height);
    ROS_ASSERT(m_latestRGBImage.width  ==  pclCloud->width);

    //Get the Gaussian kernels for gaussian sampling
    kernel_type gaussianKernel1 = produce2dGaussianKernel(kernelRad);  //Default mu/sigma values
    double mu = kernelRad;
    double sigma = 0.84089652;
    kernel_type gaussianKernel2 = produce2dGaussianKernel(kernelRad, mu, sigma);  //Tighter Gaussian curve
    
    //Sample
    std::vector<double> PCDepths;
    std::vector<double> PCDepths_weighted1;
    std::vector<double> PCDepths_weighted2;
    unsigned imageWidth = pclCloud->width;
    ROS_INFO_STREAM( "    " << "Point Cloud  width = " << pclCloud->width );
    const double x_tick = (static_cast<double>(x_delta) / numSamplePts);
    const double y_tick = (static_cast<double>(y_delta) / numSamplePts);
    ROS_ASSERT(x_tick >= 0);
    ROS_ASSERT(y_tick >= 0);
    for(unsigned row =0; row < numSamplePts; ++row) {
      for(unsigned col =0; col < numSamplePts; ++col) {
        unsigned x = xmin + static_cast<unsigned>(col * x_tick);
        unsigned y = ymin + static_cast<unsigned>(row * y_tick);
        ROS_ASSERT(x <= imageWidth);
        unsigned index = y*imageWidth + x;

        //For the unweighted point cloud depth (z-axis, from camera)
        double PCDepth = pclCloud->operator[](index).z;
        if(std::isnan(PCDepth))
          PCDepth = 0.0;
        PCDepths.push_back(PCDepth);

        //For the weighted point cloud values, apply a Gaussian convolution.

        //Gauss curve 1
        double gaussWeight1 = gaussianKernel1[row][col];              //Get the kernel weighting value
        PCDepths_weighted1.push_back(PCDepth * gaussWeight1);

        //Gauss curve 2
        double gaussWeight2 = gaussianKernel2[row][col];              //Get the kernel weighting value
        PCDepths_weighted2.push_back(PCDepth * gaussWeight2);
      }
    }

    
/*

    TEMPORARILY comment out the distance calcs for Hafiz's paper.

    
    //Apply filter to remove the max and min values (removes n-samples from each side)
    std::vector<double> PCDepths_filtered = truncate_n_return_middle_vals(PCDepths, numSamplePts);
    std::vector<double> PCDepths_dblFiltered = truncate_n_return_middle_vals(PCDepths, 2*numSamplePts);  // 2x number of samples
  
    //Raw
    std::pair<double, double> PCDepthMeanStdev = calcMeanAndStdDev(PCDepths);
    m_out << "   PCloud raw:" <<
             "   mean = " << PCDepthMeanStdev.first << "," <<
             "   std. deviation = " << PCDepthMeanStdev.second << std::endl;

    //Max-min filtered
    std::pair<double, double> PCDepthMeanStdev_filtered = calcMeanAndStdDev(PCDepths_filtered);
    m_out << "   PCloud filter:" <<
             "   mean = " << PCDepthMeanStdev_filtered.first << "," <<
             "   std. deviation = " << PCDepthMeanStdev_filtered.second << std::endl;

    //Max-min filtered with 2x
    std::pair<double, double> PCDepthMeanStdev_dblFiltered = calcMeanAndStdDev(PCDepths_dblFiltered);
    m_out << "   PCloud double filter:" <<
             "   mean = " << PCDepthMeanStdev_dblFiltered.first << "," <<
             "   std. deviation = " << PCDepthMeanStdev_dblFiltered.second << std::endl;

    //Gauss weight 1
    m_out << "   PCloud Gauss weight1 :" <<
             "   mean = " << std::accumulate(PCDepths_weighted1.begin(), PCDepths_weighted1.end(), 0.0) << std::endl;

    //Gauss weight 2
    m_out << "   PCloud Gauss weight2 :" <<
             "   mean = " << std::accumulate(PCDepths_weighted2.begin(), PCDepths_weighted2.end(), 0.0) << std::endl;

    //Using the distance and pose, calculate the position of the object
    double dist = std::accumulate(PCDepths_weighted2.begin(), PCDepths_weighted2.end(), 0.0);
    double objYaw = m_currentPose.yaw + objectAngleOffset;     //The object yaw is adjusted for its position within the image
    Pose objectPose = { m_currentPose.x + dist * cos(objYaw),
                        m_currentPose.y + dist * sin(objYaw),
                        0.0 };
    m_out << "   Object pose (x,y,yaw) = " << objectPose << std::endl;
*/
    
    m_out << std::endl;
    
    //Save the RGB image of the object as a jpeg file
    //First create the path based on the object name and its number in this series of bounding boxes
    // (in the case that there is more than 1 object of the same type in the image, this will be unique)
    std::stringstream ssObjPath;
    ssObjPath  << m_workingPath << timeStamp << "_" << itemNum << "_obj_" << objectName << ".jpeg";
    //Call the crop and save function. Save only the object in this loop
    cropAndSaveImage(m_latestRGBImage, ssObjPath.str(),
                     xmin, ymin, x_delta, y_delta);
    
    //Save the full room point cloud
    std::stringstream ssPCD;
    ssPCD << m_workingPath << timeStamp << "_fullPointCloud.pcd";
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>(ssPCD.str(), *pclCloud, false);
    
    //Extract the object PCL knowing the bounding box values
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cup_unfiltered(new pcl::PointCloud<pcl::PointXYZ>);
    //extractFrame<pcl::PointXYZ>(pclCloud, cloud_cup_unfiltered, imageWidth, xmin, xmax, ymin, ymax);
    //writer.write<pcl::PointXYZ>(workingPath + "pcd/" + std::to_string(m_numCupDetections) + "_cup.pcd", *cloud_cup_unfiltered, false);

    //Increment the item number
    ++itemNum;    
  }


  
/*
    This is older segmentation code that is not currently being used but may be soon.
    
    //If this is a cup, dump the whole point cloud within the bounding box to a new pcd file
    if(objectName == "cup") {
      
      //Save the RBG image of the cup as a jpeg file
      //First create the two paths based on the number of cup detections we have so far
      std::stringstream ssFullPath;
      ssFullPath << workingPath << "jpeg/" << "fullImage_" << m_numCupDetections << ".jpeg";
      std::stringstream ssCupPath;
      ssCupPath << workingPath << "jpeg/" << "cupImage_" << m_numCupDetections << ".jpeg";
      //Call the crop and save function
      cropAndSaveImage(m_latestRGBImage, ssFullPath.str(), ssCupPath.str(),
                xmin, ymin, x_delta, y_delta);
      
      //Save the calculated distanced values to the Distances.txt file
      m_out << std::fixed;
      m_out << std::setw(COL_WIDTH) << std::setprecision(DISTANCE_PRECISION) << PCDepthMeanStdev.first << ", " 
            << std::setw(COL_WIDTH) << std::setprecision(DISTANCE_PRECISION) << PCDepthMeanStdev_filtered.first << ", "
            << std::setw(COL_WIDTH) << std::setprecision(DISTANCE_PRECISION) << PCDepthMeanStdev_dblFiltered.first << ", "
            << std::setw(COL_WIDTH) << std::setprecision(DISTANCE_PRECISION) << std::accumulate(PCDepths_weighted1.begin(), PCDepths_weighted1.end(), 0.0) << ", "
            << std::setw(COL_WIDTH) << std::setprecision(DISTANCE_PRECISION) << std::accumulate(PCDepths_weighted2.begin(), PCDepths_weighted2.end(), 0.0) << std::endl;

      //Save the full room cloud
      pcl::PCDWriter writer;
      writer.write<pcl::PointXYZ>(workingPath + "pcd/" + std::to_string(m_numCupDetections) + "_fullRoom.pcd", *pclCloud, false);

      //Extract the cup PCL knowing the bounding box values
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cup_unfiltered(new pcl::PointCloud<pcl::PointXYZ>);
      extractFrame<pcl::PointXYZ>(pclCloud, cloud_cup_unfiltered, imageWidth, xmin, xmax, ymin, ymax);
      writer.write<pcl::PointXYZ>(workingPath + "pcd/" + std::to_string(m_numCupDetections) + "_cup.pcd", *cloud_cup_unfiltered, false);

      //Now do the filtering and segmentation (fault)
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cup_working(new pcl::PointCloud<pcl::PointXYZ>);

      //Decide if we want to down-sample (reduce data points) for this image
      if(m_useDownSampling) {
        // Create the filtering object: downsample the dataset using a leaf size of 1cm
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_cup_unfiltered);
        sor.setLeafSize(0.01f, 0.01f, 0.01f);
        sor.filter(*cloud_cup_working);
        writer.write<pcl::PointXYZ> (workingPath + "pcd/" + std::to_string(m_numCupDetections) + "_cup.downSampled.pcd", *cloud_cup_working, false);
      }
      else
        cloud_cup_working = cloud_cup_unfiltered;  //No down-sampling requested

      //Clean up the file by removing any NaNs, which is required by the Euclidean Extraction
      std::vector<int> indices;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cup_clean(new pcl::PointCloud<pcl::PointXYZ>);
      cloud_cup_working->is_dense = false;   //This is necessary for removeNanFromPointCloud() to work properly
      pcl::removeNaNFromPointCloud(*cloud_cup_working, *cloud_cup_clean, indices);
      std::cout << (cloud_cup_working->size() - cloud_cup_clean->size()) << " NaN lines removed from cup point cloud" << std::endl;
      writer.write<pcl::PointXYZ> (workingPath + "pcd/" + std::to_string(m_numCupDetections) + "_cup.NaNCleaned.pcd", *cloud_cup_clean, false);
      
      //////////////////////////////
      //Euclidean Cluster Extraction
      //
      // Creating the KdTree object for the search method of the extraction
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud(cloud_cup_clean);
      
      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance (0.01); // 2cm
      ec.setMinClusterSize (10);
      ec.setMaxClusterSize (25000);
      ec.setSearchMethod (tree);
      ec.setInputCloud(cloud_cup_clean);
      ec.extract(cluster_indices);
      std::cout << cluster_indices.size() << " clusters found in cleaned cup point-cloud." << std::endl;
      
      //Loop over all the clusters found
      for(auto it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for(auto pit = it->indices.begin (); pit != it->indices.end (); ++pit)
          cloud_cluster->push_back((*cloud_cup_clean)[*pit]);
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        
        std::cout << "Cluster extracted with " << cloud_cluster->size () << " data points." << std::endl;
        std::stringstream ss;
        ss << workingPath << "pcd/" + std::to_string(m_numCupDetections) + "_cup.cloudCluster_" << std::distance(cluster_indices.begin(), it) << ".pcd";
        writer.write<pcl::PointXYZ>(ss.str (), *cloud_cluster, false);
      }

      
      ///////////////////
      // Plane extraction
      //
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
      // Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      // Optional
      seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setMaxIterations(1000);
      seg.setDistanceThreshold(0.01);

      // Create the filtering object
      pcl::ExtractIndices<pcl::PointXYZ> extract;

      unsigned planeCount =0;
      const unsigned numPoints = cloud_cup_clean->size();
      
      // While 30% of the original cloud is still there
      while (cloud_cup_clean->size () > 0.3 * numPoints) {
        
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_cup_clean);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0) {
          std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
          break;
        }
        
        // Extract the inliers
        extract.setInputCloud (cloud_cup_clean);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_plane);
        std::cout << "PointCloud representing the planar component: "
                  << cloud_plane->width * cloud_plane->height
                  << " data points." << std::endl;
        
        std::stringstream ss;
        ss << workingPath << "pcd/" + std::to_string(m_numCupDetections) + "_cup.extractedPlane_" << planeCount << ".pcd";
        writer.write<pcl::PointXYZ>(ss.str (), *cloud_plane, false);
        
        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud_cup_clean.swap (cloud_f);

        //Put in a max threshold so if something goes wrong, we don't write a billion and 1 files
        if(planeCount > MAX_PLANE_COUNT) {
          std::cout << "Maximum threshold reached: plane count = " << MAX_PLANE_COUNT << ". Aborting." << std::endl;
          break;
        }
        
        ++planeCount;
      }

      //This is a cup so increment the counter
      ++m_numCupDetections;      
    }
  }

  //Deteremine if we're done
  if(m_numCupDetections >= MAX_CUP_DETECTIONS_BEFORE_QUIT) {
    std::cout << std::endl << std::endl
              << "Maximum number of cup detections (" << MAX_CUP_DETECTIONS_BEFORE_QUIT << ") was reached. Exiting."
              << std::endl << std::endl; 
    exit(EXIT_SUCCESS);
  }
*/

  ROS_INFO_STREAM("InventoryClerk - finished processing image");
  ROS_INFO_STREAM("m_pause = " << m_pause << "  :  m_YOLO_imageReceived = " << m_YOLO_imageReceived);
}

void UNL_Robotics::InventoryClerk::detectionImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  ROS_INFO_STREAM("InventoryClerk - detection image callback called");

  if(m_pause) {
    ROS_INFO_STREAM("InventoryClerk - currently paused. Returning");
    ROS_INFO_STREAM("m_pause = " << m_pause << "  :  m_YOLO_imageReceived = " << m_YOLO_imageReceived);
    return;
  }

  //Save off the full image for this observation
  std::string timeStamp = timestamp();
  std::stringstream ssFullPath;
  ssFullPath << m_workingPath << timeStamp << "_fullImage.jpeg";
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(m_latestRGBImage, sensor_msgs::image_encodings::BGR8);
    imwrite(ssFullPath.str(), cv_ptr->image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR_STREAM("cv_bridge exception. What = " << e.what());
    return;
  }
  ROS_INFO_STREAM("Saved full image to file: " << ssFullPath.str());
  
  //Set the yolo image received to indicate we just received an image during an un-paused period
  m_YOLO_imageReceived = true;

  //Indicate that the image was processed (i.e., captured and recorded)
  ROS_INFO_STREAM("m_pause = " << m_pause << "  :  m_YOLO_imageReceived = " << m_YOLO_imageReceived);
}

void UNL_Robotics::InventoryClerk::foundObjectCallback(const darknet_ros_msgs::ObjectCount::ConstPtr& msg)
{
  ROS_INFO_STREAM("InventoryCleck - Found object callback, count = " << static_cast<int>(msg->count));
}

void UNL_Robotics::InventoryClerk::broadcastLatestImage()
{
  //This function is generally called on a separate thread and allowed to loop until
  // the m_YOLO_imageReceived bool flag is flipped.
  
  double rate(10.0);  //Broadcast at 10 Hz
  ROS_INFO_STREAM("Thread - beginning broadcast of image at a rate of " << rate << " Hz");
  ROS_INFO_STREAM("m_pause = " << m_pause << "  :  m_YOLO_imageReceived = " << m_YOLO_imageReceived);

  ros::Rate loop_rate(rate);
  while(ros::ok()  &&  !m_YOLO_imageReceived)    //Until an image has been received during an un-paused period
  {
    m_yoloImagePublisher.publish(m_latestRGBImage);
    loop_rate.sleep();
  }
  
  ROS_INFO("Thread - finished broadcast of image");
  ROS_INFO_STREAM("m_pause = " << m_pause << "  :  m_YOLO_imageReceived = " << m_YOLO_imageReceived);
}

/////////////////////

std::ostream& operator<<(std::ostream& out, const UNL_Robotics::InventoryClerk::Pose& pose)
{
  out << "("  << pose.x << ", " << pose.y << ", " << pose.yaw << ")";
  return out;
}
