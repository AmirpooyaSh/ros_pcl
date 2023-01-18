#include "frustum.h"
#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include <pcl/filters/frustum_culling.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <string.h>
#include <eigen3/Eigen/Eigen>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/assert.h>


const std::string TOPIC_DARKNET_BB("darknet_ros/bounding_boxes");
const std::string TOPIC_POINT_CLOUD("top/velodyne_points2");  
const std::string TOPIC_RGB_IMAGE("camera_left_front/camera_left_front/image_raw");

unsigned numBoxes;
darknet_ros_msgs::BoundingBoxes::ConstPtr Working_BB;


void cloud_cb(const sensor_msgs::PointCloud2& input) {
    
    ROS_INFO("PointCloud Received");
    sensor_msgs::PointCloud2 TempCloud = input;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(TempCloud,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ> m_Cloud;
    pcl::fromPCLPointCloud2(pcl_pc2,m_Cloud);
    
    UNL_Frustum::Frustum_Trim::Position Camera_Position;
    Camera_Position.X = 0.8;
    Camera_Position.Y = 2.7;
    Camera_Position.Z = 1.2;

    UNL_Frustum::Frustum_Trim::Position LiDAR_Position;
    LiDAR_Position.X = 0;
    LiDAR_Position.Y = 0;
    LiDAR_Position.Z = 3;


    //Astra Camera Horizontal : 60 | Vertical : 49.5 | Depth : 73
    //Modeled Camera Horizontal : 80
    //Astra Camera Pro => Same

    UNL_Frustum::Frustum_Trim Initial_Object(m_Cloud,49.5,80,0.2,25,Camera_Position,45,LiDAR_Position);

    for(auto& Box: Working_BB->bounding_boxes){
        
        //Optional Part, but Nessecary when you want to trim the YOLO detected part
        // Picture Width = 800 | Height = 600 (According to the camera URDF file)

        Initial_Object.SET_BB_VIEW(Box.xmin, Box.ymin, Box.xmax, Box.ymax, 800, 600);
        Initial_Object.Trim_Cloud();

        pcl::PointCloud<pcl::PointXYZ> Trimmed_Val;
        Trimmed_Val = Initial_Object.Return_Trimed_Cloud();

        std::string Classifier = Box.Class + ".pcd";
        pcl::io::savePCDFileASCII(Classifier,Trimmed_Val);
    }

}

void BB_Detection(const darknet_ros_msgs::BoundingBoxes::ConstPtr& Bounding) {

    boost::shared_ptr<sensor_msgs::PointCloud2 const> pointCloudPtr;

    if(Bounding ->bounding_boxes.size() != 0) {
        Working_BB = Bounding;
        ROS_INFO("BoundingBoxes Captured");
        numBoxes = Bounding->bounding_boxes.size();

        ros::Duration MaxWait(2.0);
        std::string pointCloudTopicName = "/" + TOPIC_POINT_CLOUD; 
        pointCloudPtr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(pointCloudTopicName, MaxWait);

        if(pointCloudPtr == NULL) {
            ROS_ERROR_STREAM("No PointCloud received" << pointCloudTopicName);
            return;
        }
    }
    else {
        ROS_INFO("Can't Connect to Yolo's Result");
        return;   
    }

    cloud_cb(*pointCloudPtr);
}

int main (int argc, char **argv) {

    ros::init(argc ,argv, "culling");

    ROS_INFO("Converter Node Started");

    ros::NodeHandle nh;
    ros::Subscriber Sub = nh.subscribe(TOPIC_DARKNET_BB, 10, BB_Detection);
    ros::spin();

}