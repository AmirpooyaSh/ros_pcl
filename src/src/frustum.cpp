#include <frustum.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/filters/frustum_culling.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include "string.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <eigen3/Eigen/Eigen>

namespace {

  std::string addS(unsigned val)
  {
    return (val != 1 ? "s" : "");
  }
  
}

UNL_Frustum::Frustum_Trim::Frustum_Trim(pcl::PointCloud<pcl::PointXYZ> &Input_Cloud, double V_HOV, double H_HOV, double N_Plate, double F_Plate, 
    Position Cam_Position, double Camera_Theta, Position LiD_Position) {

        m_Cloud = Input_Cloud;

        Vertical_HOV=V_HOV;
        Horizontal_HOV=H_HOV;

        Nearest_Plate=N_Plate;
        Far_Plate=F_Plate;

        Remove_Surface(m_Cloud, LiD_Position);

        Transport_Matrix(LiD_Position, Cam_Position, m_Cloud);
        Rotation_Matrix(Camera_Theta, m_Cloud);

        FIX_Origins(m_Cloud);

        Working_Cloud = m_Cloud.makeShared();
        
        SET_Initial();
        SET_Cam_Pose();

        pcl::io::savePCDFileASCII("Initial.pcd",m_Cloud);

        //Trim_Cloud();
        
        //Rotation_Matrix(-Camera_Theta, Target_Cloud);
        //Transport_Matrix(Cam_Position, LiD_Position, Target_Cloud);

        std::cout<<"Frustum Culling General Initialization Done"<<std::endl;
}


//Public Transportation Matrix;
void UNL_Frustum::Frustum_Trim::Transport_Matrix(Position Origin_Point ,Position Moving_Point ,pcl::PointCloud<pcl::PointXYZ> &Edit_Cloud) {
            
    for(auto& point: Edit_Cloud) {

        point.x += (Origin_Point.X-Moving_Point.X);
        point.y += (Origin_Point.Y-Moving_Point.Y);
        point.z += (Origin_Point.Z-Moving_Point.Z);
    }
            
}

//Public Rotation Matrix;
void UNL_Frustum::Frustum_Trim::Rotation_Matrix(float Theta, pcl::PointCloud<pcl::PointXYZ> &Edit_Cloud) {
    double Correct_Theta = Theta*(M_PI/180);
    for(auto& point: Edit_Cloud) {

        float Temp_X = point.x;
        point.x = point.x*cos(Correct_Theta) + point.y*sin(Correct_Theta);
        point.y = Temp_X*(-sin(Correct_Theta)) + point.y*cos(Correct_Theta);
    }
}


void UNL_Frustum::Frustum_Trim::SET_Initial() {
    TC.setInputCloud(Working_Cloud);

    // You can Comment the Next 2 Lines if you don't want to use Yolo's Bounding box !
    //TC.setVerticalFOV(Vertical_HOV);
    //TC.setHorizontalFOV(Horizontal_HOV);

    
    TC.setNearPlaneDistance(Nearest_Plate);
    TC.setFarPlaneDistance(Far_Plate);
}


void UNL_Frustum::Frustum_Trim::FIX_Origins(pcl::PointCloud<pcl::PointXYZ> &Edit_Cloud) {
    for(auto& point: Edit_Cloud) {
        float Temp_Y=point.y;
        float Temp_Z=point.z;
        point.y = Temp_Z;
        point.z = -Temp_Y;  

    }
}


void UNL_Frustum::Frustum_Trim::SET_Cam_Pose() {
    Eigen::Matrix4f Camera_Matrix ;
    Camera_Matrix << 1, 0, 0, 0,
                     0, 1, 0, 0,
                     0, 0, 1, 0,
                     0, 0, 0, 1;
    TC.setCameraPose(Camera_Matrix);
}


void UNL_Frustum::Frustum_Trim::SET_BB_VIEW(float xmin, float ymin, float xmax, float ymax, float pic_width, float pic_height) {

    //Astra_Camera will take 640x480 Images, so for any other kind of camera => Change Dimensions here and in yolo3,cfg
    //Astra_Camera_Pro => 1280x720
    //Sizes should be the same as darknet/cfg/yolov3.cfg
    //Points should be normalized due to the frame's width and height in order to work properly
    /*
    double D_x = (xmax - xmin) / 640 ;
    double D_y = (ymax - ymin) / 480 ;

    double Center_x = (xmin-(640/2) + xmax-(640/2)) / (2 * 640);
    double Center_Y = ((-ymin+(480/2)) +(-ymax+(480/2))) / (2 * 480);
    */
    double Horizontal_Min_Angle = ((xmin-(pic_width / 2)) / (pic_width / 2) ) * 40;
    double Horizontal_Max_Angle = ((xmax-(pic_width / 2)) / (pic_width / 2) ) * 40;

    double Vertical_Max_Angle = ((-ymin+(pic_height / 2)) / (pic_height / 2) ) * 24.75;
    double Vertical_Min_Angle = ((-ymax+(pic_height / 2)) / (pic_height / 2) ) * 24.75;

    std::cout<<"Vertical Min = "<< Vertical_Min_Angle << "|" << "Vertical Max = "<< Vertical_Max_Angle << std::endl;
    std::cout<<"Horizontal Min = "<< Horizontal_Min_Angle << "|" << "Horizontal Max = "<< Horizontal_Max_Angle << std::endl;

    TC.setHorizontalFOV(Horizontal_Min_Angle,Horizontal_Max_Angle);
    TC.setVerticalFOV(Vertical_Min_Angle,Vertical_Max_Angle);
}

void UNL_Frustum::Frustum_Trim::Remove_Surface(pcl::PointCloud<pcl::PointXYZ> &Cloud, UNL_Frustum::Frustum_Trim::Position LiDAR) {
    //Removing the Surface !!
        
        for(auto& point: Cloud) {
            if (point.z+LiDAR.Z <= 0.12 && point.z+LiDAR.Z >= -0.12) {
                point.x = 0;
                point.y = 0;
                point.z = 0;
            }
        }
}