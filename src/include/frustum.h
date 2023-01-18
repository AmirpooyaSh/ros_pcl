#ifndef UNL_FRUSTUM_FRUSTUM_H
#define UNL_FRUSTUM_FRUSTUM_H

#include "sensor_msgs/PointCloud2.h"
#include <pcl/filters/frustum_culling.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <ros/ros.h>
#include <string.h>

namespace UNL_Frustum {

class Frustum_Trim {
    
    public:

    struct Position
    {
        double X;
        double Y;
        double Z;
    };

        Frustum_Trim(pcl::PointCloud<pcl::PointXYZ> &Input_Cloud, double V_HOV, double H_HOV, double N_Plate, double F_Plate, 
                    Position Cam_Position, double Camera_Theta, Position LiD_Position);


        pcl::PointCloud<pcl::PointXYZ> Return_Trimed_Cloud() {return Target_Cloud;};
        void Transport_Matrix(Position Origin_Point ,Position Moving_Point ,pcl::PointCloud<pcl::PointXYZ> &Edit_Cloud);
        void Rotation_Matrix(float Theta, pcl::PointCloud<pcl::PointXYZ> &Edit_Cloud);
        void SET_Initial();
        void FIX_Origins(pcl::PointCloud<pcl::PointXYZ> &Edit_Cloud);
        void SET_Cam_Pose();
        void Trim_Cloud() { TC.filter(Target_Cloud);};
        void SET_BB_VIEW(float xmin, float ymin, float xmax, float ymax, float pic_width, float pic_height);
        void Remove_Surface(pcl::PointCloud<pcl::PointXYZ> &Cloud, Position LiDAR);
    private:

        pcl::PointCloud<pcl::PointXYZ>::Ptr Working_Cloud;
        pcl::PointCloud<pcl::PointXYZ> m_Cloud;
        pcl::FrustumCulling<pcl::PointXYZ> TC;
        float Vertical_HOV;
        float Horizontal_HOV;
        float Nearest_Plate;
        float Far_Plate;
        pcl::PointCloud<pcl::PointXYZ> Target_Cloud;
          
};

}
#endif