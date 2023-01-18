#ifndef UNL_ROBOTICS_SEGMENTATION_PIPELINE_H
#define UNL_ROBOTICS_SEGMENTATION_PIPELINE_H

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//std
#include <string>

namespace UNL_Robotics {

  struct BoundingBox {
    unsigned xmin;
    unsigned xmax;
    unsigned ymin;
    unsigned ymax;    
  };

  enum class Normal {eX =0, eY, eZ};
  
  class SegmentationPipeline {
  public:
    SegmentationPipeline(const std::string& baseName,
                         const BoundingBox& yoloIdentifiedBoundingBox,
                         pcl::PointCloud <pcl::PointXYZ>::Ptr pointCloud);
    SegmentationPipeline(const std::string& baseName,
                         const BoundingBox& yoloIdentifiedBoundingBox,
                         std::string pcdFilepath);

    //////////////
    //Calculations
    double calculateObjectAngleOffset() const;
    double calculateDepths() const;

    ////////////
    //Extractions

    //Extract a plane matching the normal passed in if the normal meets the threshold
    void doPlaneExtraction(Normal, double minThreshold);
    void doPlaneExtraction(Normal, double minThreshold, pcl::PointCloud<pcl::PointXYZ>::Ptr destination);
    
    //Pass in a percentage to crop from all borders.
    //E.g., passing 0.15 crops 15% off each edge, leaving the middle 70%,
    //      passing 0.0 crops exactly the Yolo-identified bounding box passed to constr
    void extractObjectInBoundingBox(double cropPercentage); 
    void extractObjectInBoundingBox(double cropPercentage, pcl::PointCloud<pcl::PointXYZ>::Ptr destination);
    
    //Use a statistical outlier remover
    void removeOutliers(double meanK, double stddevMulThresh);

    //Euclidean extractions.  The biggest cluster is set as the new working cloud
    void performEuclideanExtraction();

    //Calculate the 2D bounding box in world coordinates, given the current robot pose
    // and the camera angle theta (in radians)
    void calcBoundingBoxInWorldCoords(bool visualizeBB, double x, double y, double theta);
    void calcBoundingBoxInWorldCoords2(bool visualizeBB, double x, double y, double theta);
    
    //Print the minimum and maximum points of the entire cloud at this point in the pipeline
    void printMinMax();
    void printMinMax(pcl::PointCloud<pcl::PointXYZ>::Ptr);
    
  private:
    pcl::PCDWriter m_writer;
    unsigned m_pipelineStepCount;  //What step are we in the pipeline. Major steps by 10, minor by 1
    std::string m_baseName;      //The base file name we will append and save to
    BoundingBox m_boundingBox;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;

    void removeNaNs(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr dest);
  };

}

#endif
