
#include "segmentation_pipeline.h"
#include "pclUtils.h"
#include "cvUtils.h"
#include "gaussKernel.h"
#include "convexHull.h"
//OpenCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//pcl
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
//PCL - PCA
#include <pcl/common/pca.h>
#include <pcl/features/moment_of_inertia_estimation.h>
//PCL - visualization
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
//std
#include <sstream>


//Camera specs
const unsigned CAMERA_NUM_PIXELS_WIDTH(640);
const unsigned CAMERA_NUM_PIXELS_HEIGHT(480);
const double   CAMERA_HORIZONTAL_VIEW_ANGLE(1.19);   //The angle (in radians) of what the camera views

//For Gaussian convolution filter
const unsigned KERNEL_RADIUS(5);                    //The Gaussian convolution radius
const unsigned NUM_GAUSS_SAMPLE_PTS(KERNEL_RADIUS*2 + 1);   //How many sample points for depth sampling

//PCL plane extraction - a hard threshold, especially for if something goes wrong or lots of small (insignificant) planes
const unsigned MAX_PLANE_COUNT(8);
const double PLANE_EXTRACTION_CONTINUE_THRESHOLD(0.30);   // While 30% of the original cloud is still there

//Euclidean clustering
double CLUSTER_TOLERANCE(0.10);
unsigned MIN_CLUSTER_SIZE(10);


UNL_Robotics::SegmentationPipeline::SegmentationPipeline(const std::string& baseName,
                                                         const BoundingBox& yoloIdentifiedBoundingBox,
                                                         pcl::PointCloud <pcl::PointXYZ>::Ptr pointCloud)
  : m_pipelineStepCount(10),
    m_baseName(baseName),
    m_boundingBox(yoloIdentifiedBoundingBox),
    m_cloud(pointCloud)
{
}

UNL_Robotics::SegmentationPipeline::SegmentationPipeline(const std::string& baseName,
                                                         const BoundingBox& yoloIdentifiedBoundingBox,
                                                         std::string pcdFilepath)
  : m_pipelineStepCount(10),
    m_baseName(baseName),
    m_boundingBox(yoloIdentifiedBoundingBox),
    m_cloud(new pcl::PointCloud<pcl::PointXYZ>)
{
  //Read in the pointcloud from file
  pcl::PCDReader reader;
  reader.read(pcdFilepath, *m_cloud);
}

double UNL_Robotics::SegmentationPipeline::calculateObjectAngleOffset() const
{
  unsigned x_delta = m_boundingBox.xmax - m_boundingBox.xmin;
  unsigned y_delta = m_boundingBox.ymax - m_boundingBox.ymin;
  
  //Calculate the angle offset of the picture relative to the center of the view port
  unsigned x_centerBB = m_boundingBox.xmin + static_cast<unsigned>(x_delta/2);
  unsigned y_centerBB = m_boundingBox.ymin + static_cast<unsigned>(y_delta/2);
  int x_offset = static_cast<unsigned>(CAMERA_NUM_PIXELS_WIDTH/2) - x_centerBB;   //Can be negative! This orientation assumes CCW=+
  double objectAngleOffset = CAMERA_HORIZONTAL_VIEW_ANGLE * (static_cast<double>(x_offset) / static_cast<double>(CAMERA_NUM_PIXELS_WIDTH));

  std::cout << "   " << "Bounding Box (x,y):"
            << "   Min = (" << m_boundingBox.xmin << ", " << m_boundingBox.ymin << ")"
            << "   Max = (" << m_boundingBox.xmax << ", " << m_boundingBox.ymax << ")"
            << "   Center = (" << x_centerBB << ", " << y_centerBB << ")" << std::endl;
  std::cout << "   In-image object angle offset = " << objectAngleOffset << " (rad)" << std::endl;

  return objectAngleOffset;
}

double UNL_Robotics::SegmentationPipeline::calculateDepths() const  //of despair
{
  unsigned x_delta = m_boundingBox.xmax - m_boundingBox.xmin;
  unsigned y_delta = m_boundingBox.ymax - m_boundingBox.ymin;

  //Get the Gaussian kernels for gaussian sampling
  UNL_Robotics::kernel_type gaussianKernel1 = UNL_Robotics::produce2dGaussianKernel(KERNEL_RADIUS);  //Default mu/sigma values
  double mu = KERNEL_RADIUS;
  double sigma = 0.84089652;
  UNL_Robotics::kernel_type gaussianKernel2 = UNL_Robotics::produce2dGaussianKernel(KERNEL_RADIUS, mu, sigma);  //Tighter Gaussian curve
    
  //Sampling

  //Get all the remaining point-cloud depths
  std::vector<double> PCDepths;
  for(unsigned i=0; i< m_cloud->size(); ++i)
    PCDepths.push_back(m_cloud->operator[](i).z);

  /*
  std::vector<double> PCDepths_weighted1;
  std::vector<double> PCDepths_weighted2;
  unsigned cloudWidth = m_cloud->width;
  std::cout << "    " << "Point Cloud  width = " << cloudWidth;
  const double x_tick = (static_cast<double>(x_delta) / NUM_GAUSS_SAMPLE_PTS);
  const double y_tick = (static_cast<double>(y_delta) / NUM_GAUSS_SAMPLE_PTS);
  for(unsigned row =0; row < NUM_GAUSS_SAMPLE_PTS; ++row) {
    for(unsigned col =0; col < NUM_GAUSS_SAMPLE_PTS; ++col) {
      unsigned x = m_boundingBox.xmin + static_cast<unsigned>(col * x_tick);
      unsigned y = m_boundingBox.ymin + static_cast<unsigned>(row * y_tick);
      unsigned index = y*CAMERA_NUM_PIXELS_WIDTH + x;

      //For the unweighted point cloud depth (z-axis, from camera)
      double PCDepth = m_cloud->operator[](index).z;
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
  */

  
  //Apply filter to remove the max and min values (removes n-samples from each side)
  std::vector<double> PCDepths_filtered = truncate_n_return_middle_vals(PCDepths, NUM_GAUSS_SAMPLE_PTS);
  std::vector<double> PCDepths_dblFiltered = truncate_n_return_middle_vals(PCDepths, 2*NUM_GAUSS_SAMPLE_PTS);  // 2x number of samples
  
  //Raw
  std::pair<double, double> PCDepthMeanStdev = calcMeanAndStdDev(PCDepths);
  cout << "   PCloud raw:" <<
    "   mean = " << PCDepthMeanStdev.first << "," <<
    "   std. deviation = " << PCDepthMeanStdev.second << std::endl;
  
  //Max-min filtered
  std::pair<double, double> PCDepthMeanStdev_filtered = calcMeanAndStdDev(PCDepths_filtered);
  cout << "   PCloud filter:" <<
    "   mean = " << PCDepthMeanStdev_filtered.first << "," <<
    "   std. deviation = " << PCDepthMeanStdev_filtered.second << std::endl;
  
  //Max-min filtered with 2x
  std::pair<double, double> PCDepthMeanStdev_dblFiltered = calcMeanAndStdDev(PCDepths_dblFiltered);
  cout << "   PCloud double filter:" <<
    "   mean = " << PCDepthMeanStdev_dblFiltered.first << "," <<
    "   std. deviation = " << PCDepthMeanStdev_dblFiltered.second << std::endl;

  /*
  //Gauss weight 1
  cout << "   PCloud Gauss weight1 :" <<
    "   mean = " << std::accumulate(PCDepths_weighted1.begin(), PCDepths_weighted1.end(), 0.0) << std::endl;
  
  //Gauss weight 2
  cout << "   PCloud Gauss weight2 :" <<
    "   mean = " << std::accumulate(PCDepths_weighted2.begin(), PCDepths_weighted2.end(), 0.0) << std::endl;
  */
  
  //Using the distance and pose, calculate the position of the object
  //double dist = std::accumulate(PCDepths_weighted2.begin(), PCDepths_weighted2.end(), 0.0);
  //double objYaw = m_currentPose.yaw + calculateObjectAngleOffset();     //The object yaw is adjusted for its position within the image  
  
  //Pose objectPose = { m_currentPose.x + dist * cos(objYaw),
  //                    m_currentPose.y + dist * sin(objYaw),
  //                    0.0 };
  //cout << "   Object pose (x,y,yaw) = " << objectPose << std::endl;

  return PCDepthMeanStdev.first;
}

void UNL_Robotics::SegmentationPipeline::doPlaneExtraction(Normal normal, double minThreshold)
{
  doPlaneExtraction(normal, minThreshold, m_cloud);
}

void UNL_Robotics::SegmentationPipeline::doPlaneExtraction(Normal normal, double minThreshold, pcl::PointCloud<pcl::PointXYZ>::Ptr destination)
{
  // Plane extraction
  //
  pcl::PointCloud<pcl::PointXYZ>::Ptr final_planeless_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  copyPointCloud(*m_cloud, *final_planeless_cloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_for_plane_extraction(new pcl::PointCloud<pcl::PointXYZ>);
  copyPointCloud(*m_cloud, *cloud_for_plane_extraction);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.01);

  // Create the extracting object (extraction based on indices)
  pcl::ExtractIndices<pcl::PointXYZ> extract;
    
  //Keep the removed points as NaN values to maintain the structure of the cloud
  extract.setKeepOrganized(true);
    
  unsigned planeCount =0;
  const unsigned numOriginalPoints = cloud_for_plane_extraction->size();   //How many points we originally started with
  unsigned numPointsExtracted =0;   //Keep track of how many points have so far been extracted

  //Do until we extract a certain percentage of the points -or-
  // until the maximum number of planes is met, which is checked at the end of this loop
  while(numPointsExtracted < numOriginalPoints * (1.0 - PLANE_EXTRACTION_CONTINUE_THRESHOLD)) {
      
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_for_plane_extraction);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size () == 0) {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }
    
    //Accumulate the number of points extracted for this plane
    numPointsExtracted += inliers->indices.size();
      
    // Extract the inliers into a cloud that contains exactly (and only) the plane
    extract.setInputCloud(cloud_for_plane_extraction);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_plane);

    std::cout << "   - Plane #" << planeCount << std::endl;
    std::cout << "     PointCloud representing the planar component: "
              << inliers->indices.size() << " data points." << std::endl;

    std::stringstream ss;
    ss << m_baseName << "_step" << m_pipelineStepCount << "_plane_" << planeCount << ".pcd";
    m_writer.write<pcl::PointXYZ>(ss.str(), *cloud_plane, false);

    ////////////
    //Calculate the plane normals - used to determine if the plane is a floor plane

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

    //Calculate the least-squares plane value
    float nx, ny, nz, curvature;
    ne.computePointNormal(*cloud_for_plane_extraction, inliers->indices, nx, ny, nz, curvature);

    //Now calc each normal
    ne.setInputCloud(cloud_plane);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    
    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.03);
    
    // Compute the features
    ne.compute(*cloud_normals);
    std::cout << "     Least-squares plane fit: (" << nx << ", " << ny << ", " << nz << ")"
              << ", curvature = " << curvature << std::endl;

    bool minThresholdMet = false;
    if((normal == Normal::eX) && (fabs(nx) > minThreshold))
      minThresholdMet = true;
    if((normal == Normal::eY) && (fabs(ny) > minThreshold))
      minThresholdMet = true;
    if((normal == Normal::eZ) && (fabs(nz) > minThreshold))
      minThresholdMet = true;

    if(minThresholdMet)
    {
      std::cout << endl;
      std::cout << "     ==> Minimum threshold of " << minThreshold << " met. Extracting this plane (#" << planeCount << "). <==" << std::endl;
      std::cout << endl;
      
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_minus_plane(new pcl::PointCloud<pcl::PointXYZ>);
      
      // Create another filtering object 
      pcl::ExtractIndices<pcl::PointXYZ> normalPlaneExtractor;
      normalPlaneExtractor.setInputCloud(final_planeless_cloud);
      normalPlaneExtractor.setIndices(inliers);
      normalPlaneExtractor.setNegative(true);
    
      //Keep the removed points as NaN values to maintain the structure of the cloud
      normalPlaneExtractor.setKeepOrganized(true);

      normalPlaneExtractor.filter(*cloud_minus_plane);

      std::cout << "final_planeless_cloud" << std::endl;
      std::cout << "width = " << final_planeless_cloud->width << std::endl;
      std::cout << "height = " << final_planeless_cloud->height << std::endl;

      std::cout << "cloud_minus_plane" << std::endl;
      std::cout << "width = " << cloud_minus_plane->width << std::endl;
      std::cout << "height = " << cloud_minus_plane->height << std::endl;

      //If this file already exists (i.e., part of the floor was removed in an earlier pass),
      // it will be over written here, and so just the updated, most recent version (with
      // the least amount of floor plane) will remain.
      std::stringstream ss;
      ss << m_baseName << "_step" << m_pipelineStepCount+1 << "_floorless_cloud.pcd";
      m_writer.write<pcl::PointXYZ>(ss.str(), *cloud_minus_plane, false);
      
      //Make this is our new cloud
      copyPointCloud(*cloud_minus_plane, *final_planeless_cloud);
    }

    //Report the number of points extracted after this plane extraction
    std::cout << "     " << numPointsExtracted << " points extracted so far (of a total "
              << numOriginalPoints << ")." << std::endl << std::endl;
    
    // Remove the plane from our working cloud
    extract.setNegative(true);
    extract.filter(*cloud_f);
    cloud_for_plane_extraction.swap(cloud_f);

    //Increment the current plane count
    ++planeCount;
    
    //Put in a max number of planes so if something goes wrong, we don't write a billion and 1 files
    if(planeCount >= MAX_PLANE_COUNT) {
      std::cout << std::endl << "Maximum threshold reached: plane count = " << MAX_PLANE_COUNT << ". Forcing end of plane extraction." << std::endl;
      break;
    }
  }
  
  std::stringstream ss;
  ss << m_baseName << "_step" << m_pipelineStepCount+2 << "_postPlaneExtraction.pcd";
  m_writer.write<pcl::PointXYZ>(ss.str(), *final_planeless_cloud, false);

  std::cout << "Min-max after extracting plane:" << std::endl;
  printMinMax(final_planeless_cloud);
  
  //Copy this into the destination cloud
  copyPointCloud(*final_planeless_cloud, *destination);
  
  m_pipelineStepCount += 10;
}


void UNL_Robotics::SegmentationPipeline::extractObjectInBoundingBox(double cropPercentage)
{
  extractObjectInBoundingBox(cropPercentage, m_cloud);
}

void UNL_Robotics::SegmentationPipeline::extractObjectInBoundingBox(double cropPercentage, pcl::PointCloud<pcl::PointXYZ>::Ptr destination)
{
  //Extract the object PCL knowing the bounding box values, possibly with an additional cropping border (reduced by the crop percentage)
  unsigned x_delta = ( m_boundingBox.xmax - m_boundingBox.xmin ) ;
  unsigned y_delta = ( m_boundingBox.ymax - m_boundingBox.ymin ) ;
  unsigned cloudWidth = m_cloud->width;

  std::cout << "Cloud width = " << cloudWidth << std::endl;
  std::cout << "Cloud height = " << m_cloud->height << std::endl;
  std::cout << "Crop percentage = " << (cropPercentage * 100) << "%" << std::endl;
  std::cout << "BB xmin = " << m_boundingBox.xmin << std::endl;
  std::cout << "BB xmax = " << m_boundingBox.xmax << std::endl;
  std::cout << "BB ymin = " << m_boundingBox.ymin << std::endl;
  std::cout << "BB ymax = " << m_boundingBox.ymax << std::endl;
  std::cout << "BB xmin, cropped = " << m_boundingBox.xmin + static_cast<unsigned>(x_delta * cropPercentage) << std::endl;
  std::cout << "BB xmax, cropped = " << m_boundingBox.xmax - static_cast<unsigned>(x_delta * cropPercentage) << std::endl;
  std::cout << "BB ymin, cropped = " << m_boundingBox.ymin + static_cast<unsigned>(y_delta * cropPercentage) << std::endl;
  std::cout << "BB ymax, cropped = " << m_boundingBox.ymax - static_cast<unsigned>(y_delta * cropPercentage) << std::endl;
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCrop(new pcl::PointCloud<pcl::PointXYZ>);
  UNL_Robotics::extractFrame<pcl::PointXYZ>(m_cloud, cloudCrop,
                              m_boundingBox.xmin + static_cast<unsigned>(x_delta * cropPercentage),
                              m_boundingBox.xmax - static_cast<unsigned>(x_delta * cropPercentage),
                              m_boundingBox.ymin + static_cast<unsigned>(y_delta * cropPercentage),
                              m_boundingBox.ymax - static_cast<unsigned>(y_delta * cropPercentage));

  std::stringstream ss;
  ss << m_baseName << "_step" << m_pipelineStepCount << "_extractBBcrop" << std::setprecision(2) << std::fixed << cropPercentage << ".pcd";
  m_writer.write<pcl::PointXYZ>(ss.str(), *cloudCrop, false);

  std::cout << "Min-max after extracting bounding box:" << std::endl;
  printMinMax(cloudCrop);
  
  //Copy this into the destination cloud
  copyPointCloud(*cloudCrop, *destination);
  
  m_pipelineStepCount += 10;
}
  
void UNL_Robotics::SegmentationPipeline::removeOutliers(double meanK, double stddevMulThresh)
{
  //Remove outliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_removedOutliers(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(m_cloud);
  sor.setMeanK(meanK);
  sor.setStddevMulThresh(stddevMulThresh);
  sor.filter(*cloud_removedOutliers);
  
  std::stringstream ss;
  ss << m_baseName << "_step" << m_pipelineStepCount << "_removedOutliers.pcd";
  m_writer.write<pcl::PointXYZ>(ss.str(), *cloud_removedOutliers, false);

  //Make this is our new working cloud
  copyPointCloud(*cloud_removedOutliers, *m_cloud);

  m_pipelineStepCount += 10;
}

void UNL_Robotics::SegmentationPipeline::performEuclideanExtraction()
{
  //Euclidean Cluster Extraction

  //Start by removing NaNs, if they exist
  pcl::PointCloud<pcl::PointXYZ>::Ptr nanlessCloud(new pcl::PointCloud<pcl::PointXYZ>);
  removeNaNs(m_cloud, nanlessCloud);
  copyPointCloud(*nanlessCloud, *m_cloud);
  
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(m_cloud);
    
  std::vector<pcl::PointIndices> cluster_indices;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

  std::cout << "Cluster tolearance set to: " << CLUSTER_TOLERANCE << std::endl;
  ec.setClusterTolerance(CLUSTER_TOLERANCE);

  std::cout << "Minimum cluster size set to: " << MIN_CLUSTER_SIZE << std::endl;
  ec.setMinClusterSize(MIN_CLUSTER_SIZE);

  //This is set based on the bounding box size
  unsigned maxClusterSize = (m_boundingBox.xmax - m_boundingBox.xmin) * (m_boundingBox.ymax - m_boundingBox.ymin);
  ec.setMaxClusterSize(maxClusterSize);
  std::cout << "Maximum cluster size set to: " << maxClusterSize << std::endl;
  ec.setSearchMethod(tree);
  std::cout << "Set search method to tree" << std::endl;
  m_cloud->is_dense = false;
  ec.setInputCloud(m_cloud);
  std::cout << "Set input cloud" << std::endl;
  ec.extract(cluster_indices);
  std::cout << "   " << cluster_indices.size() << " cluster" << (cluster_indices.size() != 1 ? "s" : "")
            << " found in pointcloud." << std::endl;
  
  //For the file name, set the cluster number width (so that numbers are formatted as  07  if there are more than 10)
  unsigned clusterNumberWidth = floor(log10(cluster_indices.size())) + 1;
  
  //Loop over all the clusters found
  for(auto it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for(auto pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->push_back((*m_cloud)[*pit]);
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    clusters.push_back(cloud_cluster);
    std::cout << "    - Cluster extracted with " << cloud_cluster->size () << " data points." << std::endl;
    std::stringstream ss;
    ss << m_baseName << "_step" << m_pipelineStepCount << "_euclideanCluster_" << std::setfill('0') 
       << std::setw(clusterNumberWidth) << std::distance(cluster_indices.begin(), it) << ".pcd";
    m_writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false);
  }
    
  //Make the biggest cluster (cluster_indices[0]) our new working cloud  
  if(cluster_indices.size() > 0) {
    copyPointCloud(*clusters[0], *m_cloud);
    
    /* Do this if we ever figure out how to get cluster extraction to work on the NaN-full cloud
      
      pcl::ExtractIndices<pcl::PointXYZ> clusterExtractor;
      clusterExtractor.setInputCloud(m_cloud);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices(cluster_indices[0]));
      clusterExtractor.setIndices(inliers);
      clusterExtractor.setNegative(true);
      //Keep the removed points as NaN values to maintain the structure of the cloud
      clusterExtractor.setKeepOrganized(true);
      
      pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
      clusterExtractor.filter(*clusterCloud);
      copyPointCloud(*clusterCloud, *m_cloud);
    */
  }
  
  m_pipelineStepCount += 10;  
}

void UNL_Robotics::SegmentationPipeline::calcBoundingBoxInWorldCoords(bool visualizeBB, double x, double y, double theta)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudSegmented = m_cloud;
  
  // Compute principal directions (equivalent to principal components / PCA)
  Eigen::Vector4f pcaCentroid;
  pcl::compute3DCentroid(*cloudSegmented, pcaCentroid);


  //Two methods: compute Eigen vectors, or use a PCA object

  //Method 1
  Eigen::Matrix3f covariance;
  computeCovarianceMatrixNormalized(*cloudSegmented, pcaCentroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
  //The following line is necessary for proper orientation in some cases.
  //The numbers come out the same without it, but the signs are
  //   different and the box doesn't get correctly oriented in some cases.
   eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  
  
/*
   //Method 2
   // Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PCA<pcl::PointXYZ> pca;
   pca.setInputCloud(cloudSegmented);
   pca.project(*cloudSegmented, *cloudPCAprojection);
   Eigen::Matrix3f eigenVectorsPCA = pca.getEigenVectors();
*/


  
  // Transform the original cloud to the origin where the principal components correspond to the axes.
  Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
  projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
  projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*cloudSegmented, *cloudPointsProjected, projectionTransform);
  // Get the minimum and maximum points of the transformed cloud.
  pcl::PointXYZ minPoint, maxPoint;
  pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
  const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

  // For the transformation back to world coordinates
  const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
  const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

  std::stringstream ss;
  ss << m_baseName << "_" << m_pipelineStepCount << "_PCA.pcd";
  m_writer.write<pcl::PointXYZ>(ss.str(), *cloudSegmented, false);

  std::cout << "PCA:" << std::endl;
  std::cout << eigenVectorsPCA << std::endl;
  std::cout << "Bounding box transform:" << std::endl;
  std::cout << bboxTransform << std::endl;

  Eigen::Vector3f p1(minPoint.x, minPoint.y, minPoint.z);
  Eigen::Vector3f p2(minPoint.x, minPoint.y, maxPoint.z);
  Eigen::Vector3f p3(maxPoint.x, minPoint.y, maxPoint.z);
  Eigen::Vector3f p4(maxPoint.x, minPoint.y, minPoint.z);
  Eigen::Vector3f p5(minPoint.x, maxPoint.y, minPoint.z);
  Eigen::Vector3f p6(minPoint.x, maxPoint.y, maxPoint.z);
  Eigen::Vector3f p7(maxPoint.x, maxPoint.y, maxPoint.z);
  Eigen::Vector3f p8(maxPoint.x, maxPoint.y, minPoint.z);

  std::cout << "Points - untransformed" << std::endl;
  std::cout << "P1" << std::endl << p1  << std::endl;
  std::cout << "P2" << std::endl << p2  << std::endl;
  std::cout << "P3" << std::endl << p3  << std::endl;
  std::cout << "P4" << std::endl << p4  << std::endl;
  std::cout << "P5" << std::endl << p5  << std::endl;
  std::cout << "P6" << std::endl << p6  << std::endl;
  std::cout << "P7" << std::endl << p7  << std::endl;
  std::cout << "P8" << std::endl << p8  << std::endl;

  // Transform back to world coordinates
  Eigen::Vector3f pr1 = bboxQuaternion * p1 + bboxTransform;
  Eigen::Vector3f pr2 = bboxQuaternion * p2 + bboxTransform;
  Eigen::Vector3f pr3 = bboxQuaternion * p3 + bboxTransform;
  Eigen::Vector3f pr4 = bboxQuaternion * p4 + bboxTransform;
  Eigen::Vector3f pr5 = bboxQuaternion * p5 + bboxTransform;
  Eigen::Vector3f pr6 = bboxQuaternion * p6 + bboxTransform;
  Eigen::Vector3f pr7 = bboxQuaternion * p7 + bboxTransform;
  Eigen::Vector3f pr8 = bboxQuaternion * p8 + bboxTransform;

  std::cout << std::endl << std::endl;
  std::cout << "Points with rotation/translation" << std::endl;
  std::cout << "P1" << std::endl << pr1  << std::endl;
  std::cout << "P2" << std::endl << pr2  << std::endl;
  std::cout << "P3" << std::endl << pr3  << std::endl;
  std::cout << "P4" << std::endl << pr4  << std::endl;
  std::cout << "P5" << std::endl << pr5  << std::endl;
  std::cout << "P6" << std::endl << pr6  << std::endl;
  std::cout << "P7" << std::endl << pr7  << std::endl;
  std::cout << "P8" << std::endl << pr8  << std::endl;

  //Project onto flat 2D space (in this case, x-z of the camera view), taking the
  // maximum and minimum x,z points

  std::vector<Point2D> points;
  points.push_back({pr1[0], pr1[2]});
  points.push_back({pr2[0], pr2[2]});
  points.push_back({pr3[0], pr3[2]});
  points.push_back({pr4[0], pr4[2]});
  points.push_back({pr5[0], pr5[2]});
  points.push_back({pr6[0], pr6[2]});
  points.push_back({pr7[0], pr7[2]});
  points.push_back({pr8[0], pr8[2]});


  /*  
  std::vector<Point2D> points = {{-7,8},{-4,6},{2,6},{6,4},{8,6},{7,-2},{4,-6},{8,-7},{0,0},
                                 {3,-2},{6,-10},{0,-6},{-9,-5},{-8,-2},{-8,0},{-10,3},{-2,2},
                                 {-10,4}, {-10.1, 3.5}};
  */
  std::vector<Point2D> result = findConvexHull(points);
  std::cout << "Input points are: " << std::endl;
  std::for_each(points.begin(), points.end(),
                [](Point2D point) {std::cout << point.x << ", " << point.y << std::endl;});
  std::cout << "Boundary points of convex hull are: "<<endl;
  std::for_each(result.begin(), result.end(),
                [](Point2D point) {std::cout << point.x << ", " << point.y << std::endl;});

  
  
  ///////////////////////////////////

  if(visualizeBB) {
    // This viewer has 4 windows, but is only showing images in one of them as written here.
    int argc = 1;
    char** argv;
    pcl::visualization::PCLVisualizer *visu = new pcl::visualization::PCLVisualizer(argc, argv, "PlyViewer");
    int mesh_vp_1, mesh_vp_2, mesh_vp_3, mesh_vp_4;
    //visu->createViewPort(0.0, 0.5, 0.5, 1.0,  mesh_vp_1);
    //visu->createViewPort(0.5, 0.5, 1.0, 1.0,  mesh_vp_2);
    //visu->createViewPort(0.0, 0, 0.5, 0.5,  mesh_vp_3);
    //visu->createViewPort(0.5, 0, 1.0, 0.5, mesh_vp_4);
    //visu->addPointCloud(cloudSegmented, ColorHandlerXYZ(cloudSegmented, 30, 144, 255), "bboxedCloud", mesh_vp_3);
    
    visu->createViewPort(0.0, 0.0, 2.0, 2.0, mesh_vp_3);
    visu->addPointCloud(cloudSegmented, "bboxedCloud", mesh_vp_3);
    visu->addCube(bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z, "bbox", mesh_vp_3);
    
    while(!visu->wasStopped())
    {
      visu->spinOnce();
    }
  }
}

void UNL_Robotics::SegmentationPipeline::calcBoundingBoxInWorldCoords2(bool visualizeBB, double x, double y, double theta)
{
  typedef pcl::PointXYZ PointType;
  
	pcl::PointCloud<PointType>::Ptr cloud = m_cloud;
 
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloud, pcaCentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); //correct vertical between main directions
	eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));
 
	 std::cout << "Eigenvalue va(3x1):\n" << eigenValuesPCA << std::endl;
	 std::cout << "Feature vector ve(3x3):\n" << eigenVectorsPCA << std::endl;
	 std::cout << "centroid point (4x1):\n" << pcaCentroid << std::endl;
	/*
	 // Another way to calculate the eigenvalues ​​and eigenvectors of the point cloud covariance matrix: through the pca interface in PCL as follows
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCA<pcl::PointXYZ> pca;
	pca.setInputCloud(cloudSegmented);
	pca.project(*cloudSegmented, *cloudPCAprojection);
	 std::cerr << std::endl << "EigenVectors: "<< pca.getEigenVectors() << std::endl;//Calculate the feature vector
	 std::cerr << std::endl << "EigenValues: "<< pca.getEigenValues() << std::endl;//Calculate characteristic values
	*/
	Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
	tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R.
	tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//  -R*t
	tm_inv = tm.inverse();
 
	 std::cout << "Transformation matrix tm(4x4):\n" << tm << std::endl;
	 std::cout << "inverter matrix tm'(4x4):\n" << tm_inv << std::endl;
 
	pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>);
	pcl::transformPointCloud(*cloud, *transformedCloud, tm);
 
	PointType min_p1, max_p1;
	Eigen::Vector3f c1, c;
	pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
	c1 = 0.5f*(min_p1.getVector3fMap() + max_p1.getVector3fMap());
 
	 std::cout << "Centre c1(3x1):\n" << c1 << std::endl;
 
	Eigen::Affine3f tm_inv_aff(tm_inv);
	pcl::transformPoint(c1, c, tm_inv_aff);
 
	Eigen::Vector3f whd, whd1;
	whd1 = max_p1.getVector3fMap() - min_p1.getVector3fMap();
	whd = whd1;
	 float sc1 = (whd1(0) + whd1(1) + whd1(2)) / 3; //The average scale of the point cloud, used to set the size of the main direction arrow
 
	std::cout << "width1=" << whd1(0) << endl;
	std::cout << "heght1=" << whd1(1) << endl;
	std::cout << "depth1=" << whd1(2) << endl;
	std::cout << "scale1=" << sc1 << endl;
 
	const Eigen::Quaternionf bboxQ1(Eigen::Quaternionf::Identity());
	const Eigen::Vector3f    bboxT1(c1);
 
	const Eigen::Quaternionf bboxQ(tm_inv.block<3, 3>(0, 0));
	const Eigen::Vector3f    bboxT(c);
 
 
	 //The main direction of the point cloud transformed to the origin
	PointType op;
	op.x = 0.0;
	op.y = 0.0;
	op.z = 0.0;
	Eigen::Vector3f px, py, pz;
	Eigen::Affine3f tm_aff(tm);
	pcl::transformVector(eigenVectorsPCA.col(0), px, tm_aff);
	pcl::transformVector(eigenVectorsPCA.col(1), py, tm_aff);
	pcl::transformVector(eigenVectorsPCA.col(2), pz, tm_aff);
	PointType pcaX;
	pcaX.x = sc1 * px(0);
	pcaX.y = sc1 * px(1);
	pcaX.z = sc1 * px(2);
	PointType pcaY;
	pcaY.x = sc1 * py(0);
	pcaY.y = sc1 * py(1);
	pcaY.z = sc1 * py(2);
	PointType pcaZ;
	pcaZ.x = sc1 * pz(0);
	pcaZ.y = sc1 * pz(1);
	pcaZ.z = sc1 * pz(2);
 
 
	 //The main direction of the initial point cloud
	PointType cp;
	cp.x = pcaCentroid(0);
	cp.y = pcaCentroid(1);
	cp.z = pcaCentroid(2);
	PointType pcX;
	pcX.x = sc1 * eigenVectorsPCA(0, 0) + cp.x;
	pcX.y = sc1 * eigenVectorsPCA(1, 0) + cp.y;
	pcX.z = sc1 * eigenVectorsPCA(2, 0) + cp.z;
	PointType pcY;
	pcY.x = sc1 * eigenVectorsPCA(0, 1) + cp.x;
	pcY.y = sc1 * eigenVectorsPCA(1, 1) + cp.y;
	pcY.z = sc1 * eigenVectorsPCA(2, 1) + cp.z;
	PointType pcZ;
	pcZ.x = sc1 * eigenVectorsPCA(0, 2) + cp.x;
	pcZ.y = sc1 * eigenVectorsPCA(1, 2) + cp.y;
	pcZ.z = sc1 * eigenVectorsPCA(2, 2) + cp.z;
 
 
	//visualization
	pcl::visualization::PCLVisualizer viewer;
 
	 pcl::visualization::PointCloudColorHandlerCustom<PointType> tc_handler(transformedCloud, 0, 255, 0); //Point cloud related to the origin
	viewer.addPointCloud(transformedCloud, tc_handler, "transformCloud");
	viewer.addCube(bboxT1, bboxQ1, whd1(0), whd1(1), whd1(2), "bbox1");
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox1");
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "bbox1");
 
	viewer.addArrow(pcaX, op, 1.0, 0.0, 0.0, false, "arrow_X");
	viewer.addArrow(pcaY, op, 0.0, 1.0, 0.0, false, "arrow_Y");
	viewer.addArrow(pcaZ, op, 0.0, 0.0, 1.0, false, "arrow_Z");

  /*  
	 pcl::visualization::PointCloudColorHandlerCustom<PointType> color_handler(cloud, 255, 0, 0); //The initial point cloud input is related
	viewer.addPointCloud(cloud, color_handler, "cloud");
	viewer.addCube(bboxT, bboxQ, whd(0), whd(1), whd(2), "bbox");
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox");
	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "bbox");
 
	viewer.addArrow(pcX, cp, 1.0, 0.0, 0.0, false, "arrow_x");
	viewer.addArrow(pcY, cp, 0.0, 1.0, 0.0, false, "arrow_y");
	viewer.addArrow(pcZ, cp, 0.0, 0.0, 1.0, false, "arrow_z");
  */
  
	viewer.addCoordinateSystem(0.5f*sc1);
	viewer.setBackgroundColor(1.0, 1.0, 1.0);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
}

void UNL_Robotics::SegmentationPipeline::printMinMax()
{
  printMinMax(m_cloud);
}

void UNL_Robotics::SegmentationPipeline::printMinMax(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  pcl::PointXYZ minPoint;
  pcl::PointXYZ maxPoint;
  pcl::getMinMax3D(*cloud, minPoint, maxPoint);

  std::cout << std::endl;
  std::cout << "getMinMax3D()" << std::endl;
  std::cout << "X:  " << minPoint.x << ", " <<  maxPoint.x << std::endl;
  std::cout << "Y:  " << minPoint.y << ", " <<  maxPoint.y << std::endl;
  std::cout << "Z:  " << minPoint.z << ", " <<  maxPoint.z << std::endl;
  std::cout << std::endl;
}

//////// PRIVATE ///////////

void UNL_Robotics::SegmentationPipeline::removeNaNs(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr dest)
{
  //m_cloud->is_dense = false;  
  //boost::shared_ptr<std::vector<int>> indices(new std::vector<int>);
  //pcl::removeNaNFromPointCloud(*m_cloud, *m_cloud, *indices);

  /*  
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(m_cloud);
  extract.setIndices(indices);
  extract.setNegative(true);
  extract.filter(*m_cloud);
  */

  for(pcl::PointCloud<pcl::PointXYZ>::const_iterator it = source->begin(); it != source->end(); ++it) {

    if(!(std::isnan(it->x)  ||  std::isnan(it->y)  ||  std::isnan(it->z))) {
      dest->push_back(*it);
    }
  }

  pcl::PointXYZ pt = *source->begin();
  cout << "pt = " << pt.x << " " << pt.y << " " << pt.z << std::endl;
  cout << "is nan x : " << std::isnan(pt.x) << std::endl;
  cout << "combined ! is nan :  " << !(std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) << std::endl;

  /*  
  std::copy_if(source->begin(), source->end(), dest->begin(), 
               [](pcl::PointXYZ pt)
               {
                 return(!(std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)));
               });
  */
}
