#ifndef CLOUD_FILTERING_TOOLS_H
#define CLOUD_FILTERING_TOOLS_H

#include "capsicum_pcl_types.h"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/min_cut_segmentation.h>

#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

namespace cloud_io_tools{

    void readCloud(const sensor_msgs::PointCloud2::ConstPtr msg, PointCloud::Ptr cloud);

    void writeCloudtoMsg(PointCloud::Ptr cloud, sensor_msgs::PointCloud2::Ptr msg);

    void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image);

}

namespace cloud_filtering_tools{


    PointCloud::Ptr color_based_region_growing(PointCloud::Ptr cloud);

    PointCloud::Ptr regionGrowingSegmentation(PointCloud::Ptr cloud);

    PointCloud::Ptr minCutBasedSegmentation(pcl::PointCloud <pcl::PointXYZ>::Ptr cloud, pcl::PointCloud <pcl::PointXYZ>::Ptr object, float sigma, float radius, int kNeighbours, float weight);

    PointCloud::Ptr filter_cloud(PointCloud::Ptr cloud, int meanK = 50, float std = 1.0);

    PointCloud::Ptr RadiusOutlierRemoval(PointCloud::Ptr cloud, std::string method = "Radius",int kNeighbours = 2, float radius = 0.8);

    void downSample(PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, float leafSize = 0.005f);

    void smooth_cloud(PointCloud::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr output_cloud, float radius = 0.03);

    std::vector<PointCloud::Ptr> euclideanClusterRemoval(PointCloud::Ptr cloud, float ClusterTolerance = 0.01, int minClusterSize = 500, int MaxClusterSize = 25000);

}
#endif // CLOUD_FILTERING_TOOLS_H

