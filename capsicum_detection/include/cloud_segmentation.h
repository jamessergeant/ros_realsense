#ifndef CLOUD_SEGMENTATION_H
#define CLOUD_SEGMENTATION_H

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
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/min_cut_segmentation.h>

namespace cloud_segmentation{

    PointCloud::Ptr color_based_region_growing(PointCloud::Ptr cloud);

    PointCloud::Ptr regionGrowingSegmentation(PointCloud::Ptr cloud);

    PointCloud::Ptr minCutBasedSegmentation(pcl::PointCloud <pcl::PointXYZ>::Ptr cloud, pcl::PointCloud <pcl::PointXYZ>::Ptr object, float sigma, float radius, int kNeighbours, float weight);
}
#endif // CLOUD_SEGMENTATION_H

