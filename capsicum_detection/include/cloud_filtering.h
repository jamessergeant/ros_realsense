#ifndef CLOUD_FILTERING_H
#define CLOUD_FILTERING_H

#include "capsicum_pcl_types.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>

#include <pcl/surface/mls.h>
#include <pcl/features/normal_3d.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

namespace pcl_filter{

PointCloud::Ptr filter_cloud(PointCloud::Ptr cloud, int meanK = 50, float std = 1.0);

PointCloud::Ptr RadiusOutlierRemoval(PointCloud::Ptr cloud, std::string method = "Radius",int kNeighbours = 2, float radius = 0.8);

void downSample(PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, float leafSize = 0.005f);

void smooth_cloud(PointCloud::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr output_cloud, float radius = 0.03);

std::vector<PointCloud::Ptr> euclideanClusterRemoval(PointCloud::Ptr cloud, float ClusterTolerance = 0.01, int minClusterSize = 500, int MaxClusterSize = 25000);

}
#endif // CLOUD_FILTERING_TOOLS_H

