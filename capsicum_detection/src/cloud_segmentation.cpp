#include "cloud_segmentation.h"

namespace cloud_segmentation {


pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_based_region_growing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
        pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);

         //pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);

         pcl::IndicesPtr indices (new std::vector <int>);
         pcl::PassThrough<pcl::PointXYZRGB> pass;
         pass.setInputCloud (cloud);
         pass.setFilterFieldName ("z");
         pass.setFilterLimits (0.0, 1.0);
         pass.filter (*indices);

         pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
         reg.setInputCloud (cloud);
         reg.setIndices (indices);
         reg.setSearchMethod (tree);
         reg.setDistanceThreshold (10);
         reg.setPointColorThreshold (6);
         reg.setRegionColorThreshold (5);
         reg.setMinClusterSize (600);

         std::vector <pcl::PointIndices> clusters;
         reg.extract (clusters);

         return reg.getColoredCloud ();

}

PointCloud::Ptr regionGrowingSegmentation(PointCloud::Ptr cloud){

     pcl::search::Search<PointT>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointT> > (new pcl::search::KdTree<PointT>);
     pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
     pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
     normal_estimator.setSearchMethod (tree);
     normal_estimator.setInputCloud (cloud);
     normal_estimator.setKSearch (50);
     normal_estimator.compute (*normals);

     pcl::IndicesPtr indices (new std::vector <int>);
     pcl::PassThrough<PointT> pass;
     pass.setInputCloud (cloud);
     pass.setFilterFieldName ("z");
     pass.setFilterLimits (0.0, 1.0);
     pass.filter (*indices);

     pcl::RegionGrowing<PointT, pcl::Normal> reg;
     reg.setMinClusterSize (50);
     reg.setMaxClusterSize (1000000);
     reg.setSearchMethod (tree);
     reg.setNumberOfNeighbours (30);
     reg.setInputCloud (cloud);
     //reg.setIndices (indices);
     reg.setInputNormals (normals);
     reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
     reg.setCurvatureThreshold (1.0);

     std::vector <pcl::PointIndices> clusters;
     reg.extract (clusters);

     std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
     std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
     std::cout << "These are the indices of the points of the initial" <<
       std::endl << "cloud that belong to the first cluster:" << std::endl;
     unsigned int counter = 0;
     while (counter < clusters[0].indices.size ())
     {
       std::cout << clusters[0].indices[counter] << ", ";
       counter++;
       if (counter % 10 == 0)
         std::cout << std::endl;
     }
     std::cout << std::endl;

     return reg.getColoredCloud ();


}

PointCloud::Ptr minCutBasedSegmentation(pcl::PointCloud <pcl::PointXYZ>::Ptr cloud, pcl::PointCloud <pcl::PointXYZ>::Ptr object, float sigma, float radius, int kNeighbours, float weight){

     //pcl::PointCloud <pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZ>);

      pcl::IndicesPtr indices (new std::vector <int>);
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud (cloud);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (0.0, 1.0);
      pass.filter (*indices);

      pcl::MinCutSegmentation<pcl::PointXYZ> seg;
      seg.setInputCloud (cloud);
      seg.setIndices (indices);

//      pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZ> ());
//      pcl::PointXYZ point;
//      point.x = 68.97;
//      point.y = -18.55;
//      point.z = 0.57;
//      foreground_points->points.push_back(point);
      seg.setForegroundPoints (object);

      seg.setSigma (0.25);
      seg.setRadius (3.0433856);
      seg.setNumberOfNeighbours (14);
      seg.setSourceWeight (0.8);

      std::vector <pcl::PointIndices> clusters;
      seg.extract (clusters);

      std::cout << "Maximum flow is " << seg.getMaxFlow () << std::endl;

      pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud ();
      pcl::visualization::CloudViewer viewer ("Cluster viewer");
      viewer.showCloud(colored_cloud);
      while (!viewer.wasStopped ())
      {
      }
    return seg.getColoredCloud ();

}



}

