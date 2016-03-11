#include "cloud_filtering.h"

namespace pcl_filter
{


PointCloud::Ptr filter_cloud(PointCloud::Ptr cloud, int meanK, float std){

    PointCloud::Ptr cloud_filtered (new PointCloud);

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (meanK); //50
    sor.setStddevMulThresh (std);
    sor.filter(*cloud_filtered);

    //std::cerr << "Cloud after filtering: " << std::endl;
    //std::cerr << *cloud_filtered << std::endl;

    return cloud_filtered;

}


PointCloud::Ptr RadiusOutlierRemoval(PointCloud::Ptr cloud, std::string method, int kNeighbours, float radius){

    PointCloud::Ptr cloud_filtered (new PointCloud);

    if (method == "Radius"){

        pcl::RadiusOutlierRemoval<PointT> outrem;
        // build the filter
        outrem.setInputCloud(cloud);
        outrem.setRadiusSearch(radius);
        outrem.setMinNeighborsInRadius (kNeighbours);
        // apply filter
        outrem.filter (*cloud_filtered);

    }
    else if (method == "Conditional")
    {
        // build the condition
        pcl::ConditionAnd<PointT>::Ptr range_cond (new pcl::ConditionAnd<PointT> ());

        range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::GT, 0.0)));
        range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT> ("z", pcl::ComparisonOps::LT, radius)));

        // build the filter
        pcl::ConditionalRemoval<PointT> condrem (range_cond);
        condrem.setInputCloud (cloud);
        condrem.setKeepOrganized(true);
        // apply filter
        condrem.filter (*cloud_filtered);
    }

    return cloud_filtered;

     //     std::cerr << "Cloud before filtering: " << std::endl;
     //     for (size_t i = 0; i < cloud->points.size (); ++i)
     //       std::cerr << "    " << cloud->points[i].x << " "
     //                           << cloud->points[i].y << " "
     //                           << cloud->points[i].z << std::endl;
     //     // display pointcloud after filtering
     //     std::cerr << "Cloud after filtering: " << std::endl;
     //     for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
     //       std::cerr << "    " << cloud_filtered->points[i].x << " "
     //                           << cloud_filtered->points[i].y << " "
     //                           << cloud_filtered->points[i].z << std::endl;

}


void smooth_cloud(PointCloud::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr output_cloud, float radius){
    // Load input file into a PointCloud<T> with an appropriate type

    // Create a KD-Tree
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<PointT, pcl::PointXYZRGBNormal> mls;

    //mls.setComputeNormals (true);

    // Set parameters
    mls.setInputCloud (cloud);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (radius);
    mls.process (*output_cloud);

}


pcl::PointCloud<pcl::Normal>::Ptr compute_normals(PointCloud::Ptr cloud){

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);

    // Compute the features
    ne.compute (*cloud_normals);

    return cloud_normals;


    // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
}

// Create the filtering object: downsample the dataset using a leaf size of 1cm
void downSample(PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, float leafSize){

    std::cout << "PointCloud before filtering has: " << input_cloud->points.size ()  << " data points." << std::endl; //*
    pcl::VoxelGrid<PointT> vg;
//    output_cloud = PointCloud(new PointCloud);

    vg.setInputCloud (input_cloud);
    vg.setLeafSize (leafSize, leafSize, leafSize);
    vg.filter (*output_cloud);
    std::cout << "PointCloud after filtering has: " << output_cloud->points.size ()  << " data points." << std::endl; //*
}


//returns a vector of clusters extracted from input cloud based on euclidean tolerance and min/max cluster size
std::vector<PointCloud::Ptr> euclideanClusterRemoval(PointCloud::Ptr cloud_rgb, float ClusterTolerance, int minClusterSize, int MaxClusterSize){

     std::vector<PointCloud::Ptr> clusters;
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
     pcl::copyPointCloud(*cloud_rgb, *cloud_xyz);

     // Creating the KdTree object for the search method of the extraction
     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
     tree->setInputCloud (cloud_xyz);

     std::vector<pcl::PointIndices> cluster_indices;
     pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
     ec.setClusterTolerance (ClusterTolerance); // 2cm
     ec.setMinClusterSize (minClusterSize); //100
     ec.setMaxClusterSize (MaxClusterSize); //25000
     ec.setSearchMethod (tree);
     ec.setInputCloud (cloud_xyz); //cloud_filtered
     ec.extract (cluster_indices);

     for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
     {
         PointCloud::Ptr cloud_cluster (new PointCloud);
         for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
             cloud_cluster->points.push_back (cloud_rgb->points[*pit]); //*

         cloud_cluster->width = cloud_cluster->points.size ();
         cloud_cluster->height = 1;
         cloud_cluster->is_dense = true;
         clusters.push_back(cloud_cluster);

     }

     return clusters;

}

}
