#ifndef CAPSICUM_SEGMENTATION_H
#define CAPSICUM_SEGMENTATION_H

//////////////////////////////////
//LOCAL Includes

#include "capsicum_pcl_types.h"
#include "capsicum_detector.h"
#include "cloud_segmentation.h"
#include "cloud_filtering.h"

//////////////////////////////////
//ROS Includes

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

//////////////////////////////////
//PCL Includes
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/ros/conversions.h>
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
#include <pcl/io/pcd_io.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
//////////////////////////////////
//OPENCV Includes

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "capsicum_detection/segmentCapsicum.h"


//////////////////////////////////
//STD Includes

#include <string>
#include <deque>
#include <vector>


//convenient typedefs
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

class capsicum_segmentation
{

public:

     capsicum_segmentation();
    ~capsicum_segmentation();

     void pcl_callback(const sensor_msgs::PointCloud2ConstPtr msg);
     void start();
     PointCloud::Ptr segment_capsicum(PointCloud::Ptr input_cloud);
     void image_callback(const sensor_msgs::Image::ConstPtr imageColor);
     void readCloud(const sensor_msgs::PointCloud2::ConstPtr msg, PointCloud::Ptr cloud);
     void writeCloudtoMsg(PointCloud::Ptr cloud, sensor_msgs::PointCloud2::Ptr msg);
     void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image);
     void getBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool AxisAligned);
     void getEigenVectors();
     void sampleConsensus(PointCloud::Ptr cloud,PointCloud::Ptr inliers, PointCloud::Ptr outliers, int modelType = pcl::SACMODEL_CYLINDER, float distanceThreshold = 0.05, float normalDistanceWeight = 0.1, float maxRadius = 0.1);
     bool segmentCapsicum(capsicum_detection::segmentCapsicum::Request  &req,
                               capsicum_detection::segmentCapsicum::Response &res);
     ros::NodeHandle nh_;

     //capsicum detector code
     HSV_model capsicum_model;
     capsicum_detector capsicumDetector;
     std::vector<capsicum> capsicums;

     PointCloud::Ptr result, source;

     pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
     std::deque<PointCloud::Ptr> output_clouds;

     pcl::visualization::PCLVisualizer *visualizer;
     int vp_1,vp_2;
     //pcl::visualization::CloudViewer visualizer;

     tf::TransformListener listener;

     ros::Publisher pcl_pub;
     ros::Subscriber pcl_sub;
     ros::Subscriber image_sub;

     int v1, v2;

     bool first;
};

#endif // FRUIT_RGBD

