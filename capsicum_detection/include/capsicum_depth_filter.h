#ifndef CAPSICUM_DEPTH_FILTER_H
#define CAPSICUM_DEPTH_FILTER_H

#include "capsicum_detector.h"
#include "capsicum_pcl_types.h"


#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

//#include <string>
//#include <vector>


//convenient typedefs
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2ConstPtr, sensor_msgs::PointCloud2ConstPtr> SyncPolicyCloud;


class capsicum_depth_filter
{

public:

     capsicum_depth_filter();
    ~capsicum_depth_filter();

     void callback(const sensor_msgs::PointCloud2ConstPtr msgCloud);
//     void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth);
     //void callback(const sensor_msgs::PointCloud2ConstPtr msgCloudRGB, const sensor_msgs::PointCloud2ConstPtr msgCloudDepth);
     void start();
     PointCloud::Ptr segment_capsicum(PointCloud::Ptr input_cloud);

     //Callback fuctions
     void cloudtoImageSegmentation(const sensor_msgs::PointCloud2ConstPtr msgCloud);
     void imageSegmentation(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth);
     void imageCallback(const sensor_msgs::Image::ConstPtr imageColor);

     void readCloud(const sensor_msgs::PointCloud2::ConstPtr msg, PointCloud::Ptr cloud);
     void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image);
     sensor_msgs::ImagePtr cvImagetoMsg(cv::Mat &image,std::string encoding);

     image_transport::SubscriberFilter *subImageColor, *subImageDepth;
     //message_filters::Subscriber<sensor_msgs::PointCloud2ConstPtr> *subCloudRGB, *subCloudDepth;
     message_filters::Synchronizer<SyncPolicy> *sync;
     message_filters::Synchronizer<SyncPolicyCloud> *syncCloud;

     ros::NodeHandle nh_;
     image_transport::ImageTransport it_;


     //capsicum detector code
     HSV_model capsicum_model;
     capsicum_detector capsicumDetector;
     std::vector<capsicum> capsicums;


     tf::TransformListener listener;

     image_transport::Publisher segmented_rgb_pub;
     image_transport::Publisher segmented_depth_pub;
//     ros::Publisher pcl_pub;

     ros::Subscriber subCloudRGB;
     //ros::Subscriber subCloudDepth;
     ros::Subscriber image_sub;

     std::string depth_frame_id, color_frame_id;

     bool first;
};

#endif // FRUIT_RGBD

