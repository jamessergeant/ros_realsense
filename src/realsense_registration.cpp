


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
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <string>
#include <vector>


//convenient typedefs
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2ConstPtr, sensor_msgs::PointCloud2ConstPtr> SyncPolicyCloud;


//using pcl::visualization::PointCloudColorHandlerGenericField;
//using pcl::visualization::PointCloudColorHandlerCustom;

static const std::string OPENCV_WINDOW = "Image window";
const std::string cloudName = "Output";


image_transport::Publisher rgb_pub;
image_transport::Publisher depth_pub;


void readCloud(const sensor_msgs::PointCloud2::ConstPtr msg, PointCloud::Ptr cloud){

      pcl::PCLPointCloud2 pcl_pc;
      pcl_conversions::toPCL(*msg, pcl_pc);
      pcl::fromPCLPointCloud2(pcl_pc, *cloud);
  }


void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image)
  {
      cv_bridge::CvImageConstPtr pCvImage;
      pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
      pCvImage->image.copyTo(image);
  }

  sensor_msgs::ImagePtr cvImagetoMsg(cv::Mat &image,std::string encoding, std::string frame){

    static int header_sequence_id = 0;

    cv_bridge::CvImage* cv_msg_ptr = new cv_bridge::CvImage();
    std_msgs::Header header;

    header.seq = header_sequence_id++;
    header.stamp = ros::Time::now();
    header.frame_id = frame;

    cv_msg_ptr = new cv_bridge::CvImage(header,encoding,image);

    return cv_msg_ptr->toImageMsg();
}


  void callback_cloud(const sensor_msgs::PointCloud2ConstPtr msgCloud){

      PointCloud::Ptr cloud(new PointCloud);
      readCloud(msgCloud,cloud);
      cv::Mat rgb_image(cloud->height,cloud->width,CV_8UC3);
      cv::Mat depth_image(cloud->height,cloud->width,CV_16UC1);

      cv::Mat depthCropped, rgbCropped;

      std::string frame_id = msgCloud->header.frame_id;
      //std::string frame_id = imageColor->header.frame_id;

      static int count = 0;
      if(cloud->isOrganized()){
          for(int i = 0; i < cloud->height; i++){
              count++;
              for(int j = 0; j < cloud->width; j++){
                  unsigned char b,g,r;

                  b = cloud->at(j,i).b;
                  g = cloud->at(j,i).g;
                  r = cloud->at(j,i).r;
                  if(b == 0 && g == 0 && r == 0){
                      rgb_image.at<cv::Vec3b>(i,j)[0] = 0;
                      rgb_image.at<cv::Vec3b>(i,j)[1] = 0;
                      rgb_image.at<cv::Vec3b>(i,j)[2] = 0;
                  }else{
                      rgb_image.at<cv::Vec3b>(i,j)[0] = b;
                      rgb_image.at<cv::Vec3b>(i,j)[1] = g;
                      rgb_image.at<cv::Vec3b>(i,j)[2] = r;
                  }

                  float depth = cloud->at(j,i).z;
                  if(!isnan(depth) && depth > 0){
                      depth_image.at<uint16_t>(i,j) = depth*1000;
                  }else{
                      depth_image.at<uint16_t>(i,j) = 0;
                  }
                  //rgb_image.at<cv::Vec3b>(i,j)[1] = cloud->points.at<PointT>(i,j).g;
              }
          }
      }

      //depthCropped = depth_image(cv::Rect(0,40,640,400));
      //rgbCropped = rgb_image(cv::Rect(0,40,640,400));

      rgb_pub.publish(cvImagetoMsg(rgb_image,sensor_msgs::image_encodings::BGR8,"camera_rgb_optical_frame"));
      depth_pub.publish(cvImagetoMsg(depth_image,sensor_msgs::image_encodings::TYPE_16UC1,frame_id));

  }

void callback_image(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth){

    cv::Mat color,colorResized, colorCropped, colorRegistered, depth, depthCropped;

    std::string depth_frame_id = imageDepth->header.frame_id;
    std::string color_frame_id = imageColor->header.frame_id;

    readImage(imageDepth, depth);
    readImage(imageColor, color);

    colorCropped = color(cv::Rect(150,0,1920-150,1080));
    resize(colorCropped, colorResized, cv::Size(640,400), 0, 0, cv::INTER_LANCZOS4);
    //colorRegistered = colorResized(cv::Rect(0,10,640,440));

    depthCropped = depth(cv::Rect(0,40,640,400));

    rgb_pub.publish(cvImagetoMsg(colorResized,sensor_msgs::image_encodings::BGR8,color_frame_id));
    depth_pub.publish(cvImagetoMsg(depthCropped,sensor_msgs::image_encodings::TYPE_16UC1,depth_frame_id));
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "realsense_registration");
  ros::NodeHandle nh_("~");
  image_transport::SubscriberFilter *subImageColor, *subImageDepth;
  //message_filters::Subscriber<sensor_msgs::PointCloud2ConstPtr> *subCloudRGB, *subCloudDepth;
  message_filters::Synchronizer<SyncPolicy> *sync;
  message_filters::Synchronizer<SyncPolicyCloud> *syncCloud;

  image_transport::ImageTransport it_(nh_);


  tf::TransformListener listener;

//     ros::Publisher pcl_pub;

  ros::Subscriber subCloudRGB;
  //ros::Subscriber subCloudDepth;
  ros::Subscriber image_sub;

  std::string topicColor = "/realsense/rgb/image_raw";
  std::string topicDepth = "/realsense/depth/image_raw";

  //std::string topicColor = "/camera/image/rgb_raw";
  //std::string topicDepth = "/camera/image/depth_raw";


  // std::string topicCloudRGB = "/camera/depth_registered/points";
  //std::string topicCloudDepth = "/camera/depth/points"*/;

  //subCloudRGB = nh_.subscribe(topicCloudRGB, 1);
  // subCloudRGB =  nh_.subscribe(topicCloudRGB, 1, &callback_cloud);

  subImageColor = new image_transport::SubscriberFilter(it_, topicColor, 1);
  subImageDepth = new image_transport::SubscriberFilter(it_, topicDepth, 1);

  sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(2), *subImageColor, *subImageDepth);
  sync->registerCallback(boost::bind(&callback_image, _1, _2));

  rgb_pub = it_.advertise("/camera/rgb/image_registered", 1 );
  depth_pub = it_.advertise("/camera/depth/image_registered", 1 );

  ros::spin();


  return 0;
}
