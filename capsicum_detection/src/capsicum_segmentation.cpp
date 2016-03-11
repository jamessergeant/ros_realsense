
#include "capsicum_segmentation.h"


capsicum_segmentation::capsicum_segmentation():nh_("~"),result(new PointCloud), source(new PointCloud)
{
    first = true;

    std::string topicPointCloud, topicSegmentedPoints;
    if (!nh_.getParam("topic_depth_registered_points", topicPointCloud))
        topicPointCloud = "/ros_kinfu/depth_registered/points";

    if (!nh_.getParam("topic_segmented_points", topicSegmentedPoints))
        topicSegmentedPoints = "/capsicum/points";

    //pcl_sub = nh_.subscribe(topicPointCloud, 1, &capsicum_segmentation::pcl_callback,this);
    pcl_pub = nh_.advertise<sensor_msgs::PointCloud2>(topicSegmentedPoints, 1 );


    static ros::ServiceServer service = nh_.advertiseService("/capsicum_detection/segment_capsicum", &capsicum_segmentation::segmentCapsicum, this);

    capsicum_model.hue_mean = 180.0; //183.198
    capsicum_model.saturation_mean = 1.0;//0.8323;//32.6;
    capsicum_model.value_mean = 0.3907;//115;
    capsicum_model.hue_var = 10*pow(4.3659,2); //650
    capsicum_model.saturation_var = 5*pow(0.1600,2); //412;segmentCapsicum
    capsicum_model.value_var = pow(0.1299,2); //641;

    capsicumDetector = capsicum_detector(capsicum_model);

//    visualizer = new pcl::visualization::PCLVisualizer ("PCL Visualisation");
//    visualizer->addCoordinateSystem (0.1);
//    visualizer->createViewPort (0.0, 0.25, 0.75, 0.75, v2);
//    visualizer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
//    visualizer->setBackgroundColor(0,0,0,v1);
//    visualizer->setBackgroundColor(0,0,0,v2);
}

capsicum_segmentation::~capsicum_segmentation()
{
}

void capsicum_segmentation::start(){

    ros::spin();

}

bool capsicum_segmentation::segmentCapsicum(capsicum_detection::segmentCapsicum::Request  &req,
                          capsicum_detection::segmentCapsicum::Response &res){


    PointCloud::Ptr cloud(new PointCloud);
    PointCloud::Ptr cloud_filtered(new PointCloud);
    PointCloud::Ptr cloud_segmented(new PointCloud);
    PointCloud::Ptr cloud_segmented_filtered(new PointCloud);
    PointCloud::Ptr cloud_seg_filt_dsampled(new PointCloud);
    PointCloud::Ptr capsicum_cloud(new PointCloud);
    float closest_capsicum = 10000;

    Eigen::Vector4f mean, centroid;

    pcl::fromROSMsg(req.cloud,*cloud);

    ROS_INFO_STREAM("Got Request, Starting to segment Capsicum cloud, Input Cloud has " << cloud->size () << " points" << std::endl);

    std::string input_frame_id = req.cloud.header.frame_id;

//    visualizer->removeAllPointClouds(v2);
//    visualizer->removeAllShapes(v2);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);
    //cloud_filtered = pcl_filter::filter_cloud(cloud, 50, 1.0);
    capsicumDetector.segmentCloud(capsicum_model, cloud, cloud_segmented, 10.0, false);
    pcl_filter::downSample(cloud_segmented, cloud_segmented, 0.0025);
    std::vector<PointCloud::Ptr> clusters = pcl_filter::euclideanClusterRemoval(cloud_segmented, 0.003, 100, 250000);

    //Assume biggest cluster is capsicum
    float sum_size, mean_size, euclidean_distance, distance_threshold;
    float count = 0.0;
    for (std::vector<PointCloud::Ptr>::iterator it = clusters.begin (); it != clusters.end (); ++it){
        PointCloud::Ptr cloud = *it;
        sum_size += cloud->size();
        count += 1.0;
    }

    mean_size = sum_size/count;

    for (std::vector<PointCloud::Ptr>::iterator it = clusters.begin (); it != clusters.end (); ++it){
        PointCloud::Ptr cloud = *it;
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud, centroid);
        euclidean_distance = sqrt(pow(centroid[0]-0.25,2) + pow(centroid[1]-0.25,2) + pow(centroid[2],2));

        std::cout << "euclidean distance" << euclidean_distance;
        if((euclidean_distance < closest_capsicum) && (cloud->size() >= 0.5*mean_size)){
          closest_capsicum = euclidean_distance;
          cout << "Closest capsicum is " << closest_capsicum << endl;
          capsicum_cloud = cloud;
        }
        //if(cloud->size() > capsicum_cloud->size()) capsicum_cloud = cloud;

    }

    //Mean zero the segmented cloud
//    pcl::compute3DCentroid(*capsicum_cloud, centroid);
//    pcl::demeanPointCloud<PointT> (*capsicum_cloud, centroid, *capsicum_cloud);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr smoothed_capsicum_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl_filter::smooth_cloud(capsicum_cloud, smoothed_capsicum_cloud, 0.01);

    PointCloud::Ptr output_cloud(new PointCloud);
    copyPointCloud(*smoothed_capsicum_cloud,*output_cloud);

    sensor_msgs::PointCloud2::Ptr msg_out(new sensor_msgs::PointCloud2);
    writeCloudtoMsg(output_cloud, msg_out);
    msg_out->header.stamp = ros::Time::now();
    msg_out->header.frame_id = input_frame_id;
    pcl_pub.publish(msg_out);

    res.segmented_cloud = *msg_out;

    ROS_INFO("Segmented Capsicum Sending Response");
    return true;

}

void capsicum_segmentation::pcl_callback(const sensor_msgs::PointCloud2ConstPtr msg){

    PointCloud::Ptr cloud(new PointCloud);
    PointCloud::Ptr cloud_filtered(new PointCloud);
    PointCloud::Ptr cloud_segmented(new PointCloud);
    PointCloud::Ptr cloud_segmented_filtered(new PointCloud);
    PointCloud::Ptr cloud_seg_filt_dsampled(new PointCloud);
    PointCloud::Ptr capsicum_cloud(new PointCloud);
    float closest_capsicum = 10000;

    Eigen::Vector4f mean, centroid;

    pcl::fromROSMsg(*msg,*cloud);
    std::string input_frame_id = msg->header.frame_id;

//    visualizer->removeAllPointClouds(v2);
//    visualizer->removeAllShapes(v2);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);
    //cloud_filtered = pcl_filter::filter_cloud(cloud, 50, 1.0);
    capsicumDetector.segmentCloud(capsicum_model, cloud, cloud_segmented, 10.0, false);
    pcl_filter::downSample(cloud_segmented, cloud_segmented, 0.0025);
    std::vector<PointCloud::Ptr> clusters = pcl_filter::euclideanClusterRemoval(cloud_segmented, 0.003, 100, 250000);

    //Assume biggest cluster is capsicum
    float sum_size, mean_size, euclidean_distance;
    float count = 0.0;
    for (std::vector<PointCloud::Ptr>::iterator it = clusters.begin (); it != clusters.end (); ++it){
        PointCloud::Ptr cloud = *it;
        sum_size += cloud->size();
        count += 1.0;
    }

    mean_size = sum_size/count;

    for (std::vector<PointCloud::Ptr>::iterator it = clusters.begin (); it != clusters.end (); ++it){
        PointCloud::Ptr cloud = *it;
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud, centroid);
        euclidean_distance = sqrt(pow(centroid[0]-0.25,2) + pow(centroid[1]-0.25,2) + pow(centroid[2],2));
        if((euclidean_distance < closest_capsicum) && (cloud->size() >= 0.5*mean_size)){
          closest_capsicum = euclidean_distance;
          cout << "Closest capsicum is " << closest_capsicum << endl;
          capsicum_cloud = cloud;
        }
        //if(cloud->size() > capsicum_cloud->size()) capsicum_cloud = cloud;

    }

    //Mean zero the segmented cloud
//    pcl::compute3DCentroid(*capsicum_cloud, centroid);
//    pcl::demeanPointCloud<PointT> (*capsicum_cloud, centroid, *capsicum_cloud);

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr smoothed_capsicum_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl_filter::smooth_cloud(capsicum_cloud, smoothed_capsicum_cloud, 0.01);

    PointCloud::Ptr output_cloud(new PointCloud);
    copyPointCloud(*smoothed_capsicum_cloud,*output_cloud);

    sensor_msgs::PointCloud2::Ptr msg_out(new sensor_msgs::PointCloud2);
    writeCloudtoMsg(output_cloud, msg_out);
    msg_out->header.stamp = ros::Time::now();
    msg_out->header.frame_id = input_frame_id;
    pcl_pub.publish(msg_out);
}


void capsicum_segmentation::sampleConsensus(PointCloud::Ptr cloud,PointCloud::Ptr inlier_cloud, PointCloud::Ptr outlier_cloud, int modelType, float distanceThreshold, float normalDistanceWeight, float maxRadius){

    // All the objects needed
    pcl::PassThrough<PointT> pass;
    pcl::SACSegmentation<PointT> seg;
    pcl::ExtractIndices<PointT> extract;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType(modelType);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (distanceThreshold);
    seg.setInputCloud (cloud);

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers, *coefficients);
    std::cerr << "Model Coefficients: " << *coefficients << std::endl;

    // Write the cylinder inliers to disk
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*inlier_cloud);
    if (inlier_cloud->points.empty ())
      std::cerr << "Can't find the cylindrical component." << std::endl;

    extract.setNegative (true);
    extract.filter (*outlier_cloud);
}

void capsicum_segmentation::getBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool AxisAligned)
{

    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud (cloud);
    feature_extractor.compute ();

    std::vector <float> moment_of_inertia;
    std::vector <float> eccentricity;
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

    feature_extractor.getMomentOfInertia (moment_of_inertia);
    feature_extractor.getEccentricity (eccentricity);
    feature_extractor.getAABB (min_point_AABB, max_point_AABB);
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenValues (major_value, middle_value, minor_value);
    feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter (mass_center);

    visualizer->removeAllPointClouds();
    visualizer->removeAllShapes();

    visualizer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
    Eigen::Quaternionf quat (rotational_matrix_OBB);

    if(AxisAligned){
        visualizer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
    }else{
        visualizer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
    }

    pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
    pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
    pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
    pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));

}


void capsicum_segmentation::readCloud(const sensor_msgs::PointCloud2::ConstPtr msg, PointCloud::Ptr cloud){

      pcl::PCLPointCloud2 pcl_pc;
      pcl_conversions::toPCL(*msg, pcl_pc);
      pcl::fromPCLPointCloud2(pcl_pc, *cloud);
  }

void capsicum_segmentation::writeCloudtoMsg(PointCloud::Ptr cloud, sensor_msgs::PointCloud2::Ptr msg){

      pcl::PCLPointCloud2 pcl_pc;
      pcl::toPCLPointCloud2(*cloud, pcl_pc);
      pcl_conversions::fromPCL(pcl_pc,*msg);
}

void capsicum_segmentation::readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image)
  {
      cv_bridge::CvImageConstPtr pCvImage;
      pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
      pCvImage->image.copyTo(image);
  }



int main(int argc, char** argv)
{
  ros::init(argc, argv, "capsicum_segmentation");

  capsicum_segmentation capSegmentation;

  capSegmentation.start();

  return 0;
}
