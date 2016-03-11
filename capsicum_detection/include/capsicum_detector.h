#ifndef CAPSICUM_DETECTOR_H
#define CAPSICUM_DETECTOR_H

#include "capsicum_pcl_types.h"

#include <iostream>
#include <math.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZHSV PointHSV;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointHSV> PointCloudHSV;

struct HSV_model{
public:
    double hue_mean, saturation_mean, value_mean, hue_var, saturation_var, value_var;
    double hue_std_dev, saturation_std_dev, value_std_dev;
};

struct BGR_model{
public:
    double red_mean;
    double green_mean;
    double blue_mean;

    double red_var;
    double green_var;
    double blue_var;
};


class capsicum_detector{
    public:

    capsicum_detector(HSV_model model);
    capsicum_detector();

    //capsicum_detector(HSV_model model, std::vector<std::string> &files);

    //empty Mat the save image to.


    int nCapsicumFound;

    //a vector of vectors that stores points
    cv::vector<cv::vector<cv::Point> > contours;
    //a vector that stores vectors
    cv::vector<cv::Vec4i> hierarchy;

    HSV_model capsicum_model;

    std::vector<std::vector<cv::Point> > contours_poly;
    std::vector<cv::Rect> boundRect;
    std::vector<cv::Point2f> center;
    std::vector<float> radius;
    std::vector<cv::Moments> mu;

    PointCloud::Ptr detect(PointCloud::Ptr cloud, int minArea);

    cv::Mat detect(cv::Mat &color_image, cv::Mat &depth, int minArea);

    cv::Mat detect(cv::Mat &color_image, int minArea);

    int open_image(cv::Mat &Image, std::vector<std::string> &files);

    void filterImage(cv::Mat &Image, cv::Mat &filtered_Image);

    void segmentImage(HSV_model A, cv::Mat &inputImage, cv::Mat &outputImage, double threshold);

    void segmentImage(BGR_model A, cv::Mat &inputImage, cv::Mat &outputImage, double threshold);

    void segmentImage(cv::Mat &inputImage, cv::Mat &outputImage, double threshold);

    void segmentCloud(HSV_model A, PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, double threshold, bool highlight);

    void segmentCloud(PointCloud::Ptr input_cloud, PointCloud::Ptr output_cloud, double threshold, bool highlight);



private:

    double LikelihoodOfNormal(double mean, double varance, double pixel_value);
};


struct capsicum{
	public:
    capsicum();
    capsicum(cv::Moments mu, cv::Rect boundRect);
    capsicum(pcl::PointXYZ &centroid);

	double area;
    pcl::PointXYZ centroid;
	cv::Point center_point;
	cv::Moments moments;					//m00, m01, m10
	cv::Rect bounding_box;				//top left, bottom right,
};




/*class HSV_model{
public:
    double hue_mean = 86.5283;
    double saturation_mean = 134.504;
    double value_mean = 143.904;

    double hue_var = 85.7284;
    double saturation_var = 1522.12;
    double value_var = 2576;
};*/



#endif
