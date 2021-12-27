#pragma once


#include <unordered_map>
#include <opencv2/opencv.hpp>
#include <utility>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class PCLConvert
{
public:
	PCLConvert();
	~PCLConvert();
	void setConvertParameter(double fx, double fy, double cx, double cy, double k1, double k2, double p1, double p2, double k3);	//设置转换参数
	void getPointCloud(cv::Mat img, int filterLevel);		//计算点云
	void filterCloud(int distanceFilterParameter,
		int radiusFilterRadiusParameter, int radiusFilterPointParameter, int ransacFilterParameter);
	void convertToPhoto(cv::Mat& img, cv::Mat rawImg);

	PointCloudT::Ptr pointcloud;					//点云指针 
	std::unordered_map<double, std::pair<int, int>> distanceMap;


private:
	cv::Mat undistImg(cv::Mat src);		//畸变矫正

	double fx;
	double fy;
	double cx;
	double cy;
	double k1;
	double k2;
	double p1;
	double p2;
	double k3;
};



bool comp(pcl::PointIndices a, pcl::PointIndices b);