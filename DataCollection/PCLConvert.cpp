#include "PCLConvert.h"


PCLConvert::PCLConvert()
{ 
	//设置校正参数
	setConvertParameter(306.2581793, 306.231918150968, 168.733463661578,
		127.631101483708, -0.187431077194205, 0.727853724252657, 0, 0, -0.967607914754638);
}


PCLConvert::~PCLConvert()
{
}

void PCLConvert::setConvertParameter(double fx = 296, double fy = 296, double cx = 160, double cy = 120, double k1 = 0, double k2 = 0, double p1 = 0, double p2 = 0, double k3 = 0)
{
	this->fx = fx;
	this->fy = fy;
	this->cx = cx;
	this->cy = cy;
	this->k1 = k1;
	this->k2 = k2;
	this->k3 = k3;
	this->p1 = p1;
	this->p2 = p2;

}

//点云生成
//img：原始深度图像，CV_U16格式
//colorMat：伪彩色图像，CV_U8C3格式
//colormap：点云是否染色 true点云根据伪彩色图像染色 false全白色
//filterLevel：点云过滤等级
void PCLConvert::getPointCloud(cv::Mat img, int filterLevel)
{
	pointcloud.reset(new PointCloudT());
	distanceMap.clear();

	//img = undistImg(img);		//畸变矫正

	//点云变换
	int imgWidth = img.size().width;
	int imgHeight = img.size().height;

	for (int i = 0; i < imgHeight; i++)
	{
		for (int j = 0; j < imgWidth; j++)
		{
			float picDist = sqrt((i - imgHeight / 2.0)*(i - imgHeight / 2.0) + (j - imgWidth / 2.0)*(j - imgWidth / 2.0));	//图像上点到中心的像素点个数
			float picAngle = atan2(fx*(i - imgHeight / 2.0), fy*(j - imgWidth / 2.0));												//图像上x,y和中心点角度关系
			float angle = atan(sqrt((j - imgWidth / 2.0)*(j - imgWidth / 2.0) / fx / fx + (i - imgHeight / 2.0)*(i - imgHeight / 2.0) / fy / fy));
			float dist = img.at<ushort>(i, j);				//原始图像深度

			//降低点云数量
			/*if (filterLevel)
				if ((i % filterLevel) || (j % filterLevel))
					continue;*/

			//过滤无效点
			if (dist == 0 || dist >= 30000)
				continue;

			pcl::PointXYZRGBA p;
			p.z = dist*cos(angle);									//坐标变换后的深度
			p.x = -dist*sin(angle)*cos(picAngle);
			p.y = -dist*sin(angle)*sin(picAngle);
			distanceMap[p.z] = std::make_pair(i, j);

			
				p.r = 250;
				p.g = 250;
				p.b = 250;

			pointcloud->points.push_back(p);
		}
	}

	//检查点数量
	if (pointcloud->size() <= 0)
	{
		pointcloud->resize(1);
	}
}

cv::Mat PCLConvert::undistImg(cv::Mat src)
{
	cv::Mat dst;

	//内参矩阵
	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);		//3*3单位矩阵
	cameraMatrix.at<double>(0, 0) = fx;
	cameraMatrix.at<double>(0, 1) = 0;
	cameraMatrix.at<double>(0, 2) = cx;
	cameraMatrix.at<double>(1, 1) = fy;
	cameraMatrix.at<double>(1, 2) = cy;
	cameraMatrix.at<double>(2, 2) = 1;

	//畸变参数
	cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);		//5*1全0矩阵
	distCoeffs.at<double>(0, 0) = k1;
	distCoeffs.at<double>(1, 0) = k2;
	distCoeffs.at<double>(2, 0) = p1;
	distCoeffs.at<double>(3, 0) = p2;
	distCoeffs.at<double>(4, 0) = k3;

	cv::Size imageSize = src.size();
	cv::Mat map1, map2;

	//参数1：相机内参矩阵
	//参数2：畸变矩阵
	//参数3：可选输入，第一和第二相机坐标之间的旋转矩阵
	//参数4：校正后的3X3相机矩阵
	//参数5：无失真图像尺寸
	//参数6：map1数据类型，CV_32FC1或CV_16SC2
	//参数7、8：输出X/Y坐标重映射参数
	initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, imageSize, CV_32FC1, map1, map2);	//计算畸变映射
	//参数1：畸变原始图像
	//参数2：输出图像
	//参数3、4：X\Y坐标重映射
	//参数5：图像的插值方式
	//参数6：边界填充方式
	remap(src, dst, map1, map2, cv::INTER_LINEAR);

	return dst.clone();
}


void  PCLConvert::filterCloud(int distanceFilterParameter,
	int radiusFilterRadiusParameter, int radiusFilterPointParameter, int ransacFilterParameter)
{
	// 创建滤波器对象，第一次对x卡阈值
	//距离滤波
	pcl::PassThrough<PointT> pass(true);
	pass.setInputCloud(pointcloud);
	pass.setFilterFieldName("z"); //通过x滤波
	pass.setFilterLimits(0.0, distanceFilterParameter);
	pass.setNegative(false);//设置为true，则输出范围外的点
	pass.filter(*pointcloud);

	//平面分割，去除地面
	pcl::SACSegmentation<PointT> seg; // 创建一个分割方法
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);//申明模型的参数
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);//申明存储模型的内点的索引

	seg.setOptimizeCoefficients(true); // 这一句可以选择最优化参数的因子
	seg.setModelType(pcl::SACMODEL_PLANE);	//平面模型
	seg.setMethodType(pcl::SAC_RANSAC);		//分割平面模型所使用的分割方法
	seg.setDistanceThreshold(ransacFilterParameter); //设置最小的阀值距离

	seg.setInputCloud(pointcloud); //设置输入的点云
	seg.segment(*inliers, *coefficients);

	pcl::ExtractIndices<PointT> extract; //ExtractIndices滤波器，基于某一分割算法提取点云中的一个子集
	extract.setInputCloud(pointcloud);
	extract.setIndices(inliers); //设置分割后的内点为需要提取的点集
	extract.setNegative(true); //设置提取内点而非外点 或者相反
	extract.filter(*pointcloud);
	
	//半径滤波
	//pcl::RadiusOutlierRemoval<PointT> outrem;
	//outrem.setInputCloud(pointcloud);
	//outrem.setRadiusSearch(radiusFilterRadiusParameter);
	//outrem.setMinNeighborsInRadius(radiusFilterPointParameter);
	//// apply filter
	//outrem.filter(*pointcloud);
	
	//分割出人物所在的点云团（找最大的点云）
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(pointcloud);
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance(100); // 10cm
	ec.setMinClusterSize(1000);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(pointcloud);
	ec.extract(cluster_indices);

	if (cluster_indices.size() >= 1)
	{
		pcl::PointIndices maxCluster = *std::max_element(cluster_indices.begin(), cluster_indices.end(), comp);
		if (maxCluster.indices.size() <= 0)
		{
			return;
		}
		PointCloudT::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
		for (int i = 0; i < maxCluster.indices.size(); i++)
		{
			cloud_cluster->push_back((*pointcloud)[maxCluster.indices[i]]);
		}
		cloud_cluster->width = cloud_cluster->size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		pcl::copyPointCloud(*cloud_cluster, *pointcloud);
	}
	
}


bool comp(pcl::PointIndices a, pcl::PointIndices b)
{
	return a.indices.size() < b.indices.size();
}

void PCLConvert::convertToPhoto(cv::Mat & img, cv::Mat rawImg)
{

	for (auto i : pointcloud->points)
	{
		std::pair<int, int> posi = distanceMap[i.z];
		img.at<uchar>(posi.first, posi.second) = rawImg.at<uchar>(posi.first, posi.second);
	}
}