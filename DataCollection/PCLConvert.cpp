#include "PCLConvert.h"


PCLConvert::PCLConvert()
{ 
	//����У������
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

//��������
//img��ԭʼ���ͼ��CV_U16��ʽ
//colorMat��α��ɫͼ��CV_U8C3��ʽ
//colormap�������Ƿ�Ⱦɫ true���Ƹ���α��ɫͼ��Ⱦɫ falseȫ��ɫ
//filterLevel�����ƹ��˵ȼ�
void PCLConvert::getPointCloud(cv::Mat img, int filterLevel)
{
	pointcloud.reset(new PointCloudT());
	distanceMap.clear();

	//img = undistImg(img);		//�������

	//���Ʊ任
	int imgWidth = img.size().width;
	int imgHeight = img.size().height;

	for (int i = 0; i < imgHeight; i++)
	{
		for (int j = 0; j < imgWidth; j++)
		{
			float picDist = sqrt((i - imgHeight / 2.0)*(i - imgHeight / 2.0) + (j - imgWidth / 2.0)*(j - imgWidth / 2.0));	//ͼ���ϵ㵽���ĵ����ص����
			float picAngle = atan2(fx*(i - imgHeight / 2.0), fy*(j - imgWidth / 2.0));												//ͼ����x,y�����ĵ�Ƕȹ�ϵ
			float angle = atan(sqrt((j - imgWidth / 2.0)*(j - imgWidth / 2.0) / fx / fx + (i - imgHeight / 2.0)*(i - imgHeight / 2.0) / fy / fy));
			float dist = img.at<ushort>(i, j);				//ԭʼͼ�����

			//���͵�������
			/*if (filterLevel)
				if ((i % filterLevel) || (j % filterLevel))
					continue;*/

			//������Ч��
			if (dist == 0 || dist >= 30000)
				continue;

			pcl::PointXYZRGBA p;
			p.z = dist*cos(angle);									//����任������
			p.x = -dist*sin(angle)*cos(picAngle);
			p.y = -dist*sin(angle)*sin(picAngle);
			distanceMap[p.z] = std::make_pair(i, j);

			
				p.r = 250;
				p.g = 250;
				p.b = 250;

			pointcloud->points.push_back(p);
		}
	}

	//��������
	if (pointcloud->size() <= 0)
	{
		pointcloud->resize(1);
	}
}

cv::Mat PCLConvert::undistImg(cv::Mat src)
{
	cv::Mat dst;

	//�ڲξ���
	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);		//3*3��λ����
	cameraMatrix.at<double>(0, 0) = fx;
	cameraMatrix.at<double>(0, 1) = 0;
	cameraMatrix.at<double>(0, 2) = cx;
	cameraMatrix.at<double>(1, 1) = fy;
	cameraMatrix.at<double>(1, 2) = cy;
	cameraMatrix.at<double>(2, 2) = 1;

	//�������
	cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);		//5*1ȫ0����
	distCoeffs.at<double>(0, 0) = k1;
	distCoeffs.at<double>(1, 0) = k2;
	distCoeffs.at<double>(2, 0) = p1;
	distCoeffs.at<double>(3, 0) = p2;
	distCoeffs.at<double>(4, 0) = k3;

	cv::Size imageSize = src.size();
	cv::Mat map1, map2;

	//����1������ڲξ���
	//����2���������
	//����3����ѡ���룬��һ�͵ڶ��������֮�����ת����
	//����4��У�����3X3�������
	//����5����ʧ��ͼ��ߴ�
	//����6��map1�������ͣ�CV_32FC1��CV_16SC2
	//����7��8�����X/Y������ӳ�����
	initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, imageSize, CV_32FC1, map1, map2);	//�������ӳ��
	//����1������ԭʼͼ��
	//����2�����ͼ��
	//����3��4��X\Y������ӳ��
	//����5��ͼ��Ĳ�ֵ��ʽ
	//����6���߽���䷽ʽ
	remap(src, dst, map1, map2, cv::INTER_LINEAR);

	return dst.clone();
}


void  PCLConvert::filterCloud(int distanceFilterParameter,
	int radiusFilterRadiusParameter, int radiusFilterPointParameter, int ransacFilterParameter)
{
	// �����˲������󣬵�һ�ζ�x����ֵ
	//�����˲�
	pcl::PassThrough<PointT> pass(true);
	pass.setInputCloud(pointcloud);
	pass.setFilterFieldName("z"); //ͨ��x�˲�
	pass.setFilterLimits(0.0, distanceFilterParameter);
	pass.setNegative(false);//����Ϊtrue���������Χ��ĵ�
	pass.filter(*pointcloud);

	//ƽ��ָȥ������
	pcl::SACSegmentation<PointT> seg; // ����һ���ָ��
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);//����ģ�͵Ĳ���
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);//�����洢ģ�͵��ڵ������

	seg.setOptimizeCoefficients(true); // ��һ�����ѡ�����Ż�����������
	seg.setModelType(pcl::SACMODEL_PLANE);	//ƽ��ģ��
	seg.setMethodType(pcl::SAC_RANSAC);		//�ָ�ƽ��ģ����ʹ�õķָ��
	seg.setDistanceThreshold(ransacFilterParameter); //������С�ķ�ֵ����

	seg.setInputCloud(pointcloud); //��������ĵ���
	seg.segment(*inliers, *coefficients);

	pcl::ExtractIndices<PointT> extract; //ExtractIndices�˲���������ĳһ�ָ��㷨��ȡ�����е�һ���Ӽ�
	extract.setInputCloud(pointcloud);
	extract.setIndices(inliers); //���÷ָ����ڵ�Ϊ��Ҫ��ȡ�ĵ㼯
	extract.setNegative(true); //������ȡ�ڵ������� �����෴
	extract.filter(*pointcloud);
	
	//�뾶�˲�
	//pcl::RadiusOutlierRemoval<PointT> outrem;
	//outrem.setInputCloud(pointcloud);
	//outrem.setRadiusSearch(radiusFilterRadiusParameter);
	//outrem.setMinNeighborsInRadius(radiusFilterPointParameter);
	//// apply filter
	//outrem.filter(*pointcloud);
	
	//�ָ���������ڵĵ����ţ������ĵ��ƣ�
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