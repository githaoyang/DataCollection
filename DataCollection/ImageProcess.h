#pragma once

#include <QThread>
#include <opencv2/opencv.hpp>
#include <string>
#include <QMetaType>
#include <mutex>

#include "Depthprocess.h"
#include "PCLConvert.h"


class ImageProcess : public QThread
{
	Q_OBJECT

public:
	ImageProcess(QObject *parent);
	ImageProcess();
	std::mutex m_mutex;


	PCLConvert	*g_pclConvert;			//点云转换使用
	Imagedepthprocess *g_depthprocess;	//原始图像处理类
	cv::Mat rawDepthImg;
	cv::Mat miniDepthImg;
	cv::Mat img_show;

	bool isPointCloudConvert = false;	//是否点云转换
	bool isBodyPhotoConvert = false;	//是否提取人物身体图片

signals:
	void getBodyPhoto(cv::Mat);
	void getPointCloud(PointCloudT::Ptr);	//获取点云信号
	void getCloudFrameRate(float);

protected:
	void run();							//继承自QThread，线程运行函数

private:

	bool isRun = false;					//是否运行

	int  pointFilterSize = 0;			//点云密度设置（平衡点云和速度）


};

