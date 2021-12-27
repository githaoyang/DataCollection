#pragma once

#include <QThread>
#include <opencv2/opencv.hpp>
#include <string>
#include <QMetaType>

#include "Depthprocess.h"
#include "PCLConvert.h"


class ImageProcess : public QThread
{
	Q_OBJECT

public:
	ImageProcess(QObject *parent);
	ImageProcess();





signals:
	void getBodyPhoto();
	void getPointCloud(PointCloudT::Ptr);	//获取点云信号

protected:
	void run();							//继承自QThread，线程运行函数

private:
	PCLConvert	*g_pclConvert;			//点云转换使用
	Imagedepthprocess *g_depthprocess;	//原始图像处理类

	bool isRun = false;					//是否运行
	bool isPointCloudConvert = false;	//是否点云转换
	bool isColormapPoint = false;		//是否点云伪彩色
	bool isBodyPhotoConvert = false;	//是否提取人物身体图片

	int  pointFilterSize = 3;			//点云密度设置（平衡点云和速度）


};

