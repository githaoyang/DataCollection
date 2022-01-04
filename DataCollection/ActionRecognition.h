#pragma once

#include <QThread>
#include <opencv2/opencv.hpp>
#include <string>
#include <QMetaType>
#include <mutex>

#include "PCLConvert.h"

class ActionRecognition :
	public QThread
{
	Q_OBJECT

public:
	ActionRecognition(QObject *parent);
	ActionRecognition();

	PCLConvert	*g_pclConvert;			//点云转换使用

	bool isMHIConvert = false;

signals:
	void getMHIImage(cv::Mat);


protected:
	void run();


private:
	cv::Mat mhi;
	cv::Mat currentframe;
	cv::Mat lastFrame;
	cv::Mat diff;

	int offset = 20;

	void getUpdatedMHI();

};

