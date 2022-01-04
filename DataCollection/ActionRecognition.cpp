#include "ActionRecognition.h"

ActionRecognition::ActionRecognition(QObject *parent)
	: QThread(parent)
{
	ActionRecognition();
}

ActionRecognition::ActionRecognition()
{
	//声明信号传递变量类型
	qRegisterMetaType<cv::Mat >("cv::Mat");
	qRegisterMetaType<PointCloudT::Ptr>("PointCloudT::Ptr");  
	 
}

void ActionRecognition::run()
{
	while (1)
	{
		if (!isMHIConvert || g_pclConvert->isPhotoUpdate == false)
		{
			_sleep(10);
			continue;
		} 
		currentframe = g_pclConvert->outputImg.clone();
		getUpdatedMHI();
		emit getMHIImage(mhi);

		_sleep(1);
		g_pclConvert->isPhotoUpdate = false;
	}
}

void ActionRecognition::getUpdatedMHI()
{
	if (!lastFrame.data)
	{
		mhi = cv::Mat::zeros(currentframe.size(), CV_8UC1);
		lastFrame = cv::Mat::zeros(currentframe.size(), CV_8UC1);
	}
	cv::absdiff(lastFrame, currentframe, diff);//做差求绝对值  

	threshold(diff, diff, 20, 255.0, CV_THRESH_BINARY);
	for (int i = 0; i < diff.rows; i++)
	{
		uchar* pmhi = mhi.ptr<uchar>(i);
		uchar* pcurr = lastFrame.ptr<uchar>(i);
		uchar* pdiff = diff.ptr<uchar>(i);
		for (int j = 0; j < diff.cols; j++)
		{
			//进行衰减
			if (pdiff[j] == 0 || pcurr[j] == 0)
			{
				pmhi[j] =  std::max(0, (int)pmhi[j]- offset);
			}
			else
			{
				pmhi[j] = pcurr[j];
			}
		}
	}
	lastFrame = currentframe.clone();
}
