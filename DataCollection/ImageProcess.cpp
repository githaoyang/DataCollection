#include "ImageProcess.h"

ImageProcess::ImageProcess(QObject *parent)
	: QThread(parent)
{
	ImageProcess();
}

ImageProcess::ImageProcess()
{


	//�����źŴ��ݱ�������
	qRegisterMetaType<cv::Mat >("cv::Mat");
	qRegisterMetaType<PointCloudT::Ptr>("PointCloudT::Ptr"); 
	g_pclConvert = new PCLConvert();
}



void ImageProcess::run()
{
	while(1)
	{
		clock_t startT = clock();
		cv::Mat peopleImg = cv::Mat::zeros(cv::Size(320, 240), CV_8UC1);
		if (isPointCloudConvert || isBodyPhotoConvert)
		{
			rawDepthImg = g_depthprocess->getDepth();
			//���Ʊ任
			g_pclConvert->getPointCloud(rawDepthImg);
			if (isPointCloudConvert)
			{
				g_pclConvert->filterCloud(10000, 150, 25, 200);

				emit(getPointCloud(g_pclConvert->pointcloud));
			}
			if (isBodyPhotoConvert)
			{
				cv::Mat gray = g_depthprocess->img_gray.clone();
				gray = ~gray;
				g_pclConvert->convertToPhoto(peopleImg, gray);

				emit getBodyPhoto(peopleImg);//peopleImg
			}
			//
		}
		clock_t endT = clock();
		//��ֹռ�ù�����Դ�����¿���
		_sleep(1);
		float frame_rate = CLOCKS_PER_SEC / (double)(endT - startT);
		emit getCloudFrameRate(frame_rate);
	}
}