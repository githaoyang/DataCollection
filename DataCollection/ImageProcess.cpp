#include "ImageProcess.h"

ImageProcess::ImageProcess(QObject *parent)
	: QThread(parent)
{
	imageProcess();
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

	if (isPointCloudConvert || isBodyPhotoConvert)
	{
		//���Ʊ任
		g_pclConvert.getPointCloud(g_depthprocess->getDepth(), pointFilterSize);
		if (isPointCloudConvert)
		{
			g_pclConvert.filterCloud(10000, 150, 25, 200);
			emit(getPointCloud(g_pclConvert.pointcloud));
		}
		if (isBodyPhotoConvert)
		{
			g_pclConvert.convertToPhoto(peopleImg, img_show);

			emit getBodyPhoto();//peopleImg
		}
		//
	}
}