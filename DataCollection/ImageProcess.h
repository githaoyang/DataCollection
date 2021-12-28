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


	PCLConvert	*g_pclConvert;			//����ת��ʹ��
	Imagedepthprocess *g_depthprocess;	//ԭʼͼ������
	cv::Mat rawDepthImg;
	cv::Mat miniDepthImg;
	cv::Mat img_show;

	bool isPointCloudConvert = false;	//�Ƿ����ת��
	bool isBodyPhotoConvert = false;	//�Ƿ���ȡ��������ͼƬ

signals:
	void getBodyPhoto(cv::Mat);
	void getPointCloud(PointCloudT::Ptr);	//��ȡ�����ź�
	void getCloudFrameRate(float);

protected:
	void run();							//�̳���QThread���߳����к���

private:

	bool isRun = false;					//�Ƿ�����

	int  pointFilterSize = 0;			//�����ܶ����ã�ƽ����ƺ��ٶȣ�


};

