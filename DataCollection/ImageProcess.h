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
	void getPointCloud(PointCloudT::Ptr);	//��ȡ�����ź�

protected:
	void run();							//�̳���QThread���߳����к���

private:
	PCLConvert	*g_pclConvert;			//����ת��ʹ��
	Imagedepthprocess *g_depthprocess;	//ԭʼͼ������

	bool isRun = false;					//�Ƿ�����
	bool isPointCloudConvert = false;	//�Ƿ����ת��
	bool isColormapPoint = false;		//�Ƿ����α��ɫ
	bool isBodyPhotoConvert = false;	//�Ƿ���ȡ��������ͼƬ

	int  pointFilterSize = 3;			//�����ܶ����ã�ƽ����ƺ��ٶȣ�


};

