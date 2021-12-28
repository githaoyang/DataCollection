#pragma once

#include <vtkAutoInit.h> 
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);

#include <QtWidgets/QWidget>
#include "ui_DataCollection.h"

#include <opencv2\opencv.hpp>
#include <thread>
#include <QThread>
#include <Windows.h>
#include <iostream>
#include <fstream>
#include "DCam.h"
#include <qfile.h>
#include <qfiledialog.h>  
#include <QMouseEvent>

#include "QVTKWidget.h"
#include <vtkRenderWindow.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "ImageProcess.h"

using namespace std;
using namespace cv;

class DataCollection : public QWidget
{
    Q_OBJECT

public:
    DataCollection(QWidget *parent = Q_NULLPTR);
	DCam *g_dcam;
	ImageProcess *g_imageProcess;
	void showImage(cv::Mat imshowsrc);//��ʾͼ��
	void showFrame(float frame); //��ʾ֡��
	
	
	string pathname;                        //�ļ�·����
	string image_pathname;
	char filename[256];                    //��Ƶ���
	char image_filename[256];
	int i = 1;
	int j = 1;


private:
    Ui::DataCollectionClass ui;
	int connectState = 0;					//socket����״̬�����ڸ������Ӱ�ťui 0������ 1����
	int savestate = 0;						//����״̬ 0δ���� 1���ڱ���
	bool isPCLShow = false;					//�Ƿ����ת����־
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;	//PCL���ӻ�����
	PointCloudT::Ptr cloud;					//����ָ��

private slots:
	void imageUpdateSlot(cv::Mat img, float frame, int isImg);	//����ͼ���ź�
	void bodyImageUpdateSlot(cv::Mat);
	void pointCloudUpdateSlot(PointCloudT::Ptr);	//���µ�����Ϣ
	void connectButtonPressedSlot();	//���Ӱ�ť�����
	void setIntegrationTime3DSlot();	//����3D����ʱ��
	void setIntegrationTime3DHDRSlot(); 
	void setHDRSlot();
	void bodyPhotoConcertSlot();
	void showPointCloud();
	void pclConvertSlot();			//��ʾ���Ʋ���ȡ����
	void browseButtonPressedSlot(); //ѡ���ļ�·��
	void startButtonPressedSlot();    //��ʼ��������¼��
	void finishButtonPressedSlot();   //��������¼��
	void updateCloudFrameRateSlot(float);
};
