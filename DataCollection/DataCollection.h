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
#include "Depthprocess.h"
#include "TinySocket.h"
#include "PCLConvert.h"
#include "DCam.h"
#include <qtimer.h>
#include <qfile.h>
#include <qfiledialog.h>  
#include <QMouseEvent>

#include "QVTKWidget.h"
#include <vtkRenderWindow.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


using namespace std;
using namespace cv;

class DataCollection : public QWidget
{
    Q_OBJECT

public:
    DataCollection(QWidget *parent = Q_NULLPTR);
	DCam *g_dcam;
	void showImage(cv::Mat imshowsrc);//��ʾͼ��
	void showFrame(float frame); //��ʾ֡��

private:
    Ui::DataCollectionClass ui;
	int connectState = 0;					//socket����״̬�����ڸ������Ӱ�ťui 0������ 1����
	int savestate = 0;						//����״̬ 0δ���� 1���ڱ���
	bool isPCLShow = false;					//�Ƿ����ת����־
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;	//PCL���ӻ�����
	PointCloudT::Ptr cloud;					//����ָ��

private slots:
	void imageUpdateSlot(cv::Mat img, float frame, int isImg);	//����ͼ���ź�
	void connectButtonPressedSlot();	//���Ӱ�ť�����
	void setIntegrationTime3DSlot();	//����3D����ʱ��
	void setIntegrationTime3DHDRSlot(); 
	void setHDRSlot();
	void pointCloudUpdateSlot(PointCloudT::Ptr c);	//���µ�����Ϣ
	void showPointCloud();


};
