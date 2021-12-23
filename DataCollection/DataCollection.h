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
	void showImage(cv::Mat imshowsrc);//显示图像
	void showFrame(float frame); //显示帧率

private:
    Ui::DataCollectionClass ui;
	int connectState = 0;					//socket连接状态，用于更新连接按钮ui 0无连接 1连接
	int savestate = 0;						//保存状态 0未保存 1正在保存
	bool isPCLShow = false;					//是否点云转换标志
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;	//PCL可视化窗口
	PointCloudT::Ptr cloud;					//点云指针

private slots:
	void imageUpdateSlot(cv::Mat img, float frame, int isImg);	//更新图像信号
	void connectButtonPressedSlot();	//连接按钮点击槽
	void setIntegrationTime3DSlot();	//设置3D积分时间
	void setIntegrationTime3DHDRSlot(); 
	void setHDRSlot();
	void pointCloudUpdateSlot(PointCloudT::Ptr c);	//更新点云信息
	void showPointCloud();


};
