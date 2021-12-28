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
	void showImage(cv::Mat imshowsrc);//显示图像
	void showFrame(float frame); //显示帧率
	
	
	string pathname;                        //文件路径名
	string image_pathname;
	char filename[256];                    //视频编号
	char image_filename[256];
	int i = 1;
	int j = 1;


private:
    Ui::DataCollectionClass ui;
	int connectState = 0;					//socket连接状态，用于更新连接按钮ui 0无连接 1连接
	int savestate = 0;						//保存状态 0未保存 1正在保存
	bool isPCLShow = false;					//是否点云转换标志
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;	//PCL可视化窗口
	PointCloudT::Ptr cloud;					//点云指针

private slots:
	void imageUpdateSlot(cv::Mat img, float frame, int isImg);	//更新图像信号
	void bodyImageUpdateSlot(cv::Mat);
	void pointCloudUpdateSlot(PointCloudT::Ptr);	//更新点云信息
	void connectButtonPressedSlot();	//连接按钮点击槽
	void setIntegrationTime3DSlot();	//设置3D积分时间
	void setIntegrationTime3DHDRSlot(); 
	void setHDRSlot();
	void bodyPhotoConcertSlot();
	void showPointCloud();
	void pclConvertSlot();			//显示点云并提取人体
	void browseButtonPressedSlot(); //选择文件路径
	void startButtonPressedSlot();    //开始进行样本录制
	void finishButtonPressedSlot();   //结束样本录制
	void updateCloudFrameRateSlot(float);
};
