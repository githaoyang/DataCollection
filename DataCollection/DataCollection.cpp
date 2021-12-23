#include "DataCollection.h"

DataCollection::DataCollection(QWidget *parent)
    : QWidget(parent)
{
    ui.setupUi(this);
	g_dcam = new DCam();


	//点云初始化
	cloud.reset(new PointCloudT);
	cloud->resize(1);

	//点云ui界面元素绑定
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	ui.screen->SetRenderWindow(viewer->getRenderWindow());
	viewer->setupInteractor(ui.screen->GetInteractor(), ui.screen->GetRenderWindow());
	viewer->setCameraPosition(151, -6402, -10771, 0.0133335, 0.945376, -0.325708);	//设置相机视角
	ui.screen->update();

	//lable颜色设置
	ui.statelabel->setAlignment(Qt::AlignCenter);	//连接状态剧中
	ui.statelabel->setAutoFillBackground(true);		//填充背景
	QPalette pa;
	pa.setColor(QPalette::Background, Qt::darkRed);
	ui.statelabel->setPalette(pa);					//更改颜色

	QObject::connect(ui.connectButton, SIGNAL(clicked()), this, SLOT(connectButtonPressedSlot()));
	QObject::connect(ui.IntegrationtimelineEdit, SIGNAL(editingFinished()), this, SLOT(setIntegrationTime3DSlot()));
	QObject::connect(ui.IntegrationtimeHDRlineEdit, SIGNAL(editingFinished()), this, SLOT(setIntegrationTime3DHDRSlot()));	
	QObject::connect(ui.checkBoxHDR, SIGNAL(clicked()), this, SLOT(setHDRSlot()));
	QObject::connect(g_dcam, SIGNAL(getImage(cv::Mat,float,int)), this, SLOT(imageUpdateSlot(cv::Mat,float,int)));	//设置连接槽
	QObject::connect(g_dcam, SIGNAL(getPointCloud(PointCloudT::Ptr)), this, SLOT(pointCloudUpdateSlot(PointCloudT::Ptr)));		//设置槽连接
		
		
	setHDRSlot();
	setIntegrationTime3DSlot();
	setIntegrationTime3DHDRSlot();


}

//连接按钮点击事件槽
void DataCollection::connectButtonPressedSlot()
{
	if (connectState == 0)
	{
		//启动相机，获取图像
		std::string ip = ui.IplineEdit->text().toStdString();    //获取相机IP
		int port = ui.PortlineEdit->text().toInt();      //获取相机端口号
		g_dcam->setNet(ip, port);						 //初始化相机类

		g_dcam->start();	//线程启动

		//按钮状态改变
		QPalette pa;
		pa.setColor(QPalette::Background, Qt::darkYellow);
		ui.statelabel->setPalette(pa);					//更改颜色
		ui.statelabel->setText("Connecting");			//更改文字
		ui.connectButton->setText("Disconnect");


		connectState++;
	}
	else
	{
		g_dcam->setRun(false);

		//按钮状态改变
		QPalette pa;
		pa.setColor(QPalette::Background, Qt::darkRed);
		ui.statelabel->setPalette(pa);					//更改颜色
		ui.statelabel->setText("No");
		ui.connectButton->setText("Connect");


		connectState--;
	}
}

//QT显示图像
//输入：imshowsrc 伪彩色图像
void DataCollection::showImage(Mat imshowsrc)
{
	Mat imgShow = imshowsrc.clone();

	cv::cvtColor(imgShow, imgShow, COLOR_BGR2RGB);//Opencv默认BGR存储，Qt需要RGB
	QImage img = QImage((uchar*)(imgShow.data), imgShow.cols, imgShow.rows, QImage::Format_RGB888);

	int width = ui.Img_label->size().width();
	int height = ui.Img_label->size().height();
	//自适应长宽比
	if ((height / 240.0) > (width / 320.0))
	{
		//按照宽调整大小
		height = width / 320.0 * 240.0;
	}
	else
	{
		//按照高调整大小
		width = height / 240.0 * 320.0;
	}

	img = img.scaled(width, height, Qt::KeepAspectRatio, Qt::SmoothTransformation);	//图像缩放
	ui.Img_label->setAlignment(Qt::AlignCenter);		//居中显示
	ui.Img_label->setPixmap(QPixmap::fromImage(img));	//更新图像
	ui.Img_label->repaint();
}

//更新图片槽
//输入：img 传入图像Mat格式
//输入：isImg 是否是图像，避免读取温度造成跳帧
void DataCollection::imageUpdateSlot(cv::Mat img, float frame , int isImg)
{
	if (isImg == 1)			//读入图片
	{
		if (img.size().height != 0 && g_dcam->getRunState())		//获取数据正常
		{
			//ui更新
			QPalette pa;
			pa.setColor(QPalette::Background, Qt::darkGreen);
			ui.statelabel->setPalette(pa);					//更改颜色
			ui.statelabel->setText(tr("Connected"));
			ui.connectButton->setText(tr("Disconnect"));

			//处理原始数据
			cv::Mat imshowsrc = img;
			//显示伪彩色图
			showImage(imshowsrc);
			showFrame(frame);

		}
		else							//获取数据失败
		{
			g_dcam->setRun(false);

			//ui更新
			QPalette pa;
			pa.setColor(QPalette::Background, Qt::darkRed);
			ui.statelabel->setPalette(pa);					//更改颜色
			ui.statelabel->setText("No");
			ui.connectButton->setText("Connect");
			return;
		}
	}
	else if (isImg == -1)		//读取异常
	{
		g_dcam->setRun(false);

		//ui更新
		QPalette pa;
		pa.setColor(QPalette::Background, Qt::darkRed);
		ui.statelabel->setPalette(pa);					//更改颜色
		ui.statelabel->setText("No");
		ui.connectButton->setText("Connect");
		return;
	}

	return;

}

//设置3D积分时间
void DataCollection::setIntegrationTime3DSlot()
{
	g_dcam->integrationtime3D = ui.IntegrationtimelineEdit->text();
	g_dcam->integrationtime3Dflag = 1;
}

//设置HDR积分时间槽
void DataCollection::setIntegrationTime3DHDRSlot()
{
	g_dcam->integrationtime3DHDR = ui.IntegrationtimeHDRlineEdit->text();
	g_dcam->integrationtime3DHDRflag = 1;
}

//设置HDR
void DataCollection::setHDRSlot()
{
	g_dcam->isHDR = ui.checkBoxHDR->isChecked(); //更新HDR选中状态
	g_dcam->isHDRflag = true; //更新isHDR命令发送标志
}

//帧率显示
void DataCollection::showFrame(float frame)
{
	float imgframe = frame;
	ui.FramelineEdit->setText(QString::number(imgframe));
}

//点云数据更新
//输入：c 点云指针
void DataCollection::pointCloudUpdateSlot(PointCloudT::Ptr c)
{
	//点云数据更新
	cloud = c;
	showPointCloud();
}


//点云界面更新
void DataCollection::showPointCloud()
{
	viewer->removePointCloud("cloud");
	viewer->addPointCloud(cloud, "cloud");
	viewer->updatePointCloud(cloud, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
	ui.screen->update();
}