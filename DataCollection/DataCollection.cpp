#include "DataCollection.h"

DataCollection::DataCollection(QWidget *parent)
    : QWidget(parent)
{
    ui.setupUi(this);
	g_dcam = new DCam();

	g_imageProcess = new ImageProcess();
	g_imageProcess->g_depthprocess = g_dcam->g_depthprocess;

	g_actionRecognition = new ActionRecognition();
	g_actionRecognition->g_pclConvert = g_imageProcess->g_pclConvert;

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

	QObject::connect(g_dcam, SIGNAL(getImage(cv::Mat,float,int)), this, SLOT(imageUpdateSlot(cv::Mat,float,int)));	//设置连接槽
	QObject::connect(g_imageProcess, SIGNAL(getPointCloud(PointCloudT::Ptr)), this, SLOT(pointCloudUpdateSlot(PointCloudT::Ptr)));		//设置槽连接
	QObject::connect(g_imageProcess, SIGNAL(getBodyPhoto(cv::Mat)), this, SLOT(bodyImageUpdateSlot(cv::Mat)));		//设置槽连接
	QObject::connect(g_imageProcess, SIGNAL(getCloudFrameRate(float)), this, SLOT(updateCloudFrameRateSlot(float)));		//设置槽连接
	//QObject::connect(g_actionRecognition, SIGNAL(getMHIImage(cv::Mat)), this, SLOT(updateMHISlot(cv::Mat)));		//设置槽连接


	QObject::connect(ui.browseButton, SIGNAL(clicked()), this, SLOT(browseButtonPressedSlot()));
	QObject::connect(ui.startButton, SIGNAL(clicked()), this, SLOT(startButtonPressedSlot()));
	QObject::connect(ui.finishButton, SIGNAL(clicked()), this, SLOT(finishButtonPressedSlot()));

	QObject::connect(ui.checkBoxHDR, SIGNAL(clicked()), this, SLOT(setHDRSlot()));
	QObject::connect(ui.pointCloudCheckBox, SIGNAL(clicked()), this, SLOT(pclConvertSlot()));
	QObject::connect(ui.bodySegmentCheckBox, SIGNAL(clicked()), this, SLOT(bodyPhotoConcertSlot()));
	//QObject::connect(ui.chooseButton, SIGNAL(clicked()), this, SLOT(chooseButtonPressedSlot()));
	//QObject::connect(ui.saveButton, SIGNAL(clicked()), this, SLOT(saveButtonPressedSlot()));
	
		
		
	setHDRSlot();
	setIntegrationTime3DSlot();
	setIntegrationTime3DHDRSlot();
	pclConvertSlot();


}

//连接按钮点击事件槽
void DataCollection::connectButtonPressedSlot()
{
	if (connectState == 0)
	{
		//启动相机，获取图像
		std::string ip = ui.IplineEdit->text().toStdString();    //获取相机IP
		int port = 50660;//ui.PortlineEdit->text().toInt();      //获取相机端口号
		g_dcam->setNet(ip, port);						 //初始化相机类

		g_dcam->start();	//线程启动
		g_imageProcess->start();		//线程启动

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

	//cv::cvtColor(imgShow, imgShow, COLOR_BGR2RGB);//Opencv默认BGR存储，Qt需要RGB
	QImage img = QImage((imgShow.data), imgShow.cols, imgShow.rows, QImage::Format_Grayscale8);

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
			//cv::Mat imshowsrc = img;
			//显示伪彩色图
			showImage(img);
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


void DataCollection::bodyImageUpdateSlot(cv::Mat img)
{
	//Mat img = g_imageProcess->peopleImg.clone();
	QImage image = QImage((img.data), img.cols, img.rows, QImage::Format_Grayscale8);
	ui.Seg_label->setPixmap(QPixmap::fromImage(image));
	ui.Seg_label->repaint();
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
	cloud = c;
	//点云数据更新
	//pcl::copyPointCloud(*(g_imageProcess->g_pclConvert->pointcloud), *cloud);
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

//显示点云
void DataCollection::pclConvertSlot()
{
	g_imageProcess->isPointCloudConvert = ui.pointCloudCheckBox->isChecked();
}


void DataCollection::bodyPhotoConcertSlot()
{
	g_imageProcess->isBodyPhotoConvert = ui.bodySegmentCheckBox->isChecked();
}

void DataCollection::browseButtonPressedSlot()
{
	QString path = QFileDialog::getExistingDirectory(this, tr("choose"), "/");
	if (path.length() == 0)
	{
		QMessageBox::information(NULL, tr("Path"), tr("You didn't select any path."));
	}
	else
	{
		ui.pathlineEdit->setText(path);

	}
}

void DataCollection::startButtonPressedSlot()
{
	if (ui.pathlineEdit->text().isEmpty())
	{
		QMessageBox::information(NULL, tr("Path"), tr("Choose a path."));
	}
	else
	{
		if (g_dcam->issaveVideo == 0)
		{
			sprintf(filename, "/action_sample%d.avi", i);
			string s = filename;
			string str = ui.pathlineEdit->text().toStdString();
			pathname = str + s;
			i++;
			g_dcam->write.open(pathname, CV_FOURCC('M', 'J', 'P', 'G'), 5, Size(320, 240), false);
			g_dcam->issaveVideo = 1;

			//按钮状态改变
			ui.startButton->setText("saving");

			//更改颜色
			ui.startButton->setStyleSheet("background-color: rgb(0, 255, 0);");
		}
		else
		{
			finishButtonPressedSlot();
		}
	}
}
void DataCollection::finishButtonPressedSlot()
{
	ui.startButton->setText("start");
	g_dcam->issaveVideo = 0;
	g_dcam->write.release();
	//更改颜色
	ui.startButton->setStyleSheet(ui.finishButton->styleSheet());

}

void DataCollection::updateCloudFrameRateSlot(float rate)
{
	ui.cloudFramelineEdit->setText(QString::number(rate));
}


void DataCollection::updateMHISlot(cv::Mat img)
{
	QImage image = QImage((img.data), img.cols, img.rows, QImage::Format_Grayscale8);
	ui.Gray_label->setPixmap(QPixmap::fromImage(image));
	ui.Gray_label->repaint();

}
