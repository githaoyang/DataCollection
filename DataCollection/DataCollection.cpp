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

	//���Ƴ�ʼ��
	cloud.reset(new PointCloudT);
	cloud->resize(1);

	//����ui����Ԫ�ذ�
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	ui.screen->SetRenderWindow(viewer->getRenderWindow());
	viewer->setupInteractor(ui.screen->GetInteractor(), ui.screen->GetRenderWindow());
	viewer->setCameraPosition(151, -6402, -10771, 0.0133335, 0.945376, -0.325708);	//��������ӽ�
	ui.screen->update();

	//lable��ɫ����
	ui.statelabel->setAlignment(Qt::AlignCenter);	//����״̬����
	ui.statelabel->setAutoFillBackground(true);		//��䱳��
	QPalette pa;
	pa.setColor(QPalette::Background, Qt::darkRed);
	ui.statelabel->setPalette(pa);					//������ɫ

	QObject::connect(ui.connectButton, SIGNAL(clicked()), this, SLOT(connectButtonPressedSlot()));
	QObject::connect(ui.IntegrationtimelineEdit, SIGNAL(editingFinished()), this, SLOT(setIntegrationTime3DSlot()));
	QObject::connect(ui.IntegrationtimeHDRlineEdit, SIGNAL(editingFinished()), this, SLOT(setIntegrationTime3DHDRSlot()));	

	QObject::connect(g_dcam, SIGNAL(getImage(cv::Mat,float,int)), this, SLOT(imageUpdateSlot(cv::Mat,float,int)));	//�������Ӳ�
	QObject::connect(g_imageProcess, SIGNAL(getPointCloud(PointCloudT::Ptr)), this, SLOT(pointCloudUpdateSlot(PointCloudT::Ptr)));		//���ò�����
	QObject::connect(g_imageProcess, SIGNAL(getBodyPhoto(cv::Mat)), this, SLOT(bodyImageUpdateSlot(cv::Mat)));		//���ò�����
	QObject::connect(g_imageProcess, SIGNAL(getCloudFrameRate(float)), this, SLOT(updateCloudFrameRateSlot(float)));		//���ò�����
	//QObject::connect(g_actionRecognition, SIGNAL(getMHIImage(cv::Mat)), this, SLOT(updateMHISlot(cv::Mat)));		//���ò�����


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

//���Ӱ�ť����¼���
void DataCollection::connectButtonPressedSlot()
{
	if (connectState == 0)
	{
		//�����������ȡͼ��
		std::string ip = ui.IplineEdit->text().toStdString();    //��ȡ���IP
		int port = 50660;//ui.PortlineEdit->text().toInt();      //��ȡ����˿ں�
		g_dcam->setNet(ip, port);						 //��ʼ�������

		g_dcam->start();	//�߳�����
		g_imageProcess->start();		//�߳�����

		//��ť״̬�ı�
		QPalette pa;
		pa.setColor(QPalette::Background, Qt::darkYellow);
		ui.statelabel->setPalette(pa);					//������ɫ
		ui.statelabel->setText("Connecting");			//��������
		ui.connectButton->setText("Disconnect");


		connectState++;
	}
	else
	{
		g_dcam->setRun(false);

		//��ť״̬�ı�
		QPalette pa;
		pa.setColor(QPalette::Background, Qt::darkRed);
		ui.statelabel->setPalette(pa);					//������ɫ
		ui.statelabel->setText("No");
		ui.connectButton->setText("Connect");


		connectState--;
	}
}

//QT��ʾͼ��
//���룺imshowsrc α��ɫͼ��
void DataCollection::showImage(Mat imshowsrc)
{
	Mat imgShow = imshowsrc.clone();

	//cv::cvtColor(imgShow, imgShow, COLOR_BGR2RGB);//OpencvĬ��BGR�洢��Qt��ҪRGB
	QImage img = QImage((imgShow.data), imgShow.cols, imgShow.rows, QImage::Format_Grayscale8);

	int width = ui.Img_label->size().width();
	int height = ui.Img_label->size().height();
	//����Ӧ�����
	if ((height / 240.0) > (width / 320.0))
	{
		//���տ������С
		height = width / 320.0 * 240.0;
	}
	else
	{
		//���ոߵ�����С
		width = height / 240.0 * 320.0;
	}

	img = img.scaled(width, height, Qt::KeepAspectRatio, Qt::SmoothTransformation);	//ͼ������
	ui.Img_label->setAlignment(Qt::AlignCenter);		//������ʾ
	ui.Img_label->setPixmap(QPixmap::fromImage(img));	//����ͼ��
	ui.Img_label->repaint();
}

//����ͼƬ��
//���룺img ����ͼ��Mat��ʽ
//���룺isImg �Ƿ���ͼ�񣬱����ȡ�¶������֡
void DataCollection::imageUpdateSlot(cv::Mat img, float frame , int isImg)
{
	if (isImg == 1)			//����ͼƬ
	{
		if (img.size().height != 0 && g_dcam->getRunState())		//��ȡ��������
		{
			//ui����
			QPalette pa;
			pa.setColor(QPalette::Background, Qt::darkGreen);
			ui.statelabel->setPalette(pa);					//������ɫ
			ui.statelabel->setText(tr("Connected"));
			ui.connectButton->setText(tr("Disconnect"));

			//����ԭʼ����
			//cv::Mat imshowsrc = img;
			//��ʾα��ɫͼ
			showImage(img);
			showFrame(frame);

		}
		else							//��ȡ����ʧ��
		{
			g_dcam->setRun(false);

			//ui����
			QPalette pa;
			pa.setColor(QPalette::Background, Qt::darkRed);
			ui.statelabel->setPalette(pa);					//������ɫ
			ui.statelabel->setText("No");
			ui.connectButton->setText("Connect");
			return;
		}
	}
	else if (isImg == -1)		//��ȡ�쳣
	{
		g_dcam->setRun(false);

		//ui����
		QPalette pa;
		pa.setColor(QPalette::Background, Qt::darkRed);
		ui.statelabel->setPalette(pa);					//������ɫ
		ui.statelabel->setText("No");
		ui.connectButton->setText("Connect");
		return;
	}

	return;

}

//����3D����ʱ��
void DataCollection::setIntegrationTime3DSlot()
{
	g_dcam->integrationtime3D = ui.IntegrationtimelineEdit->text();
	g_dcam->integrationtime3Dflag = 1;
}

//����HDR����ʱ���
void DataCollection::setIntegrationTime3DHDRSlot()
{
	g_dcam->integrationtime3DHDR = ui.IntegrationtimeHDRlineEdit->text();
	g_dcam->integrationtime3DHDRflag = 1;
}

//����HDR
void DataCollection::setHDRSlot()
{
	g_dcam->isHDR = ui.checkBoxHDR->isChecked(); //����HDRѡ��״̬
	g_dcam->isHDRflag = true; //����isHDR����ͱ�־
}


void DataCollection::bodyImageUpdateSlot(cv::Mat img)
{
	//Mat img = g_imageProcess->peopleImg.clone();
	QImage image = QImage((img.data), img.cols, img.rows, QImage::Format_Grayscale8);
	ui.Seg_label->setPixmap(QPixmap::fromImage(image));
	ui.Seg_label->repaint();
}

//֡����ʾ
void DataCollection::showFrame(float frame)
{
	float imgframe = frame;
	ui.FramelineEdit->setText(QString::number(imgframe));
}

//�������ݸ���
//���룺c ����ָ��
void DataCollection::pointCloudUpdateSlot(PointCloudT::Ptr c)
{
	cloud = c;
	//�������ݸ���
	//pcl::copyPointCloud(*(g_imageProcess->g_pclConvert->pointcloud), *cloud);
	showPointCloud();
}


//���ƽ������
void DataCollection::showPointCloud()
{
	viewer->removePointCloud("cloud");
	viewer->addPointCloud(cloud, "cloud");
	viewer->updatePointCloud(cloud, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
	ui.screen->update();
}

//��ʾ����
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

			//��ť״̬�ı�
			ui.startButton->setText("saving");

			//������ɫ
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
	//������ɫ
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
