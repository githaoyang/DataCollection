#include"Depthprocess.h"
using namespace cv;

Imagedepthprocess::Imagedepthprocess()
{
	_matimg_short.create(Img_height, Img_width, CV_16UC1);
	_matimg_show.create(Img_height, Img_width, CV_16UC1);
	img_color.create(imgSize, CV_8UC3);//构造RGB图像
}
Imagedepthprocess::~Imagedepthprocess()
{

}
//深度数据处理
//返回：CV_U8C3 mat类型
Mat Imagedepthprocess::depthProcess()
{
	uint16_t fameDepthArray2[bytecount];
	//深度图
	for (int j = 0; j < bytecount / 2; j++)
	{
		raw_dep = ptr_buf_unsigned[j * 2 + 1] * 256 + ptr_buf_unsigned[2 * j];
		//cout << raw_dep << " ";
		realindex = bytecount / 2 - (j / Img_width + 1) * Img_width + j % Img_width;   //镜像		
		fameDepthArray2[realindex] = raw_dep;

	}
	
	if (!isHDR)
	{
		//滤波
		calibrate(fameDepthArray2);
	}
	uint16_t depth[240][320];
	uint16_t amp[240][320];
	for (int i = 0; i < 240; i++)
	{
		for (int j = 0; j < 320; j++)
		{
			depth[i][j] = fameDepthArray2[i * 320 + j];
		}
	}
	//16bit原始数据
	for (int i = 0; i < 240; i++)
	{
		ushort* shortdata = _matimg_short.ptr<ushort>(i);
		for (int j = 0; j < 320; j++)
		{
			if (depth[i][j] > 30000)
			{
				shortdata[j] = depth[i][j];
			}
			else
			{
				//计算偏移量
				if (depth[i][j] + offset < 0 && depth[i][j] + offset < 30000)
					shortdata[j] = 0;
				else
					shortdata[j] = depth[i][j] + offset;
			}
		}
	}
	
	//翻转图像
	if (isHorizontalFlip)
	{
		flip(_matimg_short, _matimg_short, 1);		//水平翻转图像
	}
	if (isVerticalFlip)
	{
		flip(_matimg_short, _matimg_short, 0);		//垂直翻转图像
	}

	if (isHDR)
	{
		//合成hdr图
		imageInpainting();
		medianBlur(_matimg_short, _matimg_short, 3);
	}
	//深度图缩放和染色
	setColorImage();
	saveImage();

	return img_color.clone();
}
//获取深度数据
//返回：Mat类型
Mat Imagedepthprocess::getDepth()
{
	return _matimg_short.clone();
}
//滤波
//输入：图像信息的指针
void Imagedepthprocess::calibrate(ushort *img)
{
	//均值滤波
	imageAverageEightConnectivity(img);
	//温度矫正
	//calculationCorrectDRNU(img);
	//深度补偿
	calculationAddOffset(img);
}
//八均值滤波
//输入： 深度图像指针
void  Imagedepthprocess::imageAverageEightConnectivity(ushort *depthdata)
{
	int pixelCounter;
	int nCols = 320;
	int nRowsPerHalf = 120;
	int size = 320 * 240;
	ushort actualFrame[MAX_NUM_PIX];
	int i, j, index;
	int pixdata;
	memcpy(actualFrame, depthdata, size * sizeof(ushort));
	// up side
	// dowm side
	// left side and right side
	// normal part
	for (i = 1; i < 239; i++) {
		for (j = 1; j < 319; j++){
			index = i * 320 + j;
			pixelCounter = 0;
			pixdata = 0;

			ushort temp = 0;				//记录无效点类型

			if (actualFrame[index] < 30000) {
				pixelCounter++;
				pixdata += actualFrame[index];
			}
			else
				temp = actualFrame[index];
			if (actualFrame[index - 1]  < 30000) {   // left
				pixdata += actualFrame[index - 1];
				pixelCounter++;
			}
			else 
				temp = actualFrame[index - 1];
			if (actualFrame[index + 1]  < 30000) {   // right
				pixdata += actualFrame[index + 1];
				pixelCounter++;
			}
			else 
				temp = actualFrame[index + 1];
			if (actualFrame[index - 321]  < 30000) {   // left up
				pixdata += actualFrame[index - 321];
				pixelCounter++;
			}
			else 
				temp = actualFrame[index - 321];
			if (actualFrame[index - 320]  < 30000) {   // up
				pixdata += actualFrame[index - 320];
				pixelCounter++;
			}
			else 
				temp = actualFrame[index - 320];
			if (actualFrame[index - 319]  < 30000) {   // right up
				pixdata += actualFrame[index - 319];
				pixelCounter++;
			}
			else 
				temp = actualFrame[index -319];
			if (actualFrame[index + 319]  < 30000) {   // left down
				pixdata += actualFrame[index - 321];
				pixelCounter++;
			}
			else 
				temp = actualFrame[index + 319];
			if (actualFrame[index + 320]  < 30000) {   // down
				pixdata += actualFrame[index - 320];
				pixelCounter++;
			}
			else 
				temp = actualFrame[index + 320];
			if (actualFrame[index + 321]  < 30000) {   // right down
				pixdata += actualFrame[index - 319];
				pixelCounter++;
			}
			else 
				temp = actualFrame[index + 321];
			//如果周围有效数据小于6记为无效点
			if (pixelCounter < 8) {
				//无效点
				*(depthdata + index) = temp;
			}
			else {
				*(depthdata + index) = pixdata / pixelCounter;
			}
		}
	}

}
//深度补偿
//输入：图像信息指针
void  Imagedepthprocess::calculationAddOffset(ushort *img)
{
	int offset = 0;
	uint16_t maxDistanceCM = 0;
	int l = 0;
	offset = OFFSET_PHASE_DEFAULT;
	uint16_t val;
	uint16_t *pMem = img;
	int numPix = 320 * 240;

	for (l = 0; l<numPix; l++){
		val = pMem[l];
		if (val < 30000) { //if not low amplitude, not saturated and not  adc overflow
			pMem[l] = (val + offset) % MAX_DIST_VALUE;
		}
	}
}



//设置伪彩色参数
void Imagedepthprocess::setColorImage()
{
	Mat depthzip = _matimg_short.clone();
	double interdepth = 894.0 / (maxdepth - mindepth);
	//距离压缩成0到255空间
	for (int i = 0; i < 240; i++)
	{
		for (int j = 0; j < 320; j++)
		{
			ushort LOW_AMPLITUDE = LOW_AMPLITUDE_V26;
			ushort OVER_EXPOSED = OVER_EXPOSED_V26;
			switch (version)
			{
			case 20600: LOW_AMPLITUDE = LOW_AMPLITUDE_V26; OVER_EXPOSED = OVER_EXPOSED_V26; break;
			case 21200: LOW_AMPLITUDE = LOW_AMPLITUDE_V212; OVER_EXPOSED = OVER_EXPOSED_V212; break;
			default:
				break;
			}
			if (depthzip.at<ushort>(i, j) == LOW_AMPLITUDE)
			{
				//无效点
				IMG_B(img_color, i, j) = 0;
				IMG_G(img_color, i, j) = 0;
				IMG_R(img_color, i, j) = 0;
				continue;
			}
			else if (depthzip.at<ushort>(i, j) == OVER_EXPOSED)
			{
				//过曝点
				IMG_B(img_color, i, j) = 255;
				IMG_G(img_color, i, j) = 14;
				IMG_R(img_color, i, j) = 169;
				continue;
			}
			else if (depthzip.at<ushort>(i, j) < mindepth)
			{
				//过小点
				IMG_B(img_color, i, j) = 0;
				IMG_R(img_color, i, j) = 0;
				IMG_G(img_color, i, j) = 0;
				continue;
			}

			//正常点缩放
			if (depthzip.at<ushort>(i, j) > maxdepth)
			{
				depthzip.at<ushort>(i, j) = maxdepth;
			}
			else if (depthzip.at<ushort>(i, j) < mindepth)
			{
				depthzip.at<ushort>(i, j) = mindepth;
			}
			_matimg_show.at<ushort>(i, j) = (ushort)((depthzip.at<ushort>(i, j) - mindepth)*interdepth);

			unsigned short img_tmp = _matimg_show.at<ushort>(i, j);
			if (img_tmp < 64)
			{
				IMG_R(img_color, i, j) = 191 + img_tmp;
				IMG_G(img_color, i, j) = img_tmp;
				IMG_B(img_color, i, j) = 0;
			}
			else if (img_tmp < 255)
			{
				IMG_R(img_color, i, j) = 255;
				IMG_G(img_color, i, j) = img_tmp;
				IMG_B(img_color, i, j) = 0;
			}
			else if (img_tmp < 510)
			{
				img_tmp -= 255;
				IMG_B(img_color, i, j) = img_tmp;
				IMG_G(img_color, i, j) = 255;
				IMG_R(img_color, i, j) = 255 - img_tmp;
			}
			else if (img_tmp < 765)
			{
				img_tmp -= 510;
				IMG_B(img_color, i, j) = 255;
				IMG_G(img_color, i, j) = 255 - img_tmp;
				IMG_R(img_color, i, j) = 0;
			}
			else
			{
				img_tmp -= 765;
				IMG_B(img_color, i, j) = 255 - img_tmp;
				IMG_G(img_color, i, j) = 0;
				IMG_R(img_color, i, j) = 0;
			}
		}
	}

}
//保存深度图
void Imagedepthprocess::saveImage()
{
	if (saveimagestate == 1)		//连续帧
	{
		savestr = savestr.left(savestr.size() - 4);	//删除末尾的 .png
		saveAmpstr = saveAmpstr.left(savestr.size() - 4);
		imwrite(string(savestr.toLocal8Bit()) + "_" + to_string(imagecount) + ".png", _matimg_short);
		imagecount++;
	}
	else if (saveimagestate == 2)	//单帧
	{
		string tempDepth = string(savestr.toLocal8Bit());
		string tempAmp = string(saveAmpstr.toLocal8Bit());
		imwrite(tempDepth, _matimg_short);
	}
	else
	{
		imagecount = 0;
	}
}

//修复HDR图
void Imagedepthprocess::imageInpainting()
{
	Mat& depthzip = _matimg_short;
	ushort LOW_AMPLITUDE = LOW_AMPLITUDE_V26;
	ushort OVER_EXPOSED = OVER_EXPOSED_V26;
	switch (version)
	{
		case 20600: LOW_AMPLITUDE = LOW_AMPLITUDE_V26; OVER_EXPOSED = OVER_EXPOSED_V26; break;
		case 21200: LOW_AMPLITUDE = LOW_AMPLITUDE_V212; OVER_EXPOSED = OVER_EXPOSED_V212; break;
		default:
			break;
	}
	ushort* pdata = NULL;
	ushort *pdataNext = NULL;
	for (int i = 0; i < 240; i = i + 2)
	{
		pdata = _matimg_short.ptr<ushort>(i);
		pdataNext = _matimg_short.ptr<ushort>(i + 1);
		for (int j = 0; j < 320; j++)
		{
			if ((pdata[j] == LOW_AMPLITUDE) || (pdataNext[j] == LOW_AMPLITUDE))
			{
				//无效点
				if ((pdata[j] == LOW_AMPLITUDE) && (pdataNext[j] == LOW_AMPLITUDE))
				{
					continue;
				}
				else if (pdata[j] == LOW_AMPLITUDE)
				{
					pdata[j] = pdataNext[j];
					continue;
				}
				else
				{
					pdataNext[j] = pdata[j];
					continue;
				}
			}
			else if ((pdata[j] == OVER_EXPOSED) || (pdataNext[j] == OVER_EXPOSED))
			{
				//过曝点
				if ((pdata[j] == OVER_EXPOSED) && (pdataNext[j] == OVER_EXPOSED))
				{
					continue;
				}
				else if (pdata[j] == OVER_EXPOSED)
				{
					pdata[j] = pdataNext[j];
					continue;
				}
				else
				{
					pdataNext[j] = pdata[j];
					continue;
				}
			}
			else
			{
				if (pdata[j] > pdataNext[j])
				{
					pdataNext[j] = pdata[j];
				}
				else
				{
					pdata[j] = pdataNext[j];
				}
			}

		}
	}

	//_matimg_short = depthzip.clone();
}


//图像畸变矫正
//src：CV_U16C1格式图像
//return：畸变矫正后图像
void Imagedepthprocess::undistImg(Mat src, Mat &dst)
{
	remap(src, dst, map1, map2, cv::INTER_LINEAR);
}

void Imagedepthprocess::setConvertParameter(double ffx, double ffy, double fcx, double fcy, double fk1, double fk2 , double fp1, double fp2, double fk3)
{
	this->fx = ffx;
	this->fy = ffy;
	this->cx = fcx;
	this->cy = fcy;
	this->k1 = fk1;
	this->k2 = fk2;
	this->k3 = fk3;
	this->p1 = fp1;
	this->p2 = fp2;
	
	//内参矩阵
	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);		//3*3单位矩阵
	cameraMatrix.at<double>(0, 0) = fx;
	cameraMatrix.at<double>(0, 1) = 0;
	cameraMatrix.at<double>(0, 2) = cx;
	cameraMatrix.at<double>(1, 1) = fy;
	cameraMatrix.at<double>(1, 2) = cy;
	cameraMatrix.at<double>(2, 2) = 1;

	//畸变参数
	cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);		//5*1全0矩阵
	distCoeffs.at<double>(0, 0) = k1;
	distCoeffs.at<double>(1, 0) = k2;
	distCoeffs.at<double>(2, 0) = p1;
	distCoeffs.at<double>(3, 0) = p2;
	distCoeffs.at<double>(4, 0) = k3;
	//畸变参数
	cv::Mat distCoeff = cv::Mat::zeros(1, 5, CV_64F);		//5*1全0矩阵
	distCoeff.at<double>(0, 0) = k1;
	distCoeff.at<double>(0, 1) = k2;
	distCoeff.at<double>(0, 2) = p1;
	distCoeff.at<double>(0, 3) = p2;
	distCoeff.at<double>(0, 4) = k3;

	Mat newCameraMatrix = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imgSize, 0);
	//计算畸变映射
	initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), newCameraMatrix, imgSize, CV_32FC1, map1, map2);	

}