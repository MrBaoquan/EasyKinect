#include "ImageProcess.h"
#include <iostream>


#pragma comment(lib,"opencv_world320d.lib")

ImageProcess::ImageProcess()
{

}


ImageProcess::~ImageProcess()
{

}

void ImageProcess::KinectDepthImageOptimization(Mat* src,Mat* dst,int bodyIndex)
{
	if(bodyIndex==-1)
	{
		return;
	}
	using namespace std;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	Mat& srcImg = *src;
	/// Detect edges using canny
	Mat reverseOutput = Mat::zeros(srcImg.size(), CV_8UC1);
	inRange(srcImg, Scalar(bodyIndex), Scalar(bodyIndex+1), reverseOutput);
	
	// Find contours
	findContours(reverseOutput, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		/// Draw contours
	Mat drawing = Mat::zeros(srcImg.size(), CV_8UC1);
	if (!contours.empty() && !hierarchy.empty())
	{
		int idx = 0;
		for (; idx >= 0; idx = hierarchy[idx][0])
		{
			Scalar color = Scalar(255-bodyIndex);
			double area = contourArea(contours[idx]);
			if (fabs(area) < maxIndexImgKillArea)
			{
				continue;
			}
			drawContours(drawing, contours, idx, color, CV_FILLED, 8, hierarchy);
		}
	}
	*dst = drawing.clone();
}

void ImageProcess::KinectColorImageOptimization(const Kinect2::Color* fullColorImg,const Kinect2::Color* src, Kinect2::Color* dst)
{
    waitKey(5);
	Mat colorImg(Size(1920, 1080), CV_8UC4, static_cast<void*>(const_cast<Kinect2::Color*>(src)));

	Mat grayImg = Mat::zeros(colorImg.size(), CV_8UC4);
	cvtColor(colorImg, grayImg, CV_RGB2GRAY);
	Mat binImg = Mat::zeros(Size(1920, 1080), CV_8UC1);
	threshold(grayImg, binImg, 0, 255, THRESH_BINARY);

	using namespace std;
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;

	findContours(binImg, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	Mat drawing = Mat::zeros(binImg.size(), CV_8UC1);
	if (!contours.empty() && !hierarchy.empty())
	{
		int idx = 0;
		for (; idx >= 0; idx = hierarchy[idx][0])
		{
			Scalar color = Scalar(200);
			double area = contourArea(contours[idx]);
			if (fabs(area)<maxColorImgKillArea)
			{
				continue;
			}
			drawContours(drawing, contours, idx, color, CV_FILLED, 8, hierarchy);
		}
	}

	Mat element = getStructuringElement(MORPH_RECT, Size(morphoStrength, morphoStrength));
	
	
	Mat morOutput= Mat::zeros(drawing.size(), CV_8UC1);
	morphologyEx(drawing, morOutput, MORPH_CLOSE, element);
	//morphologyEx(drawing, morOutput, MORPH_OPEN, element);

	Mat blurOutput = Mat::zeros(morOutput.size(), CV_8UC1);
	GaussianBlur(morOutput, blurOutput, Size(blurStrength * 2 + 1, blurStrength * 2 + 1), 0, 0);
	for (int colorIndex = 0; colorIndex < 1920 * 1080;++colorIndex)
	{
		static uchar indexColor = 0;
		indexColor = blurOutput.data[colorIndex];
		if(indexColor!=0)
		{
			dst[colorIndex] = fullColorImg[colorIndex];
			uchar alphaVal = 0;
			if(indexColor>=175)
			{
				alphaVal = 255;
			}else
			{
				alphaVal = indexColor;
			}
			dst[colorIndex].a = alphaVal;
		}
	}
}
