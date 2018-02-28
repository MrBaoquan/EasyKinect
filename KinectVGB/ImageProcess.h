#pragma once
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "KinectVGB.h"

using namespace cv;
class ImageProcess
{
public:
	ImageProcess();
	~ImageProcess();
	
	void KinectDepthImageOptimization(Mat* src,Mat* dst,int bodyIndex);

	void KinectColorImageOptimization(const Kinect2::Color* fullColorImg,const Kinect2::Color* src, Kinect2::Color* dst);

	int maxIndexImgKillArea = 768;

	int maxColorImgKillArea = 2048;

	int morphoStrength = 3;

	int blurStrength = 11;

private:
	
};

