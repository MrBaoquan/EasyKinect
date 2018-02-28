#pragma once
#include <iostream>
#include <windows.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "../KinectVGB/KinectVGB.h"


using namespace Kinect2;
using namespace cv;

String winName = "kinect image";

int gestureCount = 0;


int main()
{
	KinectVGB::Open();
	KinectVGB::ConfigOptParameter(TEXT("E:\\Visual Studio Projects\\2015\\KinectVGB\\x64\\Release\\ImageProcess.ini"));
	KinectVGB::ConfigLimitArea(TEXT("E:\\Visual Studio Projects\\2015\\KinectVGB\\x64\\Release\\LimitArea.ini"));
//	KinectVGB::Close();
	//KinectVGB::LimitArea(-0.3, 0.3, 0.5, 2);
	//bool result = KinectVGB::CreateVGBDatabaseInstance(L"E:/Gesture1.gbd",&gestureCount);

	GestureResult* res=new GestureResult[gestureCount];

	//namedWindow(winName);
	Color* colors = new Color[1920*1080];
	
	Color* procColors = new Color[1920 * 1080];
	unsigned char* indexBuf = new unsigned char[sizeof(unsigned char) * 512 * 424];
	FJoint joints[EJointType::JointType_Count];
	
	int i = 0;
	while (i<100000)//WaitForSingleObject(h, 1000) == WAIT_TIMEOUT
	{
		++i;
		KinectVGB::UpdateJoints(joints);
		KinectVGB::CoordinateResult(colors);

		if (colors)
		{
			ZeroMemory(procColors, sizeof(Color) * 1920 * 1080);
			KinectVGB::ColorImgOptimization(colors, procColors);
			Mat colorImg(Size(1920, 1080), CV_8UC4, static_cast<void*>(const_cast<Color*>(colors)));
			imshow(winName, colorImg);
			//std::cout << "execute" << i << std::endl;
			
		}
		waitKey(20);
	}
	
	KinectVGB::Close();
	return 1;
}
