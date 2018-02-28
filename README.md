>虽然微软宣布停产了Kienct2，但其核心技术为AR/VR行业做出了不小的贡献，并且目前很多产品都采用了Kienct的相关技术。
而且目前短时间内Kienct不会死，这个DLL可以让开发者快速获取Kienct的相关人体数据。
好了，下面开始介绍这个开发库。

### 快速使用
```cpp
#pragma once
#include <iostream>
#include <windows.h>
#include "KinectVGB.h"

#pragma comment(lib,"./lib/KinectVGB.lib")

using namespace Kinect2;

int gestureCount = 0;

int main()
{
	KinectVGB::Open();
	//加载并配置 抠像优化配置文件
	KinectVGB::ConfigOptParameter(TEXT("./configs/ImageProcess.ini"));
	//加载并配置 体感限定区域
	KinectVGB::ConfigLimitArea(TEXT("./configs/LimitArea.ini"));

	bool result = KinectVGB::CreateVGBDatabaseInstance(L"./Database_sample/Seated.gbd",&gestureCount);

	if(!result)
	{
		std::cout << "invalid database path" << std::endl;
		return -1;
	}

	GestureResult* res = new GestureResult[gestureCount];

	Color* colors = new Color[1920 * 1080];

	Color* procColors = new Color[1920 * 1080];
	unsigned char* indexBuf = new unsigned char[sizeof(unsigned char) * 512 * 424];
	FJoint joints[EJointType::JointType_Count];

	int i = 0;
	while (i < 100000)
	{
		++i;
		//KinectVGB::UpdateJoints(joints);
		//KinectVGB::CoordinateResult(colors);
		// 在此处获得joints关节数据 res 姿势匹配结果
		KinectVGB::DetectGestureResult(joints, res);
		for (int index = 0; index < gestureCount;index++)
		{
			EGestureType type = res[index].type;
			switch (type)
			{
			case Kinect2::GestureType_None:
				break;
			case Kinect2::GestureType_Discrete://离散动作
				std::cout <<"离散动作_匹配度为:"<< res[index].confidence << std::endl;
				break;
			case Kinect2::GestureType_Continuous://连续性动作
				std::cout <<"连续性动作进度为:"<<res[index].progress << std::endl;
				break;
			default:
				break;
			}
			
		}
		Sleep(30);
	}

	KinectVGB::Close();
	return 0;
}

```

### 结束
上述代码可以让你快速获取一个姿势数据库与当前Kienct获取的姿势的匹配结果，如果你不熟悉怎么录制姿势数据库，那么请移步
- [Kinect for windows v2 姿势识别工具之 Kinect Studio 的使用](https://www.parful.com/blog/article/106 "Kinect for windows v2 姿势识别工具之 Kinect Studio 的使用")
- [Kinect for windows v2 姿势识别工具之 Visual Gesture Builder的使用](https://www.parful.com/blog/article/108 "Kinect for windows v2 姿势识别工具之 Visual Gesture Builder的使用")
- [Kinect for windows v2 姿势识别工具之 连续性动作检测](https://www.parful.com/blog/article/109 "Kinect for windows v2 姿势识别工具之 连续性动作检测")

通过该DLL，可以获取基本的人体关节点的位置数据以及优化的抠像纹理，更多接口请参考头文件。需要注意的是 该Dll提供的接口并不多，如需更多接口可查看[DLL源代码](https://github.com/MrBaoQuan/EasyKinect "DLL源代码")自行拓展。

####Dll demo 及 SDK 下载
链接：[Kinect快速开发库下载链接](http://pan.baidu.com/s/1pKW99wB "Kinect快速开发库下载") 密码：5f1s
