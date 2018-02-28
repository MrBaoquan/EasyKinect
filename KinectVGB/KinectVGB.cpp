#include "KinectVGB.h" 
#include "Kinect.h"
#include <iostream>
#include <windows.h>
#include "Kinect.VisualGestureBuilder.h"
#include "ImageProcess.h"

#pragma comment(lib,"Kinect20.lib")
#pragma comment(lib,"Kinect20.VisualGestureBuilder.lib")

namespace Kinect2
{
	/************************************************************************/
	/* VGB Global Variables					                                */
	/************************************************************************/
	unsigned int bodyCount = 6;

	IKinectSensor* pSensor = nullptr;
	IVisualGestureBuilderDatabase* pGestureDatabase = nullptr;
	IBodyFrameSource* pFrameSource = nullptr;
	IBodyFrameReader* pFrameReader = nullptr;

	IBody** g_bodies = nullptr;

	IVisualGestureBuilderFrameSource** gestureSources = nullptr;
	IVisualGestureBuilderFrameReader** gestureReaders = nullptr;

	IGesture** g_gestureList = nullptr;
	size_t i;

	CRITICAL_SECTION g_cs;

	unsigned int g_numGesture = 0;

	const unsigned int g_BufferSize = 300;

	float g_leftX = -1.0f;
	float g_rightX = 1.0f;
	float g_frontZ = 3.0f;
	float g_behindZ = 0.5f;

	/************************************************************************/
	/* CorrdinateMap Variables                                               */
	/************************************************************************/
	const int cColorWidth = 1920;
	const int cColorHeight = 1080;

	const int cDepthWidth = 512;
	const int cDepthHeight = 424;

	ICoordinateMapper* g_pCoordinateMapper = nullptr;
	IMultiSourceFrameReader* g_pMultiSourceFrameReader = nullptr;
	Color* g_pColorRGBX = nullptr;
	DepthSpacePoint* g_pDepthCoordinates = nullptr;
	Color* g_pOutputRGBX = nullptr;
	Color* g_pBackgroundRGBX = nullptr;

	int trackedBodyIndex = -1;

	unsigned __int64 trackedBodyId = 10010;
	
	// Safe release for interfaces
	template<class Interface>
	inline void SafeRelease(Interface *& pInterfaceToRelease)
	{
		if (pInterfaceToRelease != NULL)
		{
			pInterfaceToRelease->Release();
			pInterfaceToRelease = NULL;
		}
	}

	/************************************************************************/
	/* Optizimation                                                                     */
	/************************************************************************/
	ImageProcess imgProc;



	int GetNearestBodyIndex(IBody** param_bodies)
	{
		int bodyIndex = -1;
		float nearDis = 4.5f;
		
		for (int index = 0; index < 6; ++index)
		{
			IBody* pBody = param_bodies[index];
			BOOLEAN bTracked = false;
			if (pBody->get_IsTracked(&bTracked) == S_OK&&bTracked)
			{
				Joint joints[JointType_Count];
				if (pBody->GetJoints(static_cast<unsigned int>(JointType_Count), joints) == S_OK)
				{
					Joint& spineBase = joints[JointType_SpineBase];
					if (spineBase.TrackingState != TrackingState::TrackingState_NotTracked)
					{
						CameraSpacePoint& spineBasePoint = joints[JointType_SpineBase].Position;
						if (spineBasePoint.Z < nearDis)
						{
							if (spineBasePoint.X<g_leftX||spineBasePoint.X>g_rightX||spineBasePoint.Z>g_frontZ||spineBasePoint.Z<g_behindZ) 
							{
								continue;
							}
							//
							unsigned __int64 trackedId = 0;
							if (pBody->get_TrackingId(&trackedId) == S_OK)
							{
								if (trackedBodyId == trackedId)
								{
									return index;
								}
							}
							//
							nearDis = spineBasePoint.Z;
							bodyIndex = index;
						}
					}
				}
			}
		}
		if(bodyIndex!=-1)
		{
			IBody* pBody = param_bodies[bodyIndex];
			if(pBody->get_TrackingId(&trackedBodyId)!=S_OK)
			{
				return -1;
			}
		}
		return bodyIndex;
	}

	void ProcessFrame(INT64 nTime,
		const UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight,
		const Color* pColorBuffer, int nColorWidth, int nColorHeight,
		const BYTE* pBodyIndexBuffer, int nBodyIndexWidth, int nBodyIndexHeight)
	{
		/*Mat img(Size(nDepthWidth, nDepthHeight), CV_8UC1, static_cast<void*>(const_cast<BYTE*>(pBodyIndexBuffer)));
		
		Mat handleResult = Mat::zeros(img.size(), CV_8UC1);*/
		//imgProc.KinectDepthImageOptimization(&img,&handleResult,trackedBodyIndex);

		//imshow("test", handleResult);

		// Make sure we've received valid data
		if (g_pCoordinateMapper && g_pDepthCoordinates && g_pOutputRGBX &&
			pDepthBuffer && (nDepthWidth == cDepthWidth) && (nDepthHeight == cDepthHeight) &&
			pColorBuffer && (nColorWidth == cColorWidth) && (nColorHeight == cColorHeight) &&
			pBodyIndexBuffer && (nBodyIndexWidth == cDepthWidth) && (nBodyIndexHeight == cDepthHeight))
		{
			HRESULT hr = g_pCoordinateMapper->MapColorFrameToDepthSpace(nDepthWidth * nDepthHeight, (UINT16*)pDepthBuffer, nColorWidth * nColorHeight, g_pDepthCoordinates);
			if (SUCCEEDED(hr))
			{
				// loop over output pixels
				for (int colorIndex = 0; colorIndex < (nColorWidth*nColorHeight); ++colorIndex)
				{
					// default setting source to copy from the background pixel
					Color* pSrc = g_pBackgroundRGBX + colorIndex;

					DepthSpacePoint p = g_pDepthCoordinates[colorIndex];

					// Values that are negative infinity means it is an invalid color to depth mapping so we
					// skip processing for this pixel
					if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
					{
						int depthX = static_cast<int>(p.X + 0.5f);
						int depthY = static_cast<int>(p.Y + 0.5f);

						if ((depthX >= 0 && depthX < nDepthWidth) && (depthY >= 0 && depthY < nDepthHeight))
						{
							BYTE player = pBodyIndexBuffer[depthX + (depthY * cDepthWidth)];

							// if we're tracking a player for the current pixel, draw from the color camera
							if (player == trackedBodyIndex)
							{
								// set source for copy to the color pixel
								pSrc = g_pColorRGBX + colorIndex;
							}
						}
					}

					// write output
				    g_pOutputRGBX[colorIndex] = *pSrc;
				}
			}
		}
	}

	KINECTVGB_API bool KinectVGB::Open()
	{
		bool result = false;
		InitializeCriticalSection(&g_cs);
		if (GetDefaultKinectSensor(&pSensor) != S_OK)
		{
			return result;
		}
		if(pSensor->Open()==S_OK)
		{
			g_bodies = new IBody*[bodyCount];

			g_pColorRGBX = new Color[cColorWidth*cColorHeight];
			g_pDepthCoordinates = new DepthSpacePoint[cColorWidth*cColorHeight];
			g_pOutputRGBX = new Color[cColorWidth*cColorHeight];
			g_pBackgroundRGBX = new Color[cColorWidth*cColorHeight];

			ZeroMemory(g_pOutputRGBX,sizeof(Color)*cColorWidth*cColorHeight);

			pSensor->get_BodyFrameSource(&pFrameSource);
			pFrameSource->OpenReader(&pFrameReader);

			for (unsigned short index = 0; index < bodyCount; ++index)
			{
				g_bodies[index] = nullptr;
			}
			result = true;
		}else
		{
			result = false;
		}

		/*Coordinate Initionalize*/
		pSensor->OpenMultiSourceFrameReader(FrameSourceTypes::FrameSourceTypes_Depth |
			FrameSourceTypes::FrameSourceTypes_Color |
			FrameSourceTypes::FrameSourceTypes_BodyIndex,
			&g_pMultiSourceFrameReader);
		pSensor->get_CoordinateMapper(&g_pCoordinateMapper);

		return result;
	}



	KINECTVGB_API bool KinectVGB::CreateVGBDatabaseInstance(wchar_t* path,int* gestureCount)
	{	
		if (CreateVisualGestureBuilderDatabaseInstanceFromFile(path, &pGestureDatabase) == S_OK)
		{
			pGestureDatabase->get_AvailableGesturesCount(&g_numGesture);
			*gestureCount = g_numGesture;

			if(g_gestureList)
			{
				delete[] g_gestureList;
				g_gestureList = nullptr;
			}
			g_gestureList = new IGesture*[g_numGesture];
			pGestureDatabase->get_AvailableGestures(g_numGesture, g_gestureList);

			if(gestureReaders)
			{
				delete[] gestureReaders;
				gestureReaders = nullptr;
			}

			if(gestureSources)
			{
				delete[] gestureSources;
				gestureSources = nullptr;
			}

			gestureSources = new IVisualGestureBuilderFrameSource*[bodyCount];
			gestureReaders = new IVisualGestureBuilderFrameReader*[bodyCount];
			for (unsigned int bodyIndex = 0; bodyIndex < bodyCount; ++bodyIndex)
			{
				CreateVisualGestureBuilderFrameSource(pSensor, bodyIndex, &gestureSources[bodyIndex]);
				gestureSources[bodyIndex]->AddGestures(g_numGesture, g_gestureList);
				gestureSources[bodyIndex]->OpenReader(&gestureReaders[bodyIndex]);
			}
			return true;
		}
		return false;
	}


	KINECTVGB_API void KinectVGB::CoordinateResult(Color* colors)
	{
		EnterCriticalSection(&g_cs);
		if(!g_pMultiSourceFrameReader)
		{
			ZeroMemory(colors, sizeof(Color) * 1920 * 1080);
			return;
		}
		IMultiSourceFrame* pMultiSourceFrame = nullptr;
		IDepthFrame* pDepthFrame = nullptr;
		IColorFrame* pColorFrame = nullptr;
		IBodyIndexFrame* pBodyIndexFrame = nullptr;

		HRESULT hr = g_pMultiSourceFrameReader->AcquireLatestFrame(&pMultiSourceFrame);
		if(SUCCEEDED(hr))
		{
			IDepthFrameReference* pDepthFrameReference = nullptr;
			hr = pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference);
			if(SUCCEEDED(hr))
			{
				hr = pDepthFrameReference->AcquireFrame(&pDepthFrame);
			}
			SafeRelease(pDepthFrameReference);
		}

		if (SUCCEEDED(hr))
		{
			IColorFrameReference* pColorFrameReference = NULL;

			hr = pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);
			if (SUCCEEDED(hr))
			{
				hr = pColorFrameReference->AcquireFrame(&pColorFrame);
			}

			SafeRelease(pColorFrameReference);
		}

		if(SUCCEEDED(hr))
		{
			IBodyIndexFrameReference* pBodyIndexFrameReference = nullptr;

			hr = pMultiSourceFrame->get_BodyIndexFrameReference(&pBodyIndexFrameReference);
			if(SUCCEEDED(hr))
			{
				hr = pBodyIndexFrameReference->AcquireFrame(&pBodyIndexFrame);
			}
			SafeRelease(pBodyIndexFrameReference);
		}
		if(SUCCEEDED(hr))
		{
			INT64 nDepthTime = 0;
			IFrameDescription* pDepthFrameDescription = nullptr;
			int nDepthWidth = 0;
			int nDepthHeight = 0;
			UINT nDepthBufferSize = 0;
			UINT16 *pDepthBuffer = nullptr;

			IFrameDescription* pColorFrameDescription = nullptr;
			int nColorWidth = 0;
			int nColorHeight = 0;
			ColorImageFormat imageFormat = ColorImageFormat_None;
			UINT nColorBufferSize = 0;
			Color *pColorBuffer = nullptr;

			IFrameDescription* pBodyIndexFrameDescription = nullptr;
			int nBodyIndexWidth = 0;
			int nBodyIndexHeight = 0;
			UINT nBodyIndexBufferSize = 0;
			BYTE *pBodyIndexBuffer = nullptr;

			/*Get depth frame data*/
			hr = pDepthFrame->get_RelativeTime(&nDepthTime);
			if(SUCCEEDED(hr))
			{
				hr = pDepthFrame->get_FrameDescription(&pDepthFrameDescription);
			}

			if(SUCCEEDED(hr))
			{
				hr = pDepthFrameDescription->get_Width(&nDepthWidth);
			}

			if(SUCCEEDED(hr))
			{
				hr = pDepthFrameDescription->get_Height(&nDepthHeight);
			}

			if(SUCCEEDED(hr))
			{
				hr = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &pDepthBuffer);
				//TODO output depth image
			}

			/* Get color frame data */
			if(SUCCEEDED(hr))
			{
				hr = pColorFrame->get_FrameDescription(&pColorFrameDescription);
			}

			if(SUCCEEDED(hr))
			{
				hr = pColorFrameDescription->get_Width(&nColorWidth);
			}

			if(SUCCEEDED(hr))
			{
				hr = pColorFrameDescription->get_Height(&nColorHeight);
			}

			if(SUCCEEDED(hr))
			{
				pColorFrame->get_RawColorImageFormat(&imageFormat);
			}

			if(SUCCEEDED(hr))
			{
				if(imageFormat==ColorImageFormat_Bgra)
				{
					hr = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));
				}
				else if(g_pColorRGBX)
				{
					pColorBuffer = g_pColorRGBX;
					nColorBufferSize = cColorWidth*cColorHeight*sizeof(Color);
					hr = pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(pColorBuffer), ColorImageFormat::ColorImageFormat_Bgra);
					//TODO out put color image
				}
				else
				{
					hr = E_FAIL;
				}
			}

			/*Get body index frame data*/
			if(SUCCEEDED(hr))
			{
				hr = pBodyIndexFrame->get_FrameDescription(&pBodyIndexFrameDescription);
			}

			if(SUCCEEDED(hr))
			{
				hr = pBodyIndexFrameDescription->get_Width(&nBodyIndexWidth);
			}

			if(SUCCEEDED(hr))
			{
				hr = pBodyIndexFrameDescription->get_Height(&nBodyIndexHeight);
			}

			if(SUCCEEDED(hr))
			{
				hr = pBodyIndexFrame->AccessUnderlyingBuffer(&nBodyIndexBufferSize, &pBodyIndexBuffer);
				//TODO output body index image
			}
			if(SUCCEEDED(hr))
			{
				//TODO CORRDINATE MAPING
				ZeroMemory(g_pOutputRGBX, sizeof(Color)*cColorWidth*cDepthHeight);
				ProcessFrame(nDepthTime, pDepthBuffer, nDepthWidth, nDepthHeight,
					pColorBuffer, nColorWidth, nColorHeight,
					pBodyIndexBuffer, nBodyIndexWidth, nBodyIndexHeight);
				if(colors)
				{
					//ColorImgOptimization(g_pOutputRGBX, colors);
					ZeroMemory(colors, sizeof(Color)*nColorWidth*nColorHeight);
					memcpy(colors, g_pOutputRGBX, sizeof(Color)*nColorWidth*nColorHeight);
					//imgProc.KinectColorImageOptimization(g_pColorRGBX,g_pOutputRGBX, colors);
				}
			}
			SafeRelease(pDepthFrameDescription);
			SafeRelease(pColorFrameDescription);
			SafeRelease(pBodyIndexFrameDescription);
		}
		SafeRelease(pDepthFrame);
		SafeRelease(pColorFrame);
		SafeRelease(pBodyIndexFrame);
		SafeRelease(pMultiSourceFrame);
		LeaveCriticalSection(&g_cs);
	}

	KINECTVGB_API void KinectVGB::UpdateJoints(FJoint* joints)
	{
		IBodyFrame* pFrame = nullptr;
		if (pFrameReader->AcquireLatestFrame(&pFrame) == S_OK)
		{
			if (pFrame->GetAndRefreshBodyData(bodyCount, g_bodies) == S_OK)
			{
				int nearestBodyIndex = GetNearestBodyIndex(g_bodies);
				if (nearestBodyIndex != -1)
				{
					trackedBodyIndex = nearestBodyIndex;
					Joint _joints[JointType::JointType_Count];
					if (g_bodies[nearestBodyIndex]->GetJoints(EJointType::JointType_Count, _joints) == S_OK)
					{
						memcpy(joints, _joints, sizeof(FJoint)*JointType_Count);
					}
				}
				else
				{
					ZeroMemory(joints, sizeof(FJoint)*JointType_Count);
					trackedBodyIndex = -1;
				}
			}
		}
		SafeRelease(pFrame);
	}

	KINECTVGB_API FColorSpacePoint KinectVGB::ConvertCameraPointToColorSpace(FCameraSpacePoint& cameraPoint)
	{
		ColorSpacePoint colorPoint;
		CameraSpacePoint tmpPoint;
		tmpPoint.X = cameraPoint.X;
		tmpPoint.Y = cameraPoint.Y;
		tmpPoint.Z = cameraPoint.Z;
		g_pCoordinateMapper->MapCameraPointToColorSpace(tmpPoint, &colorPoint);
		return FColorSpacePoint(colorPoint.X,colorPoint.Y);
	}

	KINECTVGB_API void KinectVGB::ConfigLimitArea(wchar_t* path)
	{
		g_leftX = -(GetPrivateProfileInt(L"LimitArea",L"Left",50,path)/100.0f);
		g_rightX = GetPrivateProfileInt(L"LimitArea", L"Right", 50, path)/100.0f;
		g_behindZ = GetPrivateProfileInt(L"LimitArea", L"Behind", 50, path) / 100.0f;
		g_frontZ = GetPrivateProfileInt(L"LimitArea", L"Front", 300, path)/100.0f;
	}

	KINECTVGB_API void KinectVGB::ColorImgOptimization(Color* src, Color* dst)
	{
        memcpy(dst, src, sizeof(Color) * 1920 * 1080);
        return;
        //优化后存在画面偶尔卡停现象 
		if(g_pColorRGBX&&dst)
		{
			ZeroMemory(dst, sizeof(Color) * 1920 * 1080);
			imgProc.KinectColorImageOptimization(g_pColorRGBX, src, dst);
		}
	}

	KINECTVGB_API void KinectVGB::ConfigOptParameter(wchar_t* path)
	{

		std::wstring wPath = path;
		int maxIndexImgKillArea = GetPrivateProfileInt(TEXT("KinectParam"), TEXT("maxIndexImgKillArea"), 512, path);
		int maxColorImgKillArea = GetPrivateProfileInt(TEXT("KinectParam"), TEXT("maxColorImgKillArea"), 1024, path);
		int morphoStrength = GetPrivateProfileInt(TEXT("KinectParam"), TEXT("morphoStrength"), 5, path);
		int blurStrength = GetPrivateProfileInt(TEXT("KinectParam"), TEXT("blurStrength"), 11, path);
		
		imgProc.maxIndexImgKillArea = maxIndexImgKillArea;
		imgProc.maxColorImgKillArea = maxColorImgKillArea;
		imgProc.morphoStrength = morphoStrength;
		imgProc.blurStrength = blurStrength;
	}
	
	KINECTVGB_API void KinectVGB::DetectGestureResult(FJoint* joints,GestureResult* resultsOutput)
	{
		EnterCriticalSection(&g_cs);
		if(resultsOutput ==nullptr)
		{
			MessageBox(NULL,L"Null pointer of Results!",L"Notes",MB_OK);
			abort();
		}

		IBodyFrame* pFrame = nullptr;
		if (pFrameReader->AcquireLatestFrame(&pFrame) == S_OK)
		{
			if (pFrame->GetAndRefreshBodyData(bodyCount, g_bodies) == S_OK)
			{
				for (unsigned short bodyIndex = 0; bodyIndex < bodyCount; ++bodyIndex)
				{
					BOOLEAN tracked = false;
					IBody* pBody = g_bodies[bodyIndex];

					if (pBody->get_IsTracked(&tracked) == S_OK&&tracked)
					{
						UINT64 trackingId = 0;
						if (pBody->get_TrackingId(&trackingId) == S_OK)
						{
							UINT64 gestureId = 0;
							if (gestureSources[bodyIndex]->get_TrackingId(&gestureId) == S_OK)
							{
								if (gestureId != trackingId)
								{
									gestureSources[bodyIndex]->put_TrackingId(trackingId);
								}
							}
						}
					}
				}
				/************************************************************************/
				/* Visual Gesture Builder Code                                          */
				/************************************************************************/

				IVisualGestureBuilderFrame* pGestureFrame = nullptr;
				int nearestBodyIndex = GetNearestBodyIndex(g_bodies);
				if(nearestBodyIndex!=-1)
				{
					trackedBodyIndex = nearestBodyIndex;
					Joint _joints[JointType::JointType_Count];
					if (g_bodies[nearestBodyIndex]->GetJoints(EJointType::JointType_Count, _joints) == S_OK)
					{
						memcpy(joints, _joints, sizeof(FJoint)*JointType_Count);
					}

					if (gestureReaders[static_cast<unsigned int>(nearestBodyIndex)]->CalculateAndAcquireLatestFrame(&pGestureFrame) == S_OK)
					{
						BOOLEAN bGestureTracked = false;
						if (pGestureFrame->get_IsTrackingIdValid(&bGestureTracked) == S_OK&&bGestureTracked)
						{
							for (UINT gestureIndex = 0; gestureIndex < g_numGesture; ++gestureIndex)
							{
								GestureType gestureType;
								g_gestureList[gestureIndex]->get_GestureType(&gestureType);

								wchar_t gestureName[g_BufferSize];
								g_gestureList[gestureIndex]->get_Name(g_BufferSize, gestureName);
								if (gestureType == GestureType::GestureType_Discrete)
								{
									IDiscreteGestureResult* pGestureResult = nullptr;
									if (pGestureFrame->get_DiscreteGestureResult(g_gestureList[gestureIndex], &pGestureResult) == S_OK)
									{
										BOOLEAN detected = false;
										if (pGestureResult->get_Detected(&detected) == S_OK&&detected)
										{
											float confidence = 0.0f;
											pGestureResult->get_Confidence(&confidence);
											resultsOutput[gestureIndex].confidence = confidence;
											resultsOutput[gestureIndex].type = GestureType_Discrete;
											resultsOutput[gestureIndex].SetGestureName(gestureName);
											pGestureResult->Release();
										}
									}
								}
								else if (gestureType == GestureType::GestureType_Continuous)
								{
									IContinuousGestureResult* pGestureResult = nullptr;
									if (pGestureFrame->get_ContinuousGestureResult(g_gestureList[gestureIndex], &pGestureResult) == S_OK)
									{
										float progress = 0.0f;
										if (pGestureResult->get_Progress(&progress) == S_OK)
										{
											resultsOutput[gestureIndex].progress = progress;
											resultsOutput[gestureIndex].type = EGestureType::GestureType_Continuous;
											resultsOutput[gestureIndex].SetGestureName(gestureName);
										}
										pGestureResult->Release();
									}
								}
								else
								{
									resultsOutput[gestureIndex].type = reinterpret_cast<EGestureType>(GestureType_None);
									resultsOutput[gestureIndex].confidence = 0.0f;
									resultsOutput[gestureIndex].progress = 0.0f;
								}
							}
						}
						pGestureFrame->Release();
					}
				}
				else
				{
					ZeroMemory(joints, sizeof(FJoint)*JointType_Count);
				}

			}
			pFrame->Release();
		}
		LeaveCriticalSection(&g_cs);
	}

	KINECTVGB_API bool KinectVGB::Close()
	{
		bool result = false;
		EnterCriticalSection(&g_cs);
		if(g_gestureList)
		{
			for (unsigned int index = 0; index < g_numGesture; ++index)
			{
				g_gestureList[index]->Release();
			}
			if(g_gestureList)
			{
				delete[] g_gestureList;
				g_gestureList = nullptr;
			}
	
			for (unsigned int index = 0; index < bodyCount; ++index)
			{
				gestureReaders[index]->Release();
				gestureSources[index]->Release();
			}
			if(gestureReaders)
			{
				delete[] gestureReaders;
				gestureReaders = nullptr;
			}
			if(gestureSources)
			{
				delete[] gestureSources;
				gestureSources = nullptr;
			}

			pFrameReader->Release();
			pFrameSource->Release();
		}

		if(g_pOutputRGBX)
		{
			delete[] g_pOutputRGBX;
			g_pOutputRGBX = nullptr;
		}

		if(g_pBackgroundRGBX)
		{
			delete[] g_pBackgroundRGBX;
			g_pBackgroundRGBX = nullptr;
		}

		if(g_pColorRGBX)
		{
			delete[] g_pColorRGBX;
			g_pColorRGBX = nullptr;
		}

		if(g_pDepthCoordinates)
		{
			delete[] g_pDepthCoordinates;
			g_pDepthCoordinates = nullptr;
		}

		if (pSensor != nullptr)
		{
			if (pSensor->Close() == S_OK)
			{
				result = true;
			}
		}

		pSensor->Release();
		LeaveCriticalSection(&g_cs);
		DeleteCriticalSection(&g_cs);
		return result;
	}

	KinectVGB::KinectVGB()
	{

	}

	KINECTVGB_API GestureResult::GestureResult()
	{
		type = EGestureType::GestureType_None;
		confidence = 0.0f;
		progress = 0.0f;

		gestureName = new wchar_t[wcslen(L"None")+2];
		memcpy(gestureName, L"None",wcslen(L"None") + 2);

	}

	KINECTVGB_API GestureResult::GestureResult(EGestureType _type, float _confidence, float _progress, wchar_t* _name)
	{
		type = type;
		confidence = confidence;
		progress = _progress;

		gestureName = reinterpret_cast<wchar_t*>(new char[wcslen(_name) * 2 + 2]);
		memcpy(static_cast<void*>(const_cast<wchar_t*>(gestureName)), static_cast<void*>(const_cast<wchar_t*>(_name)), wcslen(_name) * 2 + 2);

	}


	KINECTVGB_API GestureResult::GestureResult(GestureResult& rhs)
	{
		gestureName = reinterpret_cast<wchar_t*>(new char[wcslen(rhs.gestureName) * 2 + 2]);
		memcpy(static_cast<void*>(const_cast<wchar_t*>(gestureName)), static_cast<void*>(const_cast<wchar_t*>(rhs.gestureName)), wcslen(rhs.gestureName) * 2 + 2);
		confidence = rhs.confidence;
		progress = rhs.progress;
		type = rhs.type;
	}

	KINECTVGB_API GestureResult& GestureResult::operator=(GestureResult& rhs)
	{
		gestureName = reinterpret_cast<wchar_t*>(new char[wcslen(rhs.gestureName) * 2 + 2]);
		memcpy(static_cast<void*>(const_cast<wchar_t*>(gestureName)), static_cast<void*>(const_cast<wchar_t*>(rhs.gestureName)), wcslen(rhs.gestureName) * 2 + 2);
		confidence = rhs.confidence;
		progress = rhs.progress;
		type = rhs.type;
		return *this;
	}

	KINECTVGB_API GestureResult::~GestureResult()
	{
		if(gestureName)
		{
			delete[] gestureName;
		}
	}

	KINECTVGB_API const wchar_t* GestureResult::GetGestureName() const
	{
		return this->gestureName;
	}

	KINECTVGB_API void GestureResult::SetGestureName(const wchar_t* name)
	{
		gestureName = reinterpret_cast<wchar_t*>(new char[wcslen(name) * 2 + 2]);
		memcpy(static_cast<void*>(const_cast<wchar_t*>(gestureName)), static_cast<void*>(const_cast<wchar_t*>(name)), wcslen(name) * 2 + 2);
	}

	KINECTVGB_API FCameraSpacePoint::FCameraSpacePoint()
	{
		X = 0.0f;
		Y = 0.0f;
		Z = 0.0f;
	}

	KINECTVGB_API FCameraSpacePoint::FCameraSpacePoint(float x, float y, float z)
	{
		X = x;
		Y = y;
		Z = z;
	}

	KINECTVGB_API FColorSpacePoint::FColorSpacePoint()
	{
		X = 0;
		Y = 0;
	}

	KINECTVGB_API FColorSpacePoint::FColorSpacePoint(float x, float y)
	{
		X = x;
		Y = y;
	}

}