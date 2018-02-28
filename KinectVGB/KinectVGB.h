#pragma once

#ifdef KINECTVGB_EXPORTS
#define KINECTVGB_API	__declspec(dllexport)
#else
#define KINECTVGB_API	__declspec(dllimport)
#endif // KINECTVGBEXPORT

namespace Kinect2
{
	enum EJointType
	{
		JointType_SpineBase = 0,
		JointType_SpineMid = 1,
		JointType_Neck = 2,
		JointType_Head = 3,
		JointType_ShoulderLeft = 4,
		JointType_ElbowLeft = 5,
		JointType_WristLeft = 6,
		JointType_HandLeft = 7,
		JointType_ShoulderRight = 8,
		JointType_ElbowRight = 9,
		JointType_WristRight = 10,
		JointType_HandRight = 11,
		JointType_HipLeft = 12,
		JointType_KneeLeft = 13,
		JointType_AnkleLeft = 14,
		JointType_FootLeft = 15,
		JointType_HipRight = 16,
		JointType_KneeRight = 17,
		JointType_AnkleRight = 18,
		JointType_FootRight = 19,
		JointType_SpineShoulder = 20,
		JointType_HandTipLeft = 21,
		JointType_ThumbLeft = 22,
		JointType_HandTipRight = 23,
		JointType_ThumbRight = 24,
		JointType_Count = (JointType_ThumbRight + 1)
	};

	enum ETrackingState
	{
		TrackingState_NotTracked = 0,
		TrackingState_Inferred = 1,
		TrackingState_Tracked = 2
	};

	enum EGestureType
	{
		GestureType_None = 0,
		GestureType_Discrete = 1,
		GestureType_Continuous = 2
	};

	struct FCameraSpacePoint
	{
		KINECTVGB_API FCameraSpacePoint();
		KINECTVGB_API FCameraSpacePoint(float x, float y, float z);
		float X = 0.0f;
		float Y = 0.0f;
		float Z = 0.0f;
	};

	struct FJoint 
	{
		EJointType JointType;
		FCameraSpacePoint Point;
		ETrackingState TrackingState=ETrackingState::TrackingState_NotTracked;
	};


	struct FColorSpacePoint
	{
		KINECTVGB_API FColorSpacePoint();
		KINECTVGB_API FColorSpacePoint(float x, float y);
		float X = 0;
		float Y = 0;
	};

	struct Color
	{
		unsigned char b;
		unsigned char g;
		unsigned char r;
		unsigned char a;
	};

	struct GestureResult
	{
	public:
		KINECTVGB_API GestureResult();
		KINECTVGB_API GestureResult(GestureResult& rhs);
		KINECTVGB_API GestureResult(EGestureType _type, float _confidence, float _progress, wchar_t* _name);
		KINECTVGB_API GestureResult& operator= (GestureResult&);
		KINECTVGB_API ~GestureResult();

		KINECTVGB_API const wchar_t* GetGestureName()const;
		KINECTVGB_API void SetGestureName(const wchar_t* name);
	public:
		EGestureType type = EGestureType::GestureType_None;
		float confidence = 0.0f;
		float progress = 0.0f;
	private:
		wchar_t* gestureName = nullptr;
	};

	class KinectVGB
	{
	public:
		static KINECTVGB_API bool Open();
		static KINECTVGB_API bool CreateVGBDatabaseInstance(wchar_t* path, int* gestureCount);
		static KINECTVGB_API void CoordinateResult(Color* colors);
		static KINECTVGB_API void UpdateJoints(FJoint* joints);
		static KINECTVGB_API FColorSpacePoint ConvertCameraPointToColorSpace(FCameraSpacePoint& cameraPoint);
		static KINECTVGB_API void ConfigLimitArea(wchar_t* path);
		static KINECTVGB_API void ColorImgOptimization(Color* src,Color* dst);
		static KINECTVGB_API void ConfigOptParameter(wchar_t* path);
		static KINECTVGB_API void DetectGestureResult(FJoint* joints, GestureResult* results);
		static KINECTVGB_API bool Close();
	private:
		KinectVGB();
	};
}

