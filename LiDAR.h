#pragma once
#include "lib/script.h"
#include "Task.hpp"

//Do not attempt to use multi threads in one single GTAV script. I tried and failed
//Shown in: https://github.com/crosire/scripthookvdotnet/issues/118

class LiDAR : public Task, public ISetDatasetParams, public IShowTaskStates
{
public:
	LiDAR();
	~LiDAR();

	friend class Server;

	enum OPERATION_MODE {
		LIDAR_NOT_INIT_YET,
		LIDAR_INIT_AS_2D,
		LIDAR_INIT_AS_3D_CONE,
		LIDAR_INIT_AS_3D_SCALED_CONE,
		LIDAR_INIT_AS_3D_SPACIALCIRCLE,
		LIDAR_INIT_AS_3D_SCALED_SPACIALCIRCLE
	};

	//Easy APIs for DeepGTAV: set up params before sending a lidar msg
	void setDatasetParams(const Value& dc, Document &d);

	void InitLiDAR();//using this to activate lidar after setup dataset parameters, vehicle ID and camera ID
	
	void showTaskStates();

	//Specific APIs for scaled sample(make point cloud sperad more evenly), call it after setDatasetParams, setVehID, setCamID
	void Init3DLiDAR_Scaled(bool isVisual = false, float maxRange = 100.0f, int totalSmplNum = 1000, float horizLeLimit = 60.0f, 
		float horizRiLimit = 300.0f, int vertiSmplNum = 20, float vertiUpLimit = 85.0f, float vertiUnLimit = 125.0f);

	//Specific APIs for average samples, call it after setDatasetParams, setVehID, setCamID
	//Note: polar-coordinate definition below: 
	//1. Horizontal angle ranges from 0 to 360 degree.
	//	[horizRiLimit, horizLeLimit), anti-clock, namely from rightside to leftside.
	//	horizRiLimit:[180, 360), horizLeLimit:[0, 180). 
	//	For example, shown as defaults in API Init2DLiDAR_SmplNum, right:[270, 360), left:[0, 90).
	//
	//2. Vertical angle ranges from 0 to 180 degree.
	//	[vertiUnLimit, vertiUpLimit), namely from underside to upside. 
	//	vertiUnLimit:[180, 90), vertiUpLimit:[90, 0).
	//	For example, shown as defaults in API Init3DLiDAR_SmplNum, downside:[135, 90), upside:[90, 45).
	//
	//3. One single frame contains :
	//	2D: HorizSmplNum ranges. 
	//	3D: VertiSmplNum * horizSmplNum ranges. 
	//
	//4. By defaults:
	//	Position: The LiDAR device is set to the same position of camera.
	void Init2DLiDAR_SmplNum(bool isVisual = false, float maxRange = 100.0f, int horizSmplNum = 1000, float horizLeLimit = 60.0f, float horizRiLimit = 300.0f);

	void Init3DLiDAR_SmplNum(bool isVisual = false, float maxRange = 100.0f, int horizSmplNum = 60, float horizLeLimit = 60.0f, float horizRiLimit = 300.0f,
		int vertiSmplNum = 20, float vertiUpLimit = 85.0f, float vertiUnLimit = 125.0f);

	void DestroyLiDAR();

	float* GeneratePointClouds();
	int getTotalSmplNum();
	int getVertiSmplNum();
	int getHorizSmplNum();
	int getCurType();

private:

	inline void GenerateSinglePoint(float *scPhi_scTheta, float *p);
	inline void GenerateHorizPointClouds(float phi, float resolu, int smplNum, float *p);
	inline void GenerateHorizPointClouds(float phi, float resolu, int smplNum, float *p, std::vector<float>&sinTheta, std::vector<float>&cosTheta);
	inline void UpdatePosAngles();

private: 
	float* _pointClouds;
	unsigned int _lenght;

	float _maxRange;		//meter
	float _lidarHeight;		//meter
	float _vertiUpLimit;	//deg, the upside limit of zenith direction, namely the min vertical angle, 0 <= up <= phiUp < 90
	float _vertiUnLimit;	//deg, the underside limit of ground direction, namely the max vertical angle, 90 <= phiLo <= un <= 180
	float _horizLeLimit;	//deg, the left limit of horizontal direction, if no limits, set to 180, 0 <= thetaLe < le < 180
	float _horizRiLimit;	//deg, the right limit of horizontal direction, if no limits, set to 180, 180 <= ri <= thetaRi < 360
	int _vertiSmplNum;
	float _vertiResolu;		//deg, vertical angle resolution

	//params for scaled samples, store the results of sin and cos to reduce computation burden
	struct ScalePhiTheta
	{
		int _horizSmplNum_scaled;
		float _horizResolu_sacled;
		std::vector<float> sinTheta;
		std::vector<float> cosTheta;
	};
	std::vector<ScalePhiTheta> _horiz_scaled_helper;
	float _minRange;
	int _totalSmplNum;

	//params for average samples
	int _horizSmplNum;
	float _horizResolu;		//deg, horizontal angle resolution

	Vector3 _curPos;
	float _quaterion[4];
	float _rotDCM[9];		//convert n-coord to b-coord
	enum OPERATION_MODE _initType;
	bool _isVisual;
};