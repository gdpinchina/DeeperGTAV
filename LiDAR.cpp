#include "LiDAR.h"
#include "defaults.h"
#include <math.h>
#include <stdio.h>
#include <ctime>

LiDAR::LiDAR() : _pointClouds(NULL)
{
	_taskName = "lidar";
	DestroyLiDAR();
}

LiDAR::~LiDAR()
{
	DestroyLiDAR();
}

void LiDAR::setDatasetParams(const Value & dc, Document &d)
{
	DestroyLiDAR();
	if (!dc["lidar"].IsNull()) { 
		int type;
		//LiDAR type
		if (!dc["lidar"][0].IsNull()) 
			type = dc["lidar"][0].GetInt();

		switch (type)
		{
		case LiDAR::LIDAR_INIT_AS_2D:
			if (!dc["lidar"][1].IsNull()) _isVisual = dc["lidar"][1].GetBool();
			else _isVisual = false;
			if (!dc["lidar"][2].IsNull()) _maxRange = dc["lidar"][2].GetFloat();
			else _maxRange = 100.0f;
			if (!dc["lidar"][3].IsNull()) _horizSmplNum = dc["lidar"][3].GetInt();
			else _horizSmplNum = 1000;
			if (!dc["lidar"][4].IsNull()) _horizLeLimit = dc["lidar"][4].GetFloat();
			else _horizLeLimit = 60.0f;
			if (!dc["lidar"][5].IsNull()) _horizRiLimit = dc["lidar"][5].GetFloat();
			else _horizRiLimit = 300.0f;
			_initType = LIDAR_INIT_AS_2D;
			break;

		case LiDAR::LIDAR_INIT_AS_3D_CONE:
		case LiDAR::LIDAR_INIT_AS_3D_SPACIALCIRCLE:
			if (!dc["lidar"][1].IsNull()) _isVisual = dc["lidar"][1].GetBool();
			else _isVisual = false;
			if (!dc["lidar"][2].IsNull()) _maxRange = dc["lidar"][2].GetFloat();
			else _maxRange = 100.0f;
			if (!dc["lidar"][3].IsNull()) _horizSmplNum = dc["lidar"][3].GetInt();
			else _horizSmplNum = 60;
			if (!dc["lidar"][4].IsNull()) _horizLeLimit = dc["lidar"][4].GetFloat();
			else _horizLeLimit = 60.0f;
			if (!dc["lidar"][5].IsNull()) _horizRiLimit = dc["lidar"][5].GetFloat();
			else _horizRiLimit = 300.0f;
			if (!dc["lidar"][6].IsNull()) _vertiSmplNum = dc["lidar"][6].GetInt();
			else _vertiSmplNum = 20;
			if (!dc["lidar"][7].IsNull()) _vertiUpLimit = dc["lidar"][7].GetFloat();
			else _vertiUpLimit = 85.0f;
			if (!dc["lidar"][8].IsNull()) _vertiUnLimit = dc["lidar"][8].GetFloat();
			else _vertiUnLimit = 125.0f;
			if(type == LIDAR_INIT_AS_3D_CONE ) _initType = LIDAR_INIT_AS_3D_CONE;
			else _initType = LIDAR_INIT_AS_3D_SPACIALCIRCLE;
			break;

		case LiDAR::LIDAR_INIT_AS_3D_SCALED_CONE:
		case LiDAR::LIDAR_INIT_AS_3D_SCALED_SPACIALCIRCLE:
			if (!dc["lidar"][1].IsNull()) _isVisual = dc["lidar"][1].GetBool();
			else _isVisual = false;
			if (!dc["lidar"][2].IsNull()) _maxRange = dc["lidar"][2].GetFloat();
			else _maxRange = 100.0f;
			if (!dc["lidar"][3].IsNull()) _totalSmplNum = dc["lidar"][3].GetInt();
			else _totalSmplNum = 1000;
			if (!dc["lidar"][4].IsNull()) _horizLeLimit = dc["lidar"][4].GetFloat();
			else _horizLeLimit = 60.0f;
			if (!dc["lidar"][5].IsNull()) _horizRiLimit = dc["lidar"][5].GetFloat();
			else _horizRiLimit = 300.0f;
			if (!dc["lidar"][6].IsNull()) _vertiSmplNum = dc["lidar"][6].GetInt();
			else _vertiSmplNum = 20;
			if (!dc["lidar"][7].IsNull()) _vertiUpLimit = dc["lidar"][7].GetFloat();
			else _vertiUpLimit = 85.0f;
			if (!dc["lidar"][8].IsNull()) _vertiUnLimit = dc["lidar"][8].GetFloat();
			else _vertiUnLimit = 125.0f;
			if (type == LIDAR_INIT_AS_3D_SCALED_CONE) _initType = LIDAR_INIT_AS_3D_SCALED_CONE;
			else _initType = LIDAR_INIT_AS_3D_SCALED_SPACIALCIRCLE;
			break;
		default:
			DestroyLiDAR();
			break;
		}
	}
}

void LiDAR::InitLiDAR()
{
	switch (_initType)
	{
	case LiDAR::LIDAR_INIT_AS_2D:
		Init2DLiDAR_SmplNum(_isVisual, _maxRange, _horizSmplNum, _horizLeLimit, _horizRiLimit);
		break;
	case LiDAR::LIDAR_INIT_AS_3D_CONE:
	case LiDAR::LIDAR_INIT_AS_3D_SPACIALCIRCLE:
		Init3DLiDAR_SmplNum(_isVisual, _maxRange, _horizSmplNum, _horizLeLimit, _horizRiLimit, _vertiSmplNum, _vertiUpLimit, _vertiUnLimit);
		break;
	case LiDAR::LIDAR_INIT_AS_3D_SCALED_CONE:
	case LiDAR::LIDAR_INIT_AS_3D_SCALED_SPACIALCIRCLE:
		Init3DLiDAR_Scaled(_isVisual, _maxRange, _totalSmplNum, _horizLeLimit, _horizRiLimit, _vertiSmplNum, _vertiUpLimit, _vertiUnLimit);
		break;
	default:
		DestroyLiDAR();
		break;
	}
}

void LiDAR::showTaskStates()
{
	switch (_initType)
	{
	case LiDAR::LIDAR_NOT_INIT_YET:
	default:
		printf("\nLiDAR is not activated");
		break;
	case LiDAR::LIDAR_INIT_AS_2D:
		printf("\nDEBUG_CONFIG: function: %s", __FUNCTION__);
		printf("\ncamera=%d, ownCar=%d, maxRange=%f, horizSmplNum=%d, horizLeLimit=%f, horizRiLimit=%f",
			_camera, _vehicle, _maxRange, _horizSmplNum, _horizLeLimit, _horizRiLimit);
		printf("\nHorizontal FOV(yaw definition): %f to %f", -_horizLeLimit, 360.0 - _horizRiLimit);
		printf("\nHorizontal angel resolution(deg): %f", _horizResolu);
		printf("\n");
		break;
	case LiDAR::LIDAR_INIT_AS_3D_CONE:
	case LiDAR::LIDAR_INIT_AS_3D_SPACIALCIRCLE:
		printf("\nDEBUG_CONFIG: function: %s", __FUNCTION__);
		if (_initType == LIDAR_INIT_AS_3D_CONE) printf("\nLiDAR's operation mode is LIDAR_INIT_AS_3D_CONE");
		else  printf("\nLiDAR's operation mode is LIDAR_INIT_AS_3D_SPACIALCIRCLE");
		printf("\ncamera=%d, ownCar=%d, maxRange=%f, horizSmplNum=%d, horizLeLimit=%f, horizRiLimit=%f, vertiSmplNum=%d, vertiUpLimit=%f, vertiUnLimit=%f",
			_camera, _vehicle, _maxRange, _horizSmplNum, _horizLeLimit, _horizRiLimit, _vertiSmplNum, _vertiUpLimit, _vertiUnLimit);
		printf("\nHorizontal FOV(yaw definition): %f to %f", -_horizLeLimit, 360.0 - _horizRiLimit);
		printf("\nVertical FOV(pitch definition): %f to %f", 90.0 - _vertiUpLimit, 90.0 - _vertiUnLimit);
		printf("\nHorizontal angel resolution(deg): %f", _horizResolu);
		printf("\nVertical angel resolution(deg): %f", _vertiResolu);
		printf("\n");
		break;
	case LiDAR::LIDAR_INIT_AS_3D_SCALED_CONE:
	case LiDAR::LIDAR_INIT_AS_3D_SCALED_SPACIALCIRCLE:
		printf("\nDEBUG_CONFIG: function: %s", __FUNCTION__);
		if (_initType == LIDAR_INIT_AS_3D_SCALED_CONE) printf("\nLiDAR's operation mode is LIDAR_INIT_AS_3D_SCALED_CONE");
		else  printf("\nLiDAR's operation mode is LIDAR_INIT_AS_3D_SCALED_SPACIALCIRCLE");
		printf("\ncamera=%d, ownCar=%d, maxRange=%f, _totalSmplNum=%d, horizLeLimit=%f, horizRiLimit=%f, vertiSmplNum=%d, vertiUpLimit=%f, vertiUnLimit=%f, minRange=%f, lidarHeight=%f",
			_camera, _vehicle, _maxRange, _totalSmplNum, _horizLeLimit, _horizRiLimit, _vertiSmplNum, _vertiUpLimit, _vertiUnLimit, _minRange, _lidarHeight);
		printf("\nHorizontal FOV(yaw definition): %f to %f", -_horizLeLimit, 360.0 - _horizRiLimit);
		printf("\nVertical FOV(pitch definition): %f to %f", 90.0 - _vertiUpLimit, 90.0 - _vertiUnLimit);
		printf("\nHorizontal angel sample number:  ");
		for (int i = 0; i < _horiz_scaled_helper.size(); ++i) {
			printf("%d,  ", _horiz_scaled_helper[i]._horizSmplNum_scaled);
		}
		printf("\nHorizontal angel resolution(deg):  ");
		for (int i = 0; i < _horiz_scaled_helper.size(); ++i) {
			printf("%f,  ", _horiz_scaled_helper[i]._horizResolu_sacled);
		}
		printf("\nVertical angel resolution(deg): %f", _vertiResolu);
		printf("\n");
		break;
	}
}

void LiDAR::Init3DLiDAR_Scaled(bool isVisual, float maxRange, int totalSmplNum, float horizLeLimit,
	float horizRiLimit, int vertiSmplNum, float vertiUpLimit, float vertiUnLimit)
{
	const int tmpArrayLen = 200;
	if (vertiSmplNum >= tmpArrayLen) {
		printf("\nToo large vertical sample number");
	}

	_maxRange = maxRange;
	_isVisual = isVisual;

	//Vertical:
	_vertiSmplNum = vertiSmplNum;
	if (vertiUnLimit <= vertiUpLimit)
	{
		printf("\nVertical FOV angle parameters error");
		return;
	}
	_vertiUpLimit = vertiUpLimit;
	_vertiUnLimit = vertiUnLimit;
	_vertiResolu = (_vertiUnLimit - _vertiUpLimit) / _vertiSmplNum;

	//Horizontal: 
	if (horizRiLimit <= horizLeLimit)
	{
		printf("\nHorizontal FOV angle parameters error");
		return;
	}
	_horizLeLimit = horizLeLimit;
	_horizRiLimit = horizRiLimit;
	_totalSmplNum = totalSmplNum;
	float angle = (180.0f - (_vertiUnLimit < 95.0f ? 95.0f : _vertiUnLimit)) * D2R;
	_lidarHeight = _CAM_OFFSET_TO_ENTITY_Z_ + ENTITY::GET_ENTITY_HEIGHT_ABOVE_GROUND(_vehicle);
	_minRange = _lidarHeight * cos(angle);
	float horizFOV = (_horizLeLimit + 360.0f - _horizRiLimit) / 2.0f;
	float eachWid[tmpArrayLen] = { 0 };
	float totalWid = 0.0f;

	//calculate each width of verical sample and total width
	float arcLen = 0.0f;
	double rangeAngle = PI - _vertiUnLimit * D2R, rangeAngleLimit = acos(_lidarHeight / _maxRange);
	for (int i = 0; i < _vertiSmplNum; ++i)
	{
		if (rangeAngle < rangeAngleLimit) {
			arcLen = _lidarHeight / cos(rangeAngle) * tan(horizFOV * D2R) * 2 * PI * horizFOV / 180.0f;
			//arcLen = _lidarHeight / tan(rangeAngle) * 2 * PI * horizFOV / 180.0f;
		}
		else {
			arcLen = _lidarHeight / cos(rangeAngleLimit) * 2 * PI * horizFOV / 180.0f;
		}
		eachWid[i] = arcLen;
		rangeAngle += _vertiResolu * D2R;
		totalWid += eachWid[i];
	}
	
	//calculate each vertical sample number and make the sum equal to _totalSmplNum
	float widResolu = totalWid / _totalSmplNum;
	int eachSmplNum[tmpArrayLen] = { 0 };
	int tmpTotalSmplNum = 0;
	for (int i = 0; i < _vertiSmplNum; ++i)//tmp is the sample number of each vertical sample
	{
		eachSmplNum[i] = eachWid[i] / widResolu;
		tmpTotalSmplNum += eachSmplNum[i];
	}
	if (tmpTotalSmplNum != _totalSmplNum) {//make the sum equal to _totalSmplNum
		if (tmpTotalSmplNum > _totalSmplNum) {
			for (int i = 0; i < tmpTotalSmplNum - _totalSmplNum; ++i) {
				eachSmplNum[_vertiSmplNum - 1 - i] = eachSmplNum[_vertiSmplNum - 1 - i] - 1;
			}
		}
		else {
			for (int i = 0; i < _totalSmplNum - tmpTotalSmplNum; ++i) {
				eachSmplNum[i] = eachSmplNum[i] + 1;
			}
		}
	}
	tmpTotalSmplNum = 0;
	for (int i = 0; i < _vertiSmplNum; ++i)
	{
		tmpTotalSmplNum += eachSmplNum[i];
	}
	if (tmpTotalSmplNum != _totalSmplNum) {
		printf("\nError: Scaled lidar horizontal sample number initializes failed");
	}

	//calculate scaled horizontal resolution
	float eachResolu[tmpArrayLen] = { 0 };
	for (int i = 0; i < _vertiSmplNum; ++i)
	{
		eachResolu[i] = 2.0*horizFOV / eachSmplNum[i];
	}

	ScalePhiTheta tmp;
	for (int i = 0; i < _vertiSmplNum; ++i)
	{
		tmp._horizSmplNum_scaled = eachSmplNum[i];
		tmp._horizResolu_sacled = eachResolu[i];
		_horiz_scaled_helper.push_back(tmp);
	}

	int i, j, k;
	//Right side:
	float theta = _horizRiLimit;
	for (k = 0; k < _vertiSmplNum; ++k) {
		for (j = 0; j < _horiz_scaled_helper[k]._horizSmplNum_scaled; ++j)
		{
			if (theta < 360.0f - _horiz_scaled_helper[k]._horizResolu_sacled)
				theta = _horizRiLimit + j * _horiz_scaled_helper[k]._horizResolu_sacled;
			else
				break;
			_horiz_scaled_helper[k].sinTheta.push_back(sin(theta*D2R));
			_horiz_scaled_helper[k].cosTheta.push_back(cos(theta*D2R));
		}
		//Left side:
		theta = theta - 360.0f;
		for (i = 0; i < eachSmplNum[k] - j; ++i)
		{
			if (theta < _horizLeLimit - _horiz_scaled_helper[k]._horizResolu_sacled)
				theta = 0.0f + i * _horiz_scaled_helper[k]._horizResolu_sacled;
			else
				break;
			_horiz_scaled_helper[k].sinTheta.push_back(sin(theta*D2R));
			_horiz_scaled_helper[k].cosTheta.push_back(cos(theta*D2R));
		}
	}

	if (!_pointClouds) free(_pointClouds);
	_lenght = _totalSmplNum * sizeof(float);
	_pointClouds = (float *)malloc(_lenght);
	if (_pointClouds == NULL)
		printf("\nLiDAR: memory alloc err");

	if (_initType == LiDAR::LIDAR_NOT_INIT_YET) {
		_initType = LiDAR::LIDAR_INIT_AS_3D_SCALED_CONE;
	}

}

void LiDAR::Init2DLiDAR_SmplNum(bool isVisual, float maxRange, int horizSmplNum, float horizLeLimit, float horizRiLimit)
{
	_horizSmplNum = horizSmplNum;
	_maxRange = maxRange;
	_isVisual = isVisual;
	if (horizRiLimit <= horizLeLimit)
	{
		printf("\nHorizontal FOV angle parameters error");
		return;
	}
	_horizLeLimit = horizLeLimit;
	_horizRiLimit = horizRiLimit;
	_horizResolu = (_horizLeLimit + 360.0f - _horizRiLimit) / _horizSmplNum;
	if (!_pointClouds) free(_pointClouds);
	_lenght = _horizSmplNum * sizeof(float);
	_pointClouds = (float *)malloc(_lenght);
	if (_pointClouds == NULL)
		printf("\nLiDAR: memory alloc err");

	_initType = LiDAR::LIDAR_INIT_AS_2D;
}

void LiDAR::Init3DLiDAR_SmplNum(bool isVisual, float maxRange, int horizSmplNum, float horizLeLimit, float horizRiLimit,
	int vertiSmplNum, float vertiUpLimit, float vertiUnLimit)
{
	_vertiSmplNum = vertiSmplNum;
	_horizSmplNum = horizSmplNum;
	_maxRange = maxRange;
	_isVisual = isVisual;

	//Vertical:
	if (vertiUnLimit <= vertiUpLimit)
	{
		printf("\nVertical FOV angle parameters error");
		return; 
	}
	_vertiUpLimit = vertiUpLimit;
	_vertiUnLimit = vertiUnLimit;
	_vertiResolu = (_vertiUnLimit - _vertiUpLimit) / _vertiSmplNum;

	//Horizontal: 
	if (horizRiLimit <= horizLeLimit)
	{
		printf("\nHorizontal FOV angle parameters error");
		return;
	}
	_horizLeLimit = horizLeLimit;
	_horizRiLimit = horizRiLimit;
	_horizResolu = (_horizLeLimit + 360.0f - _horizRiLimit) / _horizSmplNum;

	if (!_pointClouds) free(_pointClouds);
	_lenght = _vertiSmplNum * _horizSmplNum * sizeof(float);
	_pointClouds = (float *)malloc(_lenght);
	if (_pointClouds == NULL)
		printf("\nLiDAR: memory alloc err");

	if (_initType == LiDAR::LIDAR_NOT_INIT_YET) {
		_initType = LiDAR::LIDAR_INIT_AS_3D_CONE;
	}
}

void LiDAR::DestroyLiDAR()
{
	if (_pointClouds)
	{
		free(_pointClouds);
		_pointClouds = NULL;
	}
	_maxRange = 0;
	_lidarHeight = 0;
	_vertiUpLimit = 0;
	_vertiUnLimit = 0;
	_horiz_scaled_helper.clear();
	_horizLeLimit = 0;
	_horizRiLimit = 0;
	_vertiSmplNum = 0;
	_horizSmplNum = 0;
	_lenght = 0;
	_vertiResolu = 0;
	_horizResolu = 0;
	_camera = 0;
	_vehicle = 0;
	_initType = LiDAR::LIDAR_NOT_INIT_YET;
	_isVisual = false;
}

float* LiDAR::GeneratePointClouds()
{
	float phi = _vertiUnLimit;
	int horizOffset = 0;

	UpdatePosAngles();
	switch (_initType)
	{
	case LIDAR_INIT_AS_2D: 
		GenerateHorizPointClouds(90.0f, _horizResolu, _horizSmplNum, _pointClouds);
		break;
	case LIDAR_INIT_AS_3D_CONE: 
	case LIDAR_INIT_AS_3D_SPACIALCIRCLE:
		for (int k = 0; k < _vertiSmplNum; k++)
		{
			if (phi > _vertiUpLimit - _vertiResolu)
				phi = _vertiUnLimit - k * _vertiResolu;
			else
				break;
			GenerateHorizPointClouds(phi, _horizResolu, _horizSmplNum, &_pointClouds[k * _horizSmplNum]);
		}
		break;
	case LIDAR_INIT_AS_3D_SCALED_CONE:
	case LIDAR_INIT_AS_3D_SCALED_SPACIALCIRCLE:
		for (int k = 0; k < _vertiSmplNum; k++)
		{
			if (phi > _vertiUpLimit - _vertiResolu)
				phi = _vertiUnLimit - k * _vertiResolu;
			else
				break;
			horizOffset += k == 0 ? k : _horiz_scaled_helper[k - 1]._horizSmplNum_scaled;
			GenerateHorizPointClouds(
				phi, 
				_horiz_scaled_helper[k]._horizResolu_sacled, 
				_horiz_scaled_helper[k]._horizSmplNum_scaled,
				&_pointClouds[horizOffset],
				_horiz_scaled_helper[k].sinTheta, 
				_horiz_scaled_helper[k].cosTheta
				);
		}
		break;
	default:
		break;
	}
	return _pointClouds;
}

int LiDAR::getTotalSmplNum()
{
	switch (_initType)
	{
	case LIDAR_INIT_AS_2D :
		return _horizSmplNum;
	case LIDAR_INIT_AS_3D_CONE :
		return _horizSmplNum * _vertiSmplNum;
	default :
		return 0;
	}
}

int LiDAR::getVertiSmplNum()
{
	return _vertiSmplNum;
}

int LiDAR::getHorizSmplNum()
{
	return _horizSmplNum;
}

int LiDAR::getCurType()
{
	return _initType;
}

inline void LiDAR::GenerateSinglePoint(float *scPhi_scTheta, float* p)
{
	Vector3 tmp, endCoord; //tmp is target or surfaceNormal
	float range;
	
	//scPhi_scTheta: sin(phi_rad), cos(phi_rad), sin(theta_rad), cos(theta_rad)
	//I define x as forward in body frame, but in GTAV frame, y is the forward(north). That's why the equations are looked wired. But I'm tired to 
	//	make it looked better. It just works, dont worry.
	switch (_initType)
	{
	case LiDAR::LIDAR_INIT_AS_3D_CONE:
	case LiDAR::LIDAR_INIT_AS_3D_SCALED_CONE:
		endCoord.x = _maxRange * scPhi_scTheta[0] * scPhi_scTheta[2];	//rightward(east) is positive
		endCoord.y = _maxRange * scPhi_scTheta[0] * scPhi_scTheta[3];	//forward(north) is positive
		endCoord.z = _maxRange * scPhi_scTheta[1];						//upward(up) is positive
		break;
	case LiDAR::LIDAR_INIT_AS_3D_SPACIALCIRCLE:
	case LiDAR::LIDAR_INIT_AS_3D_SCALED_SPACIALCIRCLE:
		tmp.x = _maxRange * scPhi_scTheta[2];							//rightward(east) is positive
		tmp.y = 0.0f;													//forward(north) is positive
		tmp.z = _maxRange * scPhi_scTheta[3];							//upward(up) is positive
		endCoord.x = tmp.x;
		endCoord.y = tmp.z * scPhi_scTheta[0];
		endCoord.z = tmp.z * scPhi_scTheta[1];
		break;
	}

	tmp.x = _rotDCM[0] * endCoord.x + _rotDCM[1] * endCoord.y + _rotDCM[2] * endCoord.z + _curPos.x;
	tmp.y = _rotDCM[3] * endCoord.x + _rotDCM[4] * endCoord.y + _rotDCM[5] * endCoord.z + _curPos.y;
	tmp.z = _rotDCM[6] * endCoord.x + _rotDCM[7] * endCoord.y + _rotDCM[8] * endCoord.z + _curPos.z;

	//options: -1=everything
	WORLDPROBE::_GET_RAYCAST_RESULT(
		WORLDPROBE::_CAST_RAY_POINT_TO_POINT(	_curPos.x, 
												_curPos.y, 
												_curPos.z, 
												tmp.x,			//target
												tmp.y,			//target
												tmp.z,			//target
												-1, 
												_vehicle, 
												7), 
		(BOOL *)&range,	//is Hit, useless
		&endCoord, 
		&tmp,			//surfaceNormal, useless
		(int *)&tmp._paddingx);//hit entity, useless

	range = sqrt((endCoord.x - _curPos.x) * (endCoord.x - _curPos.x) +
	(endCoord.y - _curPos.y) * (endCoord.y - _curPos.y) +
		(endCoord.z - _curPos.z) * (endCoord.z - _curPos.z));

	*p = range > _maxRange ? _maxRange : range;

	//I dont kown why, activate these seem to be more fluently
	if (_isVisual) {
		GRAPHICS::DRAW_BOX(endCoord.x - 0.05f, endCoord.y - 0.05f, endCoord.z - 0.05f, endCoord.x + 0.05f, endCoord.y + 0.05f, endCoord.z + 0.05f, (short)0, (short)255, (short)0, (short)255);
	}
	GRAPHICS::DRAW_BOX(endCoord.x - 0.05f, endCoord.y - 0.05f, endCoord.z - 0.05f, endCoord.x + 0.05f, endCoord.y + 0.05f, endCoord.z + 0.05f, (short)0, (short)255, (short)0, (short)0);
#ifdef DEBUG_LOG
	printf("\nDEBUG_LOG: function: %s", __FUNCTION__);
	printf("\ntheta=%f, endcoord:x=%f, y=%f, z=%f", __FUNCTION__, theta, endCoord.x, endCoord.y, endCoord.z);
#endif //DEBUG_LOG
}

inline void LiDAR::GenerateHorizPointClouds(float phi, float resolu, int smplNum, float *p)
{
	int i, j;
	float theta, scPhi_scTheta[4] = { sin(phi*D2R), cos(phi*D2R), .0f, .0f };

	//Right side:
	theta = _horizRiLimit;
	for (j = 0; j < smplNum; ++j)
	{
		if (theta < 360.0f - resolu)
			theta = _horizRiLimit + j * resolu;
		else
			break;
		scPhi_scTheta[2] = sin(theta*D2R);
		scPhi_scTheta[3] = cos(theta*D2R);
		GenerateSinglePoint(scPhi_scTheta, p + j);
	}
	//Left side:
	theta = theta - 360.0f + resolu;
	for (i = 0; i < smplNum - j; ++i)
	{
		if (theta < _horizLeLimit)
			theta = 0.0f + i * resolu;
		else
			break;
		scPhi_scTheta[2] = sin(theta*D2R);
		scPhi_scTheta[3] = cos(theta*D2R);
		GenerateSinglePoint(scPhi_scTheta, p + i + j);
	}
}

inline void LiDAR::GenerateHorizPointClouds(float phi, float resolu, int smplNum, float *p, std::vector<float>& sinTheta, std::vector<float>& cosTheta) {
	int i, j;
	float theta, scPhi_scTheta[4] = { sin(phi*D2R), cos(phi*D2R), .0f, .0f };

	//Right side:
	theta = _horizRiLimit;
	for (j = 0; j < smplNum; ++j)
	{
		if (theta < 360.0f - resolu)
			theta = _horizRiLimit + j * resolu;
		else
			break;
		scPhi_scTheta[2] = sinTheta[j];
		scPhi_scTheta[3] = cosTheta[j];
		GenerateSinglePoint(scPhi_scTheta, p + j);
	}
	//Left side:
	theta = theta - 360.0f + resolu;
	for (i = 0; i < smplNum - j; ++i)
	{
		if (theta < _horizLeLimit)
			theta = 0.0f + i * resolu;
		else
			break;
		scPhi_scTheta[2] = sinTheta[i + j];
		scPhi_scTheta[3] = cosTheta[i + j];
		GenerateSinglePoint(scPhi_scTheta, p + i + j);
	}
}

inline void LiDAR::UpdatePosAngles()
{
	_curPos = CAM::GET_CAM_COORD(_camera);

	ENTITY::GET_ENTITY_QUATERNION(_vehicle, &_quaterion[0], &_quaterion[1], &_quaterion[2], &_quaterion[3]);
	//_quaterion: R - coord spins to b - coord
	float q00 = _quaterion[3] * _quaterion[3], q11 = _quaterion[0] * _quaterion[0], q22 = _quaterion[1] * _quaterion[1], q33 = _quaterion[2] * _quaterion[2];
	float q01 = _quaterion[3] * _quaterion[0], q02 = _quaterion[3] * _quaterion[1], q03 = _quaterion[3] * _quaterion[2], q12 = _quaterion[0] * _quaterion[1];
	float q13 = _quaterion[0] * _quaterion[2], q23 = _quaterion[1] * _quaterion[2];
	
	//convert b-vector to R-vector, CbR
	_rotDCM[0] = q00 + q11 - q22 - q33;
	_rotDCM[1] = 2 * (q12 - q03);
	_rotDCM[2] = 2 * (q13 + q02);
	_rotDCM[3] = 2 * (q12 + q03);
	_rotDCM[4] = q00 - q11 + q22 - q33;
	_rotDCM[5] = 2 * (q23 - q01);
	_rotDCM[6] = 2 * (q13 - q02);
	_rotDCM[7] = 2 * (q23 + q01);
	_rotDCM[8] = q00 - q11 - q22 + q33;
}

