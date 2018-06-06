//driving style flag:
//reference: http://gtaforums.com/topic/822314-guide-driving-styles/page-1
//00000000000000000000000000000001 - 1 - stop before vehicles
//00000000000000000000000000000010 - 2 - stop before peds
//00000000000000000000000000000100 - 4 - avoid vehicles
//00000000000000000000000000001000 - 8 - avoid empty vehicles
//00000000000000000000000000010000 - 16 - avoid peds
//00000000000000000000000000100000 - 32 - avoid objects
//00000000000000000000000001000000 - 64 - ?
//00000000000000000000000010000000 - 128 - stop at traffic lights
//00000000000000000000000100000000 - 256 - use blinkers
//00000000000000000000001000000000 - 512 - allow going wrong way, only does it if the correct lane is full, will try to reach the correct lane again as soon as possible
//00000000000000000000010000000000 - 1024 - go in reverse gear(backwards)
//00000000000000000000100000000000 - 2048 - ?
//00000000000000000001000000000000 - 4096 - ?
//00000000000000000010000000000000 - 8192 - ?
//00000000000000000100000000000000 - 16384 - ?
//00000000000000001000000000000000 - 32768 - ?
//00000000000000010000000000000000 - 65536 - ?
//00000000000000100000000000000000 - 131072 - ?
//00000000000001000000000000000000 - 262144 - Take shortest path(Removes most pathing limits, the driver even goes on dirtroads)
//00000000000010000000000000000000 - 524288 - Probably avoid offroad ?
//00000000000100000000000000000000 - 1048576 - ?
//00000000001000000000000000000000 - 2097152 - ?
//00000000010000000000000000000000 - 4194304 - Ignore roads(Uses local pathing, only works within 200~meters around the player)
//00000000100000000000000000000000 - 8388608 - ?
//00000001000000000000000000000000 - 16777216 - Ignore all pathing(Goes straight to destination)
//00000010000000000000000000000000 - 33554432 - overtake if possible
//00000100000000000000000000000000 - 67108864 - ?
//00001000000000000000000000000000 - 134217728 - ?
//00010000000000000000000000000000 - 268435456 - ?
//00100000000000000000000000000000 - 536870912 - avoid highways when possible(will use the highway if there is no other way to get to the destination)
//01000000000000000000000000000000 - 1073741824 - ?
//10000000000000000000000000000000 - 2147483648
//             1000000000010000011
#pragma once
#include "Task.hpp"
#include <set>
#include <ctime>

class DrivingMode : public Task,public ISetScenarioParams, public ISetDatasetParams, public IExeTask, public ISendTaskMsgs, public IShowTaskStates
{
public:
	DrivingMode();
	~DrivingMode();

	enum Nodetype
	{
		AnyRoad,
		Road, 
		Offroad, 
		Water
	};

	enum ROUTE_MODE
	{
		WANDERING,
		TO_COORD_ONE_WAY_TRIP,			//from A to B, then over
		TO_COORD_CIRCLE_TRIP,			//from A to B, then B to A, then A to B, then B to A, ...
		TO_COORD_ONE_WAY_TRIP_CIRCLE	//from A to B, then A to B, then A to B, then A to B, ...
	};

	enum DRIVING_PARAMS_FLAG
	{
		DEFAULT_DO_NOTHING,
		AUTO_DRIVING_WITH_AUTO_PARAMS,
		AUTO_DRIVING_WITH_MANUAL_PARAMS
	};

	enum DRIVING_STYLE_FLAGS
	{
		FOLLOWING_TRAFFICS =			0x1,//following front vehicles, following traffics
		YIELD_TO_CROSSING_PEDS =		0x2,//yield to crossing peds
		DRIVE_AROUND_VEHICLES =			0x4,//allow bypass vehicles
		DRIVE_AROUND_EMPTY_VEHICLES =	0x8,//allow bypass empty vehicles

		DRIVE_AROUND_PEDS =				0x10,
		DRIVE_AROUND_OBJECTS =			0x20,
		UNKOWN_1 =						0x40,
		STOP_AT_TRAFFIC_LIGHTS =		0x80,

		USE_BLINKERS =					0x100,

		//specific for ALLOW_GOING_WRONG_WAY:
		//If FOLLOWING_TRAFFICS is activated, the driver will usually follow traffics. That's why driving in reverse lane doesnt happen often. 
		//only does it if the correct lane is full, will try to reach the correct lane again as soon as possible. 
		ALLOW_GOING_WRONG_WAY =			0x200,	
		GO_IN_REVERSE_GEAR =			0x400,	//backwards
		UNKOWN_2 =						0x800,	

		UNKOWN_3 =						0x1000,	
		UNKOWN_4 =						0x2000,	
		UNKOWN_5 =						0x4000,
		UNKOWN_6 =						0x8000,

		UNKOWN_7 =						0x10000,
		UNKOWN_8 =						0x20000,
		TAKE_SHORTEST_PATH =			0x40000, //Removes most pathing limits, the driver even goes on dirtroads. 
		ALLOW_LANE_CHANEING =	0x80000, //try to reach desired speed

		UNKOWN_9 =						0x100000,
		UNKOWN_10 =						0x200000,
		IGNORE_ROADS =					0x400000, //Uses local pathing, only works within 200~meters around the player
		UNKOWN_11 =						0x800000,

		IGNORE_ALL_PATHING =			0x1000000, //Goes straight to destination
		UNKOWN_12 =						0x2000000, 
		UNKOWN_13 =						0x4000000,
		UNKOWN_14 =						0x8000000, 

		UNKOWN_15 =						0x10000000,
		AVOID_HIGHWAYS_WHEN_POSSIBLE =	0x20000000, //will use the highway if there is no other way to get to the destination
		UNKOWN_16 =						0x40000000
		//signed flag useless			0x80000000
	};

	enum DRIVING_STYLE
	{
		AVOID_TRAFFIC_EXTREMELY = 
			DRIVING_STYLE_FLAGS::YIELD_TO_CROSSING_PEDS			| 
			DRIVING_STYLE_FLAGS::DRIVE_AROUND_VEHICLES			|
			DRIVING_STYLE_FLAGS::UNKOWN_16,

		RUSHED =  
			DRIVING_STYLE_FLAGS::FOLLOWING_TRAFFICS				| 
			DRIVING_STYLE_FLAGS::YIELD_TO_CROSSING_PEDS			|
			DRIVING_STYLE_FLAGS::DRIVE_AROUND_VEHICLES			| 
			DRIVING_STYLE_FLAGS::DRIVE_AROUND_OBJECTS			|
			DRIVING_STYLE_FLAGS::ALLOW_GOING_WRONG_WAY			|
			DRIVING_STYLE_FLAGS::TAKE_SHORTEST_PATH				| 
			DRIVING_STYLE_FLAGS::ALLOW_LANE_CHANEING			|
			DRIVING_STYLE_FLAGS::IGNORE_ROADS					|
			DRIVING_STYLE_FLAGS::UNKOWN_16,

		NORMAL = 
			DRIVING_STYLE_FLAGS::FOLLOWING_TRAFFICS				| 
			DRIVING_STYLE_FLAGS::YIELD_TO_CROSSING_PEDS			| 
			DRIVING_STYLE_FLAGS::DRIVE_AROUND_EMPTY_VEHICLES	|
			DRIVING_STYLE_FLAGS::DRIVE_AROUND_OBJECTS			|
			DRIVING_STYLE_FLAGS::STOP_AT_TRAFFIC_LIGHTS			|
			DRIVING_STYLE_FLAGS::USE_BLINKERS					|
			DRIVING_STYLE_FLAGS::TAKE_SHORTEST_PATH				| 
			DRIVING_STYLE_FLAGS::ALLOW_LANE_CHANEING			|
			DRIVING_STYLE_FLAGS::UNKOWN_16
	};

	struct RoutePos
	{
		Vector3 org;
		Vector3 node;
	};

	struct DrivingStyle
	{
		std::string name;
		int drivingStyle;
		float desiredSpeed;
		float aggressivenss;		//0~1.0
		float ability;				//0~1.0
	};

	void setScenarioParams(const Value& sc);
	void setDatasetParams(const Value& dc, Document &d);
	void exeTask();
	void sendTaskMsgs(Document &d);
	void showTaskStates();

	void beginDrivingTask();
	void setBeginTimeWeather(int *hms, const char *weather);
	std::vector<DrivingStyle> getAllDrivingStyles();
	DrivingMode::DRIVING_PARAMS_FLAG getDrivingParamsFlag();

	std::vector<RoutePos> _route;	//start pos, pos1, pos2, ..., dest pos; xyz is origin pos; vNodeXYZ is converted node pos;			
	
private:

	inline void checkRoute();
	inline void driverSaferCheck();
	inline void surroundDriverCheck(); //TODO: it doesnt work

	//helper 
	inline bool decelerate4Dest(float dist, RoutePos & pos);
	void updateAutoDrivingParams();
	float setDrivingTask(Vector3 &curPos, DrivingMode::RoutePos &destPos, float stopRadius);
	bool getSpawnPos(Vector3 &desiredPos, Nodetype nodeType, bool sideWalk, Vector3 *outputPos, float *outputHeading);//move the vehicle to the original position to initialize that area. 
	void moveVehicle(Vector3 &dest, float heading);

	//preinstall ego driving params:
	std::vector<DrivingStyle>		_egoDrivingStyles;	
	DRIVING_PARAMS_FLAG				_egoDrivingParamFlag;
	DrivingStyle					_curDS;
	int								_curDSIndex;				//-1: manual params, >=0: auto params
	int								_routeMode;					//WANDERING, TO_COORD_ONE_WAY_TRIP, TO_COORD_CIRCLE_TRIP
	static std::vector<std::string>	_routeModeStr;
	int								_routePosIndex;				//>0 going; <0 returning;
	float							_decelDesirSpeed;
	int								_tripFrameIndex;			//data frame index in a single trip, 0 means the beginning
	float							_stopRadius4Dest;
	float							_stopRadius4MidPos;

	//preinstall surrounding driving params:
	std::vector<DrivingStyle>		_surroundingDrivingStyles;
	DRIVING_PARAMS_FLAG				_suroundDrivingParamFlag;
	DrivingStyle					_curSDS;
	int								_curSDSIndex;
	std::set<int>					_vehSet;

	//mix:
	static std::vector<std::string>	_drivingParamFlagStr;
	bool _isMsgActivate;
	std::clock_t _lastSafetyCheck;
	std::clock_t _lastSurroundDriverCheck;
	int _hour, _min, _sec;
	std::string _weather;

	bool _isReady;
	
};