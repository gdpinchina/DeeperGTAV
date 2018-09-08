#include "DrivingMode.h"
#include "defaults.h"
#include "behaviorAnalys.hpp"
#include <algorithm>
#include <Eigen/LU>


std::vector<std::string> DrivingMode::_routeModeStr = { "WANDERING", "TO_COORD_ONE_WAY_TRIP", "TO_COORD_CIRCLE_TRIP", "TO_COORD_ONE_WAY_TRIP_CIRCLE" };
std::vector<std::string> DrivingMode::_drivingParamFlagStr = { "DEFAULT_DO_NOTHING", "AUTO_DRIVING_WITH_AUTO_PARAMS", "AUTO_DRIVING_WITH_MANUAL_PARAMS" };


//test: aggressiveness: 
//_egoDrivingStyles({
//	{ "STRICT_1_AGG_0",		STRICT_1,	25.0f,	0.0f,	1.0f },
//	{ "STRICT_1_AGG_100",	STRICT_1,	25.0f,	1.0f,	1.0f },
//	{ "STRICT_2_AGG_0",		STRICT_2,	25.0f,	0.0f,	1.0f },
//	{ "STRICT_2_AGG_100",	STRICT_2,	25.0f,	1.0f,	1.0f },
//	{ "LOOSE_1_AGG_0",		LOOSE_1,	25.0f,	0.0f,	1.0f },
//	{ "LOOSE_1_AGG_100",	LOOSE_1,	25.0f,	1.0f,	1.0f },
//	}),

DrivingMode::DrivingMode() :
	_egoDrivingStyles({
		{ "STRICT_2_SPEED_15",		STRICT_2,	15.0f,	0.0f,	1.0f },
		{ "STRICT_2_SPEED_20",		STRICT_2,	20.0f,	0.0f,	1.0f },
		{ "STRICT_2_SPEED_25",		STRICT_2,	25.0f,	0.0f,	1.0f },
		{ "STRICT_2_SPEED_30",		STRICT_2,	30.0f,	0.0f,	1.0f },
		{ "LOOSE_1_SPEED_15",		LOOSE_1,	15.0f,	1.0f,	1.0f },
		{ "LOOSE_1_SPEED_20",		LOOSE_1,	20.0f,	1.0f,	1.0f },
		{ "LOOSE_1_SPEED_25",		LOOSE_1,	25.0f,	1.0f,	1.0f },
		{ "LOOSE_1_SPEED_30",		LOOSE_1,	30.0f,	1.0f,	1.0f },
		}),
	_surroundingDrivingStyles({
		{ "STRICT_1", STRICT_1, 30.0f, 0.0f, 1.0f },
		{ "LOOSE_1", LOOSE_1, 30.0f, 0.0f, 1.0f },
		{ "LOOSE_2", LOOSE_2, 30.0f, 0.0f, 1.0f }
		}),
	_stopRadius4Dest(2.0),
	_stopRadius4MidPos(25.0),
	_isMsgActivate(false)
{
	_taskName = "drivingMode";
}

DrivingMode::~DrivingMode()
{
}

void DrivingMode::setScenarioParams(const Value& sc)
{
	RoutePos tmp;
	int paramFlag;
	int paramFlagBase = DRIVING_PARAMS_FLAG::AUTO_DRIVING_WITH_MANUAL_PARAMS;
	_route.clear();				//clear route
	_routePosIndex = 0;			//start point
	_tripFrameIndex = 0;		//index of each data frame, here is the beginning
	_curSDSIndex = -1;
	
	const Value& surroundDrivingMode = sc["surroundDrivingMode"];
	const Value& route = sc["route"];
	const Value& drivingMode = sc["drivingMode"];

	_vehSet.clear();
	if (surroundDrivingMode.IsArray()) {
		paramFlag = surroundDrivingMode[0].GetInt();
		if (paramFlag >= DRIVING_PARAMS_FLAG::AUTO_DRIVING_WITH_MANUAL_PARAMS - paramFlagBase) {
			_suroundDrivingParamFlag = DRIVING_PARAMS_FLAG::AUTO_DRIVING_WITH_MANUAL_PARAMS;

			_curSDS.name = "The surrounding manual drving style params";
			_curSDS.drivingStyle = paramFlag;
			if (!surroundDrivingMode[1].IsNull()) _curSDS.desiredSpeed = surroundDrivingMode[1].GetFloat();
			else _curSDS.desiredSpeed = 15.0;
			if (!surroundDrivingMode[2].IsNull()) _curSDS.aggressivenss = surroundDrivingMode[2].GetFloat();
			else _curSDS.aggressivenss = 0.0;
			if (!surroundDrivingMode[3].IsNull()) _curSDS.ability = surroundDrivingMode[3].GetFloat();
			else _curSDS.ability = 100.0;
		}
		else if (paramFlag == DRIVING_PARAMS_FLAG::AUTO_DRIVING_WITH_AUTO_PARAMS - paramFlagBase) {
			_suroundDrivingParamFlag = DRIVING_PARAMS_FLAG::AUTO_DRIVING_WITH_AUTO_PARAMS;

			if (!surroundDrivingMode[1].IsNull()) _curSDSIndex = surroundDrivingMode[1].GetInt();
			else _curSDSIndex = 0;
			_curSDSIndex = _curSDSIndex > _surroundingDrivingStyles.size() - 1 ? 0 : _curSDSIndex;
			_curSDS = _surroundingDrivingStyles[_curSDSIndex];
		}
		else{	//use default GTAV AI as surrounding drivers
			_suroundDrivingParamFlag = DRIVING_PARAMS_FLAG::DEFAULT_DO_NOTHING;
		}
	}
	else {
		_suroundDrivingParamFlag = DRIVING_PARAMS_FLAG::DEFAULT_DO_NOTHING;
	}

	//_route[0] is the starting point
	if (route.IsArray()) {
		if (!route[0].IsNull()) tmp.org.x = route[0].GetFloat();
		else tmp.org.x = 5000 * ((float)rand() / RAND_MAX) - 2500;
		if (!route[1].IsNull()) tmp.org.y = route[1].GetFloat();
		else tmp.org.y = 8000 * ((float)rand() / RAND_MAX) - 2000;
		if (!route[2].IsNull()) tmp.org.z = route[2].GetFloat();
		else tmp.org.z = 100 * ((float)rand() / RAND_MAX);
		_route.push_back(tmp);

		//destination position
		if (route.Size() > 3 && (route.Size() - 3) % 3 == 0)
		{
			for (unsigned int i = 0; i < (route.Size() - 3) / 3; i++)
			{
				tmp.org.x = route[3 + 3 * i].GetFloat();
				tmp.org.y = route[4 + 3 * i].GetFloat();
				tmp.org.z = route[5 + 3 * i].GetFloat();
				_route.push_back(tmp);
			}
		}
	}
	else {
		tmp.org.x = 5000 * ((float)rand() / RAND_MAX) - 2500;
		tmp.org.y = 8000 * ((float)rand() / RAND_MAX) - 2000;
		tmp.org.z = 100 * ((float)rand() / RAND_MAX);
		_route.push_back(tmp);
	}

	if (drivingMode.IsArray()) {
		// paramFlag: the meaning of the 1st param sent by client:
		//	==-2			Manual driving:	just drive
		//	==-1			Auto driving:	auto selecting preinstall driving styles, desired speed. manual setting route mode by turns when the vehicle reach the destination
		//	>= 0			Auto driving:	manual setting  driving styles, desired speed, driving aggressiveness, driving ability and route mode
		paramFlag = drivingMode[0].GetInt();
		if (paramFlag >= DRIVING_PARAMS_FLAG::AUTO_DRIVING_WITH_MANUAL_PARAMS - paramFlagBase) {
			//driving params flag
			_egoDrivingParamFlag = AUTO_DRIVING_WITH_MANUAL_PARAMS;

			//driving route mode, if no destination position, set to WANDERING
			if (_route.size() > 1)
			{
				if (!drivingMode[1].IsNull()) {
					_routeMode = drivingMode[1].GetInt();
					_routeMode = _routeMode < _routeModeStr.size() ? _routeMode : ROUTE_MODE::WANDERING;
				}
				else _routeMode = ROUTE_MODE::WANDERING;
			}
			else _routeMode = ROUTE_MODE::WANDERING;

			_curDSIndex = -1;//means useless

			//driving mode name
			_curDS.name = "manual params";

			//driving style int
			_curDS.drivingStyle = paramFlag;

			//driving speed
			if (!drivingMode[2].IsNull()) _curDS.desiredSpeed = drivingMode[2].GetFloat();
			else _curDS.desiredSpeed = (float)1.0*(rand() % 20);

			//driving aggressiveness
			if (!drivingMode[3].IsNull()) _curDS.aggressivenss = drivingMode[3].GetFloat();
			else _curDS.aggressivenss = 0.0;

			//driving ability
			if (!drivingMode[4].IsNull()) _curDS.ability = drivingMode[4].GetFloat();
			else _curDS.ability = 100.0;
		}
		else if (paramFlag == DRIVING_PARAMS_FLAG::AUTO_DRIVING_WITH_AUTO_PARAMS - paramFlagBase) {
			//driving params flag
			_egoDrivingParamFlag = DRIVING_PARAMS_FLAG::AUTO_DRIVING_WITH_AUTO_PARAMS;

			//driving route mode, if no destination position, set to WANDERING
			if (_route.size() > 1)
			{
				if (!drivingMode[1].IsNull()) {
					_routeMode = drivingMode[1].GetInt();
					_routeMode = _routeMode < _routeModeStr.size() ? _routeMode : ROUTE_MODE::WANDERING;
				}
				else _routeMode = ROUTE_MODE::WANDERING;
			}
			else _routeMode = ROUTE_MODE::WANDERING;

			_curDSIndex = 0;
			_curDS = _egoDrivingStyles[_curDSIndex];
		}
		else{//manual driving
			_egoDrivingParamFlag = DRIVING_PARAMS_FLAG::DEFAULT_DO_NOTHING;
			_curDSIndex = -1;	//-1 means not activated
			_curDS.drivingStyle = -1;
			_curDS.desiredSpeed = -1;
			_curDS.aggressivenss = -1;
			_curDS.ability = -1;
		}
	}
	else {
		_egoDrivingParamFlag = DRIVING_PARAMS_FLAG::DEFAULT_DO_NOTHING; //manual driving
		_curDSIndex = -1;	//-1 means not activated
		_curDS.drivingStyle = -1;
		_curDS.desiredSpeed = -1;
		_curDS.aggressivenss = -1;
		_curDS.ability = -1;
	}
}

void DrivingMode::setDatasetParams(const Value& dc, Document &d)
{
	if (!dc["drivingModeMsg"].IsNull()) _isMsgActivate = dc["drivingModeMsg"].GetBool();
	else _isMsgActivate = _DRIVING_MODE_MSG_;
	if (_isMsgActivate) addMenber2Doc(_taskName.data(), Task::DATA_TYPE::MSG_ARRAY, d);
}

void DrivingMode::beginDrivingTask()
{

	Vector3 pos, check;
	float heading, travelDist, stopRadius;
	DrivingMode::Nodetype nodeType = DrivingMode::Nodetype::AnyRoad;

	//setup AI driver
	AI::CLEAR_PED_TASKS(_ped);

	//verify all original positions and converto vehicle node position
	for (int i = 0; i < _route.size(); i++) {
		getSpawnPos(_route[i].org, nodeType, false, &pos, 0);
		_route[i].node = pos;
	}

	getSpawnPos(_route[0].org, nodeType, false, &pos, &heading);
	assert(_route[0].node.x - pos.x < 1e-5 && _route[0].node.y - pos.y < 1e-5 && _route[0].node.z - pos.z < 1e-5);

	moveVehicle(_route[0].node, heading);

	//there are some differents between entity coordination and vehicle node coordination, more than 0.8 meter
	check = ENTITY::GET_ENTITY_COORDS(_vehicle, false);
	if (SYSTEM::VDIST(_route[0].node.x, _route[0].node.y, _route[0].node.z, check.x, check.y, check.z) < 1.6) printf("\nVehicle is set correctly!");
	while (!VEHICLE::IS_VEHICLE_STOPPED(_vehicle)) WAIT(0);

	//setup weather
	TIME::SET_CLOCK_TIME(_hour, _min, _sec);
	GAMEPLAY::SET_WEATHER_TYPE_NOW_PERSIST((char *)_weather.data());

	_vehSet.insert(_vehicle);
	surroundDriverCheck();
	WAIT(700);//wait for scenerio initializing

	if (_egoDrivingParamFlag != DRIVING_PARAMS_FLAG::DEFAULT_DO_NOTHING)
	{
		AI::SET_DRIVE_TASK_DRIVING_STYLE(_ped, _curDS.drivingStyle);
		//setup initial driving tasks
		switch (_routeMode)
		{
		case ROUTE_MODE::WANDERING:
			AI::TASK_VEHICLE_DRIVE_WANDER(_ped, _vehicle, _curDS.desiredSpeed, _curDS.drivingStyle);
			break;
		case ROUTE_MODE::TO_COORD_ONE_WAY_TRIP:
		case ROUTE_MODE::TO_COORD_CIRCLE_TRIP:
		case ROUTE_MODE::TO_COORD_ONE_WAY_TRIP_CIRCLE:
			_routePosIndex = 1;//the 2rd point
			stopRadius = _route.size() > 2 ? _stopRadius4MidPos : _stopRadius4Dest;
			travelDist = setDrivingTask(pos, _route[1], stopRadius);
			printf("\nBegin driving task, travel distance=%f", travelDist);
			break;
		default:
			break;
		}
	}
	_lastSafetyCheck = std::clock();
	_lastSurroundDriverCheck = std::clock();
}

void DrivingMode::setBeginTimeWeather(int * hms, const char * weather)
{
	_hour = hms[0];
	_min = hms[1];
	_sec = hms[2];
	_weather = weather;
}

void DrivingMode::exeTask()
{
	if (_isMsgActivate) return;
	checkRoute(); //if set to send msgs, dont execute the task. Because the task is executed in exeTaskMsg
}

void DrivingMode::sendTaskMsgs(Document &d)
{
	switch (_isMsgActivate)
	{
	case true:
		checkRoute();
		Document::AllocatorType& allocator = d.GetAllocator();
		Value drivingMode(kArrayType);
		drivingMode.
			PushBack(_tripFrameIndex, allocator).
			PushBack(_curDSIndex, allocator).
			PushBack(_curDS.drivingStyle, allocator).
			PushBack(_curDS.desiredSpeed, allocator).
			PushBack(_curDS.aggressivenss, allocator).
			PushBack(_curDS.ability, allocator);
		_tripFrameIndex++;
		d[_taskName.data()] = drivingMode;
		break;
	}
}

void DrivingMode::showTaskStates()
{
#ifdef DEBUG_CONFIG
	printf("\nDEBUG_CONFIG: function: %s", __FUNCTION__);
	for (int i = 0; i < _route.size(); ++i) {
		printf("\nroute: %d, x=%f, y=%f, z=%f, nodex=%f, nodey=%f, nodez=%f",
			i, _route[i].org.x, _route[i].org.y, _route[i].org.z, _route[i].node.x, _route[i].node.y, _route[i].node.z);
	}

	printf("\negoDrivingParamsFlag=%s, surroundingDrivingParamsFlag=%s", 
		_drivingParamFlagStr[_egoDrivingParamFlag].data(), _drivingParamFlagStr[_suroundDrivingParamFlag].data());
	printf("\nis sending msg=%d", _isMsgActivate);

	if (_suroundDrivingParamFlag != DRIVING_PARAMS_FLAG::DEFAULT_DO_NOTHING) {
		printf("\nSurrounding driving style:");
		printf("\ndrivingStyleIndex=%d, drivingStyle=%d, desiredSpeed=%f,  _curDS.aggressivenss=%f, _curDS.ability=%f", 
			_curSDSIndex, _curSDS.drivingStyle, _curSDS.desiredSpeed, _curSDS.aggressivenss, _curSDS.ability);
	}

	if (_egoDrivingParamFlag != DRIVING_PARAMS_FLAG::DEFAULT_DO_NOTHING) {
		printf("\nEgo driving style:");
		printf("\nrouteMode=%s", _routeModeStr[_routeMode].data());
		printf("\ndrivingStyleIndex=%d, drivingStyle=%d, desiredSpeed=%f,  _curDS.aggressivenss=%f, _curDS.ability=%f", 
			_curDSIndex, _curDS.drivingStyle, _curDS.desiredSpeed, _curDS.aggressivenss, _curDS.ability);
		if (_egoDrivingParamFlag == DRIVING_PARAMS_FLAG::AUTO_DRIVING_WITH_AUTO_PARAMS) {
			assert(_curDSIndex >= 0);
			printf("\ndrivingStyleName=%s", _egoDrivingStyles[_curDSIndex].name.data());
		}
	}
	printf("\n");
#endif // DEBUG_CONFIG
}

inline void DrivingMode::checkRoute()
{
	driverSaferCheck();
	surroundDriverCheck();

	if (_egoDrivingParamFlag == DRIVING_PARAMS_FLAG::DEFAULT_DO_NOTHING) return;

	Vector3 ePos;
	RoutePos rPos;
	float dist, travelDist;

	switch (_routeMode)
	{
	case ROUTE_MODE::WANDERING:
		break;
	case ROUTE_MODE::TO_COORD_ONE_WAY_TRIP:
	case ROUTE_MODE::TO_COORD_ONE_WAY_TRIP_CIRCLE:
		ePos = ENTITY::GET_ENTITY_COORDS(_vehicle, false);
		rPos = _route[_routePosIndex];
		dist = GAMEPLAY::GET_DISTANCE_BETWEEN_COORDS(ePos.x, ePos.y, 0, rPos.node.x, rPos.node.y, 0, false);
		if (_routePosIndex < _route.size() - 1) {			//is a middle node?(not the destination)
			if (dist > _stopRadius4MidPos) break;					//going?
			else {											//reached a node, get the next one
				_routePosIndex++;
				rPos = _route[_routePosIndex];
				travelDist = setDrivingTask(ePos, rPos, _stopRadius4MidPos);
				printf("\nGoing to next middle position, travel distance=%f", travelDist);
			}
		}
		else if (decelerate4Dest(dist, rPos)) {			//reached destination?		
			if (_routeMode == ROUTE_MODE::TO_COORD_ONE_WAY_TRIP_CIRCLE) {
				printf("\nArrived!(TO_COORD_ONE_WAY_TRIP_CIRCLE) Now set the vehicle to the start point");
				beginDrivingTask();
			}
		}
		break;
	case ROUTE_MODE::TO_COORD_CIRCLE_TRIP:
		ePos = ENTITY::GET_ENTITY_COORDS(_vehicle, false);
		rPos = _route[abs(_routePosIndex)];
		dist = GAMEPLAY::GET_DISTANCE_BETWEEN_COORDS(ePos.x, ePos.y, 0, rPos.node.x, rPos.node.y, 0, false);
		if (_routePosIndex > 0)								//going to the destination?
		{
			if (_routePosIndex < _route.size() - 1) {			//is a middle node?(not the destination)		
				if (dist > _stopRadius4MidPos) break;				//going?
				else {
					_routePosIndex++;								//reached a node, get the next one
					rPos = _route[_routePosIndex];
					travelDist = setDrivingTask(ePos, rPos, _stopRadius4MidPos);
					printf("\nGoing to next middle position, travel distance=%f", travelDist);
				}
			}
			else if (decelerate4Dest(dist, rPos)) {			//reached the destination and now returning from destination		
				_routePosIndex = 2 - _route.size();
				rPos = _route[-_routePosIndex];
				travelDist = setDrivingTask(ePos, rPos, _stopRadius4Dest);
				printf("\nArrived!(TO_COORD_CIRCLE_TRIP) Now going to the start point, travelDist=%f, posX=%f, posY=%f, posZ=%f", travelDist, rPos.node.x, rPos.node.y, rPos.node.z);
			}
		}
		else {												//return from the destination?
			if (-_routePosIndex > 0) {							//is a middle node?(not the start point)	
				if (dist > _stopRadius4MidPos) break;						//going?
				else {
					_routePosIndex++;								//reached a node, get the next one
					rPos = _route[-_routePosIndex];
					travelDist = setDrivingTask(ePos, rPos, _stopRadius4MidPos);
					printf("\nGoing to next middle position, travel distance=%f", travelDist);
				}
			}
			else if (decelerate4Dest(dist, rPos)) {			//reached the start point and now going to the destination
				_routePosIndex = 1;
				rPos = _route[_routePosIndex];
				travelDist = setDrivingTask(ePos, rPos, _stopRadius4Dest);
				printf("\nArrived!(TO_COORD_CIRCLE_TRIP) Now going to the destination, travelDist=%f, posX=%f, posY=%f, posZ=%f", travelDist, rPos.node.x, rPos.node.y, rPos.node.z);
			}
		}
		break;
	default:
		break;
	}
#ifdef DEBUG_GRAPHICS_DRIVING_MODE
	for (int i = 0; i < _route.size(); i++)
		GRAPHICS::DRAW_BOX(_route[i].node.x - 0.5, _route[i].node.y - 0.5, _route[i].node.z - 0.5, _route[i].node.x + 0.5, _route[i].node.y + 0.5, _route[i].node.z + 0.5, 255, 0, 0, 255);
#endif // DEBUG_GRAPHICS_DRIVING_MODE
}

inline void DrivingMode::driverSaferCheck()
{
	//Every X sec check whether the vehicle is safe
	std::clock_t now = std::clock();

	//safety check
	if (((float)(now - _lastSafetyCheck)) / CLOCKS_PER_SEC > 20) {
		_lastSafetyCheck = std::clock();
		//Avoid bad things such as getting killed by the police, robbed, dying in car accidents or other horrible stuff
		PLAYER::SET_EVERYONE_IGNORE_PLAYER(_player, TRUE);
		PLAYER::SET_POLICE_IGNORE_PLAYER(_player, TRUE);
		PLAYER::CLEAR_PLAYER_WANTED_LEVEL(_player); // Never wanted
			   
		PED::SET_PED_CONFIG_FLAG(_ped, 32, FALSE);// Put on seat belt

		// Invincible vehicle
		VEHICLE::SET_VEHICLE_TYRES_CAN_BURST(_vehicle, FALSE);
		VEHICLE::SET_VEHICLE_WHEELS_CAN_BREAK(_vehicle, FALSE);
		VEHICLE::SET_VEHICLE_HAS_STRONG_AXLES(_vehicle, TRUE);

		VEHICLE::SET_VEHICLE_CAN_BE_VISIBLY_DAMAGED(_vehicle, FALSE);
		ENTITY::SET_ENTITY_INVINCIBLE(_vehicle, TRUE);
		ENTITY::SET_ENTITY_PROOFS(_vehicle, 1, 1, 1, 1, 1, 1, 1, 1);

		// Player invincible
		PLAYER::SET_PLAYER_INVINCIBLE(_player, TRUE);
	}
}

inline void DrivingMode::surroundDriverCheck()
{
	//Every X se
	std::clock_t now = std::clock();

	if (((float)(now - _lastSurroundDriverCheck)) / CLOCKS_PER_SEC > 2) {
		_lastSurroundDriverCheck = std::clock();

		if (_suroundDrivingParamFlag != DRIVING_PARAMS_FLAG::DEFAULT_DO_NOTHING) {
			//These doesnt work:
			const int ARR_SIZE = 1024;
			Vehicle vehArray[ARR_SIZE] = { 0 };
			Ped driver;
			int count = worldGetAllVehicles(vehArray, ARR_SIZE), i = 0;
			for (; i < count; ++i) {
				if (_vehSet.count(vehArray[i])) continue;
				_vehSet.insert(vehArray[i]);
				driver = VEHICLE::GET_PED_IN_VEHICLE_SEAT(vehArray[i], -1);//-1 == driver
				if (driver != 0 && PED::IS_PED_IN_ANY_VEHICLE(driver, false)) {
					AI::SET_DRIVE_TASK_DRIVING_STYLE(driver, _curSDS.drivingStyle);
					AI::SET_DRIVE_TASK_MAX_CRUISE_SPEED(driver, _curSDS.desiredSpeed);
					PED::SET_DRIVER_AGGRESSIVENESS(driver, _curSDS.aggressivenss);
					PED::SET_DRIVER_ABILITY(driver, _curSDS.ability);
				}
			}
		}
	}
}

float DrivingMode::setDrivingTask(Vector3 &curPos, DrivingMode::RoutePos &destPos, float stopRadius)
{
	PED::SET_DRIVER_AGGRESSIVENESS(_ped, _curDS.aggressivenss);
	PED::SET_DRIVER_ABILITY(_ped, _curDS.ability);
	AI::TASK_VEHICLE_DRIVE_TO_COORD_LONGRANGE(_ped, _vehicle, destPos.node.x, destPos.node.y, destPos.node.z, _curDS.desiredSpeed, _curDS.drivingStyle, stopRadius);
	return PATHFIND::CALCULATE_TRAVEL_DISTANCE_BETWEEN_POINTS(curPos.x, curPos.y, curPos.z, destPos.node.x, destPos.node.y, destPos.node.z);
}

std::vector<DrivingMode::DrivingStyle> DrivingMode::getAllDrivingStyles()
{
	return _egoDrivingStyles;
}

DrivingMode::DRIVING_PARAMS_FLAG DrivingMode::getDrivingParamsFlag()
{
	return _egoDrivingParamFlag;
}

bool DrivingMode::decelerate4Dest(float dist, RoutePos & pos)
{
	if (dist > _stopRadius4Dest + 5.0f + 2.4f*_curDS.desiredSpeed) {
		_decelDesirSpeed = _curDS.desiredSpeed;			//arriving
		return false;
	}
	if (dist < _stopRadius4Dest + 3.0f) {						//arrived
		//stop the vehicle
		CONTROLS::_SET_CONTROL_NORMAL(27, 72, 1);
		if (ROUTE_MODE::TO_COORD_ONE_WAY_TRIP == _routeMode) VEHICLE::SET_VEHICLE_HANDBRAKE(_vehicle, true);

		//sth to do if you reach the destination
		updateAutoDrivingParams();					//get the auto params, only work in AUTO_DRIVING_WITH_AUTO_PARAMS
		_tripFrameIndex = 0;					//data frame index in a single trip, this is the beginning. However, checkRoute and generatingMsg might be asynchronous. So _tripFrameIndex++ is in generatingMsg		
		return true;
	}
	float deltaDist[] = {
		_stopRadius4Dest,
		_stopRadius4Dest + 5.0f,
		_stopRadius4Dest + 5.0f + 0.8f*_curDS.desiredSpeed,
		_stopRadius4Dest + 5.0f + 1.6f*_curDS.desiredSpeed,
		_stopRadius4Dest + 5.0f + 2.4f*_curDS.desiredSpeed };
	float deltaSpeed[] = {
		3.0f,
		0.2f*_curDS.desiredSpeed + 3.0f,
		0.4f*_curDS.desiredSpeed + 3.0f,
		0.6f*_curDS.desiredSpeed + 3.0f };

	for (int i = 0; i < 4; i++) {					//decelerating
		if (deltaDist[i] < dist && dist < deltaDist[i + 1]) {
			if (abs(_decelDesirSpeed - deltaSpeed[i]) < 1e-5);
			else {
				_decelDesirSpeed = deltaSpeed[i];
				AI::TASK_VEHICLE_DRIVE_TO_COORD_LONGRANGE(_ped, _vehicle, pos.node.x, pos.node.y, pos.node.z, _decelDesirSpeed, _curDS.drivingStyle, _stopRadius4Dest);
				//AI::SET_DRIVE_TASK_CRUISE_SPEED(_ped, _decelDesirSpeed);//this will do, too
			}
			break;
		}
	}
	return false;
}

void DrivingMode::updateAutoDrivingParams()
{
	if (_egoDrivingParamFlag == DRIVING_PARAMS_FLAG::AUTO_DRIVING_WITH_AUTO_PARAMS) {
		DrivingStyle tmp;
		printf("\nSet drivingStyleIndex, drvingStyle, desiredSpeed, aggressivenss, ability  from %d, %d, %f, %f, %f  to", _curDSIndex, _curDS.drivingStyle , _curDS.desiredSpeed, _curDS.aggressivenss, _curDS.ability);
		_curDSIndex = _curDSIndex < _egoDrivingStyles.size() - 1 ? _curDSIndex + 1 : 0;
		tmp = _egoDrivingStyles[_curDSIndex];
		_curDS.drivingStyle = tmp.drivingStyle;
		_curDS.desiredSpeed = tmp.desiredSpeed;
		_curDS.aggressivenss = tmp.aggressivenss;
		_curDS.ability = tmp.ability;
		printf("  %d, %d, %f, %f, %f", _curDSIndex, _curDS.drivingStyle, _curDS.desiredSpeed, _curDS.aggressivenss, _curDS.ability);
	}
}

bool DrivingMode::getSpawnPos(Vector3 &desiredPos, Nodetype nodeType, bool sideWalk, Vector3 *outPos, float *outVehHeading)
{
	//ref: http://gtaforums.com/topic/843561-pathfind-node-types
	//PATHFINE MODE
	//ref: http://gtaforums.com/topic/843561-pathfind-node-types    and     http://dev-c.com/nativedb/
	//There seems to be 4 types of nodes :
	//
	//1. Only asphalt road(0, 4, 8, etc).
	//
	//2. Simple path / asphalt road(1, 5, 9, etc).
	//
	//3. Under the map at always the same coords(probably x, y, z all set to 0), somewhat in the middle of the map(2, 6, 10, etc).
	//
	//4. Water(3, 7, 11, 15, 19, 23, 27, 31, 35, 39... 239, etc).
	//
	//The node types follows a pattern.They seems to repeat every fourth node.Asphalt road starts at 0 (0, 4, 8, etc), 
	//simple path / asphalt road at 1 (1, 5, 9, etc), under the map at 2 (2, 6, 10, etc) and water at 3 (3, 7, 11, 15, 19, 23, 27, 31, 35, 39... 239, etc).
	//
	//	either 1 or 12. 1 means any path/road. 12, 8, 0 means node in the middle of the closest main (asphalt) road.
	//_P5_ 300.0f
	//	p5, p6 and p7 seems to be about the same as p4, p5 and p6 for GET_CLOSEST_VEHICLE_NODE. 
	//	p6 and/or p7 has something to do with finding a node on the same path/road and same direction
	//_P6_ 300.0f
	//_P7_ 0

	Vector3 finalPos, checkPos;
	float finalHeading, dist;
	bool forceOffroad = false;
	int nodeNumber = 1, type = 0, nodeID = 0;

	if (nodeType == Nodetype::AnyRoad) type = 1;
	if (nodeType == Nodetype::Road) type = 0;
	if (nodeType == Nodetype::Offroad) { type = 1; forceOffroad = true; }
	if (nodeType == Nodetype::Water) type = 3;

	//move the vehicle to the desired position to initialize that area, or it will retrun wrong vehicle node position
	moveVehicle(desiredPos, 0.0);

	nodeID = PATHFIND::GET_NTH_CLOSEST_VEHICLE_NODE_ID(desiredPos.x, desiredPos.y, desiredPos.z, nodeNumber, type, 300.0f, 300.0f);
	if (forceOffroad) {
		while (!PATHFIND::_GET_IS_SLOW_ROAD_FLAG(nodeID) && nodeNumber < 500) {
			nodeNumber++;
			nodeID = PATHFIND::GET_NTH_CLOSEST_VEHICLE_NODE_ID(desiredPos.x, desiredPos.y, desiredPos.z, nodeNumber, type, 300.0f, 300.0f);
		}
	}
	PATHFIND::GET_VEHICLE_NODE_POSITION(nodeID, &finalPos);
	PATHFIND::GET_CLOSEST_VEHICLE_NODE_WITH_HEADING(finalPos.x, finalPos.y, finalPos.z, &checkPos, &finalHeading, type, 300.0f, 0);
	dist = GAMEPLAY::GET_DISTANCE_BETWEEN_COORDS(finalPos.x, finalPos.y, finalPos.z, checkPos.x, checkPos.y, checkPos.z, true);
	if (dist > 1e-5) {
		printf("\nERROR: function: %s: error in finding position", __FUNCTION__);
		printf("\ndesiredPos : x=%f, y=%f, z=%f", desiredPos.x, desiredPos.y, desiredPos.z);
		printf("\ndist=%f, finalPos: x=%f, y=%f, z=%f; checkPos : x=%f, y=%f, z=%f", dist, finalPos.x, finalPos.y, finalPos.z, checkPos.x, checkPos.y, checkPos.z);
		return false;
	}

	if (sideWalk) {
		if (PATHFIND::GET_SAFE_COORD_FOR_PED(finalPos.x, finalPos.y, finalPos.z, true, &checkPos, 0)) finalPos = checkPos;
		else if (PATHFIND::GET_SAFE_COORD_FOR_PED(finalPos.x, finalPos.y, finalPos.z, false, &checkPos, 0)) finalPos = checkPos;
		else {
			printf("\nERROR: function: %s: error in finding safe position", __FUNCTION__);
			printf("\ndesiredPos : x=%f, y=%f, z=%f", desiredPos.x, desiredPos.y, desiredPos.z);
			printf("\ndist=%f, finalPos: x=%f, y=%f, z=%f; checkPos : x=%f, y=%f, z=%f", dist, finalPos.x, finalPos.y, finalPos.z, checkPos.x, checkPos.y, checkPos.z);
			return false;
		} 
	}

	if (outPos) *outPos = finalPos;
	if (outVehHeading) *outVehHeading = finalHeading;
	return true;
}

void DrivingMode::moveVehicle(Vector3 &dest, float heading)
{
	ENTITY::SET_ENTITY_COORDS(_vehicle, dest.x, dest.y, dest.z + 1.0, 0, 0, 0, 1);
	ENTITY::SET_ENTITY_HEADING(_vehicle, heading);
	Vector3 check = ENTITY::GET_ENTITY_ROTATION(_vehicle, 0);
	CAM::SET_CAM_ROT(_camera, check.x, check.y, check.z, 0);
	VEHICLE::SET_VEHICLE_ON_GROUND_PROPERLY(_vehicle);
	WAIT(700);
}

