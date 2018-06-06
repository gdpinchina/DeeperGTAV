#include "ScenarioMsg.h"
#include "defaults.h"
#include "VisionNative.h"
#include "Server.h"
#include <algorithm>
#include <Eigen/LU>

typedef void (ScenarioMsg::*TASK) (Document &d);

std::vector<ScenarioMsg::Content> ScenarioMsg::_allTaskList = {
{ "throttle",			&ScenarioMsg::throttle,		false, 	_THROTTLE_,		 Task::DATA_TYPE::MSG_FLOAT },
{ "brake",				&ScenarioMsg::brake,		false, 	_BRAKE_,		 Task::DATA_TYPE::MSG_FLOAT },
{ "steering",			&ScenarioMsg::steering,		false, 	_STEERING_,		 Task::DATA_TYPE::MSG_FLOAT },
{ "speed",				&ScenarioMsg::speed,		false, 	_SPEED_,		 Task::DATA_TYPE::MSG_FLOAT },
{ "acceleration",		&ScenarioMsg::acceleration,	false, 	_ACCELERATION_,  Task::DATA_TYPE::MSG_FLOAT },
{ "yaw",				&ScenarioMsg::yaw,			false, 	_YAW_,			 Task::DATA_TYPE::MSG_FLOAT },
{ "yawRate",			&ScenarioMsg::yawRate,		false, 	_YAW_RATE_,		 Task::DATA_TYPE::MSG_FLOAT },
{ "time",				&ScenarioMsg::time,			false, 	_TIME_,			 Task::DATA_TYPE::MSG_INT },
{ "isCollide",			&ScenarioMsg::isCollide,	false, 	_IS_COLLIDE_,	 Task::DATA_TYPE::MSG_INT },
{ "eulerAngles",		&ScenarioMsg::eulerAngles,	false,	_EULER_ANGLES_,	 Task::DATA_TYPE::MSG_ARRAY },
{ "location",			&ScenarioMsg::location,		false,	_LOCATION_,		 Task::DATA_TYPE::MSG_ARRAY },
{ "cameraInfo",			&ScenarioMsg::cameraInfo,	false,	_CAMERA_INFO_,	 Task::DATA_TYPE::MSG_ARRAY },
{ "rageMatrices",		&ScenarioMsg::rageMatrices,	false,	_RAGE_MATRIX_,	 Task::DATA_TYPE::MSG_ARRAY },
{ "vehicles",			&ScenarioMsg::vehicles,		false,	_VEHICLES_,		 Task::DATA_TYPE::MSG_ARRAY },
{ "peds",				&ScenarioMsg::peds,			false,	_PEDS_,			 Task::DATA_TYPE::MSG_ARRAY },
{ "trafficSigns",		&ScenarioMsg::trafficSigns,	false,	_TRAFFIC_SIGNS_, Task::DATA_TYPE::MSG_ARRAY } };

ScenarioMsg::ScenarioMsg()
{
	_taskName = "ScenarioMsg";
}

ScenarioMsg::~ScenarioMsg()
{
}

void ScenarioMsg::setDatasetParams(const Value & dc, Document &d)
{
	Document::AllocatorType& allocator = d.GetAllocator();
	_taskList.clear();
	for (int i = 0; i < _allTaskList.size(); ++i) {
		_allTaskList[i].isActivate = _allTaskList[i].defaultState;	//default state

		if (!dc[_allTaskList[i].taskName.data()].IsNull()) 
			_allTaskList[i].isActivate = dc["throttle"].GetBool();
		if (_allTaskList[i].isActivate) {
			addMenber2Doc(_allTaskList[i].taskName.data(), _allTaskList[i].dataType, d);
			_taskList.push_back(&_allTaskList[i]);
		}
	}
}

void ScenarioMsg::sendTaskMsgs(Document & d)
{
	TASK t;
	for (int i = 0; i < _taskList.size(); ++i) {
		t = _taskList[i]->task;
		(this->*t)(d); //or //(this->*((TASK)_taskList[i]->task))(d);
	}
}

void ScenarioMsg::showTaskStates()
{
	//helper initialize
	curSpeed = ENTITY::GET_ENTITY_SPEED(_vehicle);
#ifdef DEBUG_CONFIG
	printf("\nDEBUG_CONFIG: function: %s", __FUNCTION__);
	for (int i = 0; i < _allTaskList.size(); i++) printf("\n%s=%d", _allTaskList[i].taskName.data(), _allTaskList[i].isActivate);
	printf("\n");
#endif // DEBUG_CONFIG
}

void ScenarioMsg::throttle(Document & d)
{
	d["throttle"] = abs(getFloatValue(_vehicle, 0x92C));//[0,1]
}

void ScenarioMsg::brake(Document & d)
{
	d["brake"] = abs(getFloatValue(_vehicle, 0x930));//[0,1]
}

void ScenarioMsg::steering(Document & d)
{
	d["steering"] = -getFloatValue(_vehicle, 0x924) / 0.6981317008;//[-1,1]
}

void ScenarioMsg::speed(Document & d)
{
	d["speed"] = ENTITY::GET_ENTITY_SPEED(_vehicle);
}

void ScenarioMsg::acceleration(Document & d)
{
	float intv = (float)(std::clock() - Server::getLastSentTime()) / CLOCKS_PER_SEC;
	float speed = ENTITY::GET_ENTITY_SPEED(_vehicle);
	d["acceleration"] = (speed - curSpeed) / intv;
	curSpeed = speed;
}

void ScenarioMsg::yaw(Document & d)
{
	d["yaw"] = ENTITY::GET_ENTITY_ROTATION(_vehicle, 0).z * D2R;
}

void ScenarioMsg::yawRate(Document & d)
{
	d["yawRate"] = ENTITY::GET_ENTITY_ROTATION_VELOCITY(_vehicle).z;		//rad/s
}

void ScenarioMsg::time(Document & d)
{
	d["time"] = TIME::GET_CLOCK_HOURS();
}

void ScenarioMsg::isCollide(Document & d)
{
	d["isCollide"] = ENTITY::HAS_ENTITY_COLLIDED_WITH_ANYTHING(_vehicle);
}

void ScenarioMsg::eulerAngles(Document &d)
{
	Document::AllocatorType& allocator = d.GetAllocator();
	Value eA(kArrayType);
	Vector3 nbAttitude = ENTITY::GET_ENTITY_ROTATION(_vehicle, 0);
	eA.PushBack(nbAttitude.x*D2R, allocator).PushBack(nbAttitude.y*D2R, allocator).PushBack(nbAttitude.z*D2R, allocator);
	d["eulerAngles"] = eA;
}

void ScenarioMsg::location(Document &d) {
	Document::AllocatorType& allocator = d.GetAllocator();
	Value lc(kArrayType);
	Vector3 nPos = ENTITY::GET_ENTITY_COORDS(_vehicle, false);
	lc.PushBack(nPos.x, allocator).PushBack(nPos.y, allocator).PushBack(nPos.z, allocator);
	d["location"] = lc;
}

void ScenarioMsg::cameraInfo(Document &d)
{
	Document::AllocatorType& allocator = d.GetAllocator();
	Value cI(kObjectType);
	Value jsVec(kArrayType);

	Vector3 cameraPos = CAM::GET_CAM_COORD(_camera);
	Vector3 cameraRot = CAM::GET_CAM_ROT(_camera, 1);
	float cameraFOV = CAM::GET_CAM_FOV(_camera);

	jsVec.PushBack(cameraPos.x, allocator).PushBack(cameraPos.y, allocator).PushBack(cameraPos.z, allocator);
	cI.AddMember("cameraPos", jsVec, allocator);
	jsVec.SetArray();

	jsVec.PushBack(cameraRot.x, allocator).PushBack(cameraRot.y, allocator).PushBack(cameraRot.z, allocator);
	cI.AddMember("cameraRot", jsVec, allocator);
	jsVec.SetArray();

	jsVec.PushBack(cameraFOV, allocator);
	cI.AddMember("cameraFOV", jsVec, allocator);
	jsVec.SetArray();

	d["cameraInfo"] = cI;
}

void ScenarioMsg::rageMatrices(Document &d)
{
	Document::AllocatorType& allocator = d.GetAllocator();
	rage_matrices const* rm = VisionNative::GetInstance()->GetConstants();

	Value rM(kObjectType);
	Value jsVec(kArrayType);

	if (rm != NULL)
	{
		//transform sequence of the following matrices is from right to left
		//alias: worldview=WV, worldViewProjection=WVP
		//the actual sequence: V=VW*inv(W), P=PVW*inv(VW)=PVW*inv(W)*inv(V)
		Eigen::Matrix4f I = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f W = rm->world;
		Eigen::Matrix4f V = rm->worldView * W.lu().solve(I);
		Eigen::Matrix4f P = rm->worldViewProjection * rm->worldView.lu().solve(I);
		int i;
		for (i = 0; i < W.size(); i++)
			jsVec.PushBack(W(i), allocator);
		rM.AddMember("worldMatrix", jsVec, allocator);
		jsVec.SetArray();
		for (i = 0; i < V.size(); i++)
			jsVec.PushBack(V(i), allocator);
		rM.AddMember("viewMatrix", jsVec, allocator);
		jsVec.SetArray();
		for (i = 0; i < P.size(); i++)
			jsVec.PushBack(P(i), allocator);
		rM.AddMember("projectionMatrix", jsVec, allocator);
		jsVec.SetArray();
	}

	d["rageMatrices"] = rM;
}

void ScenarioMsg::vehicles(Document & d)
{
	const int ARR_SIZE = 1024;
	const float vehListMaxRange = 22500; //150^2=22500.
	Vehicle vehArray[ARR_SIZE];
	Value vehsJsVal(kArrayType);
	Document::AllocatorType& allocator = d.GetAllocator();

	Vector3 FUR; //Front Upper Right, the first element of 3D bounding box
	Vector3 BLL; //Back Lower Lelft, the second element of 3D bounding box
	Vector3 dim; //Vehicle dimensions
	Vector3 upVector, rightVector, forwardVector, eulerAttiAngles, position; //Vehicles' attitude angles(xroll,ypitch,zhead) and position
	Hash model;
	Vector3 min, max;
	Vector3 speedVector;
	float dist, heading, direction, speed, accelerate;
	int classid;

	Vector3 currentPos = ENTITY::GET_ENTITY_COORDS(_vehicle, false);
	Vector3 currentForwardVector = ENTITY::GET_ENTITY_FORWARD_VECTOR(_vehicle);

	int count = worldGetAllVehicles(vehArray, ARR_SIZE);
	for (int i = 0; i < count; i++) {
		if (vehArray[i] == _vehicle) continue; //Don't process own car!
		if (ENTITY::IS_ENTITY_ON_SCREEN(vehArray[i])) {//Check if it is in screen
			ENTITY::GET_ENTITY_MATRIX(vehArray[i], (Any *)&rightVector, (Any *)&forwardVector, &upVector, &position); //Blue or red pill
			if (abs(position.z - currentPos.z) > 15) continue;//excluding vehs on viaducts above														  
			if (SYSTEM::VDIST2(currentPos.x, currentPos.y, currentPos.z, position.x, position.y, position.z) < vehListMaxRange) {//SYSTEM::VDIST2 return without sqrt, namely 150m distance equal 22500
				if (ENTITY::HAS_ENTITY_CLEAR_LOS_TO_ENTITY(_vehicle, vehArray[i], 19)) {//Check if we see it (not occluded)			
					model = ENTITY::GET_ENTITY_MODEL(vehArray[i]);
					GAMEPLAY::GET_MODEL_DIMENSIONS(model, &min, &max);
								
					speed = ENTITY::GET_ENTITY_SPEED(vehArray[i]);
					speedVector = ENTITY::GET_ENTITY_SPEED_VECTOR(vehArray[i], false);
					if (speed > 0) {
						heading = GAMEPLAY::GET_HEADING_FROM_VECTOR_2D(speedVector.x - currentForwardVector.x, speedVector.y - currentForwardVector.y);
					}
					else {
						heading = GAMEPLAY::GET_HEADING_FROM_VECTOR_2D(forwardVector.x - currentForwardVector.x, forwardVector.y - currentForwardVector.y);
					}
					dist = GAMEPLAY::GET_DISTANCE_BETWEEN_COORDS(position.x, position.y, position.z, currentPos.x, currentPos.y, currentPos.z, true);
					direction = GAMEPLAY::GET_HEADING_FROM_VECTOR_2D(position.x - currentPos.x, position.y - currentPos.y);//0~360.0£¬ relative to ego car
					direction = (direction > 180.0 ? direction - 360.0 : direction) * D2R;//left:0~+180, right:0~-180
					eulerAttiAngles = ENTITY::GET_ENTITY_ROTATION(vehArray[i], 0);
					accelerate = VEHICLE::GET_VEHICLE_ACCELERATION(_vehicle);

					if (VEHICLE::IS_THIS_MODEL_A_CAR(model)) classid = 0;
					else if (VEHICLE::IS_THIS_MODEL_A_BIKE(model)) classid = 1;
					else if (VEHICLE::IS_THIS_MODEL_A_BICYCLE(model)) classid = 2;
					else if (VEHICLE::IS_THIS_MODEL_A_QUADBIKE(model)) classid = 3;
					else if (VEHICLE::IS_THIS_MODEL_A_BOAT(model)) classid = 4;
					else if (VEHICLE::IS_THIS_MODEL_A_PLANE(model)) classid = 5;
					else if (VEHICLE::IS_THIS_MODEL_A_HELI(model)) classid = 6;
					else if (VEHICLE::IS_THIS_MODEL_A_TRAIN(model)) classid = 7;
					else if (VEHICLE::_IS_THIS_MODEL_A_SUBMERSIBLE(model)) classid = 8;
					else classid = 9; //unknown (ufo?)

									  //Calculate size
					dim.x = 0.5*(max.x - min.x);
					dim.y = 0.5*(max.y - min.y);
					dim.z = 0.5*(max.z - min.z);

					FUR.x = position.x + dim.y*rightVector.x + dim.x*forwardVector.x + dim.z*upVector.x;
					FUR.y = position.y + dim.y*rightVector.y + dim.x*forwardVector.y + dim.z*upVector.y;
					FUR.z = position.z + dim.y*rightVector.z + dim.x*forwardVector.z + dim.z*upVector.z;
					//GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(FUR.x, FUR.y, FUR.z, &(FUR.z), 0);
					//FUR.z += 2 * dim.z;

					BLL.x = position.x - dim.y*rightVector.x - dim.x*forwardVector.x - dim.z*upVector.x;
					BLL.y = position.y - dim.y*rightVector.y - dim.x*forwardVector.y - dim.z*upVector.y;
					BLL.z = position.z - dim.y*rightVector.z - dim.x*forwardVector.z - dim.z*upVector.z;
					//GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(BLL.x, BLL.y, 1000.0, &(BLL.z), 0);

					Value vehJsVal(kObjectType);
					Value jsVec(kArrayType);
#if 0
					jsVec.PushBack(FUR.x - currentPos.x, allocator).PushBack(FUR.y - currentPos.y, allocator).PushBack(FUR.z - currentPos.z, allocator);
					vehJsVal.AddMember("FUR", jsVec, allocator);
					jsVec.SetArray();
					jsVec.PushBack(BLL.x - currentPos.x, allocator).PushBack(BLL.y - currentPos.y, allocator).PushBack(BLL.z - currentPos.z, allocator);
					vehJsVal.AddMember("BLL", jsVec, allocator);
					jsVec.SetArray();
					for (int j = 0; j < 8; j++)
						jsVec.PushBack(edges[j].x - currentPos.x, allocator).PushBack(edges[j].y - currentPos.y, allocator).PushBack(edges[j].z - currentPos.z, allocator);
					vehJsVal.AddMember("bbEdges", jsVec, allocator);
					jsVec.SetArray();
					jsVec.PushBack(position.x, allocator).PushBack(position.y, allocator).PushBack(position.z, allocator);
					vehJsVal.AddMember("position", jsVec, allocator);
					jsVec.SetArray();
					jsVec.PushBack(eulerAttiAngles.x*D2R, allocator).PushBack(eulerAttiAngles.y*D2R, allocator).PushBack(eulerAttiAngles.z*D2R, allocator);
					vehJsVal.AddMember("eulerAngles", jsVec, allocator);
					jsVec.SetArray();
					vehJsVal.AddMember("heading", heading*D2R, allocator);
#endif // 0
					vehJsVal.AddMember("distance", dist, allocator).AddMember("direction", direction, allocator).AddMember("yaw", eulerAttiAngles.z*D2R, allocator).AddMember("speed", speed, allocator)
						.AddMember("acceleration", accelerate, allocator).AddMember("classID", classid, allocator);

					vehsJsVal.PushBack(vehJsVal, allocator);

#ifdef DEBUG_GRAPHICS_VEH
					Vector3 edges[8];
					edges[0] = BLL;
					edges[4] = FUR;

					edges[1].x = edges[0].x + 2 * dim.y*rightVector.x;
					edges[1].y = edges[0].y + 2 * dim.y*rightVector.y;
					edges[1].z = edges[0].z + 2 * dim.y*rightVector.z;

					edges[2].x = edges[1].x + 2 * dim.z*upVector.x;
					edges[2].y = edges[1].y + 2 * dim.z*upVector.y;
					edges[2].z = edges[1].z + 2 * dim.z*upVector.z;

					edges[3].x = edges[0].x + 2 * dim.z*upVector.x;
					edges[3].y = edges[0].y + 2 * dim.z*upVector.y;
					edges[3].z = edges[0].z + 2 * dim.z*upVector.z;

					edges[5].x = edges[4].x - 2 * dim.y*rightVector.x;
					edges[5].y = edges[4].y - 2 * dim.y*rightVector.y;
					edges[5].z = edges[4].z - 2 * dim.y*rightVector.z;

					edges[6].x = edges[5].x - 2 * dim.z*upVector.x;
					edges[6].y = edges[5].y - 2 * dim.z*upVector.y;
					edges[6].z = edges[5].z - 2 * dim.z*upVector.z;

					edges[7].x = edges[4].x - 2 * dim.z*upVector.x;
					edges[7].y = edges[4].y - 2 * dim.z*upVector.y;
					edges[7].z = edges[4].z - 2 * dim.z*upVector.z;
					GRAPHICS::DRAW_LINE(edges[0].x, edges[0].y, edges[0].z, edges[1].x, edges[1].y, edges[1].z, 0, 255, 0, 200);
					GRAPHICS::DRAW_LINE(edges[0].x, edges[0].y, edges[0].z, edges[3].x, edges[3].y, edges[3].z, 0, 255, 0, 200);
					GRAPHICS::DRAW_LINE(edges[1].x, edges[1].y, edges[1].z, edges[2].x, edges[2].y, edges[2].z, 0, 255, 0, 200);
					GRAPHICS::DRAW_LINE(edges[2].x, edges[2].y, edges[2].z, edges[3].x, edges[3].y, edges[3].z, 0, 255, 0, 200);

					GRAPHICS::DRAW_LINE(edges[4].x, edges[4].y, edges[4].z, edges[5].x, edges[5].y, edges[5].z, 0, 255, 0, 200);
					GRAPHICS::DRAW_LINE(edges[4].x, edges[4].y, edges[4].z, edges[7].x, edges[7].y, edges[7].z, 0, 255, 0, 200);
					GRAPHICS::DRAW_LINE(edges[5].x, edges[5].y, edges[5].z, edges[6].x, edges[6].y, edges[6].z, 0, 255, 0, 200);
					GRAPHICS::DRAW_LINE(edges[6].x, edges[6].y, edges[6].z, edges[7].x, edges[7].y, edges[7].z, 0, 255, 0, 200);

					GRAPHICS::DRAW_LINE(edges[0].x, edges[0].y, edges[0].z, edges[6].x, edges[6].y, edges[6].z, 0, 255, 0, 200);
					GRAPHICS::DRAW_LINE(edges[1].x, edges[1].y, edges[1].z, edges[7].x, edges[7].y, edges[7].z, 0, 255, 0, 200);
					GRAPHICS::DRAW_LINE(edges[2].x, edges[2].y, edges[2].z, edges[4].x, edges[4].y, edges[4].z, 0, 255, 0, 200);
					GRAPHICS::DRAW_LINE(edges[3].x, edges[3].y, edges[3].z, edges[5].x, edges[5].y, edges[5].z, 0, 255, 0, 200);
#endif//DEBUG_GRAPHICS_VEH
				}
			}
		}
	}

	d["vehicles"] = vehsJsVal;
}

void ScenarioMsg::peds(Document & d)
{
	const int ARR_SIZE = 1024;
	const float pedListMaxRange = 22500;//150^2=22500.
	Ped pedArray[ARR_SIZE];
	Value pedsJsVal(kArrayType);
	Document::AllocatorType& allocator = d.GetAllocator();

	Vector3 FUR; //Front Upper Right
	Vector3 BLL; //Back Lower Lelft
	Vector3 dim; //Vehicle dimensions
	Vector3 upVector, rightVector, forwardVector, eulerAttiAngles, position; //Peds' attitude angles(xroll,ypitch,zhead) and position
	Hash model;
	Vector3 min, max;
	Vector3 speedVector;
	float dist, heading, direction, speed;
	int classid;
	
	Vector3 currentPos = ENTITY::GET_ENTITY_COORDS(_vehicle, false);
	Vector3 currentForwardVector = ENTITY::GET_ENTITY_FORWARD_VECTOR(_vehicle);

	int count = worldGetAllPeds(pedArray, ARR_SIZE);
	for (int i = 0; i < count; i++) {
		if (PED::IS_PED_IN_ANY_VEHICLE(pedArray[i], TRUE)) continue; //Don't process peds in vehicles!
		if (ENTITY::IS_ENTITY_ON_SCREEN(pedArray[i])) {//Check if it is in screen
			ENTITY::GET_ENTITY_MATRIX(pedArray[i], (Any *)&rightVector, (Any *)&forwardVector, &upVector, &position); //Blue or red pill
			if (abs(position.z - currentPos.z) > 15) continue;//excluding birds											  
			if (SYSTEM::VDIST2(currentPos.x, currentPos.y, currentPos.z, position.x, position.y, position.z) < pedListMaxRange) {//SYSTEM::VDIST2 return without sqrt, namely 150m distance equal 22500

				if (ENTITY::HAS_ENTITY_CLEAR_LOS_TO_ENTITY(_ped, pedArray[i], 19)) {//Check if we see it (not occluded)			
					model = ENTITY::GET_ENTITY_MODEL(pedArray[i]);
					GAMEPLAY::GET_MODEL_DIMENSIONS(model, &min, &max);

					speed = ENTITY::GET_ENTITY_SPEED(pedArray[i]);
					speedVector = ENTITY::GET_ENTITY_SPEED_VECTOR(pedArray[i], false);
					if (speed > 0) {
						heading = GAMEPLAY::GET_HEADING_FROM_VECTOR_2D(speedVector.x - currentForwardVector.x, speedVector.y - currentForwardVector.y);
					}
					else {
						heading = GAMEPLAY::GET_HEADING_FROM_VECTOR_2D(forwardVector.x - currentForwardVector.x, forwardVector.y - currentForwardVector.y);
					}
					dist = GAMEPLAY::GET_DISTANCE_BETWEEN_COORDS(position.x, position.y, position.z, currentPos.x, currentPos.y, currentPos.z, true);
					direction = GAMEPLAY::GET_HEADING_FROM_VECTOR_2D(position.x - currentPos.x, position.y - currentPos.y);//0.0~360.0
					direction = (direction > 180.0 ? direction - 360.0 : direction) * D2R;//left:0~+180, right:0~-180£¬relative to ego car
					eulerAttiAngles = ENTITY::GET_ENTITY_ROTATION(pedArray[i], 0);

					if (PED::GET_PED_TYPE(pedArray[i]) == 28) classid = 11; //animal
					else classid = 10;

					//Calculate size
					dim.x = 0.5*(max.x - min.x);
					dim.y = 0.5*(max.y - min.y);
					dim.z = 0.5*(max.z - min.z);

					FUR.x = position.x + dim.y*rightVector.x + dim.x*forwardVector.x + dim.z*upVector.x;
					FUR.y = position.y + dim.y*rightVector.y + dim.x*forwardVector.y + dim.z*upVector.y;
					FUR.z = position.z + dim.y*rightVector.z + dim.x*forwardVector.z + dim.z*upVector.z;
					//GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(FUR.x, FUR.y, 1000.0, &(FUR.z), 0);
					//FUR.z += 2 * dim.z;

					BLL.x = position.x - dim.y*rightVector.x - dim.x*forwardVector.x - dim.z*upVector.x;
					BLL.y = position.y - dim.y*rightVector.y - dim.x*forwardVector.y - dim.z*upVector.y;
					BLL.z = position.z - dim.y*rightVector.z - dim.x*forwardVector.z - dim.z*upVector.z;
					//GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(BLL.x, BLL.y, 1000.0, &(BLL.z), 0);

					Value pedJsVal(kObjectType);
					Value jsVec(kArrayType);
#if 0
					jsVec.PushBack(FUR.x - currentPos.x, allocator).PushBack(FUR.y - currentPos.y, allocator).PushBack(FUR.z - currentPos.z, allocator);
					pedJsVal.AddMember("FUR", jsVec, allocator);
					jsVec.SetArray();
					jsVec.PushBack(BLL.x - currentPos.x, allocator).PushBack(BLL.y - currentPos.y, allocator).PushBack(BLL.z - currentPos.z, allocator);
					pedJsVal.AddMember("BLL", jsVec, allocator);
					jsVec.SetArray();
					for (int j = 0; j < 8; j++)
						jsVec.PushBack(edges[j].x - currentPos.x, allocator).PushBack(edges[j].y - currentPos.y, allocator).PushBack(edges[j].z - currentPos.z, allocator);
					pedJsVal.AddMember("bbEdges", jsVec, allocator);
					jsVec.SetArray();
					jsVec.PushBack(position.x, allocator).PushBack(position.y, allocator).PushBack(position.z, allocator);
					pedJsVal.AddMember("position", jsVec, allocator);
					jsVec.SetArray();
					jsVec.PushBack(eulerAttiAngles.x*D2R, allocator).PushBack(eulerAttiAngles.y*D2R, allocator).PushBack(eulerAttiAngles.z*D2R, allocator);
					pedJsVal.AddMember("eulerAngles", jsVec, allocator);
					jsVec.SetArray();
					pedJsVal.AddMember("heading", heading*D2R, allocator);
#endif // 0
					pedJsVal.AddMember("distance", dist, allocator).AddMember("direction", direction, allocator).AddMember("yaw", eulerAttiAngles.z * D2R, allocator)
						.AddMember("speed", speed, allocator).AddMember("classID", classid, allocator);
					pedsJsVal.PushBack(pedJsVal, allocator);

#ifdef DEBUG_GRAPHICS_PED
					Vector3 edges[8];
					edges[0] = BLL;
					edges[4] = FUR;

					edges[1].x = edges[0].x + 2 * dim.y*rightVector.x;
					edges[1].y = edges[0].y + 2 * dim.y*rightVector.y;
					edges[1].z = edges[0].z + 2 * dim.y*rightVector.z;

					edges[2].x = edges[1].x + 2 * dim.z*upVector.x;
					edges[2].y = edges[1].y + 2 * dim.z*upVector.y;
					edges[2].z = edges[1].z + 2 * dim.z*upVector.z;

					edges[3].x = edges[0].x + 2 * dim.z*upVector.x;
					edges[3].y = edges[0].y + 2 * dim.z*upVector.y;
					edges[3].z = edges[0].z + 2 * dim.z*upVector.z;

					edges[5].x = edges[4].x - 2 * dim.y*rightVector.x;
					edges[5].y = edges[4].y - 2 * dim.y*rightVector.y;
					edges[5].z = edges[4].z - 2 * dim.y*rightVector.z;

					edges[6].x = edges[5].x - 2 * dim.z*upVector.x;
					edges[6].y = edges[5].y - 2 * dim.z*upVector.y;
					edges[6].z = edges[5].z - 2 * dim.z*upVector.z;

					edges[7].x = edges[4].x - 2 * dim.z*upVector.x;
					edges[7].y = edges[4].y - 2 * dim.z*upVector.y;
					edges[7].z = edges[4].z - 2 * dim.z*upVector.z;
					GRAPHICS::DRAW_LINE(edges[0].x, edges[0].y, edges[0].z, edges[1].x, edges[1].y, edges[1].z, 255, 0, 0, 200);
					GRAPHICS::DRAW_LINE(edges[0].x, edges[0].y, edges[0].z, edges[3].x, edges[3].y, edges[3].z, 255, 0, 0, 200);
					GRAPHICS::DRAW_LINE(edges[1].x, edges[1].y, edges[1].z, edges[2].x, edges[2].y, edges[2].z, 255, 0, 0, 200);
					GRAPHICS::DRAW_LINE(edges[2].x, edges[2].y, edges[2].z, edges[3].x, edges[3].y, edges[3].z, 255, 0, 0, 200);

					GRAPHICS::DRAW_LINE(edges[4].x, edges[4].y, edges[4].z, edges[5].x, edges[5].y, edges[5].z, 255, 0, 0, 200);
					GRAPHICS::DRAW_LINE(edges[4].x, edges[4].y, edges[4].z, edges[7].x, edges[7].y, edges[7].z, 255, 0, 0, 200);
					GRAPHICS::DRAW_LINE(edges[5].x, edges[5].y, edges[5].z, edges[6].x, edges[6].y, edges[6].z, 255, 0, 0, 200);
					GRAPHICS::DRAW_LINE(edges[6].x, edges[6].y, edges[6].z, edges[7].x, edges[7].y, edges[7].z, 255, 0, 0, 200);

					GRAPHICS::DRAW_LINE(edges[0].x, edges[0].y, edges[0].z, edges[6].x, edges[6].y, edges[6].z, 255, 0, 0, 200);
					GRAPHICS::DRAW_LINE(edges[1].x, edges[1].y, edges[1].z, edges[7].x, edges[7].y, edges[7].z, 255, 0, 0, 200);
					GRAPHICS::DRAW_LINE(edges[2].x, edges[2].y, edges[2].z, edges[4].x, edges[4].y, edges[4].z, 255, 0, 0, 200);
					GRAPHICS::DRAW_LINE(edges[3].x, edges[3].y, edges[3].z, edges[5].x, edges[5].y, edges[5].z, 255, 0, 0, 200);
#endif//DEBUG_GRAPHICS_PED

				}
			}
		}
	}
	d["peds"] = pedsJsVal;
}

void ScenarioMsg::trafficSigns(Document & d)
{
}


