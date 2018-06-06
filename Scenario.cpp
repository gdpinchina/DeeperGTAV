#include "Scenario.h"
#include "lib/utils.h"
#include "lib/rapidjson/writer.h"
#include "Rewarders\GeneralRewarder.h"
#include "Rewarders\LaneRewarder.h"
#include "Rewarders\SpeedRewarder.h"
#include "defaults.h"
#include "Server.h"
#include <time.h>

char* Scenario::weatherList[14] = { "CLEAR", "EXTRASUNNY", "CLOUDS", "OVERCAST", "RAIN", "CLEARING", "THUNDER", "SMOG", "FOGGY", "XMAS", "SNOWLIGHT", "BLIZZARD", "NEUTRAL", "SNOW" };
char* Scenario::vehicleList[3] = { "blista", "voltic", "packer" };


void Scenario::parseScenarioConfig(const Value& sc) {
	const Value& time = sc["time"];
	const Value& weather = sc["weather"];
	const Value& vehicle = sc["vehicle"];

	if (time.IsArray()) {
		if (!time[0].IsNull()) _hour = time[0].GetInt();
		else _hour = rand() % 24;

		if (!time[1].IsNull()) _minute = time[1].GetInt();
		else _minute = rand() % 60;
	}
	else {
		_hour = rand() % 24;
		_minute = rand() % 60;
	}

	if (!weather.IsNull()) _weatherStr = weather.GetString();
	else _weatherStr = weatherList[rand() % 14];

	if (!vehicle.IsNull()) _vehicleStr = vehicle.GetString();
	else _vehicleStr = vehicleList[rand() % 3];

	//drivingMode init
	dm.setScenarioParams(sc);

#ifdef DEBUG_CONFIG
	printf("\nDEBUG_CONFIG: function: %s", __FUNCTION__);	
	printf("\nhour=%d, minutes=%d", _hour, _minute);
	printf("\n_weather=%s, _vehicle=%s", _weatherStr, _vehicleStr);
	printf("\n");
#endif // DEBUG_CONFIG
}

void Scenario::parseDatasetConfig(const Value& dc) {
	if (!dc["rate"].IsNull()) rate = dc["rate"].GetInt();
	else rate = _RATE_;
	scenarioIntv = 1.0 / rate * CLOCKS_PER_SEC;
	
	if (dc["reward"].IsArray()) {
		if (dc["reward"][0].IsFloat() && dc["reward"][1].IsFloat()) {
			rewardSpeed = dc["reward"][0].GetFloat();
			rewardAggressive = dc["reward"][1].GetFloat();
			reward = true;
		}
		else reward = _REWARD_;
	}
	else reward = _REWARD_;

	//setup the vision native library
	if (!dc["rageMatrices"].IsNull())
		VisionNative::GetInstance()->loadLib();
	else
		VisionNative::GetInstance()->unLoadLib();

	//Create JSON DOM
	d.SetObject();
	Document::AllocatorType& allocator = d.GetAllocator();
	Value a(kArrayType);
	if (reward) d.AddMember("reward", 0.0, allocator);

	//setup screenCapturer
	screenCapturer.setDatasetParams(dc, d);

	//driving mode msg task
	dm.setDatasetParams(dc, d);

	//scalar msg task
	npMsg.setDatasetParams(dc, d);

	//liDAR msg task
	liDARDevice.setDatasetParams(dc, d);

#ifdef DEBUG_CONFIG
	printf("\nDEBUG_CONFIG: function: %s", __FUNCTION__);
	printf("\nrate=%d", rate);
	if (dc["reward"].IsArray())
		if (dc["reward"][0].IsFloat() && dc["reward"][1].IsFloat())
			printf("\nreward[0]=%f, reward[1]=%f", dc["reward"][0].GetFloat(), dc["reward"][1].GetFloat());
	printf("\n");
#endif // DEBUG_CONFIG
}

void Scenario::buildScenario() {
	Vector3 pos, rotation, checkPos;
	Hash vehicleHash;

	GAMEPLAY::SET_RANDOM_SEED(std::time(NULL));
	while (!PATHFIND::_0xF7B79A50B905A30D(-8192.0f, 8192.0f, -8192.0f, 8192.0f)) WAIT(0);
	//use the starting position directly to initialize the scenario, remaining jobs the drivingmode will do.
	pos = dm._route[0].org;

	//setup vehicle
	ENTITY::DELETE_ENTITY(&_vehicle);
	vehicleHash = GAMEPLAY::GET_HASH_KEY((char*)_vehicleStr);
	STREAMING::REQUEST_MODEL(vehicleHash);
	while (!STREAMING::HAS_MODEL_LOADED(vehicleHash)) WAIT(0);
	while (!ENTITY::DOES_ENTITY_EXIST(_vehicle)) {
		_vehicle = VEHICLE::CREATE_VEHICLE(vehicleHash, pos.x, pos.y, pos.z, 0.0, FALSE, FALSE);
		WAIT(0);
	}
	VEHICLE::SET_VEHICLE_ON_GROUND_PROPERLY(_vehicle);

	//setup player
	while (!ENTITY::DOES_ENTITY_EXIST(_ped)) {
		_ped = PLAYER::PLAYER_PED_ID();
		WAIT(0);
	}
	_player = PLAYER::PLAYER_ID();
	PLAYER::START_PLAYER_TELEPORT(_player, pos.x, pos.y, pos.z, 0.0, 0, 0, 0);
	while (PLAYER::IS_PLAYER_TELEPORT_ACTIVE()) WAIT(0);
	PED::SET_PED_INTO_VEHICLE(_ped, _vehicle, -1);
	STREAMING::SET_MODEL_AS_NO_LONGER_NEEDED(vehicleHash);

	//setup weather
	TIME::SET_CLOCK_TIME(_hour, _minute, 0);
	GAMEPLAY::SET_WEATHER_TYPE_NOW_PERSIST((char*)_weatherStr);

	//setup camera
	rotation = ENTITY::GET_ENTITY_ROTATION(_vehicle, 0);
	CAM::DESTROY_ALL_CAMS(TRUE);
	_camera = CAM::CREATE_CAM("DEFAULT_SCRIPTED_CAMERA", TRUE);
	if (strcmp(_vehicleStr, "packer") == 0) CAM::ATTACH_CAM_TO_ENTITY(_camera, _vehicle, 0, 2.35, 1.7, TRUE);
	else CAM::ATTACH_CAM_TO_ENTITY(_camera, _vehicle, _CAM_OFFSET_TO_ENTITY_X_, _CAM_OFFSET_TO_ENTITY_Y_, _CAM_OFFSET_TO_ENTITY_Z_, TRUE);
	CAM::SET_CAM_FOV(_camera, 60);
	CAM::SET_CAM_ACTIVE(_camera, TRUE);
	CAM::SET_CAM_ROT(_camera, rotation.x, rotation.y, rotation.z, 0);
	CAM::SET_CAM_INHERIT_ROLL_VEHICLE(_camera, TRUE);
	CAM::RENDER_SCRIPT_CAMS(TRUE, FALSE, 0, TRUE, TRUE);
	while (1)//Wait the camera initated. If camera isn't initiated, the first frame of LiDAR is wrong. 
	{
		Vector3 source = CAM::GET_CAM_COORD(_camera);
		if (abs(source.x - 128000.0) < 1e-5 && abs(source.y - 128000.0) < 1e-5) WAIT(100);
		else break;
	}

	//setup reward
	if (reward)//reward must be initiated after vehicle initiating.
	{
		if (rewarder != NULL) { delete rewarder; rewarder = NULL; }
		rewarder = new GeneralRewarder((char*)(GetCurrentModulePath() + "paths.xml").c_str(), rewardSpeed, rewardAggressive);
	}

	//setup AI driver
	dm.setVehID(_vehicle);
	dm.setPedID(_ped);
	dm.setCamID(_camera);
	dm.setPlayerID(_player);

	//setup no params msg task
	npMsg.setVehID(_vehicle);
	npMsg.setPedID(_ped);
	npMsg.setCamID(_camera);

	//setup lidar
	liDARDevice.setVehID(_vehicle);
	liDARDevice.setCamID(_camera);
	liDARDevice.InitLiDAR();

	//begin!
	int hms[] = { _hour, _minute, 0 };
	dm.setBeginTimeWeather(hms, _weatherStr);
	dm.beginDrivingTask();

	//show states
	dm.showTaskStates();
	npMsg.showTaskStates();
	liDARDevice.showTaskStates();
}

Scenario::Scenario() :rewarder(NULL)
{
}

Scenario::~Scenario()
{
	screenCapturer.DestroyCapturer();
	liDARDevice.DestroyLiDAR();
	if (rewarder) delete rewarder;
	VisionNative::DestroyInstance();
}

void Scenario::start(const Value& sc, const Value& dc) {
	if (running) return;

	//Parse options
	srand(std::time(NULL));
	parseScenarioConfig(sc);
	parseDatasetConfig(dc);

	//Build scenario
	buildScenario();

	running = true;
}

void Scenario::config(const Value& sc, const Value& dc) {
	if (!running) return;

	running = false;

	//Parse options
	srand(std::time(NULL));
	parseScenarioConfig(sc);
	parseDatasetConfig(dc);

	//Build scenario
	buildScenario();

	running = true;
}

void Scenario::run() {
	if (running) {
		Vector3 nbAtti = ENTITY::GET_ENTITY_ROTATION(_vehicle, 0);	
		CAM::SET_CAM_ROT(_camera, nbAtti.x, nbAtti.y, nbAtti.z, 0);
		if (dm.getDrivingParamsFlag() == DrivingMode::DRIVING_PARAMS_FLAG::DEFAULT_DO_NOTHING) {	//manual drive
			CONTROLS::_SET_CONTROL_NORMAL(27, 71, currentThrottle); //[0,1]
			CONTROLS::_SET_CONTROL_NORMAL(27, 72, currentBrake); //[0,1]
			CONTROLS::_SET_CONTROL_NORMAL(27, 59, currentSteering); //[-1,1]
		}
		else dm.exeTask();			//auto drive, the task will be carried out by drivingMode msg sending
	}
	scriptWait(0);
}

void Scenario::stop() {
	if (!running) return;
	running = false;
	CAM::DESTROY_ALL_CAMS(TRUE);
	CAM::RENDER_SCRIPT_CAMS(FALSE, TRUE, 500, FALSE, FALSE);
	AI::CLEAR_PED_TASKS(_ped);
	setCommands(0.0, 0.0, 0.0);
}

void Scenario::setCommands(float throttle, float brake, float steering) {
	currentThrottle = throttle;
	currentBrake = brake;
	currentSteering = steering;
}

void Scenario::generateMessage() {
	buffer.Clear();
	Writer<StringBuffer> writer(buffer);

	dm.sendTaskMsgs(d);

	screenCapturer.capture();
	liDARDevice.GeneratePointClouds();
	npMsg.sendTaskMsgs(d);
	if (reward) setReward();
	d.Accept(writer);//all txt works are put into the buffer through write

	chmessage = buffer.GetString();
	messageSize = buffer.GetSize();
}

void Scenario::setReward() {
	d["reward"] = rewarder->computeReward(_vehicle);
}




