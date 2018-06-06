#pragma once

#include <stdlib.h>
#include <ctime>
#include <vector>

#include "lib/script.h"
#include "lib/utils.h"

#include "lib/rapidjson/document.h"
#include "lib/rapidjson/stringbuffer.h"

#include "ScreenCapturer.h"
#include "Rewarders\Rewarder.h"
#include "VisionNative.h"
#include "LiDAR.h"
#include "DrivingMode.h"
#include "ScenarioMsg.h"
#include "defaults.h"

using namespace rapidjson;

class Scenario {
private:
	//mix
	static char* weatherList[14];
	static char* vehicleList[3];
	Vehicle _vehicle = NULL;
	Player _player = NULL;
	Ped _ped = NULL;
	Cam _camera = NULL;
	int _hour, _minute;
	const char* _weatherStr;
	const char* _vehicleStr;

	//manual control params
	float currentThrottle = 0.0;
	float currentBrake = 0.0;
	float currentSteering = 0.0;

	//running flag
	bool running = false;

	DrivingMode dm;
	ScenarioMsg npMsg;

	//lane and velo rewarder
	Rewarder* rewarder;
	float rewardSpeed, rewardAggressive;
	bool reward;		//msg flag

	//rapjson object for msg
	Document d;
	StringBuffer buffer;

public:
	Scenario();
	~Scenario();
	int rate;
	int scenarioIntv;								//ms

	void start(const Value& sc, const Value& dc);	//Apply configs to start scenario, if configs aren't defined, apply defaults instead. 
	void stop();
	void config(const Value& sc, const Value& dc);	//Apply configs to start scenario, if configs aren't defined, deprecate instead. 
	void setCommands(float throttle, float brake, float steering);
	void run();										//Update the camera's direction. If the ego vehicle gets in trouble, help it.
	void generateMessage();

	ScreenCapturer screenCapturer;

	LiDAR liDARDevice;

	const char *chmessage;
	unsigned int messageSize;

private:
	//core
	void parseScenarioConfig(const Value& sc);
	void parseDatasetConfig(const Value& dc);
	void buildScenario();

	//for msg
	void setReward();
};