#pragma once
#include "Task.hpp"
#include <ctime>

/*
Some APIs about IMU:
Vector3 nPos = ENTITY::GET_ENTITY_COORDS(_vehicle, false);				//ENU, m
Vector3 nSpeed = ENTITY::GET_ENTITY_SPEED_VECTOR(_vehicle, false);		//ENU, m/s
Vector3 bSpeed = ENTITY::GET_ENTITY_SPEED_VECTOR(_vehicle, true);		//right, forward, upward, m/s
Vector3 nbAttitude = ENTITY::GET_ENTITY_ROTATION(_vehicle, 0);			//roll, pitch, head, degree
Vector3 bAngleRate = ENTITY::GET_ENTITY_ROTATION_VELOCITY(_vehicle);	//right, forward, upward, rad/s
float vehMaxSpeed = VEHICLE::_GET_VEHICLE_SPEED(_vehicle);
float vehMaxAccel = VEHICLE::GET_VEHICLE_ACCELERATION(_vehicle);
*/

class ScenarioMsg : public Task, public ISetDatasetParams, public ISendTaskMsgs, public IShowTaskStates
{
public:
	ScenarioMsg();
	~ScenarioMsg();

	struct Content
	{
		std::string taskName;
		void (ScenarioMsg::*task) (Document &d);
		bool isActivate;
		bool defaultState;	//false
		Task::DATA_TYPE dataType;
	};

	void setDatasetParams(const Value& dc, Document &d);
	void sendTaskMsgs(Document &d); 	//virtual func
	void showTaskStates();
	static std::vector<Content> _allTaskList;
	std::vector<Content *> _taskList;

private:
	void throttle(Document &d);
	void brake(Document &d);
	void steering(Document &d);
	void speed(Document &d);
	void acceleration(Document &d);
	void yaw(Document &d);			//rad
	void yawRate(Document &d);		//rad/s
	void time(Document &d);
	void isCollide(Document &d);
	void eulerAngles(Document &d);	//own car's Euler attitude angles, roll, pitch, head, rad
	void location(Document &d);
	void cameraInfo(Document &d);
	void rageMatrices(Document &d);	//worldMatrix, viewMatrix, projectionMatrix
	void vehicles(Document &d);
	void peds(Document &d);
	void trafficSigns(Document &d);	//TODO, may use this API to set the state of traffic signs: ENTITY::SET_ENTITY_TRAFFICLIGHT_OVERRIDE

private:
	//helper
	float curSpeed;
};
