#pragma once
#include "lib/script.h"
#include "lib/utils.h"

#include "lib/rapidjson/document.h"
#include "lib/rapidjson/stringbuffer.h"

#include <string>
#include <vector>
using namespace rapidjson;

class ISetScenarioParams{ public: void virtual setScenarioParams(const Value& sc) = 0; };

class ISetDatasetParams{ public:  void virtual setDatasetParams(const Value& dc, Document &d) = 0; };

class IExeTask{ public:  void virtual exeTask() = 0; };

class ISendTaskMsgs{ public:  void virtual sendTaskMsgs(Document &d) = 0; };

class IShowTaskStates{ public:  void virtual showTaskStates() = 0; };


class Task {
public:
	enum DATA_TYPE {
		MSG_INT,
		MSG_FLOAT,
		MSG_ARRAY,
		MSG_STRING
	};

	void setVehID(Vehicle veh) { _vehicle = veh; }
	void setPedID(Ped ped) { _ped = ped; }
	void setPlayerID(Player pl) { _player = pl; }
	void setCamID(Camera cam) { _camera = cam; }
	const virtual  char *getTaskName() { return _taskName.data(); }

protected:
	std::string _taskName;
	Vehicle _vehicle;
	Ped _ped;
	Player _player;
	Camera _camera;

	void virtual addMenber2Doc(const char *name, enum DATA_TYPE type, Document &d) {
		Document::AllocatorType& allocator = d.GetAllocator();
		Value arrayType(kArrayType), stringType(rapidjson::kStringType);
		switch (type)
		{
		case Task::MSG_INT:
			d.AddMember(rapidjson::StringRef(name), 0, allocator);
			break;
		case Task::MSG_FLOAT:
			d.AddMember(rapidjson::StringRef(name), 0.0, allocator);
			break;
		case Task::MSG_ARRAY:
			d.AddMember(rapidjson::StringRef(name), arrayType, allocator);
			break;
		case Task::MSG_STRING:
			d.AddMember(rapidjson::StringRef(name), stringType, allocator);
			break;
		default:
			break;
		}
	}
};