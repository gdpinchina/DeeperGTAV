#include "SpeedRewarder.h"

SpeedRewarder::SpeedRewarder(float _desirSpeed) {
	setSpeed = _desirSpeed;
}

float SpeedRewarder::computeReward(Vehicle vehicle) {
	float reward = ENTITY::GET_ENTITY_SPEED(vehicle) / setSpeed;
	if (reward > 1.0f) reward = (1.0f - reward);
	if (reward < -1.0f) reward = -1.0f;
	return reward;
}