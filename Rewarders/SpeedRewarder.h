#pragma once

#include "Rewarder.h"

class SpeedRewarder : public Rewarder {
private:
	float setSpeed;
public:
	SpeedRewarder(float _desirSpeed);
	float computeReward(Vehicle vehicle);
};