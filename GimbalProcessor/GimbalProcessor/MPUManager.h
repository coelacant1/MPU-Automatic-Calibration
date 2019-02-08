#pragma once

#include "Mathematics.h"
#include "MPUState.h"
#include "Quaternion.h"
#include "Vector.h"
#include <Vector>

//filter
//estimate
//calibrate

//calculate reliance factor for mpu, 0->100%

//input is angular velocity and local acceleration


class MPUManager {
private:
	std::vector<MPUState> mpuData;
	std::vector<MPUState> filteredMPUData;

public:
	MPUManager(int TCAPin);
	void AddMPUData(Vector3D angularVelocity, Vector3D localAcceleration);
	void CorrectScaleOffset();
	void CorrectLinearBias();


};
