#pragma once

#include "Vector.h"
#include "Quaternion.h"

//store current estimated quaternion readout from sensor
//store current estimated position readout from sensor
//store current quaternion from motor positions adjusted for position offset, calculate based on interpolation from start to end of signal
//store current position from motor positions adjusted for position offset, 

class MPUState {
private:
	Vector3D angularVelocity;
	Vector3D localAcceleration;

public:
	MPUState(Vector3D angularVelocity, Vector3D localAcceleration);

	Vector3D GetAngularVelocity();
	Vector3D GetLocalAcceleration();

};
