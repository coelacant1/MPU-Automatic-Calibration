#pragma once

#include "EulerAngles.h"
#include "Quaternion.h"

//calculates estimated mpu values through series of motor positions over time

//position input is partial position vector and euler rotation

//interpolate position vector () and rotation (quaternion slerp)

class GimbalManager {
private:
	EulerAngles startRotation;
	EulerAngles endRotation;
	Vector3D startPosition;
	Vector3D endPosition;
	Quaternion InterpolateRotation(double ratio);//same for all MPUs
	Vector3D InterpolatePosition(double ratio);

public:
	GimbalManager(EulerAngles startRotation, EulerAngles endRotation, Vector3D startPosition, Vector3D endPosition);

	Vector3D GetMPUAcceleration(Vector3D mpuPositionOffset, double ratio);
	Vector3D GetMPUAngularVelocity(double ratio);

};