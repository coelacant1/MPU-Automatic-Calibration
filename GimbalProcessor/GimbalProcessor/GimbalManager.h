#pragma once

#include "EulerAngles.h"
#include "Quaternion.h"
#include "Rotation.h"

//calculates estimated mpu values through series of motor positions over time

//position input is partial position vector and euler rotation

//interpolate position vector () and rotation (quaternion slerp)

class GimbalManager {
private:
	Vector3D startRotation;
	Vector3D endRotation;
	EulerOrder eulerOrder;
	Vector3D startPosition;
	Vector3D endPosition;
	Quaternion InterpolateRotation(double ratio);//same for all MPUs
	Vector3D InterpolatePosition(double ratio);
	double InterpolateDouble(double ratio, double inMin, double inMax, double outMin, double outMax);
	Vector3D InterpolateVector(double ratio, Vector3D start, Vector3D end);

public:
	GimbalManager();

	Vector3D GetMPUAcceleration(Vector3D mpuPositionOffset, double ratio);
	Vector3D GetMPUAngularVelocity(double ratio);

	void SetStartEndRotation(Vector3D startRotation, Vector3D endRotation, EulerOrder eulerOrder);
	void SetStartEndPosition(Vector3D startPosition, Vector3D endPosition);
	void SetTime(long milliseconds);


};