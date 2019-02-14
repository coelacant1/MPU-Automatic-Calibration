#include "GimbalManager.h"

GimbalManager::GimbalManager(bool cubic) {
	this->cubic = cubic;
}

Vector3D GimbalManager::GetMPUAcceleration(Vector3D mpuPositionOffset, double ratio, double period) {
	Vector3D gimbalAngularVelocity = GetMPUAngularVelocity(ratio, period);
	Vector3D gimbalAngularAcceleration = (GetMPUAngularVelocity(ratio, period) + GetMPUAngularVelocity(ratio + (period / constantVelocityTime), period)) / period;
	Vector3D gimbalLinearAcceleration = (InterpolatePosition(ratio) + InterpolatePosition(ratio + (period / constantVelocityTime))) / period;

	Vector3D mpuTangentialAcceleration = mpuPositionOffset * gimbalAngularAcceleration;
	Vector3D mpuCentripetalAcceleration = mpuPositionOffset * (gimbalAngularVelocity * 2.0);

	Vector3D mpuAcceleration = gimbalLinearAcceleration + mpuTangentialAcceleration + mpuCentripetalAcceleration;

	return mpuAcceleration;
}

Vector3D GimbalManager::GetMPUAngularVelocity(double ratio, double period) {
	double r1 = ratio;
	double r2 = ratio + (period / constantVelocityTime);

	Quaternion q1 = InterpolateRotation(r1);
	Quaternion q2 = InterpolateRotation(r2);
	Quaternion dif = q2 * q1.Conjugate();

	//Convert difference quaternion to axis angle space
	AxisAngle aa = Rotation(dif).GetAxisAngle();

	//angular velocity is equal to the axis * angle / dT
	Vector3D angularVelocity =  (aa.Axis * aa.Rotation) / period;

	return angularVelocity;
}

void GimbalManager::SetStartEndRotation(Vector3D startRotation, Vector3D endRotation, EulerOrder eulerOrder) {
	this->startRotation = startRotation;
	this->endRotation = endRotation;
	this->eulerOrder = eulerOrder;
}

void GimbalManager::SetStartEndPosition(Vector3D startPosition, Vector3D endPosition) {
	this->startPosition = startPosition;
	this->endPosition = endPosition;
}

void GimbalManager::SetTime(long milliseconds) {
	this->constantVelocityTime = milliseconds;
}

Quaternion GimbalManager::InterpolateRotation(double ratio) {
	Vector3D interpolatedVector;

	if (cubic) {
		interpolatedVector = CubicInterpolateVector(ratio, startRotation, endRotation);
	}
	else {
		interpolatedVector = LinearInterpolateVector(ratio, startRotation, endRotation);
	}

	return Rotation(EulerAngles(Vector3D(), eulerOrder)).GetQuaternion();;
}

Vector3D GimbalManager::InterpolatePosition(double ratio) {
	if (cubic) {
		return CubicInterpolateVector(ratio, startPosition, endPosition);
	}
	else {
		return LinearInterpolateVector(ratio, startPosition, endPosition);
	}
}


double GimbalManager::LinearInterpolateDouble(double ratio, double inMin, double inMax, double outMin, double outMax) {
	return (ratio - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}


Vector3D GimbalManager::LinearInterpolateVector(double ratio, Vector3D start, Vector3D end) {
	return Vector3D(
		LinearInterpolateDouble(ratio, 0.0, 1.0, start.X, end.X),
		LinearInterpolateDouble(ratio, 0.0, 1.0, start.Y, end.Y),
		LinearInterpolateDouble(ratio, 0.0, 1.0, start.Z, end.Z)
	);
}


double GimbalManager::CubicInterpolateDouble(double ratio, double inMin, double inMax, double outMin, double outMax) {
	return 0;
}


Vector3D GimbalManager::CubicInterpolateVector(double ratio, Vector3D start, Vector3D end) {
	return Vector3D();
}
