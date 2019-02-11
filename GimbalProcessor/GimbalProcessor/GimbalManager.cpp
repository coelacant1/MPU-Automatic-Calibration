#include "GimbalManager.h"

GimbalManager::GimbalManager() {

}

Vector3D GimbalManager::GetMPUAcceleration(Vector3D mpuPositionOffset, double ratio) {

}

Vector3D GimbalManager::GetMPUAngularVelocity(double ratio) {

}

void GimbalManager::SetStartEndRotation(Vector3D startRotation, Vector3D endRotation, EulerOrder eulerOrder) {

}

void GimbalManager::SetStartEndPosition(Vector3D startPosition, Vector3D endPosition) {

}

void GimbalManager::SetTime(long milliseconds) {

}


Quaternion GimbalManager::InterpolateRotation(double ratio) {
	Vector3D interpolatedVector = InterpolateVector(ratio, startRotation, endRotation);

	Quaternion interpolated = Rotation(EulerAngles(Vector3D(), eulerOrder)).GetQuaternion();

	return Quaternion();
}

Vector3D GimbalManager::InterpolatePosition(double ratio) {

}

double GimbalManager::InterpolateDouble(double ratio, double inMin, double inMax, double outMin, double outMax) {
	return (ratio - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}


Vector3D GimbalManager::InterpolateVector(double ratio, Vector3D start, Vector3D end) {
	return Vector3D(
		InterpolateDouble(ratio, 0.0, 1.0, start.X, end.X),
		InterpolateDouble(ratio, 0.0, 1.0, start.Y, end.Y),
		InterpolateDouble(ratio, 0.0, 1.0, start.Z, end.Z)
	);
}
