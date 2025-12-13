//
//  Body.cpp
//
#include "Body.h"

/*
====================================================
Body::Body
====================================================
*/
Body::Body() :
	m_position(0.0f),
	m_orientation(0.0f, 0.0f, 0.0f, 1.0f),
	m_linearVelocity(0.0f),
	m_shape( NULL ) {
}

Vec3 Body::GetCenterOfMassWorldSpace() const {
	const Vec3 centerOfMass = m_shape->GetCenterOfMass();
	const Vec3 pos = m_position + m_orientation.RotatePoint(centerOfMass);
	return pos;
}
Vec3 Body::GetCenterOfMassModelSpace() const {
	const Vec3 centerOfMass = m_shape->GetCenterOfMass();
	return centerOfMass;
}
Vec3 Body::WorldSpaceToBodySpace(const Vec3& worldPt) const {
	Vec3 tmp = worldPt - GetCenterOfMassWorldSpace();
	Quat inverseOrient = m_orientation.Inverse();
	Vec3 bodySpace = inverseOrient.RotatePoint(tmp);
	return bodySpace;
}
Vec3 Body::BodySpaceToWorldSpace(const Vec3& worldPt) const {
	Vec3 worldSpace = GetCenterOfMassWorldSpace() + m_orientation.RotatePoint(worldPt);
	return worldSpace;
}


Mat3 Body::GetInverseInertiaTensorBodySpace() const {
	Mat3 inertiaTensor = m_shape->GetInertiaTensor();
	Mat3 invInertiaTensor = inertiaTensor.Inverse() * m_invMass;
	return invInertiaTensor;
}
Mat3 Body::GetInverseInertiaTensorWorldSpace() const {
	Mat3 inertiaTensor = m_shape->GetInertiaTensor();
	Mat3 invInertiaTensor = inertiaTensor.Inverse() * m_invMass;
	Mat3 orient = m_orientation.ToMat3();
	invInertiaTensor = orient * invInertiaTensor * orient.Transpose();
	return invInertiaTensor;
}


void Body::ApplyImpulse(const Vec3& impulsePoint, const Vec3& linearImpulse) {
	if (0.0f == m_invMass)
		return;

	ApplyImpulseLinear(linearImpulse);

	Vec3 pivotPoint = GetCenterOfMassWorldSpace();
	Vec3 positionVector = impulsePoint - pivotPoint;
	Vec3 angularImpulse = positionVector.Cross(linearImpulse);
	ApplyImpulseAngular(angularImpulse);
}
void Body::ApplyImpulseLinear(const Vec3& linearImpulse) {
	if (0.0f == m_invMass)
		return;

	m_linearVelocity += linearImpulse * m_invMass;
}
void Body::ApplyImpulseAngular(const Vec3& angularImpulse) {
	if (0.0f == m_invMass)
		return;

	m_angularVelocity += GetInverseInertiaTensorWorldSpace() * angularImpulse;

	// if the angular velocity is too high, modify it to the arbitrary limit
	const float maxAngularSpeed = 30.0f;
	if (m_angularVelocity.GetLengthSqr() > maxAngularSpeed * maxAngularSpeed) {
		m_angularVelocity.Normalize();
		m_angularVelocity *= maxAngularSpeed;
	}
}


void Body::Update(const float deltaSecond) {
    m_position += m_linearVelocity * deltaSecond;

    Vec3 centerOfMass = GetCenterOfMassWorldSpace();
    Vec3 comToPos = m_position - centerOfMass;

    Mat3 orientation = m_orientation.ToMat3();
    // Transform the inertia tensor from body space to world space
    Mat3 inertiaTensor = orientation * m_shape->GetInertiaTensor() * orientation.Transpose();
    // Compute the angular acceleration
    Vec3 acceleration = inertiaTensor.Inverse() * (m_angularVelocity.Cross(inertiaTensor * m_angularVelocity));
    m_angularVelocity += acceleration * deltaSecond; // angular acceleration times delta time = delta angular velocity


    // Update orientation
    // This vector's direction is the axis of rotation, and its magnitude is the angle (in radians)
    Vec3 deltaAngle = m_angularVelocity * deltaSecond;
    Quat deltaQuat = Quat(deltaAngle, deltaAngle.GetMagnitude());
    m_orientation = deltaQuat * m_orientation;
    // Normalize the quaternion to prevent numerical drift (quaternions must have magnitude 1)
    m_orientation.Normalize();

    // Update the reference position by rotating the offset vector around the center of mass
    // This ensures the position follows the body's rotation although m_position isn’t the center of mass
    m_position = centerOfMass + deltaQuat.RotatePoint(comToPos);
}
