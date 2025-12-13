//
//  Shapes.cpp
//
#include "ShapeBox.h"

/*
========================================================================================================

ShapeBox

========================================================================================================
*/

/*
====================================================
ShapeBox::Build
====================================================
*/
void ShapeBox::Build(const Vec3* pts, const int num) {
	for (int currentIndex = 0; currentIndex < num; ++currentIndex)
		m_bounds.Expand(pts[currentIndex]);

	m_points.clear();
	m_points.push_back(Vec3(m_bounds.mins.x, m_bounds.mins.y, m_bounds.mins.z));
	m_points.push_back(Vec3(m_bounds.maxs.x, m_bounds.mins.y, m_bounds.mins.z));
	m_points.push_back(Vec3(m_bounds.mins.x, m_bounds.maxs.y, m_bounds.mins.z));
	m_points.push_back(Vec3(m_bounds.mins.x, m_bounds.mins.y, m_bounds.maxs.z));

	m_points.push_back(Vec3(m_bounds.maxs.x, m_bounds.maxs.y, m_bounds.maxs.z));
	m_points.push_back(Vec3(m_bounds.mins.x, m_bounds.maxs.y, m_bounds.maxs.z));
	m_points.push_back(Vec3(m_bounds.maxs.x, m_bounds.mins.y, m_bounds.maxs.z));
	m_points.push_back(Vec3(m_bounds.maxs.x, m_bounds.maxs.y, m_bounds.mins.z));

	m_centerOfMass = (m_bounds.maxs + m_bounds.mins) * 0.5f;
}

/*
====================================================
ShapeBox::GetSupportPoint
====================================================
*/
Vec3 ShapeBox::GetSupportPoint( const Vec3 & dir, const Vec3 & pos, const Quat & orient, const float bias ) const {
	// Find the point in furthest in direction
	Vec3 currentSupportPoint = orient.RotatePoint(m_points[0]) + pos;
	float currentMaximumDistance = dir.Dot(currentSupportPoint);
	for (int currentIndex = 1; currentIndex < m_points.size(); ++currentIndex) {
		const Vec3 currentPoint = orient.RotatePoint(m_points[currentIndex]) + pos;
		const float currentDistance = dir.Dot(currentPoint);

		if (currentDistance > currentMaximumDistance) {
			currentMaximumDistance = currentDistance;
			currentSupportPoint = currentPoint;
		}
	}

	Vec3 normal = dir;
	normal.Normalize();
	normal *= bias;

	return currentSupportPoint + normal;
}

/*
====================================================
ShapeBox::GetInertiaTensor
====================================================
*/
Mat3 ShapeBox::GetInertiaTensor() const {
	// Inertia tensor for box centered around zero
	const float deltaX = m_bounds.maxs.x - m_bounds.mins.x;
	const float deltaY = m_bounds.maxs.y - m_bounds.mins.y;
	const float deltaZ = m_bounds.maxs.z - m_bounds.mins.z;

	// center of mass tensor
	Mat3 tensorCOM;
	tensorCOM.Zero();
	tensorCOM.rows[0][0] = (deltaY * deltaY + deltaZ * deltaZ) / 12.0f;
	tensorCOM.rows[1][1] = (deltaX * deltaX + deltaZ * deltaZ) / 12.0f;
	tensorCOM.rows[2][2] = (deltaX * deltaX + deltaY * deltaY) / 12.0f;

	// Now we need to use the parallel axis theroem to get the inertia tensor for a box
	//  that is not centered arond the origin
	Vec3 centerOfMass;
	centerOfMass.x = (m_bounds.maxs.x + m_bounds.mins.x) * 0.5f;
	centerOfMass.y = (m_bounds.maxs.y + m_bounds.mins.y) * 0.5f;
	centerOfMass.z = (m_bounds.maxs.z + m_bounds.mins.z) * 0.5f;

	// the displacement from center of mass to the origin
	const Vec3 displacement = Vec3(0, 0, 0) - centerOfMass;
	const float lengthSqrOfDisplacement = displacement.GetLengthSqr();
	// parallel axis therom tensor
	Mat3 tensorPAT;
	tensorPAT.rows[0] = Vec3(
		lengthSqrOfDisplacement - displacement.x * displacement.x
		, displacement.x * displacement.y
		, displacement.x * displacement.z
	);
	tensorPAT.rows[1] = Vec3(
		displacement.y * displacement.x
		, lengthSqrOfDisplacement - displacement.y * displacement.y
		, displacement.y * displacement.z
	);
	tensorPAT.rows[2] = Vec3(
		displacement.z * displacement.x
		, displacement.z * displacement.y
		, lengthSqrOfDisplacement - displacement.z * displacement.z
	);

	// Now we need to add the center of mass tensor and the parallel axis therom tensor together
	tensorCOM += tensorPAT;
	return tensorCOM;
}

/*
====================================================
ShapeBox::GetBounds
====================================================
*/
Bounds ShapeBox::GetBounds( const Vec3 & pos, const Quat & orient ) const {
	Vec3 corners[8];
	corners[0] = Vec3(m_bounds.mins.x, m_bounds.mins.y, m_bounds.mins.z);
	corners[1] = Vec3(m_bounds.mins.x, m_bounds.mins.y, m_bounds.maxs.z);
	corners[2] = Vec3(m_bounds.mins.x, m_bounds.maxs.y, m_bounds.mins.z);
	corners[3] = Vec3(m_bounds.maxs.x, m_bounds.mins.y, m_bounds.mins.z);

	corners[4] = Vec3(m_bounds.maxs.x, m_bounds.maxs.y, m_bounds.maxs.z);
	corners[5] = Vec3(m_bounds.maxs.x, m_bounds.maxs.y, m_bounds.mins.z);
	corners[6] = Vec3(m_bounds.maxs.x, m_bounds.mins.y, m_bounds.maxs.z);
	corners[7] = Vec3(m_bounds.mins.x, m_bounds.maxs.y, m_bounds.maxs.z);

	Bounds bounds;
	for (int currentIndex = 0; currentIndex < 8; ++currentIndex) {
		corners[currentIndex] = orient.RotatePoint(corners[currentIndex]) + pos;
		bounds.Expand(corners[currentIndex]);
	}

	return bounds;
}

/*
====================================================
ShapeBox::GetFastestLinearSpeed
====================================================
*/
float ShapeBox::GetFastestLinearSpeed( const Vec3 & angularVelocity, const Vec3 & dir ) const {
	float maxSpeed = 0.0f, currentSpeed = maxSpeed;
	Vec3 offsetFromCOM, linearVelocity;
	for (int currentIndex = 0; currentIndex < m_points.size(); ++currentIndex) {
		offsetFromCOM = m_points[currentIndex] - m_centerOfMass;
		linearVelocity = angularVelocity.Cross(offsetFromCOM);
		currentSpeed = dir.Dot(linearVelocity);
		if (currentSpeed > maxSpeed)
			maxSpeed = currentSpeed;
	}

	return maxSpeed;
}
