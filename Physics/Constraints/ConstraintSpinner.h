//
//	ConstraintSpinner.h
//
#pragma once
#include "ConstraintBase.h"

/*
================================
ConstraintSpinner
================================
*/
class ConstraintSpinner : public Constraint {
public:
	ConstraintSpinner() : Constraint(), m_Jacobian(4, 12) {
		m_motorTargetSpeed = 0.0f;
		m_motorAxis = Vec3(0, 0, 1);
		m_baumgarte = 0.0f;
	}

	void PreSolve(const float deltaSecond) override;
	void Solve() override;

	float m_motorTargetSpeed;
	Vec3 m_motorAxis;						// Motor Axis in BodyA's local space
	Quat m_targetRelativeOrientation;		// The initial relative quaternion q1^-1 * q2

	MatMN m_Jacobian;

	Vec3 m_baumgarte;
};
