//
//	ConstraintConstantVelocity.h
//
#pragma once
#include "ConstraintBase.h"

/*
================================
ConstraintConstantVelocity
================================
*/
class ConstraintConstantVelocity : public Constraint {
public:
	ConstraintConstantVelocity() : Constraint(), m_cachedLagrange( 2 ), m_Jacobian( 2, 12 ) {
		m_cachedLagrange.Zero();
		m_baumgarte = 0.0f;
	}
	void PreSolve( const float deltaSecond ) override;
	void Solve() override;
	void PostSolve() override;

	Quat m_targetRelativeOrientation;	// The initial relative quaternion q1 * q2^-1

	VecN m_cachedLagrange;
	MatMN m_Jacobian;

	float m_baumgarte;
};

/*
================================
ConstraintConstantVelocityLimited
================================
*/
class ConstraintConstantVelocityLimited : public Constraint {
public:
	ConstraintConstantVelocityLimited() : Constraint(), m_cachedLagrange( 4 ), m_Jacobian( 4, 12 ) {
		m_cachedLagrange.Zero();
		m_baumgarte = 0.0f;
		m_isAngleViolatedU = false;
		m_isAngleViolatedV = false;
		m_relativeAngleU = 0.0f;
		m_relativeAngleV = 0.0f;
	}
	void PreSolve( const float deltaSecond ) override;
	void Solve() override;
	void PostSolve() override;

	Quat m_targetRelativeOrientation;	// The initial relative quaternion q1^-1 * q2

	VecN m_cachedLagrange;
	MatMN m_Jacobian;

	float m_baumgarte;

	bool m_isAngleViolatedU;
	bool m_isAngleViolatedV;
	float m_relativeAngleU;
	float m_relativeAngleV;
};
