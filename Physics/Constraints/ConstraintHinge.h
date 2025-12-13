//
//	ConstraintHinge.h
//
#pragma once
#include "ConstraintBase.h"

/*
================================
ConstraintHinge
================================
*/
class ConstraintHinge : public Constraint {
public:
	ConstraintHinge() : Constraint(), m_cachedLagrange( 3 ), m_Jacobian( 3, 12 ) {
		m_cachedLagrange.Zero();
		m_baumgarte = 0.0f;
	}
	void PreSolve( const float deltaSecond ) override;
	void Solve() override;
	void PostSolve() override;

	Quat m_targetRelativeOrientation;	// The initial relative quaternion q1^-1 * q2

	VecN m_cachedLagrange;
	MatMN m_Jacobian;

	float m_baumgarte;
};

/*
================================
ConstraintHingeLimited
================================
*/
class ConstraintHingeLimited : public Constraint {
public:
	ConstraintHingeLimited() : Constraint(), m_cachedLagrange( 4 ), m_Jacobian( 4, 12 ) {
		m_cachedLagrange.Zero();
		m_baumgarte = 0.0f;
		m_isAngleViolated = false;
		m_relativeAngle = 0.0f;
	}
	void PreSolve( const float deltaSecond) override;
	void Solve() override;
	void PostSolve() override;

	Quat m_targetRelativeOrientation;	// The initial relative quaternion q1^-1 * q2

	VecN m_cachedLagrange;
	MatMN m_Jacobian;

	float m_baumgarte;

	bool m_isAngleViolated;
	float m_relativeAngle;
};
