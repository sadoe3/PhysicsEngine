//
//	ConstraintOrientation.h
//
#pragma once
#include "ConstraintBase.h"

/*
================================
ConstraintOrientation
================================
*/
class ConstraintOrientation : public Constraint {
public:
	ConstraintOrientation() : Constraint(), m_Jacobian( 4, 12 ) {
		m_baumgarte = 0.0f;
	}

	void PreSolve( const float deltaSecond ) override;
	void Solve() override;

	Quat m_targetRelativeOrientation;			// The initial relative quaternion q1^-1 * q2

	MatMN m_Jacobian;

	float m_baumgarte;
};
