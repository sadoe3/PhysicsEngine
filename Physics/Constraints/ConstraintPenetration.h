//
//	ConstraintPenetration.h
//
#pragma once
#include "ConstraintBase.h"

/*
================================
ConstraintPenetration
================================
*/
class ConstraintPenetration : public Constraint {
public:
	ConstraintPenetration() : Constraint(), m_cachedLagrange( 3 ), m_Jacobian( 3, 12 ) {
		m_cachedLagrange.Zero();
		m_baumgarte = 0.0f;
		m_friction = 0.0f;
	}

	void PreSolve( const float deltaSecond ) override;
	void Solve() override;

	VecN m_cachedLagrange;

	// in Body A's local space
	// it's headed towards Body B
	Vec3 m_collisionNormal;		

	MatMN m_Jacobian;

	float m_baumgarte;
	float m_friction;
};
