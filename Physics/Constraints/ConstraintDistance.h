//
//	ConstraintDistance.h
//
#pragma once
#include "ConstraintBase.h"

/*
================================
ConstraintDistance
================================
*/
class ConstraintDistance : public Constraint {
public:
	ConstraintDistance() : Constraint(),
		m_cachedLagrange( 1 ),
		m_Jacobian( 1, 12 ) {
		m_cachedLagrange.Zero();
		m_baumgarte = 0.0f;
	}

	void PreSolve( const float dt_sec ) override;
	void Solve() override;
	void PostSolve() override;

private:
	MatMN m_Jacobian;

	VecN m_cachedLagrange;
	float m_baumgarte;
};
