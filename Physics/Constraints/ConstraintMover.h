//
//	ConstraintMover.h
//
#pragma once
#include "ConstraintBase.h"

/*
====================================================
ConstraintMoverSimple
====================================================
*/
class ConstraintMoverSimple : public Constraint {
public:
	ConstraintMoverSimple() : Constraint(), m_accumulatedTime( 0 ) {}

	void PreSolve( const float deltaTime ) override;

	float m_accumulatedTime;
};
