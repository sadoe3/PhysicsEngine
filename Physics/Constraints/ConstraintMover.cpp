//
//  ConstraintMover.cpp
//
#include "ConstraintMover.h"

/*
====================================================
ConstraintMoverSimple::PreSolve
====================================================
*/
void ConstraintMoverSimple::PreSolve( const float deltaTime) {
	m_accumulatedTime += deltaTime;
	m_bodyA->m_linearVelocity.y = cosf(m_accumulatedTime * 0.25f) * 4.0f;
}
