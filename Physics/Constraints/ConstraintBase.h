//
//	ConstraintBase.h
//
#pragma once
#include "../Body.h"

/*
====================================================
Constraint
====================================================
*/
class Constraint {
public:
	virtual void PreSolve( const float deltaSecond ) {}
	virtual void Solve() {}
	virtual void PostSolve() {}

	static Mat4 GetQuatLeftMatrix(const Quat& inputQaut);
	static Mat4 GetQuatRightMatrix(const Quat& inputQaut);

protected:
	MatMN GetInverseMassMatrix() const;
	VecN GetVelocities() const;
	void ApplyImpulses( const VecN & impulses );

public:
	Body * m_bodyA;
	Body * m_bodyB;

	Vec3 m_anchorA;		// The anchor location in bodyA's space
	Vec3 m_axisA;		// The axis direction in bodyA's space

	Vec3 m_anchorB;		// The anchor location in bodyB's space
	Vec3 m_axisB;		// The axis direction in bodyB's space
};

/*
====================================================
Constraint::GetInverseMassMatrix
====================================================
*/
inline MatMN Constraint::GetInverseMassMatrix() const {
	MatMN invMassMatrix(12, 12);
	invMassMatrix.Zero();

	invMassMatrix.rows[0][0] = m_bodyA->m_invMass;
	invMassMatrix.rows[1][1] = m_bodyA->m_invMass;
	invMassMatrix.rows[2][2] = m_bodyA->m_invMass;

	Mat3 invInertiaMatrixA = m_bodyA->GetInverseInertiaTensorWorldSpace();
	for (int currentRow = 0; currentRow < 3; ++currentRow) {
		invMassMatrix.rows[3 + currentRow][3 + 0] = invInertiaMatrixA.rows[currentRow][0];
		invMassMatrix.rows[3 + currentRow][3 + 1] = invInertiaMatrixA.rows[currentRow][1];
		invMassMatrix.rows[3 + currentRow][3 + 2] = invInertiaMatrixA.rows[currentRow][2];
	}

	invMassMatrix.rows[6][6] = m_bodyB->m_invMass;
	invMassMatrix.rows[7][7] = m_bodyB->m_invMass;
	invMassMatrix.rows[8][8] = m_bodyB->m_invMass;

	Mat3 invInertiaMatrixB = m_bodyB->GetInverseInertiaTensorWorldSpace();
	for (int currentRow = 0; currentRow < 3; ++currentRow) {
		invMassMatrix.rows[9 + currentRow][9 + 0] = invInertiaMatrixB.rows[currentRow][0];
		invMassMatrix.rows[9 + currentRow][9 + 1] = invInertiaMatrixB.rows[currentRow][1];
		invMassMatrix.rows[9 + currentRow][9 + 2] = invInertiaMatrixB.rows[currentRow][2];
	}

	return invMassMatrix;
}

/*
====================================================
Constraint::GetVelocities
====================================================
*/
inline VecN Constraint::GetVelocities() const {
	VecN velocitiesVector(12);

	velocitiesVector[0] = m_bodyA->m_linearVelocity.x;
	velocitiesVector[1] = m_bodyA->m_linearVelocity.y;
	velocitiesVector[2] = m_bodyA->m_linearVelocity.z;

	velocitiesVector[3] = m_bodyA->m_angularVelocity.x;
	velocitiesVector[4] = m_bodyA->m_angularVelocity.y;
	velocitiesVector[5] = m_bodyA->m_angularVelocity.z;

	velocitiesVector[6] = m_bodyB->m_linearVelocity.x;
	velocitiesVector[7] = m_bodyB->m_linearVelocity.y;
	velocitiesVector[8] = m_bodyB->m_linearVelocity.z;

	velocitiesVector[9] = m_bodyB->m_angularVelocity.x;
	velocitiesVector[10] = m_bodyB->m_angularVelocity.y;
	velocitiesVector[11] = m_bodyB->m_angularVelocity.z;

	return velocitiesVector;
}

/*
====================================================
Constraint::ApplyImpulses
====================================================
*/
inline void Constraint::ApplyImpulses( const VecN & impulses ) {
	Vec3 forceInternalA(0.0f);
	Vec3 torqueInternalA(0.0f);
	Vec3 forceInternalB(0.0f);
	Vec3 torqueInternalB(0.0f);

	forceInternalA[0] = impulses[0];
	forceInternalA[1] = impulses[1];
	forceInternalA[2] = impulses[2];

	torqueInternalA[0] = impulses[3];
	torqueInternalA[1] = impulses[4];
	torqueInternalA[2] = impulses[5];

	forceInternalB[0] = impulses[6];
	forceInternalB[1] = impulses[7];
	forceInternalB[2] = impulses[8];

	torqueInternalB[0] = impulses[9];
	torqueInternalB[1] = impulses[10];
	torqueInternalB[2] = impulses[11];

	m_bodyA->ApplyImpulseLinear(forceInternalA);
	m_bodyA->ApplyImpulseAngular(torqueInternalA);

	m_bodyB->ApplyImpulseLinear(forceInternalB);
	m_bodyB->ApplyImpulseAngular(torqueInternalB);
}


/*
====================================================
Constraint::GetQuatLeftMatrix
====================================================
*/
inline Mat4 Constraint::GetQuatLeftMatrix(const Quat& inputQaut) {
	Mat4 leftMatrix;
	leftMatrix.rows[0] = Vec4(inputQaut.w, -inputQaut.x, -inputQaut.y, -inputQaut.z);
	leftMatrix.rows[1] = Vec4(inputQaut.x, inputQaut.w, -inputQaut.z, inputQaut.y);
	leftMatrix.rows[2] = Vec4(inputQaut.y, inputQaut.z, inputQaut.w, -inputQaut.x);
	leftMatrix.rows[3] = Vec4(inputQaut.z, -inputQaut.y, inputQaut.x, inputQaut.w);

	return leftMatrix.Transpose();
}

/*
====================================================
Constraint::GetQuatRightMatrix
====================================================
*/
inline Mat4 Constraint::GetQuatRightMatrix(const Quat& inputQaut) {
	Mat4 rightMatrix;
	rightMatrix.rows[0] = Vec4(inputQaut.w, -inputQaut.x, -inputQaut.y, -inputQaut.z);
	rightMatrix.rows[1] = Vec4(inputQaut.x, inputQaut.w, inputQaut.z, -inputQaut.y);
	rightMatrix.rows[2] = Vec4(inputQaut.y, -inputQaut.z, inputQaut.w, inputQaut.x);
	rightMatrix.rows[3] = Vec4(inputQaut.z, inputQaut.y, -inputQaut.x, inputQaut.w);

	return rightMatrix.Transpose();
}
