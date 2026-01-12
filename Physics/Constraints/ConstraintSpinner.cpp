//
//  ConstraintSpinner.cpp
//
#include "PCH.h"
#include "ConstraintSpinner.h"

/*
================================
ConstraintSpinner::PreSolve
================================
*/
void ConstraintSpinner::PreSolve(const float deltaSecond) {
	// Get the world space position of the hinge from anchorA's orientation
	const Vec3 worldAnchorA = m_bodyA->BodySpaceToWorldSpace(m_anchorA);

	// Get the world space position of the hinge from anchorB's orientation
	const Vec3 worldAnchorB = m_bodyB->BodySpaceToWorldSpace(m_anchorB);

	const Vec3 anchorAToAnchorB = worldAnchorB - worldAnchorA;
	const Vec3 centerToAnchorA = worldAnchorA - m_bodyA->GetCenterOfMassWorldSpace();
	const Vec3 centerToAnchorB = worldAnchorB - m_bodyB->GetCenterOfMassWorldSpace();
	const Vec3 anchorA = worldAnchorA;
	const Vec3 anchorB = worldAnchorB;

	// Get the orientation information of the bodies
	const Quat orientationA = m_bodyA->m_orientation;
	const Quat orientationB = m_bodyB->m_orientation;
	const Quat targetOrientationInv = m_targetRelativeOrientation.Inverse();
	const Quat orientationAInv = orientationA.Inverse();

	const Vec3 motorAxis = m_bodyA->m_orientation.RotatePoint(m_motorAxis);
	Vec3 motorU;
	Vec3 motorV;
	motorAxis.GetOrtho(motorU, motorV);

	const Vec3 u = motorU;
	const Vec3 v = motorV;
	const Vec3 h = motorAxis;

	Mat4 projectionMatrix;
	projectionMatrix.rows[0] = Vec4(0, 0, 0, 0);
	projectionMatrix.rows[1] = Vec4(0, 1, 0, 0);
	projectionMatrix.rows[2] = Vec4(0, 0, 1, 0);
	projectionMatrix.rows[3] = Vec4(0, 0, 0, 1);
	Mat4 pTranspose = projectionMatrix.Transpose();	// I know it's pointless to do this with our particular matrix implementations.  But I like its self commenting.

	const Mat4 MatA = projectionMatrix * GetQuatLeftMatrix(orientationAInv) * GetQuatRightMatrix(orientationB * targetOrientationInv) * pTranspose * -0.5f;
	const Mat4 MatB = projectionMatrix * GetQuatLeftMatrix(orientationAInv) * GetQuatRightMatrix(orientationB * targetOrientationInv) * pTranspose * 0.5f;
	
	//	The distance constraint
	m_Jacobian.Zero();

	// First row is the primary distance constraint that holds the anchor points together
	Vec3 J1 = (anchorA - anchorB) * 2.0f;
	m_Jacobian.rows[0][0] = J1.x;
	m_Jacobian.rows[0][1] = J1.y;
	m_Jacobian.rows[0][2] = J1.z;

	Vec3 J2 = centerToAnchorA.Cross((anchorA - anchorB) * 2.0f);
	m_Jacobian.rows[0][3] = J2.x;
	m_Jacobian.rows[0][4] = J2.y;
	m_Jacobian.rows[0][5] = J2.z;

	Vec3 J3 = (anchorB - anchorA) * 2.0f;
	m_Jacobian.rows[0][6] = J3.x;
	m_Jacobian.rows[0][7] = J3.y;
	m_Jacobian.rows[0][8] = J3.z;

	Vec3 J4 = centerToAnchorB.Cross((anchorB - anchorA) * 2.0f);
	m_Jacobian.rows[0][9] = J4.x;
	m_Jacobian.rows[0][10] = J4.y;
	m_Jacobian.rows[0][11] = J4.z;



	// The quaternion jacobians
	const int ONE = 1;

	Vec4 tempVector;
	{
		J1.Zero();
		m_Jacobian.rows[1][0] = J1.x;
		m_Jacobian.rows[1][1] = J1.y;
		m_Jacobian.rows[1][2] = J1.z;

		tempVector = MatA * Vec4(0, u.x, u.y, u.z);
		J2 = Vec3(tempVector[ONE + 0], tempVector[ONE + 1], tempVector[ONE + 2]);
		m_Jacobian.rows[1][3] = J2.x;
		m_Jacobian.rows[1][4] = J2.y;
		m_Jacobian.rows[1][5] = J2.z;

		J3.Zero();
		m_Jacobian.rows[1][6] = J3.x;
		m_Jacobian.rows[1][7] = J3.y;
		m_Jacobian.rows[1][8] = J3.z;

		tempVector = MatB * Vec4(0, u.x, u.y, u.z);
		J4 = Vec3(tempVector[ONE + 0], tempVector[ONE + 1], tempVector[ONE + 2]);
		m_Jacobian.rows[1][9] = J4.x;
		m_Jacobian.rows[1][10] = J4.y;
		m_Jacobian.rows[1][11] = J4.z;
	}
	{
		J1.Zero();
		m_Jacobian.rows[2][0] = J1.x;
		m_Jacobian.rows[2][1] = J1.y;
		m_Jacobian.rows[2][2] = J1.z;

		tempVector = MatA * Vec4(0, v.x, v.y, v.z);
		J2 = Vec3(tempVector[ONE + 0], tempVector[ONE + 1], tempVector[ONE + 2]);
		m_Jacobian.rows[2][3] = J2.x;
		m_Jacobian.rows[2][4] = J2.y;
		m_Jacobian.rows[2][5] = J2.z;

		J3.Zero();
		m_Jacobian.rows[2][6] = J3.x;
		m_Jacobian.rows[2][7] = J3.y;
		m_Jacobian.rows[2][8] = J3.z;

		tempVector = MatB * Vec4(0, v.x, v.y, v.z);
		J4 = Vec3(tempVector[ONE + 0], tempVector[ONE + 1], tempVector[ONE + 2]);
		m_Jacobian.rows[2][9] = J4.x;
		m_Jacobian.rows[2][10] = J4.y;
		m_Jacobian.rows[2][11] = J4.z;
	}
	{
		J1.Zero();
		m_Jacobian.rows[3][0] = J1.x;
		m_Jacobian.rows[3][1] = J1.y;
		m_Jacobian.rows[3][2] = J1.z;

		tempVector = MatA * Vec4(0, h.x, h.y, h.z);
		J2 = Vec3(tempVector[ONE + 0], tempVector[ONE + 1], tempVector[ONE + 2]);
		m_Jacobian.rows[3][3] = J2.x;
		m_Jacobian.rows[3][4] = J2.y;
		m_Jacobian.rows[3][5] = J2.z;

		J3.Zero();
		m_Jacobian.rows[3][6] = J3.x;
		m_Jacobian.rows[3][7] = J3.y;
		m_Jacobian.rows[3][8] = J3.z;

		tempVector = MatB * Vec4(0, h.x, h.y, h.z);
		J4 = Vec3(tempVector[ONE + 0], tempVector[ONE + 1], tempVector[ONE + 2]);
		m_Jacobian.rows[3][9] = J4.x;
		m_Jacobian.rows[3][10] = J4.y;
		m_Jacobian.rows[3][11] = J4.z;
	}

	//	Calculate the baumgarte stabilization
	const float Beta = 0.05f;
	const float violatedDistance = anchorAToAnchorB.Dot(anchorAToAnchorB);

	const Quat relativeOrientationAB = m_bodyA->m_orientation.Inverse() * m_bodyB->m_orientation;
	const Quat currentRelativeOrientation = relativeOrientationAB * targetOrientationInv;	// Relative orientation in BodyA's space

	// Get the world space axis for the relative rotation
	const Vec3 axisA = m_bodyA->m_orientation.RotatePoint(currentRelativeOrientation.xyz());

	m_baumgarte.Zero();
	m_baumgarte[0] = (Beta / deltaSecond) * violatedDistance;
	m_baumgarte[1] = motorU.Dot(axisA) * (Beta / deltaSecond);
	m_baumgarte[2] = motorV.Dot(axisA) * (Beta / deltaSecond);
}

/*
================================
ConstraintSpinner::Solve
================================
*/
void ConstraintSpinner::Solve() {
	const Vec3 motorAxis = m_bodyA->m_orientation.RotatePoint(m_motorAxis);

	VecN negativeDesiredVelocities(12);
	negativeDesiredVelocities.Zero();
	negativeDesiredVelocities[3] = motorAxis[0] * -m_motorTargetSpeed;
	negativeDesiredVelocities[4] = motorAxis[1] * -m_motorTargetSpeed;
	negativeDesiredVelocities[5] = motorAxis[2] * -m_motorTargetSpeed;

	negativeDesiredVelocities[9] = motorAxis[0] * m_motorTargetSpeed;
	negativeDesiredVelocities[10] = motorAxis[1] * m_motorTargetSpeed;
	negativeDesiredVelocities[11] = motorAxis[2] * m_motorTargetSpeed;



	const MatMN JacobianTranspose = m_Jacobian.Transpose();

	// Build the system of equations
	// By subtracting by the desired velocity, the solver is tricked into applying the impulse to give us that velocity
	const VecN velocities = GetVelocities() - negativeDesiredVelocities;	
	const MatMN invMassMatrix = GetInverseMassMatrix();
	// Solve for the Lagrange multipliers based on this formula
	// J * M^-1 * J^T * lambda = -J*v
	// lambda here is the Lagrange multipliers that we need to handle
	const MatMN lhs = m_Jacobian * invMassMatrix * JacobianTranspose;
	VecN rhs = m_Jacobian * velocities * -1.0f;
	for (int currentIndex = 0; currentIndex < 3; ++currentIndex)
		rhs[currentIndex] -= m_baumgarte[currentIndex];

	// Solve for the Lagrange multipliers
	VecN lagrangeMultipliers = Solve_LCP_GaussSeidel(lhs, rhs);

	// Apply the impulses
	const VecN impulses = JacobianTranspose * lagrangeMultipliers;
	ApplyImpulses(impulses);
}
