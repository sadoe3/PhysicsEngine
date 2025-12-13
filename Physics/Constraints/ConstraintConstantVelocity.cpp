//
//  ConstraintConstantVelocity.cpp
//
#include "ConstraintConstantVelocity.h"

/*
================================================================================================
ConstraintConstantVelocity
================================================================================================
*/
/*
================================
ConstraintConstantVelocity::PreSolve
================================
*/
void ConstraintConstantVelocity::PreSolve(const float deltaSecond) {
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

	// This axis is defined in the local space of bodyA
	Vec3 u;
	Vec3 v;
	Vec3 restrictedAxis = m_axisA;
	restrictedAxis.GetOrtho(u, v);

	Mat4 projectionMatrix;
	projectionMatrix.rows[0] = Vec4(0, 0, 0, 0);
	projectionMatrix.rows[1] = Vec4(0, 1, 0, 0);
	projectionMatrix.rows[2] = Vec4(0, 0, 1, 0);
	projectionMatrix.rows[3] = Vec4(0, 0, 0, 1);
	Mat4 pTranspose = projectionMatrix.Transpose();	// currentIndex know it's pointless to do this with our particular matrix implementations.  But currentIndex like its self commenting.

	const Mat4 MatA = projectionMatrix * GetQuatLeftMatrix(orientationAInv) * GetQuatRightMatrix(orientationB * targetOrientationInv) * pTranspose * -0.5f;
	const Mat4 MatB = projectionMatrix * GetQuatLeftMatrix(orientationAInv) * GetQuatRightMatrix(orientationB * targetOrientationInv) * pTranspose * 0.5f;

	m_Jacobian.Zero();

	// Distance constraint
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
	{
		const int ONE = 1;

		Vec4 tempVector;

		J1.Zero();
		m_Jacobian.rows[1][0] = J1.x;
		m_Jacobian.rows[1][1] = J1.y;
		m_Jacobian.rows[1][2] = J1.z;

		tempVector = MatA * Vec4(0, restrictedAxis.x, restrictedAxis.y, restrictedAxis.z) * -0.5f;
		J2 = Vec3(tempVector[ONE + 0], tempVector[ONE + 1], tempVector[ONE + 2]);
		m_Jacobian.rows[1][3] = J2.x;
		m_Jacobian.rows[1][4] = J2.y;
		m_Jacobian.rows[1][5] = J2.z;

		J3.Zero();
		m_Jacobian.rows[1][6] = J3.x;
		m_Jacobian.rows[1][7] = J3.y;
		m_Jacobian.rows[1][8] = J3.z;

		tempVector = MatB * Vec4(0, restrictedAxis.x, restrictedAxis.y, restrictedAxis.z) * 0.5f;
		J4 = Vec3(tempVector[ONE + 0], tempVector[ONE + 1], tempVector[ONE + 2]);
		m_Jacobian.rows[1][9] = J4.x;
		m_Jacobian.rows[1][10] = J4.y;
		m_Jacobian.rows[1][11] = J4.z;
	}

	// Apply warm starting from last frame
	const VecN impulses = m_Jacobian.Transpose() * m_cachedLagrange;
	ApplyImpulses(impulses);

	//	Calculate the baumgarte stabilization
	float violatedDistance = anchorAToAnchorB.Dot(anchorAToAnchorB);
	violatedDistance = std::max(0.0f, violatedDistance - 0.01f);
	const float Beta = 0.05f;
	m_baumgarte = (Beta / deltaSecond) * violatedDistance;
}

/*
================================
ConstraintConstantVelocity::Solve
================================
*/
void ConstraintConstantVelocity::Solve() {
	const MatMN JacobianTranspose = m_Jacobian.Transpose();

	// Build the system of equations
	const VecN velocities = GetVelocities();
	const MatMN invMassMatrix = GetInverseMassMatrix();
	// Solve for the Lagrange multipliers based on this formula
	// J * M^-1 * J^T * lambda = -J*v
	// lambda here is the Lagrange multipliers that we need to handle
	const MatMN lhs = m_Jacobian * invMassMatrix * JacobianTranspose;
	VecN rhs = m_Jacobian * velocities * -1.0f;
	rhs[0] -= m_baumgarte;

	// Solve for the Lagrange multipliers
	const VecN lagrangeMultipliers = Solve_LCP_GaussSeidel(lhs, rhs);

	// Apply the impulses
	const VecN impulses = JacobianTranspose * lagrangeMultipliers;
	ApplyImpulses(impulses);

	// Accumulate the impulses for warm starting
	m_cachedLagrange += lagrangeMultipliers;
}

/*
================================
ConstraintConstantVelocity::PostSolve
================================
*/
void ConstraintConstantVelocity::PostSolve() {
	// Limit the warm starting to reasonable limits
	for (int currentIndex = 0; currentIndex < m_cachedLagrange.N; ++currentIndex) {
		if (m_cachedLagrange[currentIndex] * 0.0f != m_cachedLagrange[currentIndex] * 0.0f) 
			m_cachedLagrange[currentIndex] = 0.0f;
		const float limit = 20.0f;
		if (m_cachedLagrange[currentIndex] > limit) 
			m_cachedLagrange[currentIndex] = limit;
		if (m_cachedLagrange[currentIndex] < -limit) 
			m_cachedLagrange[currentIndex] = -limit;
	}
}



/*
================================================================================================
ConstraintConstantVelocityLimited
================================================================================================
*/
/*
================================
ConstraintConstantVelocityLimited::PreSolve
================================
*/
void ConstraintConstantVelocityLimited::PreSolve(const float deltaSecond) {
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

	// This axis is defined in the local space of bodyA
	Vec3 u;
	Vec3 v;
	Vec3 restrictedAxis = m_axisA;
	restrictedAxis.GetOrtho(u, v);

	Mat4 projectionMatrix;
	projectionMatrix.rows[0] = Vec4(0, 0, 0, 0);
	projectionMatrix.rows[1] = Vec4(0, 1, 0, 0);
	projectionMatrix.rows[2] = Vec4(0, 0, 1, 0);
	projectionMatrix.rows[3] = Vec4(0, 0, 0, 1);
	const Mat4 pTranspose = projectionMatrix.Transpose();	// currentIndex know it's pointless to do this with our particular matrix implementations.  But currentIndex like its self commenting.

	const Mat4 MatA = projectionMatrix * GetQuatLeftMatrix(orientationAInv) * GetQuatRightMatrix(orientationB * targetOrientationInv) * pTranspose * -0.5f;
	const Mat4 MatB = projectionMatrix * GetQuatLeftMatrix(orientationAInv) * GetQuatRightMatrix(orientationB * targetOrientationInv) * pTranspose * 0.5f;

	// Check the constraint's angular limits
	const float pi = acosf(-1.0f);
	const Quat relativeOrientationAB = orientationAInv * orientationB;
	const Quat currentRelativeOrientation = relativeOrientationAB * targetOrientationInv;
	m_relativeAngleU = 2.0f * asinf(currentRelativeOrientation.xyz().Dot(u)) * 180.0f / pi;
	m_relativeAngleV = 2.0f * asinf(currentRelativeOrientation.xyz().Dot(v)) * 180.0f / pi;

	m_isAngleViolatedU = false;
	const float angleLimit = 45.0f;
	if (m_relativeAngleU > angleLimit) 
		m_isAngleViolatedU = true;
	if (m_relativeAngleU < -angleLimit) 
		m_isAngleViolatedU = true;

	m_isAngleViolatedV = false;
	if (m_relativeAngleV > angleLimit) 
		m_isAngleViolatedV = true;
	if (m_relativeAngleV < -angleLimit) 
		m_isAngleViolatedV = true;

	// First row is the primary distance constraint that holds the anchor points together
	m_Jacobian.Zero();

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

		tempVector = MatA * Vec4(0, restrictedAxis.x, restrictedAxis.y, restrictedAxis.z) * -0.5f;
		J2 = Vec3(tempVector[ONE + 0], tempVector[ONE + 1], tempVector[ONE + 2]);
		m_Jacobian.rows[1][3] = J2.x;
		m_Jacobian.rows[1][4] = J2.y;
		m_Jacobian.rows[1][5] = J2.z;

		J3.Zero();
		m_Jacobian.rows[1][6] = J3.x;
		m_Jacobian.rows[1][7] = J3.y;
		m_Jacobian.rows[1][8] = J3.z;

		tempVector = MatB * Vec4(0, restrictedAxis.x, restrictedAxis.y, restrictedAxis.z) * 0.5f;
		J4 = Vec3(tempVector[ONE + 0], tempVector[ONE + 1], tempVector[ONE + 2]);
		m_Jacobian.rows[1][9] = J4.x;
		m_Jacobian.rows[1][10] = J4.y;
		m_Jacobian.rows[1][11] = J4.z;
	}

	if (m_isAngleViolatedU) {
		tempVector.Zero();

		J1.Zero();
		m_Jacobian.rows[2][0] = J1.x;
		m_Jacobian.rows[2][1] = J1.y;
		m_Jacobian.rows[2][2] = J1.z;

		tempVector = MatA * Vec4(0, u.x, u.y, u.z) * -0.5f;
		J2 = Vec3(tempVector[ONE + 0], tempVector[ONE + 1], tempVector[ONE + 2]);
		m_Jacobian.rows[2][3] = J2.x;
		m_Jacobian.rows[2][4] = J2.y;
		m_Jacobian.rows[2][5] = J2.z;

		J3.Zero();
		m_Jacobian.rows[2][6] = J3.x;
		m_Jacobian.rows[2][7] = J3.y;
		m_Jacobian.rows[2][8] = J3.z;

		tempVector = MatB * Vec4(0, u.x, u.y, u.z) * 0.5f;
		J4 = Vec3(tempVector[ONE + 0], tempVector[ONE + 1], tempVector[ONE + 2]);
		m_Jacobian.rows[2][9] = J4.x;
		m_Jacobian.rows[2][10] = J4.y;
		m_Jacobian.rows[2][11] = J4.z;
	}

	if (m_isAngleViolatedV) {
		tempVector.Zero();

		J1.Zero();
		m_Jacobian.rows[3][0] = J1.x;
		m_Jacobian.rows[3][1] = J1.y;
		m_Jacobian.rows[3][2] = J1.z;

		tempVector = MatA * Vec4(0, v.x, v.y, v.z) * -0.5f;
		J2 = Vec3(tempVector[ONE + 0], tempVector[ONE + 1], tempVector[ONE + 2]);
		m_Jacobian.rows[3][3] = J2.x;
		m_Jacobian.rows[3][4] = J2.y;
		m_Jacobian.rows[3][5] = J2.z;

		J3.Zero();
		m_Jacobian.rows[3][6] = J3.x;
		m_Jacobian.rows[3][7] = J3.y;
		m_Jacobian.rows[3][8] = J3.z;

		tempVector = MatB * Vec4(0, v.x, v.y, v.z) * 0.5f;
		J4 = Vec3(tempVector[ONE + 0], tempVector[ONE + 1], tempVector[ONE + 2]);
		m_Jacobian.rows[3][9] = J4.x;
		m_Jacobian.rows[3][10] = J4.y;
		m_Jacobian.rows[3][11] = J4.z;
	}

	// Apply warm starting from last frame
	const VecN impulses = m_Jacobian.Transpose() * m_cachedLagrange;
	ApplyImpulses(impulses);

	//	Calculate the baumgarte stabilization
	float violatedDistance = anchorAToAnchorB.Dot(anchorAToAnchorB);
	violatedDistance = std::max(0.0f, violatedDistance - 0.01f);
	const float Beta = 0.05f;
	m_baumgarte = (Beta / deltaSecond) * violatedDistance;
}

/*
================================
ConstraintConstantVelocityLimited::Solve
================================
*/
void ConstraintConstantVelocityLimited::Solve() {
	const MatMN JacobianTranspose = m_Jacobian.Transpose();

	// Build the system of equations
	const VecN velocities = GetVelocities();
	const MatMN invMassMatrix = GetInverseMassMatrix();
	// Solve for the Lagrange multipliers based on this formula
	// J * M^-1 * J^T * lambda = -J*v
	// lambda here is the Lagrange multipliers that we need to handle
	const MatMN lhs = m_Jacobian * invMassMatrix * JacobianTranspose;
	VecN rhs = m_Jacobian * velocities * -1.0f;
	rhs[0] -= m_baumgarte;

	// Solve for the Lagrange multipliers
	VecN lagrangeMultipliers = Solve_LCP_GaussSeidel(lhs, rhs);

	// Clamp the torque from the angle constraint.
	// We need to make sure it's anchorA restorative torque.
	if (m_isAngleViolatedU) {
		if (m_relativeAngleU > 0.0f) 
			lagrangeMultipliers[2] = std::min(0.0f, lagrangeMultipliers[2]);
		if (m_relativeAngleU < 0.0f) 
			lagrangeMultipliers[2] = std::max(0.0f, lagrangeMultipliers[2]);
	}
	if (m_isAngleViolatedV) {
		if (m_relativeAngleV > 0.0f) 
			lagrangeMultipliers[3] = std::min(0.0f, lagrangeMultipliers[3]);
		if (m_relativeAngleV < 0.0f) 
			lagrangeMultipliers[3] = std::max(0.0f, lagrangeMultipliers[3]);
	}

	// Apply the impulses
	const VecN impulses = JacobianTranspose * lagrangeMultipliers;
	ApplyImpulses(impulses);

	// Accumulate the impulses for warm starting
	m_cachedLagrange += lagrangeMultipliers;
}

/*
================================
ConstraintConstantVelocityLimited::PostSolve
================================
*/
void ConstraintConstantVelocityLimited::PostSolve() {
	// Limit the warm starting to reasonable limits
	for (int currentIndex = 0; currentIndex < m_cachedLagrange.N; ++currentIndex) {
		if (currentIndex > 0) 
			m_cachedLagrange[currentIndex] = 0.0f;

		if (m_cachedLagrange[currentIndex] * 0.0f != m_cachedLagrange[currentIndex] * 0.0f) 
			m_cachedLagrange[currentIndex] = 0.0f;
		const float limit = 20.0f;
		if (m_cachedLagrange[currentIndex] > limit) 
			m_cachedLagrange[currentIndex] = limit;
		if (m_cachedLagrange[currentIndex] < -limit) 
			m_cachedLagrange[currentIndex] = -limit;
	}
}
