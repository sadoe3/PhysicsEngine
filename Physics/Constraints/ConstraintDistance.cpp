//
//  ConstraintDistance.cpp
//
#include "PCH.h"
#include "ConstraintDistance.h"

/*
================================
ConstraintDistance::PreSolve
================================
*/
void ConstraintDistance::PreSolve(const float deltaSecond) {
	// Get the world space position of the hinge from A's orientation
	const Vec3 worldAnchorA = m_bodyA->BodySpaceToWorldSpace(m_anchorA);

	// Get the world space position of the hinge from B's orientation
	const Vec3 worldAnchorB = m_bodyB->BodySpaceToWorldSpace(m_anchorB);

	const Vec3 anchorAToAnchorB = worldAnchorB - worldAnchorA;
	const Vec3 centerToAnchorA = worldAnchorA - m_bodyA->GetCenterOfMassWorldSpace();
	const Vec3 centerToAnchorB = worldAnchorB - m_bodyB->GetCenterOfMassWorldSpace();
	const Vec3 anchorA = worldAnchorA;
	const Vec3 anchorB = worldAnchorB;

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

	// Apply warm starting from last frame
	const VecN impulsesFromPreviousFrame = m_Jacobian.Transpose() * m_cachedLagrange;
	ApplyImpulses(impulsesFromPreviousFrame);


	// Calculate the baumgarte stabilization
	float violatedDistance = anchorAToAnchorB.Dot(anchorAToAnchorB);
	// we don't stabilize if the distance is less than 0.01 
	violatedDistance = std::max(0.0f, violatedDistance - 0.01f);
	const float Beta = 0.05f;
	m_baumgarte = (Beta / deltaSecond) * violatedDistance;
}

/*
================================
ConstraintDistance::Solve
================================
*/
void ConstraintDistance::Solve() {
	const MatMN JacobianTranspose = m_Jacobian.Transpose();

	// Build the system of equations
	const VecN velocities = GetVelocities();
	const MatMN invMassMatrix = GetInverseMassMatrix();
	// Solve for the Lagrange multipliers based on this formula
	// J * M^-1 * J^T * lambda = -J*v
	// lambda here is the Lagrange multipliers that we need to handle
	const MatMN lhs = m_Jacobian * invMassMatrix * JacobianTranspose;
	VecN rhs = m_Jacobian * velocities * -1.0f;
	// apply stabilization
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
ConstraintDistance::PostSolve
================================
*/
void ConstraintDistance::PostSolve() {
	// Limit the warm starting to reasonable limits
	if (m_cachedLagrange[0] * 0.0f != m_cachedLagrange[0] * 0.0f) 
		m_cachedLagrange[0] = 0.0f;


	const float limit = 1e5f;
	if (m_cachedLagrange[0] > limit) 
		m_cachedLagrange[0] = limit;
	if (m_cachedLagrange[0] < -limit)
		m_cachedLagrange[0] = -limit;
}
