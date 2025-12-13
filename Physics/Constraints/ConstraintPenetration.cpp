//
//  ConstraintPenetration.cpp
//
#include "ConstraintPenetration.h"

/*
================================
ConstraintPenetration::PreSolve
================================
*/
void ConstraintPenetration::PreSolve(const float deltaSecond) {
	// Get the world space position of the hinge from A's orientation
	const Vec3 worldAnchorA = m_bodyA->BodySpaceToWorldSpace(m_anchorA);

	// Get the world space position of the hinge from B's orientation
	const Vec3 worldAnchorB = m_bodyB->BodySpaceToWorldSpace(m_anchorB);

	const Vec3 centerToAnchorA = worldAnchorA - m_bodyA->GetCenterOfMassWorldSpace();
	const Vec3 centerToAnchorB = worldAnchorB - m_bodyB->GetCenterOfMassWorldSpace();
	const Vec3 anchorA = worldAnchorA;
	const Vec3 anchorB = worldAnchorB;

	m_Jacobian.Zero();
	// Convert collision normal from local space to world space
	Vec3 normal = m_bodyA->m_orientation.RotatePoint(m_collisionNormal);

	// Penetration Constraint (parallel to collision normal)
	// First row is the primary distance constraint that holds the anchor points together
	Vec3 J1 = normal * -1.0f;
	m_Jacobian.rows[0][0] = J1.x;
	m_Jacobian.rows[0][1] = J1.y;
	m_Jacobian.rows[0][2] = J1.z;

	Vec3 J2 = centerToAnchorA.Cross(normal * -1.0f);
	m_Jacobian.rows[0][3] = J2.x;
	m_Jacobian.rows[0][4] = J2.y;
	m_Jacobian.rows[0][5] = J2.z;

	Vec3 J3 = normal * 1.0f;
	m_Jacobian.rows[0][6] = J3.x;
	m_Jacobian.rows[0][7] = J3.y;
	m_Jacobian.rows[0][8] = J3.z;

	Vec3 J4 = centerToAnchorB.Cross(normal * 1.0f);
	m_Jacobian.rows[0][9] = J4.x;
	m_Jacobian.rows[0][10] = J4.y;
	m_Jacobian.rows[0][11] = J4.z;


	// Friction Jacobians (orthogonal to collision normal)
	const float frictionA = m_bodyA->m_friction;
	const float frictionB = m_bodyB->m_friction;
	m_friction = frictionA * frictionB;

	// the unit vectors for each axis that are perpendicular to the collision normal and perpendicular to each other
	Vec3 u;
	Vec3 v;
	m_collisionNormal.GetOrtho(u, v);

	u = m_bodyA->m_orientation.RotatePoint(u);
	v = m_bodyA->m_orientation.RotatePoint(v);
	if (m_friction > 0.0f) {
		// second row
		J1 = u * -1.0f;
		m_Jacobian.rows[1][0] = J1.x;
		m_Jacobian.rows[1][1] = J1.y;
		m_Jacobian.rows[1][2] = J1.z;

		J2 = centerToAnchorA.Cross(u * -1.0f);
		m_Jacobian.rows[1][3] = J2.x;
		m_Jacobian.rows[1][4] = J2.y;
		m_Jacobian.rows[1][5] = J2.z;

		J3 = u * 1.0f;
		m_Jacobian.rows[1][6] = J3.x;
		m_Jacobian.rows[1][7] = J3.y;
		m_Jacobian.rows[1][8] = J3.z;

		J4 = centerToAnchorB.Cross(u * 1.0f);
		m_Jacobian.rows[1][9] = J4.x;
		m_Jacobian.rows[1][10] = J4.y;
		m_Jacobian.rows[1][11] = J4.z;


		// third row
		J1 = v * -1.0f;
		m_Jacobian.rows[2][0] = J1.x;
		m_Jacobian.rows[2][1] = J1.y;
		m_Jacobian.rows[2][2] = J1.z;

		J2 = centerToAnchorA.Cross(v * -1.0f);
		m_Jacobian.rows[2][3] = J2.x;
		m_Jacobian.rows[2][4] = J2.y;
		m_Jacobian.rows[2][5] = J2.z;

		J3 = v * 1.0f;
		m_Jacobian.rows[2][6] = J3.x;
		m_Jacobian.rows[2][7] = J3.y;
		m_Jacobian.rows[2][8] = J3.z;

		J4 = centerToAnchorB.Cross(v * 1.0f);
		m_Jacobian.rows[2][9] = J4.x;
		m_Jacobian.rows[2][10] = J4.y;
		m_Jacobian.rows[2][11] = J4.z;
	}


	// Apply warm starting from last frame
	const VecN impulsesFromPreviousFrame = m_Jacobian.Transpose() * m_cachedLagrange;
	ApplyImpulses(impulsesFromPreviousFrame);

	// Calculate the baumgarte stabilization
	float violatedDistance = (anchorB - anchorA).Dot(normal);
	// we don't stabilize if the penetration depth is less than 0.02
	violatedDistance = std::min(0.0f, violatedDistance + 0.02f);
	float Beta = 0.25f;
	m_baumgarte = Beta * violatedDistance / deltaSecond;
}


/*
================================
ConstraintPenetration::PreSolve
================================
*/
void ConstraintPenetration::Solve() {
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
	VecN currentMultipliers = Solve_LCP_GaussSeidel(lhs, rhs);


	// Accumulate the impulses and clamp to within the constraint limits
	VecN previousAccumulatedMultipliers = m_cachedLagrange;
	m_cachedLagrange += currentMultipliers;
	const float lambdaLimit = 0.0f;
	if (m_cachedLagrange[0] < lambdaLimit) 
		m_cachedLagrange[0] = lambdaLimit;

	if (m_friction > 0.0f) {
		// two possible limit value
		const float gravityBasedLimit = m_friction * 10.0f * 1.0f / (m_bodyA->m_invMass + m_bodyB->m_invMass);
		const float normalBasedLimit = fabsf(currentMultipliers[0] * m_friction);
		const float maxForce = (gravityBasedLimit > normalBasedLimit) ? gravityBasedLimit : normalBasedLimit;

		if (m_cachedLagrange[1] > maxForce) 
			m_cachedLagrange[1] = maxForce;
		if (m_cachedLagrange[1] < -maxForce) 
			m_cachedLagrange[1] = -maxForce;


		if (m_cachedLagrange[2] > maxForce) 
			m_cachedLagrange[2] = maxForce;
		if (m_cachedLagrange[2] < -maxForce) 
			m_cachedLagrange[2] = -maxForce;
	}
	// Modify currentMultipliers regarding the limit values
	currentMultipliers = m_cachedLagrange - previousAccumulatedMultipliers;

	// Apply the impulses constructed by the modified multipliers
	const VecN impulses = JacobianTranspose * currentMultipliers;
	ApplyImpulses(impulses);
}
