//
//  Intersections.cpp
//
#include "PCH.h"
#include "Intersections.h"
#include "GJK.h"


/*
====================================================
Continuous Collision Checking
====================================================
*/
bool DoesHit_RaySphere(const Vec3& rayStart, const Vec3& rayDir, const Vec3& sphereCenter, const float sphereRadius, float& time1, float& time2) {
	const Vec3 vectorRayOriginToSphereCenter = sphereCenter - rayStart;
	const float a = rayDir.Dot(rayDir);
	// The standard derivation suggests b should be -1.0f * rayDir.Dot(vectorRayOriginToSphereCenter) for the quadratic form a*t^2 + 2b*t + c = 0.
	// However, this code uses b = rayDir.Dot(vectorRayOriginToSphereCenter) (positive) and adjusts the solution formula accordingly.
	// This avoids the negation operation.
	const float b = vectorRayOriginToSphereCenter.Dot(rayDir);
	const float c = vectorRayOriginToSphereCenter.Dot(vectorRayOriginToSphereCenter) - sphereRadius * sphereRadius;

	// because b^2 is equal to (-b)^2, the operation below makes sense
	const float discriminantSquared = b * b - a * c;
	const float invA = 1.0f / a;

	if (discriminantSquared < 0)
		return false;		// they don't collide

	const float discriminant = sqrtf(discriminantSquared);
	// Since b is caluclated without negation, the solution below makes sense although b is not negated
	time1 = invA * (b - discriminant);
	time2 = invA * (b + discriminant);
	return true;
}
bool DoesIntersect_SphereSphereDynamic(const ShapeSphere* shapeA, const ShapeSphere* shapeB, const Vec3& posA, const Vec3& posB, const Vec3& velA, const Vec3& velB,
	const float deltaTime, Vec3& pointOnA, Vec3& pointOnB, float& timeOfImpact) {
	const Vec3 relativeVelocity = velA - velB;

	Vec3 a = posA;
	Vec3 b = posA + relativeVelocity * deltaTime;
	const Vec3 rayDir = b - a;

	float time1 = 0;	// - discriminant
	float time2 = 0;	// + discriminant
	if (rayDir.GetLengthSqr() < 0.001f * 0.001f) {
		// ab is too short, just check if already intersecting
		Vec3 ab = posB - posA;
		float radiusPlusRadius = shapeA->m_radius + shapeB->m_radius + 0.001f;
		if (ab.GetLengthSqr() > radiusPlusRadius * radiusPlusRadius)
			return false;
	}
	else if (!DoesHit_RaySphere(posA, rayDir, posB, shapeA->m_radius + shapeB->m_radius, time1, time2))
		return false;

	// If the control flow reaches this point, it means that the ray representing 
	// the future movement of sphereA is not currently intersecting with sphereB.
	// However, sphereA will collide with sphereB after some time (time1 or time2).

	// change from [0,1] range to [0,dt] range
	time1 *= deltaTime;
	time2 *= deltaTime;

	// if the collision is only in the past, then there's no future collision in this frame
	if (time2 < 0.0f)
		return false;

	// get the earliest positive time of impact
	timeOfImpact = (time1 < 0.0f) ? 0.0f : time1;

	// if the earliest collision is too far in the future, then there's no collision in this frame
	if (timeOfImpact > deltaTime)
		return false;
	
	// at this point, timeOfImpact can be zero or positive
	// if it's zero, then it means that the two spheres are currently intersecting
	// otherwise, they will collide soon
	
	// get the points on the respective points of collision and return true
	a = posA + velA * timeOfImpact;
	b = posB + velB * timeOfImpact;
	Vec3 ab = b - a;
	ab.Normalize();

	pointOnA = a + ab * shapeA->m_radius;
	pointOnB = b - ab * shapeB->m_radius;
	return true;
}



/*
====================================================
DoesIntersect_SphereSphereStatic
====================================================
*/
bool DoesIntersect_SphereSphereStatic(const ShapeSphere* sphereA, const ShapeSphere* sphereB, const Vec3& posA, const Vec3& posB, Vec3& ptOnA, Vec3& ptOnB) {
	const Vec3 ab = posB - posA;
	Vec3 norm = ab;
	norm.Normalize();

	ptOnA = posA + norm * sphereA->m_radius;
	ptOnB = posB - norm * sphereB->m_radius;

	const float radiusAB = sphereA->m_radius + sphereB->m_radius;
	const float lengthSquare = ab.GetLengthSqr();
	if (lengthSquare <= (radiusAB * radiusAB)) {
		return true;
	}

	return false;
}
/*
====================================================
DoesIntersect
====================================================
*/
bool DoesIntersect(Body* bodyA, Body* bodyB, contact_t& contact) {
	contact.bodyA = bodyA;
	contact.bodyB = bodyB;
	contact.timeOfImpact = 0.0f;

	if (bodyA->m_shape->GetType() == Shape::SHAPE_SPHERE && bodyB->m_shape->GetType() == Shape::SHAPE_SPHERE) {
		const ShapeSphere* sphereA = (const ShapeSphere*)bodyA->m_shape;
		const ShapeSphere* sphereB = (const ShapeSphere*)bodyB->m_shape;

		Vec3 posA = bodyA->m_position;
		Vec3 posB = bodyB->m_position;

		if (DoesIntersect_SphereSphereStatic(sphereA, sphereB, posA, posB, contact.ptOnA_WorldSpace, contact.ptOnB_WorldSpace)) {
			contact.normal = posA - posB;
			contact.normal.Normalize();

			contact.ptOnA_LocalSpace = bodyA->WorldSpaceToBodySpace(contact.ptOnA_WorldSpace);
			contact.ptOnB_LocalSpace = bodyB->WorldSpaceToBodySpace(contact.ptOnB_WorldSpace);

			Vec3 ab = bodyB->m_position - bodyA->m_position;
			float r = ab.GetMagnitude() - (sphereA->m_radius + sphereB->m_radius);
			contact.separationDistance = r;
			return true;
		}
	}
	else {
		Vec3 ptOnA;
		Vec3 ptOnB;
		const float bias = 0.001f;
		if (DoesIntersect_GJK(bodyA, bodyB, bias, ptOnA, ptOnB)) {
			// There was an intersection, so get the contact data
			Vec3 normal = ptOnB - ptOnA;
			normal.Normalize();

			ptOnA -= normal * bias;
			ptOnB += normal * bias;

			contact.normal = normal;

			contact.ptOnA_WorldSpace = ptOnA;
			contact.ptOnB_WorldSpace = ptOnB;

			contact.ptOnA_LocalSpace = bodyA->WorldSpaceToBodySpace(contact.ptOnA_WorldSpace);
			contact.ptOnB_LocalSpace = bodyB->WorldSpaceToBodySpace(contact.ptOnB_WorldSpace);

			Vec3 ab = bodyB->m_position - bodyA->m_position;
			float r = (ptOnA - ptOnB).GetMagnitude();
			contact.separationDistance = -r;
			return true;
		}

		// There was no collision, but we still want the contact data, so get it
		FindClosestPoints_GJK(bodyA, bodyB, ptOnA, ptOnB);
		contact.ptOnA_WorldSpace = ptOnA;
		contact.ptOnB_WorldSpace = ptOnB;

		contact.ptOnA_LocalSpace = bodyA->WorldSpaceToBodySpace(contact.ptOnA_WorldSpace);
		contact.ptOnB_LocalSpace = bodyB->WorldSpaceToBodySpace(contact.ptOnB_WorldSpace);

		Vec3 ab = bodyB->m_position - bodyA->m_position;
		float r = (ptOnA - ptOnB).GetMagnitude();
		contact.separationDistance = r;
	}
	return false;
}


/*
====================================================
DoesIntersect_ConservativeAdvance
====================================================
*/
bool DoesIntersect_ConservativeAdvance(Body* bodyA, Body* bodyB, float deltaTime, contact_t& contact) {
	contact.bodyA = bodyA;
	contact.bodyB = bodyB;

	float toi = 0.0f;

	int currentIterationCount = 0;

	// Advance the positions of the bodies until they touch or there's not time left
	while (deltaTime > 0.0f) {
		// Check for intersection
		bool didIntersect = DoesIntersect(bodyA, bodyB, contact);
		if (didIntersect) {
			contact.timeOfImpact = toi;
			bodyA->Update(-toi);
			bodyB->Update(-toi);
			return true;
		}

		++currentIterationCount;
		if (currentIterationCount > 10)
			break;

		// Get the vector from the closest point on A to the closest point on B
		Vec3 ab = contact.ptOnB_WorldSpace - contact.ptOnA_WorldSpace;
		ab.Normalize();

		// project the relative velocity onto the ray of shortest distance
		Vec3 relativeVelocity = bodyA->m_linearVelocity - bodyB->m_linearVelocity;
		float orthoSpeed = relativeVelocity.Dot(ab);

		// Add to the orthoSpeed the maximum angular speeds of the relative shapes
		float angularSpeedA = bodyA->m_shape->GetFastestLinearSpeed(bodyA->m_angularVelocity, ab);
		float angularSpeedB = bodyB->m_shape->GetFastestLinearSpeed(bodyB->m_angularVelocity, ab * -1.0f);
		orthoSpeed += angularSpeedA + angularSpeedB;
		if (orthoSpeed <= 0.0f) 
			break;

		float timeToGo = contact.separationDistance / orthoSpeed;
		if (timeToGo > deltaTime) 
			break;

		deltaTime -= timeToGo;
		toi += timeToGo;
		bodyA->Update(timeToGo);
		bodyB->Update(timeToGo);
	}

	// unwind the clock
	bodyA->Update(-toi);
	bodyB->Update(-toi);
	return false;
}
/*
====================================================
DoesIntersect
====================================================
*/
bool DoesIntersect(Body* bodyA, Body* bodyB, const float deltaTime, contact_t& contact) {
	contact.bodyA = bodyA;
	contact.bodyB = bodyB;

	if (bodyA->m_shape->GetType() == Shape::SHAPE_SPHERE && bodyB->m_shape->GetType() == Shape::SHAPE_SPHERE) {
		const ShapeSphere* sphereA = (const ShapeSphere*)bodyA->m_shape;
		const ShapeSphere* sphereB = (const ShapeSphere*)bodyB->m_shape;

		Vec3 posA = bodyA->m_position;
		Vec3 posB = bodyB->m_position;

		Vec3 velA = bodyA->m_linearVelocity;
		Vec3 velB = bodyB->m_linearVelocity;

		if (DoesIntersect_SphereSphereDynamic(sphereA, sphereB, posA, posB, velA, velB, deltaTime, contact.ptOnA_WorldSpace, contact.ptOnB_WorldSpace, contact.timeOfImpact)) {
			// Step bodies forward to get local space collision points
			bodyA->Update(contact.timeOfImpact);
			bodyB->Update(contact.timeOfImpact);

			// Convert world space contacts to local space
			contact.ptOnA_LocalSpace = bodyA->WorldSpaceToBodySpace(contact.ptOnA_WorldSpace);
			contact.ptOnB_LocalSpace = bodyB->WorldSpaceToBodySpace(contact.ptOnB_WorldSpace);

			contact.normal = bodyA->m_position - bodyB->m_position;
			contact.normal.Normalize();

			// Unwind time step
			bodyA->Update(-contact.timeOfImpact);
			bodyB->Update(-contact.timeOfImpact);

			// Calculate the separation distance
			Vec3 ab = bodyB->m_position - bodyA->m_position;
			float distance = ab.GetMagnitude() - (sphereA->m_radius + sphereB->m_radius);
			contact.separationDistance = distance;
			return true;
		}
	}
	else {
		// Use GJK to perform conservative advancement
		bool result = DoesIntersect_ConservativeAdvance(bodyA, bodyB, deltaTime, contact);
		return result;
	}
	return false;
}
