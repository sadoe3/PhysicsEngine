//
//  Manifold.cpp
//
#include "PCH.h"
#include "Manifold.h"


/*
================================================================================================
ManifoldCollector
================================================================================================
*/

/*
================================
ManifoldCollector::AddContact
================================
*/
void ManifoldCollector::AddContact(const contact_t& contact) {
	// Try to find the previously existing manifold for contacts between these two bodies
	int targetIndex = -1;
	for (int currentIndex = 0; currentIndex < m_manifolds.size(); ++currentIndex) {
		const Manifold& currentManifold = m_manifolds[currentIndex];
		bool hasA = (currentManifold.m_bodyA == contact.bodyA || currentManifold.m_bodyB == contact.bodyA);
		bool hasB = (currentManifold.m_bodyA == contact.bodyB || currentManifold.m_bodyB == contact.bodyB);
		if (hasA && hasB) {
			targetIndex = currentIndex;
			break;
		}
	}

	// Add contact to manifolds
	if (targetIndex >= 0) 
		m_manifolds[targetIndex].AddContact(contact);
	else {
		Manifold manifold;
		manifold.m_bodyA = contact.bodyA;
		manifold.m_bodyB = contact.bodyB;

		manifold.AddContact(contact);
		m_manifolds.push_back(manifold);
	}
}

/*
================================
ManifoldCollector::RemoveExpired
================================
*/
void ManifoldCollector::RemoveExpired() {
	// Remove expired manifolds
	for (int currentIndex = static_cast<int>(m_manifolds.size()) - 1; currentIndex >= 0; --currentIndex) {
		Manifold& currentManifold = m_manifolds[currentIndex];
		currentManifold.RemoveExpiredContacts();

		if (0 == currentManifold.m_contactsCount) 
			m_manifolds.erase(m_manifolds.begin() + currentIndex);
	}
}

/*
================================
ManifoldCollector::PreSolve
================================
*/
void ManifoldCollector::PreSolve(const float deltaSecond) {
	for (int currentIndex = 0; currentIndex < m_manifolds.size(); ++currentIndex) 
		m_manifolds[currentIndex].PreSolve(deltaSecond);
}
/*
================================
ManifoldCollector::Solve
================================
*/
void ManifoldCollector::Solve() {
	for (int currentIndex = 0; currentIndex < m_manifolds.size(); ++currentIndex)
		m_manifolds[currentIndex].Solve();
}
/*
================================
Manifold::PostSolve
================================
*/
void ManifoldCollector::PostSolve() {
	for (int currentIndex = 0; currentIndex < m_manifolds.size(); ++currentIndex)
		m_manifolds[currentIndex].PostSolve();
}



/*
================================================================================================
Manifold
================================================================================================
*/

/*
================================
Manifold::RemoveExpiredContacts
================================
*/
void Manifold::RemoveExpiredContacts() {
	Body* bodyA = nullptr;
	Body* bodyB = nullptr;

	// remove any contacts that have drifted too far
	for (int currentIndex = 0; currentIndex < m_contactsCount; ++currentIndex) {
		contact_t& currentContact = m_contacts[currentIndex];

		bodyA = currentContact.bodyA;
		bodyB = currentContact.bodyB;

		// Get the tangential distance of the point on A and the point on B
		const Vec3& contactPointA = bodyA->BodySpaceToWorldSpace(currentContact.ptOnA_LocalSpace);
		const Vec3& contactPointB = bodyB->BodySpaceToWorldSpace(currentContact.ptOnB_LocalSpace);

		Vec3 collisionNormal = m_constraints[currentIndex].m_collisionNormal;
		collisionNormal = bodyA->m_orientation.RotatePoint(collisionNormal);

		// Calculate the tangential separation and penetration depth
		const Vec3 originalAB = contactPointB - contactPointA;
		float penetrationDepth = collisionNormal.Dot(originalAB);
		Vec3 parallelComponentAB = collisionNormal * penetrationDepth;
		Vec3 perpendicularComponentAB = originalAB - parallelComponentAB;

		// If the tangential displacement is less than a specific threshold, it's okay to keep it
		const float distanceThreshold = 0.02f;
		if (perpendicularComponentAB.GetLengthSqr() < distanceThreshold * distanceThreshold && penetrationDepth <= 0.0f)
			continue;

		// This contact has moved beyond its threshold and should be removed
		for (int currentNestedIndex = currentIndex; currentNestedIndex < MAX_CONTACTS - 1; ++currentNestedIndex) {
			m_constraints[currentNestedIndex] = m_constraints[currentNestedIndex + 1];
			m_contacts[currentNestedIndex] = m_contacts[currentNestedIndex + 1];
			if (currentNestedIndex >= m_contactsCount)
				m_constraints[currentNestedIndex].m_cachedLagrange.Zero();
		}
		--m_contactsCount;
		--currentIndex;
	}
}

/*
================================
Manifold::AddContact
================================
*/
void Manifold::AddContact(const contact_t& contact_old) {
	// Make sure the contact's BodyA and BodyB are of the correct order
	contact_t targetContact = contact_old;
	if (contact_old.bodyA != m_bodyA || contact_old.bodyB != m_bodyB) {
		targetContact.ptOnA_LocalSpace = contact_old.ptOnB_LocalSpace;
		targetContact.ptOnB_LocalSpace = contact_old.ptOnA_LocalSpace;
		targetContact.ptOnA_WorldSpace = contact_old.ptOnB_WorldSpace;
		targetContact.ptOnB_WorldSpace = contact_old.ptOnA_WorldSpace;

		targetContact.bodyA = m_bodyA;
		targetContact.bodyB = m_bodyB;
	}

	// If this contact is close to another contact, then keep the old contact
	for (int currentIndex = 0; currentIndex < m_contactsCount; currentIndex++) {
		const Body* bodyA = m_contacts[currentIndex].bodyA;
		const Body* bodyB = m_contacts[currentIndex].bodyB;

		const Vec3 contactPointExistingA = bodyA->BodySpaceToWorldSpace(m_contacts[currentIndex].ptOnA_LocalSpace);
		const Vec3 contactPointExistingB = bodyB->BodySpaceToWorldSpace(m_contacts[currentIndex].ptOnB_LocalSpace);

		const Vec3 contactPointNewA = targetContact.bodyA->BodySpaceToWorldSpace(targetContact.ptOnA_LocalSpace);
		const Vec3 contactPointNewB = targetContact.bodyB->BodySpaceToWorldSpace(targetContact.ptOnB_LocalSpace);

		const Vec3 distanceExistingToNewA = contactPointNewA - contactPointExistingA;
		const Vec3 distanceExistingToNewB = contactPointNewB - contactPointExistingB;

		const float distanceThreshold = 0.02f;
		if (distanceExistingToNewA.GetLengthSqr() < distanceThreshold * distanceThreshold) 
			return;
		if (distanceExistingToNewB.GetLengthSqr() < distanceThreshold * distanceThreshold) 
			return;
	}

	// If we're all full on contacts, then keep the contacts that are furthest away from each other
	int targetSlot = m_contactsCount;
	if (targetSlot >= MAX_CONTACTS) {
		Vec3 averagePoint = Vec3(0, 0, 0);
		for (int currentContactIndex = 0; currentContactIndex < MAX_CONTACTS; ++currentContactIndex)
			averagePoint += m_contacts[currentContactIndex].ptOnA_LocalSpace;
		averagePoint += targetContact.ptOnA_LocalSpace;
		averagePoint *= 0.2f;

		float currentMinDistance = (averagePoint - targetContact.ptOnA_LocalSpace).GetLengthSqr();
		int targetIndex = -1;
		for (int currentIndex = 0; currentIndex < MAX_CONTACTS; ++currentIndex) {
			float currentDistance = (averagePoint - m_contacts[currentIndex].ptOnA_LocalSpace).GetLengthSqr();

			if (currentDistance < currentMinDistance) {
				currentMinDistance = currentDistance;
				targetIndex = currentIndex;
			}
		}

		if (-1 != targetIndex) 
			targetSlot = targetIndex;
		else 
			return;
	}

	m_contacts[targetSlot] = targetContact;

	m_constraints[targetSlot].m_bodyA = targetContact.bodyA;
	m_constraints[targetSlot].m_bodyB = targetContact.bodyB;
	m_constraints[targetSlot].m_anchorA = targetContact.ptOnA_LocalSpace;
	m_constraints[targetSlot].m_anchorB = targetContact.ptOnB_LocalSpace;

	// Get the collisionNormal in BodyA's space
	Vec3 collisionNormal = m_bodyA->m_orientation.Inverse().RotatePoint(targetContact.normal * -1.0f);
	m_constraints[targetSlot].m_collisionNormal = collisionNormal;
	m_constraints[targetSlot].m_collisionNormal.Normalize();

	m_constraints[targetSlot].m_cachedLagrange.Zero();

	if (targetSlot == m_contactsCount)
		++m_contactsCount;
}

/*
================================
Manifold::PreSolve
================================
*/
void Manifold::PreSolve(const float deltaSecond) {
	for (int currentIndex = 0; currentIndex < m_contactsCount; ++currentIndex) 
		m_constraints[currentIndex].PreSolve(deltaSecond);
}
/*
================================
Manifold::Solve
================================
*/
void Manifold::Solve() {
	for (int currentIndex = 0; currentIndex < m_contactsCount; ++currentIndex) 
		m_constraints[currentIndex].Solve();
}
/*
================================
Manifold::PostSolve
================================
*/
void Manifold::PostSolve() {
	for (int currentIndex = 0; currentIndex < m_contactsCount; ++currentIndex)
		m_constraints[currentIndex].PostSolve();
}
