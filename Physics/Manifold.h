//
//	Manifold.h
//
#pragma once
#include "Body.h"
#include "Constraints.h"
#include "Contact.h"

/*
================================
Manifold
================================
*/
class Manifold {
public:
	Manifold() : m_bodyA( nullptr ), m_bodyB( nullptr ), m_contactsCount( 0 ) {}

	void AddContact( const contact_t & contact );
	void RemoveExpiredContacts();

	void PreSolve( const float deltaSecond );
	void Solve();
	void PostSolve();

	contact_t GetContact( const int contactIndex ) const { return m_contacts[contactIndex]; }
	int GetContactsCount() const { return m_contactsCount; }

	Body* m_bodyA;
	Body* m_bodyB;
private:
	static const int MAX_CONTACTS = 4;
	contact_t m_contacts[ MAX_CONTACTS ];

	int m_contactsCount;

	ConstraintPenetration m_constraints[ MAX_CONTACTS ];

	friend class ManifoldCollector;
};

/*
================================
ManifoldCollector
================================
*/
class ManifoldCollector {
public:
	ManifoldCollector() {}

	void AddContact( const contact_t & contact );

	void PreSolve( const float deltaSecond );
	void Solve();
	void PostSolve();

	void RemoveExpired();
	void Clear() { m_manifolds.clear(); }	// For resetting the demo

public:
	std::vector< Manifold > m_manifolds;
};
