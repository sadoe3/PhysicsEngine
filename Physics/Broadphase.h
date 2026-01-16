//
//	Broadphase.h
//
#pragma once
#include "Body.h"

struct collisionPair_t {
	int a;
	int b;

	bool operator == ( const collisionPair_t & rhs ) const {
		return ( ( ( a == rhs.a ) && ( b == rhs.b ) ) || ( ( a == rhs.b ) && ( b == rhs.a ) ) );
	}
	bool operator != ( const collisionPair_t & rhs ) const {
		return !( *this == rhs );
	}
};

struct pseudoBody_t {
	int id;
	float value;
	bool isMin;
};

int CompareSAP(const void* lhs, const void* rhs);
void SortBodiesBounds(const Body* bodies, const int numBodies, pseudoBody_t* sortedArray, const float deltaSecond);
void BuildPairs(std::vector<collisionPair_t>& collisionPairs, const pseudoBody_t* sortedBodies, const int numBodies);
void SweepAndPrune1D(const Body* bodies, const int numBodies, std::vector<collisionPair_t>& finalPairs, const float deltaSecond);
void BroadPhase( const Body * bodies, const int numBodies, std::vector< collisionPair_t > & finalPairs, const float deltaSecond);
