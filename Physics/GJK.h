//
//	GJK.h
//
#pragma once
#include "Body.h"

bool DoesIntersect_GJK( const Body * bodyA, const Body * bodyB );
bool DoesIntersect_GJK( const Body * bodyA, const Body * bodyB, const float bias, Vec3 & ptOnA, Vec3 & ptOnB );
void FindClosestPoints_GJK( const Body * bodyA, const Body * bodyB, Vec3 & ptOnA, Vec3 & ptOnB );
