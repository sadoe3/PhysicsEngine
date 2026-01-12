//
//  ShapeConvex.cpp
//
#include "PCH.h"
#include "ShapeConvex.h"

/*
Random Number Generator
*/
std::mt19937 Random::m_engine(0);
std::uniform_real_distribution<float> Random::m_distribution(0.0f, 1.0f);


/*
====================================================
FindPointFurthestInDir
====================================================
*/
int FindPointFurthestInDir(const Vec3* points, const int totalNumberOfPoints, const Vec3& dir) {
	int currentSupportIndex = 0;
	float currentMaxDistance = dir.Dot(points[0]), currentDistance;
	for (int currentIndex = 1; currentIndex < totalNumberOfPoints; ++currentIndex) {
		currentDistance = dir.Dot(points[currentIndex]);
		if (currentDistance > currentMaxDistance) {
			currentMaxDistance = currentDistance;
			currentSupportIndex = currentIndex;
		}
	}
	return currentSupportIndex;
}
/*
====================================================
GetDistanceFromLine
====================================================
*/
float GetDistanceFromLine(const Vec3& a, const Vec3& b, const Vec3& pointTarget) {
	Vec3 ab = b - a;
	ab.Normalize();

	Vec3 vectorToTarget = pointTarget - a;
	Vec3 projectedVector = ab * vectorToTarget.Dot(ab);	// project vectorToTarget onto ab
	Vec3 perpendicularVector = vectorToTarget - projectedVector;
	return perpendicularVector.GetMagnitude();
}
/*
====================================================
FindPointFurthestFromLine
====================================================
*/
Vec3 FindPointFurthestFromLine(const Vec3* points, const int totalNumberOfPoints, const Vec3 &lineStart, const Vec3 &lineEnd) {
	int currentSupportIndex = 0;
	float currentMaxDistance = GetDistanceFromLine(lineStart, lineEnd, points[0]), currentDistance;
	for (int currentIndex = 1; currentIndex < totalNumberOfPoints; ++currentIndex) {
		currentDistance = GetDistanceFromLine(lineStart, lineEnd, points[currentIndex]);
		if (currentDistance > currentMaxDistance) {
			currentMaxDistance = currentDistance;
			currentSupportIndex = currentIndex;
		}
	}
	return points[currentSupportIndex];
}
/*
====================================================
GetDistanceFromTriangle
====================================================
*/
float GetDistanceFromTriangle(const Vec3&a, const Vec3&b, const Vec3 &c, const Vec3 &pointTarget) {
	Vec3 ab = b - a;
	Vec3 ac = c - a;
	Vec3 normal = ab.Cross(ac);
	normal.Normalize();

	Vec3 vectorToTarget = pointTarget - a;
	float distance = vectorToTarget.Dot(normal);
	return distance;
}
/*
====================================================
FindPointFurthestFromTriangle
====================================================
*/
Vec3 FindPointFurthestFromTriangle(const Vec3 *points, const int totalNumberOfPoints, const Vec3 &triangleA, const Vec3& triangleB, const Vec3 & triangleC) {
	int currentSupportIndex = 0;
	float currentMaxDistance = GetDistanceFromTriangle(triangleA, triangleB, triangleC, points[0]), currentDistance;
	for (int currentIndex = 1; currentIndex < totalNumberOfPoints; ++currentIndex) {
		currentDistance = GetDistanceFromTriangle(triangleA, triangleB, triangleC, points[currentIndex]);
		if (currentDistance * currentDistance > currentMaxDistance * currentMaxDistance) {
			currentMaxDistance = currentDistance;
			currentSupportIndex = currentIndex;
		}
	}
	return points[currentSupportIndex];
}
/*
====================================================
BuildTetrahedron
====================================================
*/
void BuildTetrahedron(const Vec3* vertices, const int totalNumberOfVertices, std::vector< Vec3 >& hullPoints, std::vector< tri_t >& hullTris) {
	hullPoints.clear();
	hullTris.clear();

	Vec3 finalSimplex[4];

	int supportIndex = FindPointFurthestInDir(vertices, totalNumberOfVertices, Vec3(1, 0, 0));
	finalSimplex[0] = vertices[supportIndex];
	supportIndex = FindPointFurthestInDir(vertices, totalNumberOfVertices, finalSimplex[0] * -1.0f);
	finalSimplex[1] = vertices[supportIndex];
	finalSimplex[2] = FindPointFurthestFromLine(vertices, totalNumberOfVertices, finalSimplex[0], finalSimplex[1]);
	finalSimplex[3] = FindPointFurthestFromTriangle(vertices, totalNumberOfVertices, finalSimplex[0], finalSimplex[1], finalSimplex[2]);

	// This is important for making sure the ordering is CCW for all faces
	float distance = GetDistanceFromTriangle(finalSimplex[0], finalSimplex[1], finalSimplex[2], finalSimplex[3]);
	if (distance > 0.0f)
		std::swap(finalSimplex[0], finalSimplex[1]);

	// Builid the tetrahedron
	hullPoints.push_back(finalSimplex[0]);
	hullPoints.push_back(finalSimplex[1]);
	hullPoints.push_back(finalSimplex[2]);
	hullPoints.push_back(finalSimplex[3]);

	tri_t triangle;
	triangle.a = 0;
	triangle.b = 1;
	triangle.c = 2;
	hullTris.push_back(triangle);

	triangle.a = 0;
	triangle.b = 2;
	triangle.c = 3;
	hullTris.push_back(triangle);

	triangle.a = 2;
	triangle.b = 1;
	triangle.c = 3;
	hullTris.push_back(triangle);

	triangle.a = 1;
	triangle.b = 0;
	triangle.c = 3;
	hullTris.push_back(triangle);
}
/*
====================================================
RemoveInternalPoints
====================================================
*/
void RemoveInternalPoints(std::vector<Vec3>& hullPoints, std::vector<tri_t>& hullTris, std::vector<Vec3>& validPoints) {
	float distance;
	bool isExternal;
	for (int currentIndex = 0; currentIndex < validPoints.size(); ++currentIndex) {
		const Vec3& currentPoint = validPoints[currentIndex];

		isExternal = false;
		for (int currenntIndexTris = 0; currenntIndexTris < hullTris.size(); ++currenntIndexTris) {
			const tri_t& currentTriangle = hullTris[currenntIndexTris];
			const Vec3& a = hullPoints[currentTriangle.a];
			const Vec3& b = hullPoints[currentTriangle.b];
			const Vec3& c = hullPoints[currentTriangle.c];
			
			// if the point is in fron of any triangle then it's external
			distance = GetDistanceFromTriangle(a, b, c, currentPoint);
			if (distance > 0.0f) {
				isExternal = true;
				break;
			}
		}

		// if it's not external, then it's inside the polyhedron and should be removed
		if (isExternal == false) {
			validPoints.erase(validPoints.begin() + currentIndex);
			--currentIndex;
		}
	}

	// Moreover remove any points that are just a little too close to the hull points
	Vec3 currentHullPoint, tempVector;
	bool isTooClose;
	for (int currentIndex = 0; currentIndex < validPoints.size(); ++currentIndex) {
		const Vec3& currentPoint = validPoints[currentIndex];

		isTooClose = false;
		for (int currentIndexHullPoints = 0; currentIndexHullPoints < hullPoints.size(); ++currentIndexHullPoints) {
			currentHullPoint = hullPoints[currentIndexHullPoints];
			tempVector = currentHullPoint - currentPoint;
			if (tempVector.GetLengthSqr() < 0.01f * 0.01f) {
				isTooClose = true;
				break;
			}
		}

		if (isTooClose) {
			validPoints.erase(validPoints.begin() + currentIndex);
			--currentIndex;
		}
	}
}
/*
====================================================
IsEdgeUnique
====================================================
*/
bool IsEdgeUnique(std::vector<tri_t>& tris, std::vector<int>& facingTris, const int triangleIndexToIgnore, const edge_t &existingEdge) {
	edge_t edges[3];
	for (int currentIndex = 0; currentIndex < facingTris.size(); ++currentIndex) {
		const int currentTriangleIndex = facingTris[currentIndex];
		if (triangleIndexToIgnore == currentTriangleIndex)
			continue;

		const tri_t& currentTriangle = tris[currentTriangleIndex];
		edges[0].a = currentTriangle.a;
		edges[0].b = currentTriangle.b;

		edges[1].a = currentTriangle.b;
		edges[1].b = currentTriangle.c;

		edges[2].a = currentTriangle.c;
		edges[2].b = currentTriangle.a;

		for (int currentIndexEdge = 0; currentIndexEdge < 3; ++currentIndexEdge) {
			if (existingEdge == edges[currentIndexEdge])
				return false;
		}
	}
	return true;
}
/*
====================================================
AddPoint
====================================================
*/
void AddPoint(std::vector<Vec3>& hullPoints, std::vector<tri_t>& hullTris, Vec3& newPoint) {
	// This point is outside
	// Now se need to remove old triangles and build new ones

	// Find all trianglees that face this point
	std::vector<int> facingTris;
	for (int currentIndex = static_cast<int>(hullTris.size()) - 1; currentIndex >= 0; --currentIndex) {
		const tri_t& currentTriangle = hullTris[currentIndex];

		const Vec3& a = hullPoints[currentTriangle.a];
		const Vec3& b = hullPoints[currentTriangle.b];
		const Vec3& c = hullPoints[currentTriangle.c];
		
		if (GetDistanceFromTriangle(a, b, c, newPoint) > 0.0f)
			facingTris.push_back(currentIndex);
	}

	// Now find all edges that are unique to the tris, these will be edges that form the new triangles
	std::vector<edge_t> uniqueEdges;
	edge_t edges[3];
	for (int currentIndex = 0; currentIndex < facingTris.size(); ++currentIndex) {
		const int currentTriangleIndex = facingTris[currentIndex];
		const tri_t& currentTriangle = hullTris[currentTriangleIndex];

		edges[0].a = currentTriangle.a;
		edges[0].b = currentTriangle.b;

		edges[1].a = currentTriangle.b;
		edges[1].b = currentTriangle.c;

		edges[2].a = currentTriangle.c;
		edges[2].b = currentTriangle.a;

		for (int currentIndexEdge = 0; currentIndexEdge < 3; ++currentIndexEdge) {
			if (IsEdgeUnique(hullTris, facingTris, currentTriangleIndex, edges[currentIndexEdge]))
				uniqueEdges.push_back(edges[currentIndexEdge]);
		}
	}

	// now remove the old facing tris
	for (int currentIndex = 0; currentIndex < facingTris.size(); ++currentIndex)
		hullTris.erase(hullTris.begin() + facingTris[currentIndex]);

	// now add the new point
	hullPoints.push_back(newPoint);
	const int newPointIndex = static_cast<int>(hullPoints.size()) - 1;

	// now add triangles for each unique edge
	tri_t currentTriangle;
	for (int currentIndex = 0; currentIndex < uniqueEdges.size(); ++currentIndex) {
		const edge_t& currentEdge = uniqueEdges[currentIndex];

		currentTriangle.a = currentEdge.a;
		currentTriangle.b = currentEdge.b;
		currentTriangle.c = newPointIndex;
		hullTris.push_back(currentTriangle);
	}
}
/*
====================================================
RemoveUnreferencedVertices
====================================================
*/
void RemoveUnreferencedVertices(std::vector<Vec3>& hullPoints, std::vector<tri_t>& hullTris) {
	for (int currentIndex = 0; currentIndex < hullPoints.size(); ++currentIndex) {
		bool isUsed = false;
		for (int currentIndexTris = 0; currentIndexTris < hullTris.size(); ++currentIndexTris) {
			const tri_t& currentTriangle = hullTris[currentIndexTris];

			if (currentTriangle.a == currentIndex || currentTriangle.b == currentIndex || currentTriangle.c == currentIndex) {
				isUsed = true;
				break;
			}
		}
		if (isUsed)
			continue;

		for (int currentIndexTris = 0; currentIndexTris < hullTris.size(); ++currentIndexTris) {
			tri_t& currentTriangle = hullTris[currentIndexTris];
			if (currentTriangle.a > currentIndex)
				--currentTriangle.a;
			if (currentTriangle.b > currentIndex)
				--currentTriangle.b;
			if (currentTriangle.c > currentIndex)
				--currentTriangle.c;
		}

		hullPoints.erase(hullPoints.begin() + currentIndex);
		--currentIndex;
	}
}
/*
====================================================
ExpandConvexHull
====================================================
*/
void ExpandConvexHull(std::vector< Vec3 >& hullPoints, std::vector< tri_t >& hullTris, const std::vector<Vec3>& inputVertices) {
	std::vector<Vec3> externalVertices = inputVertices;
	RemoveInternalPoints(hullPoints, hullTris, externalVertices);

	while (externalVertices.size() > 0) {
		int supportIndex = FindPointFurthestInDir(externalVertices.data(), (int)externalVertices.size(), externalVertices[0]);
		Vec3 supportPoint = externalVertices[supportIndex];

		// remove this element
		externalVertices.erase(externalVertices.begin() + supportIndex);
		AddPoint(hullPoints, hullTris, supportPoint);
		RemoveInternalPoints(hullPoints, hullTris, externalVertices);
	}
	RemoveUnreferencedVertices(hullPoints, hullTris);
}


/*
====================================================
IsExternal
====================================================
*/
bool IsExternal(const std::vector<Vec3>& points, const std::vector<tri_t>& tris, const Vec3& targetPoint) {
	bool isExternal = false;
	for (int t = 0; t < tris.size(); t++) {
		const tri_t& tri = tris[t];
		const Vec3& a = points[tri.a];
		const Vec3& b = points[tri.b];
		const Vec3& c = points[tri.c];

		// If the point is in front of any triangle then it's external
		float dist = GetDistanceFromTriangle(a, b, c, targetPoint);
		if (dist > 0.0f) {
			isExternal = true;
			break;
		}
	}

	return isExternal;
}

/*
====================================================
CalculateCenterOfMass
====================================================
*/
Vec3 CalculateCenterOfMass(const std::vector< Vec3 >& points, const std::vector< tri_t >& tris) {
	const int NUMBER_OF_SAMPLES = 10000;

	Bounds bounds;
	bounds.Expand(points.data(), points.size());

	// now it's revised with the Monte Carlo method
	Vec3 centerOfMass(0.0f), currentSamplePoint;
	int currentNumberOfSamples = 0;
	for (int currentCount = 0; currentCount < NUMBER_OF_SAMPLES; ++currentCount) {
		currentSamplePoint.x = bounds.mins.x + Random::Get() * bounds.WidthX();
		currentSamplePoint.y = bounds.mins.y + Random::Get() * bounds.WidthY();
		currentSamplePoint.z = bounds.mins.z + Random::Get() * bounds.WidthZ();

		if (IsExternal(points, tris, currentSamplePoint))
			continue;

		centerOfMass += currentSamplePoint;
		++currentNumberOfSamples;
	}

	centerOfMass /= static_cast<float>(currentNumberOfSamples);
	return centerOfMass;
}
/*
====================================================
CalculateInertiaTensor
====================================================
*/
Mat3 CalculateInertiaTensor(const std::vector< Vec3 >& points, const std::vector<tri_t>& tris, const Vec3& centerOfMass) {
	const int NUMBER_OF_SAMPLES = 10000;

	Bounds bounds;
	bounds.Expand(points.data(), (int)points.size());

	Mat3 inertiaTensor;
	inertiaTensor.Zero();

	// now it's revised with the Monte Carlo method
	Vec3 currentSamplePoint;
	int currentNumberOfSamples = 0;
	for (int currentCount = 0; currentCount < NUMBER_OF_SAMPLES; ++currentCount) {
		currentSamplePoint.x = bounds.mins.x + Random::Get() * bounds.WidthX();
		currentSamplePoint.y = bounds.mins.y + Random::Get() * bounds.WidthY();
		currentSamplePoint.z = bounds.mins.z + Random::Get() * bounds.WidthZ();

		if (IsExternal(points, tris, currentSamplePoint))
			continue;

		// Get the point relative to the center of mass
		currentSamplePoint -= centerOfMass;

		inertiaTensor.rows[0][0] += currentSamplePoint.y * currentSamplePoint.y + currentSamplePoint.z * currentSamplePoint.z;
		inertiaTensor.rows[1][1] += currentSamplePoint.z * currentSamplePoint.z + currentSamplePoint.x * currentSamplePoint.x;
		inertiaTensor.rows[2][2] += currentSamplePoint.x * currentSamplePoint.x + currentSamplePoint.y * currentSamplePoint.y;

		inertiaTensor.rows[0][1] += -1.0f * currentSamplePoint.x * currentSamplePoint.y;
		inertiaTensor.rows[0][2] += -1.0f * currentSamplePoint.x * currentSamplePoint.z;
		inertiaTensor.rows[1][2] += -1.0f * currentSamplePoint.y * currentSamplePoint.z;

		inertiaTensor.rows[1][0] += -1.0f * currentSamplePoint.x * currentSamplePoint.y;
		inertiaTensor.rows[2][0] += -1.0f * currentSamplePoint.x * currentSamplePoint.z;
		inertiaTensor.rows[2][1] += -1.0f * currentSamplePoint.y * currentSamplePoint.z;

		++currentNumberOfSamples;
	}

	inertiaTensor *= 1.0f / static_cast<float>(currentNumberOfSamples);
	return inertiaTensor;
}


/*
====================================================
CalculateTetrahedronVolume
====================================================
*/
float CalculateTetrahedronVolume(const Vec3& pointA, const Vec3& pointB, const Vec3& pointC, const Vec3& pointD) {
	const Vec3 AC = pointC - pointA;
	const Vec3 AB = pointB - pointA;
	const Vec3 AD = pointD - pointA;

	float volumeOfParallelepiped = AD.Dot(AB.Cross(AC));
	float volumeOfTetrahedron = volumeOfParallelepiped / 6.0f;
	return fabsf(volumeOfTetrahedron);
}
/*
====================================================
CalculateCenterOfMassUsingTetrahedrons
====================================================
*/
Vec3 CalculateCenterOfMassUsingTetrahedrons(const std::vector< Vec3 >& points, const std::vector< tri_t >& tris) {
	// it's the revised approach using decomposition of the convex shape
	std::vector<Vec3> centerOfMasses;
	std::vector<float> volumes;
	float totalVolume = 0.0f;
	centerOfMasses.reserve(tris.size());
	volumes.reserve(tris.size());

	Vec3 centerPoint(0.0f);
	for (int currentIndex = 0, endIndex = points.size(); currentIndex < endIndex; ++currentIndex)
		centerPoint += points[currentIndex];
	centerPoint *= 1.0f / static_cast<float>(points.size());

	for (int currentIndex = 0, endIndex = tris.size(); currentIndex < endIndex; ++currentIndex) {
		const tri_t& currentTriangle = tris[currentIndex];

		const Vec3 pointA = centerPoint;
		const Vec3 pointB = points[currentTriangle.a];
		const Vec3 pointC = points[currentTriangle.b];
		const Vec3 pointD = points[currentTriangle.c];

		const Vec3 currentCenterOfMass = (pointA + pointB + pointC + pointD) * 0.25f;
		const float currentVolume = CalculateTetrahedronVolume(pointA, pointB, pointC, pointD);
		centerOfMasses.push_back(currentCenterOfMass);
		volumes.push_back(currentVolume);
		totalVolume += currentVolume;
	}

	Vec3 centerOfMass(0.0f);
	for (int currentTetrahedronIndex = 0, endIndex = centerOfMasses.size(); currentTetrahedronIndex < endIndex; ++currentTetrahedronIndex)
		centerOfMass += centerOfMasses[currentTetrahedronIndex] * volumes[currentTetrahedronIndex];
	centerOfMass *= 1.0f / totalVolume;
	return centerOfMass;
}
/*
====================================================
CalculateTermsForDiagonal
====================================================
*/
inline float CalculateTermsForDiagonal(const float c1, const float c2, const float c3) {
	// this is the support function which helps the calculation for the diagonal element of the inertia tensor
	// the target formula is
	// c_1^2 + c_1c_2 + c_2^2 + c_1c_3 + c_2c_3 + c_3^2
	return (c1*c1 + c1*c2 + c2*c2 + c1*c3 + c2*c3 + c3*c3);
}
/*
====================================================
CalculateTermsForOffDiagonal
====================================================
*/
inline float CalculateTermsForOffDiagonal(const float a1, const float a2, const float a3, const float b1, const float b2, const float b3) {
	// this is the support function which helps the calculation for the off-diagonal element of the inertia tensor
	// the target formula is
	// a_1b_1 + a_1b_2 + a_2b_1 + a_2b_2 + a_1b_3 + a_3b_1 + a_2b_3 + a_3b_2 + a_3b_3
	return (a1*b1 + a1*b2 + a2*b1 + a2*b2 + a1*b3 + a3*b1 + a2*b3 + a3*b2 + a3*b3);
}

/*
====================================================
CalculateInertiaTensorOfTetrahedron
====================================================
*/
Mat3 CalculateInertiaTensorOfTetrahedron(const Vec3& pointB, const Vec3& pointC, const Vec3& pointD, const float mass) {
	// this function assumes that the three points are translates so that pointA becomes the origin
	// since the origin is not involved during calculation, it's manually excluded in this function

	Mat3 inertiaTensor;
	// Diagonal Elements
	float coefficient = mass / 20.0f;
	inertiaTensor.rows[0][0] = coefficient * (CalculateTermsForDiagonal(pointB.y, pointC.y, pointD.y) + CalculateTermsForDiagonal(pointB.z, pointC.z, pointD.z));
	inertiaTensor.rows[1][1] = coefficient * (CalculateTermsForDiagonal(pointB.x, pointC.x, pointD.x) + CalculateTermsForDiagonal(pointB.z, pointC.z, pointD.z));
	inertiaTensor.rows[2][2] = coefficient * (CalculateTermsForDiagonal(pointB.x, pointC.x, pointD.x) + CalculateTermsForDiagonal(pointB.y, pointC.y, pointD.y));


	// Off-Diagonal Elements
	coefficient *= -1.0f;
	inertiaTensor.rows[0][1] = coefficient * CalculateTermsForOffDiagonal(pointB.x, pointC.x, pointD.x, pointB.y, pointC.y, pointD.y);
	inertiaTensor.rows[1][0] = inertiaTensor.rows[0][1];
	inertiaTensor.rows[1][2] = coefficient * CalculateTermsForOffDiagonal(pointB.y, pointC.y, pointD.y, pointB.z, pointC.z, pointD.z);
	inertiaTensor.rows[2][1] = inertiaTensor.rows[1][2];
	inertiaTensor.rows[0][2] = coefficient * CalculateTermsForOffDiagonal(pointB.x, pointC.x, pointD.x, pointB.z, pointC.z, pointD.z);
	inertiaTensor.rows[2][0] = inertiaTensor.rows[0][2];
	
	return inertiaTensor;
}
/*
====================================================
CalculateInertiaTensorUsingTetrahedrons
====================================================
*/
Mat3 CalculateInertiaTensorUsingTetrahedrons(const std::vector< Vec3 >& points, const std::vector<tri_t>& tris, const Vec3& centerOfMass) {
	// it's the revised approach using decomposition of the convex shape
	Mat3 inertiaTensor;
	inertiaTensor.Zero();
	float totalVolume = 0.0f;

	for (int currentIndex = 0, endIndex = tris.size(); currentIndex < endIndex; ++currentIndex) {
		const tri_t& currentTriangle = tris[currentIndex];

		// translate each sub-tetrahedron so that pointA becomes the origin (0,0,0)
		const Vec3 pointA = centerOfMass - centerOfMass;
		const Vec3 pointB = points[currentTriangle.a] - centerOfMass;
		const Vec3 pointC = points[currentTriangle.b] - centerOfMass;
		const Vec3 pointD = points[currentTriangle.c] - centerOfMass;
		
		const float currentVolume = CalculateTetrahedronVolume(pointA, pointB, pointC, pointD);
		totalVolume += currentVolume;

		// this function assumes that the object's density is 1
		const Mat3 currentInertiaTensor = CalculateInertiaTensorOfTetrahedron(pointB, pointC, pointD, currentVolume);
		inertiaTensor += currentInertiaTensor;
	}
	inertiaTensor *= 1.0f / totalVolume;
	return inertiaTensor;
}



/*
====================================================
BuildConvexHull
====================================================
*/
void BuildConvexHull(const std::vector< Vec3 >& vertices, std::vector< Vec3 >& hullPoints, std::vector< tri_t >& hullTris) {
	if (vertices.size() < 4)
		return;

	BuildTetrahedron(vertices.data(), static_cast<int>(vertices.size()), hullPoints, hullTris);
	ExpandConvexHull(hullPoints, hullTris, vertices);
}


/*
========================================================================================================

ShapeConvex

========================================================================================================
*/

/*
====================================================
ShapeConvex::Build
====================================================
*/
void ShapeConvex::Build(const Vec3* points, const int totalNumberOfPoints) {
	m_points.clear();
	m_points.reserve(totalNumberOfPoints);
	for (int currentPointIndex = 0; currentPointIndex < totalNumberOfPoints; ++currentPointIndex)
		m_points.push_back(points[currentPointIndex]);

	// Expand into a convex hull
	std::vector< Vec3 > hullPoints;
	std::vector< tri_t > hullTriangles;
	BuildConvexHull(m_points, hullPoints, hullTriangles);
	m_points = hullPoints;
	// todo
	m_triangles = hullTriangles;

	// Expand the bounds
	m_bounds.Clear();
	m_bounds.Expand(m_points.data(), m_points.size());

	m_centerOfMass = CalculateCenterOfMassUsingTetrahedrons(hullPoints, hullTriangles);

	m_inertiaTensor = CalculateInertiaTensorUsingTetrahedrons(hullPoints, hullTriangles, m_centerOfMass);
}

/*
====================================================
ShapeConvex::GetSupportPoint
====================================================
*/
Vec3 ShapeConvex::GetSupportPoint(const Vec3& dir, const Vec3& pos, const Quat& orient, const float bias) const {
	// Find the point in furthest in direction
	Vec3 supportPoint = orient.RotatePoint(m_points[0]) + pos;
	float currentMaxDistance = dir.Dot(supportPoint);
	for (int i = 1; i < m_points.size(); i++) {
		const Vec3 currentPoint = orient.RotatePoint(m_points[i]) + pos;
		const float currentDistance = dir.Dot(currentPoint);

		if (currentDistance > currentMaxDistance) {
			currentMaxDistance = currentDistance;
			supportPoint = currentPoint;
		}
	}

	Vec3 norm = dir;
	norm.Normalize();
	norm *= bias;

	return supportPoint + norm;
}

/*
====================================================
ShapeConvex::GetBounds
====================================================
*/
Bounds ShapeConvex::GetBounds( const Vec3 & pos, const Quat & orient ) const {
	Vec3 corners[8];
	corners[0] = Vec3(m_bounds.mins.x, m_bounds.mins.y, m_bounds.mins.z);
	corners[1] = Vec3(m_bounds.mins.x, m_bounds.mins.y, m_bounds.maxs.z);
	corners[2] = Vec3(m_bounds.mins.x, m_bounds.maxs.y, m_bounds.mins.z);
	corners[3] = Vec3(m_bounds.maxs.x, m_bounds.mins.y, m_bounds.mins.z);

	corners[4] = Vec3(m_bounds.maxs.x, m_bounds.maxs.y, m_bounds.maxs.z);
	corners[5] = Vec3(m_bounds.maxs.x, m_bounds.maxs.y, m_bounds.mins.z);
	corners[6] = Vec3(m_bounds.maxs.x, m_bounds.mins.y, m_bounds.maxs.z);
	corners[7] = Vec3(m_bounds.mins.x, m_bounds.maxs.y, m_bounds.maxs.z);

	Bounds bounds;
	for (int currentIndex = 0; currentIndex < 8; ++currentIndex) {
		corners[currentIndex] = orient.RotatePoint(corners[currentIndex]) + pos;
		bounds.Expand(corners[currentIndex]);
	}

	return bounds;
}

/*
====================================================
ShapeConvex::GetFastestLinearSpeed
====================================================
*/
float ShapeConvex::GetFastestLinearSpeed( const Vec3 & angularVelocity, const Vec3 & dir ) const {
	float maxSpeed = 0.0f, currentSpeed = maxSpeed;
	Vec3 offsetFromCOM, linearVelocity;
	for (int currentIndex = 0; currentIndex < m_points.size(); ++currentIndex) {
		offsetFromCOM = m_points[currentIndex] - m_centerOfMass;
		linearVelocity = angularVelocity.Cross(offsetFromCOM);
		currentSpeed = dir.Dot(linearVelocity);
		if (currentSpeed > maxSpeed)
			maxSpeed = currentSpeed;
	}

	return maxSpeed;
}
