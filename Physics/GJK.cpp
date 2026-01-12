//
//  GJK.cpp
//
#include "PCH.h"
#include "GJK.h"

struct point_t;
float Expand_EPA(const Body* bodyA, const Body* bodyB, const float bias, const point_t simplexPoints[4], Vec3& ptOnA, Vec3& ptOnB);

/*
================================================================================================

Signed Volumes

================================================================================================
*/

/*
================================
GetBarycentricCoordinatesFromLineToOrigin
================================
*/
Vec2 GetBarycentricCoordinatesFromLineToOrigin(const Vec3& s1, const Vec3& s2) {
	Vec3 ab = s2 - s1;			// Ray from a to b
	Vec3 ao = Vec3(0.0f) - s1;	// Ray from a to origin
	Vec3 ap = s1 + ab * ab.Dot(ao) / ab.GetLengthSqr();	// projection of the origin onto the line

	// Choose the axis with the greatest difference/length
	int targetAxis = 0;
	float maxDifference = 0;
	for (int currentAxis = 0; currentAxis < 3; ++currentAxis) {
		float currentDifference = s2[currentAxis] - s1[currentAxis];
		if (currentDifference * currentDifference > maxDifference * maxDifference) {
			maxDifference = currentDifference;
			targetAxis = currentAxis;
		}
	}

	// Project the simplex points and projected origin onto the axis with greatest length
	const float a = s1[targetAxis];
	const float b = s2[targetAxis];
	const float p = ap[targetAxis];

	// Get the signed distance from A to P and from P to B
	const float DISTANCE_AP = p - a;
	const float DISTANCE_PB = b - p;

	// if P is between [A,B]
	if ((p > a && p < b) || (p > b && p < a)) {
		Vec2 lambdas;
		lambdas[0] = DISTANCE_PB / maxDifference;
		lambdas[1] = DISTANCE_AP / maxDifference;
		return lambdas;
	}

	// if P is on the far side of A
	if ((a <= b && p <= a) || (a >= b && p >= a))
		return Vec2(1.0f, 0.0f);

	// P must be on the far side of B
	return Vec2(0.0f, 1.0f);
}

/*
================================
CompareSigns
================================
*/
int CompareSigns(float a, float b) {
	if (a > 0.0f && b > 0.0f) {
		return 1;
	}
	if (a < 0.0f && b < 0.0f) {
		return 1;
	}
	return 0;
}

/*
================================
GetBarycentricCoordinatesFromTriangleToOrigin
================================
*/
Vec3 GetBarycentricCoordinatesFromTriangleToOrigin(const Vec3& s1, const Vec3& s2, const Vec3& s3) {
	Vec3 normal = (s2 - s1).Cross(s3 - s1);		// surface normal of triangle
	Vec3 projectedOA = normal * s1.Dot(normal) / normal.GetLengthSqr(); // project the vector from origin to s1 onto the normal

	// Find the axis with the greatest projected area
	int targetAxis = 0;
	float maxArea = 0;
	for (int currentAxis = 0; currentAxis < 3; ++currentAxis) {
		int axisA = (currentAxis + 1) % 3;
		int axisB = (currentAxis + 2) % 3;

		Vec2 a = Vec2(s1[axisA], s1[axisB]);
		Vec2 b = Vec2(s2[axisA], s2[axisB]);
		Vec2 c = Vec2(s3[axisA], s3[axisB]);
		Vec2 ab = b - a;
		Vec2 ac = c - a;

		float currentArea = ab.x * ac.y - ab.y * ac.x;
		if (currentArea * currentArea > maxArea * maxArea) {
			targetAxis = currentAxis;
			maxArea = currentArea;
		}
	}

	// Project onto the appropriate axis
	int axisA = (targetAxis + 1) % 3;
	int axisB = (targetAxis + 2) % 3;
	Vec2 projectedTriangle[3];
	projectedTriangle[0] = Vec2(s1[axisA], s1[axisB]);
	projectedTriangle[1] = Vec2(s2[axisA], s2[axisB]);
	projectedTriangle[2] = Vec2(s3[axisA], s3[axisB]);
	Vec2 projectedOriginVertex = Vec2(projectedOA[axisA], projectedOA[axisB]);

	// Get the sub-areas of the triangles formed from the projected origin and the edges
	Vec3 areas;
	for (int currentVertex = 0; currentVertex < 3; ++currentVertex) {
		int vertexA = (currentVertex + 1) % 3;
		int vertexB = (currentVertex + 2) % 3;

		Vec2 a = projectedOriginVertex;
		Vec2 b = projectedTriangle[vertexA];
		Vec2 c = projectedTriangle[vertexB];
		Vec2 ab = b - a;
		Vec2 ac = c - a;

		areas[currentVertex] = ab.x * ac.y - ab.y * ac.x;
	}

	// If the projected origin is inside the triangle, then return the barycentric points
	if (CompareSigns(maxArea, areas[0]) > 0 && CompareSigns(maxArea, areas[1]) > 0 && CompareSigns(maxArea, areas[2]) > 0) {
		Vec3 lambdas = areas / maxArea;
		return lambdas;
	}

	// If we make it here, then we need to project onto the edges and determine the closest point
	float closestDistance = 1e10;
	Vec3 lambdas = Vec3(1, 0, 0);
	Vec3 edgesPoints[3];
	for (int currentCount = 0; currentCount < 3; ++currentCount) {
		int indexA = (currentCount + 1) % 3;
		int indexB = (currentCount + 2) % 3;

		edgesPoints[0] = s1;
		edgesPoints[1] = s2;
		edgesPoints[2] = s3;

		Vec2 lambdaEdge = GetBarycentricCoordinatesFromLineToOrigin(edgesPoints[indexA], edgesPoints[indexB]);
		Vec3 currentPoint = edgesPoints[indexA] * lambdaEdge[0] + edgesPoints[indexB] * lambdaEdge[1];
		if (currentPoint.GetLengthSqr() < closestDistance) {
			closestDistance = currentPoint.GetLengthSqr();
			lambdas[currentCount] = 0;
			lambdas[indexA] = lambdaEdge[0];
			lambdas[indexB] = lambdaEdge[1];
		}
	}

	return lambdas;
}

/*
================================
GetBarycentricCoordinatesFromTetrahedronToOrigin
================================
*/
Vec4 GetBarycentricCoordinatesFromTetrahedronToOrigin(const Vec3& s1, const Vec3& s2, const Vec3& s3, const Vec3& s4) {
	Mat4 simplexMatrix;
	simplexMatrix.rows[0] = Vec4(s1.x, s2.x, s3.x, s4.x);
	simplexMatrix.rows[1] = Vec4(s1.y, s2.y, s3.y, s4.y);
	simplexMatrix.rows[2] = Vec4(s1.z, s2.z, s3.z, s4.z);
	simplexMatrix.rows[3] = Vec4(1.0f, 1.0f, 1.0f, 1.0f);

	Vec4 cofactors;
	cofactors[0] = simplexMatrix.Cofactor(3, 0);
	cofactors[1] = simplexMatrix.Cofactor(3, 1);
	cofactors[2] = simplexMatrix.Cofactor(3, 2);
	cofactors[3] = simplexMatrix.Cofactor(3, 3);

	const float signedTetrahedronVolume = cofactors[0] + cofactors[1] + cofactors[2] + cofactors[3];

	// If the barycentric coordinates put the origin inside the simplex, then return them
	if (CompareSigns(signedTetrahedronVolume, cofactors[0]) > 0 && CompareSigns(signedTetrahedronVolume, cofactors[1]) > 0 && CompareSigns(signedTetrahedronVolume, cofactors[2]) > 0 && CompareSigns(signedTetrahedronVolume, cofactors[3]) > 0) {
		Vec4 lambdas = cofactors * (1.0f / signedTetrahedronVolume);
		return lambdas;
	}

	// If we get here, then we need to project the origin onto the faces and determine the closest one
	Vec4 lambdas;
	float closestDistance = 1e10;
	Vec3 tetrahedronVertices[4];
	for (int currentVertex = 0; currentVertex < 4; ++currentVertex) {
		int vertexA = (currentVertex + 1) % 4;
		int vertexB = (currentVertex + 2) % 4;

		tetrahedronVertices[0] = s1;
		tetrahedronVertices[1] = s2;
		tetrahedronVertices[2] = s3;
		tetrahedronVertices[3] = s4;

		Vec3 lambdasFace = GetBarycentricCoordinatesFromTriangleToOrigin(tetrahedronVertices[currentVertex], tetrahedronVertices[vertexA], tetrahedronVertices[vertexB]);
		Vec3 currentPoint = tetrahedronVertices[currentVertex] * lambdasFace[0] + tetrahedronVertices[vertexA] * lambdasFace[1] + tetrahedronVertices[vertexB] * lambdasFace[2];
		if (currentPoint.GetLengthSqr() < closestDistance) {
			closestDistance = currentPoint.GetLengthSqr();
			lambdas.Zero();
			lambdas[currentVertex] = lambdasFace[0];
			lambdas[vertexA] = lambdasFace[1];
			lambdas[vertexB] = lambdasFace[2];
		}
	}

	return lambdas;
}



/*
================================================================================================

Gilbert Johnson Keerthi

================================================================================================
*/

struct point_t {
	Vec3 xyz;	// The point on the minkowski sum
	Vec3 ptA;	// The point on bodyA
	Vec3 ptB;	// The point on bodyB

	point_t() : xyz(0.0f), ptA(0.0f), ptB(0.0f) {}

	const point_t& operator = (const point_t& rhs) {
		xyz = rhs.xyz;
		ptA = rhs.ptA;
		ptB = rhs.ptB;
		return *this;
	}

	bool operator == (const point_t& rhs) const {
		return ((ptA == rhs.ptA) && (ptB == rhs.ptB) && (xyz == rhs.xyz));
	}
};

/*
================================
GetSupportPoint
================================
*/
point_t GetSupportPoint(const Body* bodyA, const Body* bodyB, Vec3 dir, const float bias) {
	dir.Normalize();

	point_t point;

	// Find the point in A furthest in direction
	point.ptA = bodyA->m_shape->GetSupportPoint(dir, bodyA->m_position, bodyA->m_orientation, bias);

	dir *= -1.0f;

	// Find the point in B furthest in the opposite direction
	point.ptB = bodyB->m_shape->GetSupportPoint(dir, bodyB->m_position, bodyB->m_orientation, bias);

	// Return the point, in the minkowski sum, furthest in the direction
	point.xyz = point.ptA - point.ptB;
	return point;
}

/*
================================
SimplexSignedVolumes

Projects the origin onto the simplex to acquire the new search direction,
also checks if the origin is "inside" the simplex.
================================
*/
bool GetBarycentricCoordinatesToOrigin(point_t* points, const int totalNumberOfPoints, Vec3& newDir, Vec4& barycentricCoordinates) {
	const float epsilonf = 0.0001f * 0.0001f;
	barycentricCoordinates.Zero();

	bool doesIntersect = false;
	Vec3 closestPointOnSimplexToOrigin(0.0f);
	switch (totalNumberOfPoints) {
	default:
	case 2: {
		Vec2 lambdas = GetBarycentricCoordinatesFromLineToOrigin(points[0].xyz, points[1].xyz);
		for (int currentPoint = 0; currentPoint < 2; ++currentPoint)
			closestPointOnSimplexToOrigin += points[currentPoint].xyz * lambdas[currentPoint];

		newDir = closestPointOnSimplexToOrigin * -1.0f;
		doesIntersect = (closestPointOnSimplexToOrigin.GetLengthSqr() < epsilonf);
		barycentricCoordinates[0] = lambdas[0];
		barycentricCoordinates[1] = lambdas[1];
	} break;
	case 3: {
		Vec3 lambdas = GetBarycentricCoordinatesFromTriangleToOrigin(points[0].xyz, points[1].xyz, points[2].xyz);
		for (int currentPoint = 0; currentPoint < 3; ++currentPoint)
			closestPointOnSimplexToOrigin += points[currentPoint].xyz * lambdas[currentPoint];

		newDir = closestPointOnSimplexToOrigin * -1.0f;
		doesIntersect = (closestPointOnSimplexToOrigin.GetLengthSqr() < epsilonf);
		barycentricCoordinates[0] = lambdas[0];
		barycentricCoordinates[1] = lambdas[1];
		barycentricCoordinates[2] = lambdas[2];
	} break;
	case 4: {
		Vec4 lambdas = GetBarycentricCoordinatesFromTetrahedronToOrigin(points[0].xyz, points[1].xyz, points[2].xyz, points[3].xyz);
		for (int currentPoint = 0; currentPoint < 4; ++currentPoint)
			closestPointOnSimplexToOrigin += points[currentPoint].xyz * lambdas[currentPoint];

		newDir = closestPointOnSimplexToOrigin * -1.0f;
		doesIntersect = (closestPointOnSimplexToOrigin.GetLengthSqr() < epsilonf);
		barycentricCoordinates[0] = lambdas[0];
		barycentricCoordinates[1] = lambdas[1];
		barycentricCoordinates[2] = lambdas[2];
		barycentricCoordinates[3] = lambdas[3];
	} break;
	};

	return doesIntersect;
}

/*
================================
IsAlreadyAdded

Checks whether the new point already exists in the simplex
================================
*/
bool IsAlreadyAdded(const point_t simplexPoints[4], const point_t& pointToCheck) {
	const float precision = 1e-6f;

	for (int currentPoint = 0; currentPoint < 4; ++currentPoint) {
		Vec3 delta = simplexPoints[currentPoint].xyz - pointToCheck.xyz;
		if (delta.GetLengthSqr() < precision * precision)
			return true;
	}
	return false;
}

/*
================================
SortValidSupportPoints

Sorts the valid support points to the beginning of the array
================================
*/
void SortValidSupportPoints(point_t simplexPoints[4], Vec4& lambdas) {
	bool validFlags[4];
	for (int currentVertex = 0; currentVertex < 4; ++currentVertex) {
		validFlags[currentVertex] = true;
		if (lambdas[currentVertex] == 0.0f)
			validFlags[currentVertex] = false;
	}

	Vec4 validLambdas(0.0f);
	int validCount = 0;
	point_t validPoints[4];
	memset(validPoints, 0, sizeof(point_t) * 4);
	for (int currentVertex = 0; currentVertex < 4; ++currentVertex) {
		if (validFlags[currentVertex]) {
			validPoints[validCount] = simplexPoints[currentVertex];
			validLambdas[validCount] = lambdas[currentVertex];
			++validCount;
		}
	}

	// Copy the validPoints back into simplexPoints
	for (int currentVertex = 0; currentVertex < 4; ++currentVertex) {
		simplexPoints[currentVertex] = validPoints[currentVertex];
		lambdas[currentVertex] = validLambdas[currentVertex];
	}
}

/*
================================
GetNumberOfValidPoints
================================
*/
static int GetNumberOfValidPoints(const Vec4& lambdas) {
	int numberOfValidPoints = 0;
	for (int currentPoint = 0; currentPoint < 4; ++currentPoint) {
		if (0.0f != lambdas[currentPoint])
			++numberOfValidPoints;
	}
	return numberOfValidPoints;
}

/*
================================
DoesIntersect_GJK
================================
*/
bool DoesIntersect_GJK(const Body* bodyA, const Body* bodyB) {
	const Vec3 ORIGIN(0.0f);

	int numberOfTotalPoints = 1;
	point_t simplexPoints[4];
	simplexPoints[0] = GetSupportPoint(bodyA, bodyB, Vec3(1, 1, 1), 0.0f);

	float currentClosestDistance = 1e10f;
	bool doesContainOrigin = false;
	Vec3 newDir = simplexPoints[0].xyz * -1.0f;
	do {
		// Get the new point to check on
		point_t newPoint = GetSupportPoint(bodyA, bodyB, newDir, 0.0f);

		// If the new point is the same as a previous point, then we can't expand any further
		if (IsAlreadyAdded(simplexPoints, newPoint)) {
			break;
		}

		simplexPoints[numberOfTotalPoints] = newPoint;
		++numberOfTotalPoints;

		// If this new point hasn't moved passed the origin, then the
		// origin cannot be in the set. And therefore there is no collision.
		float dotResult = newDir.Dot(newPoint.xyz - ORIGIN);
		if (dotResult < 0.0f)
			break;

		Vec4 lambdas;
		doesContainOrigin = GetBarycentricCoordinatesToOrigin(simplexPoints, numberOfTotalPoints, newDir, lambdas);
		if (doesContainOrigin)
			break;

		// Check that the new projection of the origin onto the simplex is closer than the previous
		float currentDistance = newDir.GetLengthSqr();
		if (currentDistance >= currentClosestDistance)
			break;
		currentClosestDistance = currentDistance;

		// Use the lambdas that support the new search direction, and invalidate any points that don't support it
		SortValidSupportPoints(simplexPoints, lambdas);
		numberOfTotalPoints = GetNumberOfValidPoints(lambdas);
		doesContainOrigin = (4 == numberOfTotalPoints);
	} while (!doesContainOrigin);

	return doesContainOrigin;
}

/*
================================
DoesIntersect_GJK
================================
*/
bool DoesIntersect_GJK(const Body* bodyA, const Body* bodyB, const float bias, Vec3& contactPointOnA, Vec3& contactPointOnB) {
	const Vec3 ORIGIN(0.0f);

	int numberOfTotalPoints = 1;
	point_t simplexPoints[4];
	simplexPoints[0] = GetSupportPoint(bodyA, bodyB, Vec3(1, 1, 1), 0.0f);

	float currentClosestDistance = 1e10f;
	bool doesContainOrigin = false;
	Vec3 newDir = simplexPoints[0].xyz * -1.0f;

	// 
	// TODO: temporary fix (remove this logic after applying optimization)
	unsigned currentIterationCount = 0;
	const unsigned MAX_ITERATION_COUNT = 9;
	do {
		if (currentIterationCount > MAX_ITERATION_COUNT)
			break;

		// Get the new point to check on
		point_t newPoint = GetSupportPoint(bodyA, bodyB, newDir, 0.0f);

		// If the new point is the same as a previous point, then we can't expand any further
		if (IsAlreadyAdded(simplexPoints, newPoint))
			break;

		simplexPoints[numberOfTotalPoints] = newPoint;
		++numberOfTotalPoints;

		// If this new point hasn't moved passed the origin, then the
		// origin cannot be in the set. And therefore there is no collision.
		float dotResult = newDir.Dot(newPoint.xyz - ORIGIN);
		if (dotResult < 0.0f)
			break;

		Vec4 lambdas;
		doesContainOrigin = GetBarycentricCoordinatesToOrigin(simplexPoints, numberOfTotalPoints, newDir, lambdas);
		if (doesContainOrigin)
			break;

		// Check that the new projection of the origin onto the simplex is closer than the previous
		float currentDistance = newDir.GetLengthSqr();
		if (currentDistance >= currentClosestDistance)
			break;
		currentClosestDistance = currentDistance;

		// Use the lambdas that support the new search direction, and invalidate any points that don't support it
		SortValidSupportPoints(simplexPoints, lambdas);
		numberOfTotalPoints = GetNumberOfValidPoints(lambdas);
		doesContainOrigin = (4 == numberOfTotalPoints);

		++currentIterationCount;
	} while (!doesContainOrigin);

	// Exit if there's no collision
	if (!doesContainOrigin) {
		return false;
	}

	//
	//	Check that we have a 3-simplex (EPA expects a tetrahedron)
	//
	if (1 == numberOfTotalPoints) {
		Vec3 searchDir = simplexPoints[0].xyz * -1.0f;
		point_t newPoint = GetSupportPoint(bodyA, bodyB, searchDir, 0.0f);
		simplexPoints[numberOfTotalPoints] = newPoint;
		++numberOfTotalPoints;
	}
	if (2 == numberOfTotalPoints) {
		Vec3 ab = simplexPoints[1].xyz - simplexPoints[0].xyz;
		Vec3 u, v;
		ab.GetOrtho(u, v);

		Vec3 newDir = u;
		point_t newPoint = GetSupportPoint(bodyA, bodyB, newDir, 0.0f);
		simplexPoints[numberOfTotalPoints] = newPoint;
		++numberOfTotalPoints;
	}
	if (3 == numberOfTotalPoints) {
		Vec3 ab = simplexPoints[1].xyz - simplexPoints[0].xyz;
		Vec3 ac = simplexPoints[2].xyz - simplexPoints[0].xyz;
		Vec3 norm = ab.Cross(ac);

		Vec3 newDir = norm;
		point_t newPoint = GetSupportPoint(bodyA, bodyB, newDir, 0.0f);
		simplexPoints[numberOfTotalPoints] = newPoint;
		++numberOfTotalPoints;
	}

	//
	// Expand the simplex by the bias amount
	//

	// Get the center point of the simplex
	Vec3 centerPoint = Vec3(0, 0, 0);
	for (int currentPoint = 0; currentPoint < 4; ++currentPoint)
		centerPoint += simplexPoints[currentPoint].xyz;
	centerPoint *= 0.25f;

	// Now expand the simplex by the bias amount
	for (int currentPoint = 0; currentPoint < numberOfTotalPoints; ++currentPoint) {
		point_t& currentVertex = simplexPoints[currentPoint];

		Vec3 dir = currentVertex.xyz - centerPoint;	// ray from "center" to witness point
		dir.Normalize();
		currentVertex.ptA += dir * bias;
		currentVertex.ptB -= dir * bias;
		currentVertex.xyz = currentVertex.ptA - currentVertex.ptB;
	}

	//
	// Perform EPA expansion of the simplex to find the closest face on the CSO
	//
	Expand_EPA(bodyA, bodyB, bias, simplexPoints, contactPointOnA, contactPointOnB);
	return true;
}

/*
================================
FindClosestPoints_GJK
================================
*/
void FindClosestPoints_GJK(const Body* bodyA, const Body* bodyB, Vec3& pointOnA, Vec3& pointOnB) {
	float currentClosestDistance = 1e10f;
	const float bias = 0.0f;

	int numberOfTotalPoints = 1;
	point_t simplexPoints[4];
	simplexPoints[0] = GetSupportPoint(bodyA, bodyB, Vec3(1, 1, 1), bias);

	Vec4 lambdas = Vec4(1, 0, 0, 0);
	Vec3 newDir = simplexPoints[0].xyz * -1.0f;
	do {
		// Get the new point to check on
		point_t newPoint = GetSupportPoint(bodyA, bodyB, newDir, bias);

		// If the new point is the same as a previous point, then we can't expand any further
		if (IsAlreadyAdded(simplexPoints, newPoint))
			break;

		// Add point and get new search direction
		simplexPoints[numberOfTotalPoints] = newPoint;
		++numberOfTotalPoints;

		GetBarycentricCoordinatesToOrigin(simplexPoints, numberOfTotalPoints, newDir, lambdas);
		SortValidSupportPoints(simplexPoints, lambdas);
		numberOfTotalPoints = GetNumberOfValidPoints(lambdas);

		// Check that the new projection of the origin onto the simplex is closer than the previous
		float currentDistance = newDir.GetLengthSqr();
		if (currentDistance >= currentClosestDistance)
			break;
		currentClosestDistance = currentDistance;
	} while (numberOfTotalPoints < 4);

	pointOnA.Zero();
	pointOnB.Zero();
	for (int currentPoint = 0; currentPoint < 4; ++currentPoint) {
		pointOnA += simplexPoints[currentPoint].ptA * lambdas[currentPoint];
		pointOnB += simplexPoints[currentPoint].ptB * lambdas[currentPoint];
	}
}


/*
================================================================================================

Expanding Polytope Algorithm

================================================================================================
*/

/*
================================
GetBarycentricCoordinates

This borrows our signed volume code to perform the barycentric coordinates.
================================
*/
Vec3 GetBarycentricCoordinates(Vec3 s1, Vec3 s2, Vec3 s3, const Vec3& targetPoint) {
	s1 = s1 - targetPoint;
	s2 = s2 - targetPoint;
	s3 = s3 - targetPoint;

	Vec3 normal = (s2 - s1).Cross(s3 - s1);
	Vec3 projectedPA = normal * s1.Dot(normal) / normal.GetLengthSqr(); // project the vector from targetPoint to s1 onto the normal

	// Find the axis with the greatest projected area
	int targetAxis = 0;
	float maxArea = 0;
	for (int currentAxis = 0; currentAxis < 3; ++currentAxis) {
		int axisA = (currentAxis + 1) % 3;
		int axisB = (currentAxis + 2) % 3;

		Vec2 a = Vec2(s1[axisA], s1[axisB]);
		Vec2 b = Vec2(s2[axisA], s2[axisB]);
		Vec2 c = Vec2(s3[axisA], s3[axisB]);
		Vec2 ab = b - a;
		Vec2 ac = c - a;

		float currentArea = ab.x * ac.y - ab.y * ac.x;
		if (currentArea * currentArea > maxArea * maxArea) {
			targetAxis = currentAxis;
			maxArea = currentArea;
		}
	}

	// Project onto the appropriate axis
	int axisA = (targetAxis + 1) % 3;
	int axisB = (targetAxis + 2) % 3;
	Vec2 projectedTriangle[3];
	projectedTriangle[0] = Vec2(s1[axisA], s1[axisB]);
	projectedTriangle[1] = Vec2(s2[axisA], s2[axisB]);
	projectedTriangle[2] = Vec2(s3[axisA], s3[axisB]);
	Vec2 projectedTargetVertex = Vec2(projectedPA[axisA], projectedPA[axisB]);

	// Get the sub-areas of the triangles formed from the projected origin and the edges
	Vec3 areas;
	for (int currentVertex = 0; currentVertex < 3; ++currentVertex) {
		int vertexA = (currentVertex + 1) % 3;
		int vertexB = (currentVertex + 2) % 3;

		Vec2 a = projectedTargetVertex;
		Vec2 b = projectedTriangle[vertexA];
		Vec2 c = projectedTriangle[vertexB];
		Vec2 ab = b - a;
		Vec2 ac = c - a;

		areas[currentVertex] = ab.x * ac.y - ab.y * ac.x;
	}

	Vec3 lambdas = areas / maxArea;
	if (!lambdas.IsValid())
		lambdas = Vec3(1, 0, 0);
	return lambdas;
}

/*
================================
GetNormalOfTriangle
================================
*/
Vec3 GetNormalOfTriangle(const tri_t& targetTriangle, const std::vector< point_t >& points) {
	const Vec3& a = points[targetTriangle.a].xyz;
	const Vec3& b = points[targetTriangle.b].xyz;
	const Vec3& c = points[targetTriangle.c].xyz;

	Vec3 ab = b - a;
	Vec3 ac = c - a;
	Vec3 normal = ab.Cross(ac);
	normal.Normalize();
	return normal;
}

/*
================================
GetSignedDistanceFromPointToTriangle
================================
*/
float GetSignedDistanceFromPointToTriangle(const tri_t& triangle, const Vec3& targetPoint, const std::vector< point_t >& points) {
	const Vec3 normal = GetNormalOfTriangle(triangle, points);
	const Vec3& vertexA = points[triangle.a].xyz;
	const Vec3 aToPoint = targetPoint - vertexA;
	const float distance = normal.Dot(aToPoint);
	return distance;
}

/*
================================
GetClosestTriangleToOrigin
================================
*/
int GetClosestTriangleToOrigin(const std::vector< tri_t >& triangles, const std::vector< point_t >& points) {
	float minDistanceSquared = 1e10;

	int targetTriangleIndex = -1;
	for (int currentTriangleIndex = 0; currentTriangleIndex < triangles.size(); ++currentTriangleIndex) {
		const tri_t& currentTriangle = triangles[currentTriangleIndex];

		float currentDistance = GetSignedDistanceFromPointToTriangle(currentTriangle, Vec3(0.0f), points);
		float currentDistanceSquared = currentDistance * currentDistance;
		if (currentDistanceSquared < minDistanceSquared) {
			targetTriangleIndex = currentTriangleIndex;
			minDistanceSquared = currentDistanceSquared;
		}
	}

	return targetTriangleIndex;
}

/*
================================
IsAlreadyAdded
================================
*/
bool IsAlreadyAdded(const Vec3& targetPoint, const std::vector< tri_t > triangles, const std::vector< point_t >& points) {
	const float epsilons = 0.001f * 0.001f;
	Vec3 delta;

	for (int currentTriangleIndex = 0; currentTriangleIndex < triangles.size(); ++currentTriangleIndex) {
		const tri_t& currentTriangle = triangles[currentTriangleIndex];

		delta = targetPoint - points[currentTriangle.a].xyz;
		if (delta.GetLengthSqr() < epsilons)
			return true;
		delta = targetPoint - points[currentTriangle.b].xyz;
		if (delta.GetLengthSqr() < epsilons)
			return true;
		delta = targetPoint - points[currentTriangle.c].xyz;
		if (delta.GetLengthSqr() < epsilons)
			return true;
	}
	return false;
}

/*
================================
RemoveTrianglesFacingPoint
================================
*/
int RemoveTrianglesFacingPoint(const Vec3& targetPoint, std::vector< tri_t >& triangles, const std::vector< point_t >& points) {
	int numOfRemovedTriangles = 0;
	for (int currentTriangleIndex = 0; currentTriangleIndex < triangles.size(); ++currentTriangleIndex) {
		const tri_t& currentTriangle = triangles[currentTriangleIndex];

		float distance = GetSignedDistanceFromPointToTriangle(currentTriangle, targetPoint, points);
		if (distance > 0.0f) {
			// This triangle faces the point.  Remove it.
			triangles.erase(triangles.begin() + currentTriangleIndex);
			--currentTriangleIndex;
			numOfRemovedTriangles++;
		}
	}
	return numOfRemovedTriangles;
}

/*
================================
FindDanglingEdges
================================
*/
void FindDanglingEdges(std::vector< edge_t >& danglingEdges, const std::vector< tri_t >& triangles) {
	danglingEdges.clear();

	for (int currentTriangleIndex = 0; currentTriangleIndex < triangles.size(); ++currentTriangleIndex) {
		const tri_t& currentTriangle = triangles[currentTriangleIndex];

		edge_t edges[3];
		edges[0].a = currentTriangle.a;
		edges[0].b = currentTriangle.b;

		edges[1].a = currentTriangle.b;
		edges[1].b = currentTriangle.c;

		edges[2].a = currentTriangle.c;
		edges[2].b = currentTriangle.a;

		int counts[3];
		counts[0] = 0;
		counts[1] = 0;
		counts[2] = 0;

		for (int currentNestedIndex = 0; currentNestedIndex < triangles.size(); ++currentNestedIndex) {
			if (currentNestedIndex == currentTriangleIndex)
				continue;

			const tri_t& currentNestedTriangle = triangles[currentNestedIndex];

			edge_t edgesNested[3];
			edgesNested[0].a = currentNestedTriangle.a;
			edgesNested[0].b = currentNestedTriangle.b;

			edgesNested[1].a = currentNestedTriangle.b;
			edgesNested[1].b = currentNestedTriangle.c;

			edgesNested[2].a = currentNestedTriangle.c;
			edgesNested[2].b = currentNestedTriangle.a;

			for (int currentNestedNestedIndex = 0; currentNestedNestedIndex < 3; ++currentNestedNestedIndex) {
				if (edges[currentNestedNestedIndex] == edgesNested[0]) 
					counts[currentNestedNestedIndex]++;

				if (edges[currentNestedNestedIndex] == edgesNested[1]) 
					counts[currentNestedNestedIndex]++;

				if (edges[currentNestedNestedIndex] == edgesNested[2]) 
					counts[currentNestedNestedIndex]++;
			}
		}

		// An edge that isn't shared, is dangling 
		for (int currentNestedNestedIndex = 0; currentNestedNestedIndex < 3; ++currentNestedNestedIndex) {
			if (0 == counts[currentNestedNestedIndex]) 
				danglingEdges.push_back(edges[currentNestedNestedIndex]);
		}
	}
}

/*
================================
Expand_EPA
================================
*/
float Expand_EPA(const Body* bodyA, const Body* bodyB, const float bias, const point_t simplexPoints[4], Vec3& pointOnA, Vec3& pointOnB) {
	std::vector< point_t > points;
	std::vector< tri_t > triangles;
	std::vector< edge_t > danglingEdges;

	Vec3 centerPoint(0.0f);
	for (int currentVertex = 0; currentVertex < 4; ++currentVertex) {
		points.push_back(simplexPoints[currentVertex]);
		centerPoint += simplexPoints[currentVertex].xyz;
	}
	centerPoint *= 0.25f;

	// Build the triangles
	for (int currentVertex = 0; currentVertex < 4; ++currentVertex) {
		int vertexA = (currentVertex + 1) % 4;
		int vertexB = (currentVertex + 2) % 4;
		tri_t currentTriangle;
		currentTriangle.a = currentVertex;
		currentTriangle.b = vertexA;
		currentTriangle.c = vertexB;

		int unusedPoint = (currentVertex + 3) % 4;
		float distance = GetSignedDistanceFromPointToTriangle(currentTriangle, points[unusedPoint].xyz, points);

		// The unused point is always on the negative/inside of the triangle.. make sure the normal points away
		if (distance > 0.0f) 
			std::swap(currentTriangle.a, currentTriangle.b);

		triangles.push_back(currentTriangle);
	}

	//
	//	Expand the simplex to find the closest face of the CSO to the origin
	//
	while (true) {
		const int closestTriangleIndex = GetClosestTriangleToOrigin(triangles, points);
		Vec3 normal = GetNormalOfTriangle(triangles[closestTriangleIndex], points);
		const point_t newSupportPoint = GetSupportPoint(bodyA, bodyB, normal, bias);

		// if w already exists, then just stop
		// because it means we can't expand any further
		if (IsAlreadyAdded(newSupportPoint.xyz, triangles, points)) 
			break;

		float distance = GetSignedDistanceFromPointToTriangle(triangles[closestTriangleIndex], newSupportPoint.xyz, points);
		if (distance <= 0.0f) 
			break;	// can't expand

		const int newSupportPointIndex = (int)points.size();
		points.push_back(newSupportPoint);

		// Remove Triangles that face this point
		int numOfRemovedTriangles = RemoveTrianglesFacingPoint(newSupportPoint.xyz, triangles, points);
		if (0 == numOfRemovedTriangles) 
			break;

		// Find Dangling Edges
		danglingEdges.clear();
		FindDanglingEdges(danglingEdges, triangles);
		if (0 == danglingEdges.size())
			break;


		// In theory the edges should be a proper CCW order
		// So we only need to add the new point as 'a' in order
		// to create new triangles that face away from origin
		for (int currentEdgeIndex = 0; currentEdgeIndex < danglingEdges.size(); ++currentEdgeIndex) {
			const edge_t& currentEdge = danglingEdges[currentEdgeIndex];

			tri_t currentTriangle;
			currentTriangle.a = newSupportPointIndex;
			currentTriangle.b = currentEdge.b;
			currentTriangle.c = currentEdge.a;

			// Make sure it's oriented properly
			float distance = GetSignedDistanceFromPointToTriangle(currentTriangle, centerPoint, points);
			if (distance > 0.0f) 
				std::swap(currentTriangle.b, currentTriangle.c);

			triangles.push_back(currentTriangle);
		}
	}

	// Get the projection of the origin on the closest triangle
	const int closestTriangleIndex = GetClosestTriangleToOrigin(triangles, points);
	const tri_t& closestTriangle = triangles[closestTriangleIndex];
	Vec3 triangleVertexA = points[closestTriangle.a].xyz;
	Vec3 triangleVertexB = points[closestTriangle.b].xyz;
	Vec3 triangleVertexC = points[closestTriangle.c].xyz;
	Vec3 lambdas = GetBarycentricCoordinates(triangleVertexA, triangleVertexB, triangleVertexC, Vec3(0.0f));

	// Get the point on shape A
	Vec3 shapeA_VertexA = points[closestTriangle.a].ptA;
	Vec3 shapeA_VertexB = points[closestTriangle.b].ptA;
	Vec3 shapeA_VertexC = points[closestTriangle.c].ptA;
	pointOnA = shapeA_VertexA * lambdas[0] + shapeA_VertexB * lambdas[1] + shapeA_VertexC * lambdas[2];

	// Get the point on shape B
	Vec3 shapeB_VertexA = points[closestTriangle.a].ptB;
	Vec3 shapeB_VertexB = points[closestTriangle.b].ptB;
	Vec3 shapeB_VertexC = points[closestTriangle.c].ptB;
	pointOnB = shapeB_VertexA * lambdas[0] + shapeB_VertexB * lambdas[1] + shapeB_VertexC * lambdas[2];

	// Return the penetration distance
	Vec3 delta = pointOnB - pointOnA;
	return delta.GetMagnitude();
}
