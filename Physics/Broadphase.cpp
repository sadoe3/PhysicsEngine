//
//  Broadphase.cpp
//
#include "PCH.h"
#include "Broadphase.h"


int CompareSAP(const void* lhs, const void* rhs) {
	const psuedoBody_t* left = reinterpret_cast<const psuedoBody_t*>(lhs);
	const psuedoBody_t* right = reinterpret_cast<const psuedoBody_t*>(rhs);

	if (left->value < right->value)
		return -1;
	return 1;
}
void SortBodiesBounds(const Body* bodies, const int numBodies, psuedoBody_t* sortedArray, const float deltaSecond) {

    // --- 1. Dynamic Axis Selection for Optimal Sweep-and-Prune ---
    float minX = FLT_MAX, minY = FLT_MAX, minZ = FLT_MAX;
    float maxX = -FLT_MAX, maxY = -FLT_MAX, maxZ = -FLT_MAX;

    // First pass to determine the global bounds and select the axis with the largest extent.
    for (int i = 0; i < numBodies; ++i) {
        const Body& body = bodies[i];
        Bounds bounds = body.m_shape->GetBounds(body.m_position, body.m_orientation);

        // Expand bounds for continuous collision detection (CCD)
        bounds.Expand(bounds.mins + body.m_linearVelocity * deltaSecond);
        bounds.Expand(bounds.maxs + body.m_linearVelocity * deltaSecond);

        // Optional epsilon expansion to prevent issues at boundaries
        const float epsilon = 0.01f;
        bounds.Expand(bounds.mins + Vec3(-1, -1, -1) * epsilon);
        bounds.Expand(bounds.maxs + Vec3(1, 1, 1) * epsilon);

        // Update global bounds
        minX = std::min(minX, bounds.mins.x); maxX = std::max(maxX, bounds.maxs.x);
        minY = std::min(minY, bounds.mins.y); maxY = std::max(maxY, bounds.maxs.y);
        minZ = std::min(minZ, bounds.mins.z); maxZ = std::max(maxZ, bounds.maxs.z);
    }

    // Select the axis with the largest extent (best for minimizing overlaps)
    float extentX = maxX - minX;
    float extentY = maxY - minY;
    float extentZ = maxZ - minZ;

    Vec3 axis;
    if (extentX >= extentY && extentX >= extentZ) {
        axis = Vec3(1, 0, 0); // X axis chosen
    }
    else if (extentY >= extentX && extentY >= extentZ) {
        axis = Vec3(0, 1, 0); // Y axis chosen
    }
    else {
        axis = Vec3(0, 0, 1); // Z axis chosen
    }

    // --- 2. Project Bounds onto the Chosen Axis ---
    // Note: If Vec3(1, 1, 1) was intentional for a custom non-axis aligned sweep, 
    // the axis selection logic should be removed, but this is less common for SAP.

    for (int currentBodyIndex = 0; currentBodyIndex < numBodies; ++currentBodyIndex) {
        const Body& body = bodies[currentBodyIndex];
        // Must recalculate bounds if they were optimized away in the first loop
        Bounds bounds = body.m_shape->GetBounds(body.m_position, body.m_orientation);

        // Expand bounds for continuous collision detection (CCD)
        bounds.Expand(bounds.mins + body.m_linearVelocity * deltaSecond);
        bounds.Expand(bounds.maxs + body.m_linearVelocity * deltaSecond);

        const float epsilon = 0.01f;
        bounds.Expand(bounds.mins + Vec3(-1, -1, -1) * epsilon);
        bounds.Expand(bounds.maxs + Vec3(1, 1, 1) * epsilon);

        // Project min endpoint
        sortedArray[currentBodyIndex * 2 + 0].id = currentBodyIndex;
        sortedArray[currentBodyIndex * 2 + 0].value = axis.Dot(bounds.mins);
        sortedArray[currentBodyIndex * 2 + 0].isMin = true;

        // Project max endpoint
        sortedArray[currentBodyIndex * 2 + 1].id = currentBodyIndex;
        sortedArray[currentBodyIndex * 2 + 1].value = axis.Dot(bounds.maxs);
        sortedArray[currentBodyIndex * 2 + 1].isMin = false;
    }

    // 3. Sort the endpoints array. Assuming CompareSAP handles float comparison correctly.
    qsort(sortedArray, numBodies * 2, sizeof(psuedoBody_t), CompareSAP);
}
void BuildPairs(std::vector<collisionPair_t>& collisionPairs, const psuedoBody_t* sortedBodies, const int numBodies) {
    collisionPairs.clear();

    // The Active List stores the ID of bodies currently overlapping along the sweep axis.
    std::vector<int> activeList;

    const int doubleNumBodies = numBodies * 2;
    for (int targetBodyIndex = 0; targetBodyIndex < doubleNumBodies; ++targetBodyIndex) {
        const psuedoBody_t& targetBody = sortedBodies[targetBodyIndex];
        int bodyId = targetBody.id;

        if (targetBody.isMin) {
            // Min Endpoint: The body enters the sweep axis.

            // Generate pairs with all bodies currently in the Active List (i.e., overlapping bodies).
            for (int activeId : activeList) {
                collisionPair_t pair;
                // Ensure pair.a < pair.b for canonical pairing (optional but good practice)
                pair.a = std::min(bodyId, activeId);
                pair.b = std::max(bodyId, activeId);
                collisionPairs.push_back(pair);
            }

            // Add the entering body to the Active List.
            activeList.push_back(bodyId);

        }
        else {
            // Max Endpoint: The body exits the sweep axis.

            // Remove the exiting body from the Active List.
            // Using std::remove_if + erase idiom for safety and correctness.
            auto it = std::remove_if(activeList.begin(), activeList.end(),
                [bodyId](int id) { return id == bodyId; });
            activeList.erase(it, activeList.end());
        }
    }
}
void SweepAndPrune1D(const Body* bodies, const int numBodies, std::vector<collisionPair_t>& finalPairs, const float deltaSecond) {
    // reserve space on the stack for 2 endpoints per body.
    std::vector<psuedoBody_t> sortedBodies(numBodies * 2);

    SortBodiesBounds(bodies, numBodies, sortedBodies.data(), deltaSecond);

    // The core issue was here: BuildPairs was doing an O(N^2) scan instead of using the Active List.
    BuildPairs(finalPairs, sortedBodies.data(), numBodies);
}


/*
====================================================
BroadPhase
====================================================
*/
void BroadPhase( const Body * bodies, const int num, std::vector< collisionPair_t > & finalPairs, const float deltaSecond ) {
	finalPairs.clear();
	SweepAndPrune1D(bodies, num, finalPairs, deltaSecond);
}
