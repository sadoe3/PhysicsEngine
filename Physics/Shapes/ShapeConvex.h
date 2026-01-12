//
//	ShapeConvex.h
//
#pragma once
#include "ShapeBase.h"


class Random {
public:
	static float Get() { return m_distribution(m_engine); }
private:
	static std::mt19937 m_engine;
	static std::uniform_real_distribution<float> m_distribution;
};


struct tri_t {
	int a;
	int b;
	int c;
};

struct edge_t {
	int a;
	int b;

	bool operator == ( const edge_t & rhs ) const {
		return ( ( a == rhs.a && b == rhs.b ) || ( a == rhs.b && b == rhs.a ) );
	}
};



void BuildConvexHull( const std::vector< Vec3 > & verts, std::vector< Vec3 > & hullPts, std::vector< tri_t > & hullTris );

/*
====================================================
ShapeConvex
====================================================
*/
class ShapeConvex : public Shape {
public:
	explicit ShapeConvex(const Vec3* pts, const int num) {
		Build(pts, num);
	}
	void Build(const Vec3* pts, const int num) override;

	Vec3 GetSupportPoint(const Vec3& dir, const Vec3& pos, const Quat& orient, const float bias) const override;

	Mat3 GetInertiaTensor() const override { return m_inertiaTensor; }

	Bounds GetBounds(const Vec3& pos, const Quat& orient) const override;
	Bounds GetBounds() const override { return m_bounds; }

	float GetFastestLinearSpeed(const Vec3& angularVelocity, const Vec3& dir) const override;

	shapeType_t GetType() const override { return SHAPE_CONVEX; }

public:
	std::vector< Vec3 > m_points;
	// todo
	std::vector< tri_t >m_triangles;
	Bounds m_bounds;
	Mat3 m_inertiaTensor;
};
