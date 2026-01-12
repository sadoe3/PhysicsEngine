//
//	Body.h
//
#pragma once

/*
====================================================
Body
====================================================
*/
class Body {
public:
	Body();

	Vec3		m_position;
	Quat		m_orientation;
	Vec3		m_linearVelocity;
	Vec3		m_angularVelocity;

	float		m_invMass = 0.0f;
	float		m_elasticity;
	float 		m_friction;
	Shape*		m_shape;
	
	//TODO
	std::string m_geometryName;
	std::string m_objectName;
	std::string m_materialName;
	unsigned	m_id;


	Vec3 GetCenterOfMassWorldSpace() const;
	Vec3 GetCenterOfMassModelSpace() const;
	Vec3 WorldSpaceToBodySpace(const Vec3& pt) const;
	Vec3 BodySpaceToWorldSpace(const Vec3& pt) const;

	Mat3 GetInverseInertiaTensorBodySpace() const;
	Mat3 GetInverseInertiaTensorWorldSpace() const;

	void ApplyImpulse(const Vec3& impulsePoint, const Vec3& linearImpulse);
	void ApplyImpulseLinear(const Vec3& linearImpulse);
	void ApplyImpulseAngular(const Vec3& angularImpulse);

	void Update(const float deltaSecond);
};
