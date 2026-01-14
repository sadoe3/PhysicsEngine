#include "PCH.h"
#include "SceneConfiguration.h"
#include "../Renderer/GeneralConstants.h"
#include "../Physics/Constraints.h"

int AddSpheres(std::vector< Body >& bodies, std::vector<std::pair<unsigned int, unsigned int>>& indices, const int startIndex
	, const int stressLevel, const float startHeight, bool isDense) {
	Body body;
	int lastIndex = startIndex;

	float positionOffset = 2.0f;
	if (isDense == false)
		positionOffset = 30.0f;
	for (int x = 1; x <= stressLevel; ++x) {
		for (int y = 1; y <= stressLevel; ++y) {
			float radius = 0.5f;
			float xx = float(x - (stressLevel/2.0f)) * radius * positionOffset;
			float yy = float(y - (stressLevel/2.0f)) * radius * positionOffset;
			body.m_position = Vec3(xx, yy, startHeight);
			body.m_orientation = Quat(0, 0, 0, 1);
			body.m_linearVelocity.Zero();
			body.m_angularVelocity.Zero();
			body.m_invMass = 1.0f;
			body.m_elasticity = 0.5f;
			body.m_friction = 0.5f;
			body.m_shape = new ShapeSphere(radius);
			body.m_geometryName = GeneralData::Geometry::RenderObjectNames::Spheres;
			body.m_objectName = std::to_string(lastIndex);
			body.m_materialName = GeneralData::Material::TextureNames::Brick;
			body.m_id = lastIndex;
			bodies.push_back(body);
			++lastIndex;
		}
	}

	indices.emplace_back(startIndex, lastIndex);

	return lastIndex;
}
int AddDiamonds(std::vector< Body >& bodies, std::vector<std::pair<unsigned int, unsigned int>>& indices, const int startIndex
	, const int stressLevel, const float startHeight, bool isDense) {
	Body body;
	int lastIndex = startIndex;

	float positionOffset = 2.0f;
	if (isDense == false)
		positionOffset = 15.0f;
	for (int x = 1; x <= stressLevel; ++x) {
		for (int y = 1; y <= stressLevel; ++y) {
			float gap = 1.0f;
			float xx = float(x - (stressLevel / 2.0f)) * gap * positionOffset;
			float yy = float(y - (stressLevel / 2.0f)) * gap * positionOffset;
			body.m_position = Vec3(xx, yy, startHeight);
			body.m_orientation = Quat(0, 0, 0, 1);
			body.m_linearVelocity.Zero();
			body.m_angularVelocity.Zero();
			body.m_invMass = 1.0f;
			body.m_elasticity = 0.5f;
			body.m_friction = 0.5f;
			body.m_shape = new ShapeConvex(g_diamond, sizeof(g_diamond) / sizeof(Vec3));
			body.m_geometryName = GeneralData::Geometry::RenderObjectNames::Diamonds;
			body.m_objectName = std::to_string(lastIndex);
			body.m_materialName = GeneralData::Material::TextureNames::Brick;
			body.m_id = lastIndex;
			bodies.push_back(body);
			++lastIndex;
		}
	}

	indices.emplace_back(startIndex, lastIndex);

	return lastIndex;
}
int AddFloor(std::vector< Body >& bodies, std::vector<std::pair<unsigned int, unsigned int>>& indices, const int startIndex, bool isDense) {
	Body body;
	int lastIndex = startIndex;

	float radius = 80.0f;
	if (isDense == false)
		radius *= 2;
	for (unsigned x = 0; x < 3; ++x) {
		for (unsigned y = 0; y < 3; ++y) {
			float xx = float(x - 1) * radius * 0.25f;
			float yy = float(y - 1) * radius * 0.25f;
			body.m_position = Vec3(xx - 5.0f, yy - 5.0f, -radius);
			body.m_orientation = Quat(0, 0, 0, 1);
			body.m_linearVelocity.Zero();
			body.m_angularVelocity.Zero();
			body.m_invMass = 0.0f;
			body.m_elasticity = 0.99f;
			body.m_friction = 0.5f;
			body.m_shape = new ShapeSphere(radius);

			body.m_geometryName = GeneralData::Geometry::RenderObjectNames::Floor;
			body.m_objectName = std::to_string(lastIndex);
			body.m_materialName = GeneralData::Material::TextureNames::Brick;
			body.m_id = lastIndex;
			bodies.push_back(body);
			++lastIndex;
		}
	}

	indices.emplace_back(startIndex, lastIndex);

	return lastIndex;
}


int AddStack(std::vector< Body >& bodies, std::vector<std::pair<unsigned int, unsigned int>>& indices, const int startIndex) {
	Body body;
	std::string geometryName = GeneralData::Geometry::RenderObjectNames::Stack;
	int lastIndex = startIndex;

	const int stackHeight = 5;
	for (int x = 0; x < 1; x++) {
		for (int y = 0; y < 1; y++) {
			for (int z = 0; z < stackHeight; z++) {
				float offset = ((z & 1) == 0) ? 0.0f : 0.15f;
				float xx = (float)x + offset;
				float yy = (float)y + offset;
				float delta = 0.04f;
				float scaleHeight = 2.0f + delta;
				float deltaHeight = 1.0f + delta;
				body.m_position = Vec3((float)xx * scaleHeight, (float)yy * scaleHeight, deltaHeight + (float)z * scaleHeight);
				body.m_orientation = Quat(0, 0, 0, 1);
				body.m_shape = new ShapeBox(g_boxUnit, sizeof(g_boxUnit) / sizeof(Vec3));
				body.m_invMass = 1.0f;
				body.m_elasticity = 0.5f;
				body.m_friction = 0.5f;

				body.m_geometryName = geometryName;
				switch (z) {
				case 0: body.m_objectName = GeneralData::Geometry::RenderItemNames::Stack::BoxA; break;
				case 1: body.m_objectName = GeneralData::Geometry::RenderItemNames::Stack::BoxB; break;
				case 2: body.m_objectName = GeneralData::Geometry::RenderItemNames::Stack::BoxC; break;
				case 3: body.m_objectName = GeneralData::Geometry::RenderItemNames::Stack::BoxD; break;
				case 4: body.m_objectName = GeneralData::Geometry::RenderItemNames::Stack::BoxE; break;
				}
				body.m_materialName = GeneralData::Material::TextureNames::Brick;
				body.m_id = lastIndex;
				bodies.push_back(body);
				++lastIndex;
			}
		}
	}

	indices.emplace_back(startIndex, lastIndex);

	return lastIndex;

}
int AddMover(std::vector< Body >& bodies, std::vector<Constraint*>& constraints
	, std::vector<std::pair<unsigned int, unsigned int>>& indices, const int startIndex) {
	Body body;
	std::string geometryName = GeneralData::Geometry::RenderObjectNames::Mover;
	int lastIndex = startIndex;

	body.m_position = Vec3(10, 0, 5);
	body.m_linearVelocity = Vec3(0, 0, 0);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_shape = new ShapeBox(g_boxPlatform, sizeof(g_boxPlatform) / sizeof(Vec3));
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.1f;
	body.m_friction = 0.9f;
	body.m_geometryName = geometryName;
	body.m_objectName = GeneralData::Geometry::RenderItemNames::Mover::Platform;
	body.m_materialName = GeneralData::Material::TextureNames::Brick;
	body.m_id = lastIndex;
	bodies.push_back(body);
	++lastIndex;
	{
		ConstraintMoverSimple* mover = new ConstraintMoverSimple();
		mover->m_bodyA = &bodies[bodies.size() - 1];
		mover->m_bodyB = &bodies[bodies.size() - 1];

		constraints.push_back(mover);
	}

	body.m_position = Vec3(10, 0, 6.3f);
	body.m_linearVelocity = Vec3(0, 0, 0);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_shape = new ShapeBox(g_boxUnit, sizeof(g_boxUnit) / sizeof(Vec3));
	body.m_invMass = 1.0f;
	body.m_elasticity = 0.1f;
	body.m_friction = 0.9f;
	body.m_geometryName = geometryName;
	body.m_objectName = GeneralData::Geometry::RenderItemNames::Mover::Unit;
	body.m_materialName = GeneralData::Material::TextureNames::Brick;
	body.m_id = lastIndex;
	bodies.push_back(body);
	++lastIndex;


	indices.emplace_back(startIndex, lastIndex);

	return lastIndex;
}

int AddChain(std::vector< Body >& bodies, std::vector<Constraint*>& constraints
	, std::vector<std::pair<unsigned int, unsigned int>>& indices, const int startIndex) {
	Body body;
	std::string geometryName = GeneralData::Geometry::RenderObjectNames::Chain;
	int lastIndex = startIndex;

	const int numJoints = 5;
	for (int i = 0; i < numJoints; i++) {
		if (i == 0) {
			body.m_position = Vec3(0.0f, 5.0f, (float)numJoints + 3.0f);
			body.m_orientation = Quat(0, 0, 0, 1);
			body.m_shape = new ShapeBox(g_boxSmall, sizeof(g_boxSmall) / sizeof(Vec3));
			body.m_invMass = 0.0f;
			body.m_elasticity = 1.0f;
			body.m_geometryName = geometryName;
			body.m_objectName = GeneralData::Geometry::RenderItemNames::Chain::TopBox;
			body.m_materialName = GeneralData::Material::TextureNames::Brick;
			body.m_id = lastIndex;
			bodies.push_back(body);
			++lastIndex;
		}

		body.m_linearVelocity = Vec3(0, 0, 0);

		Body* bodyA = &bodies[bodies.size() - 1];

		const Vec3 jointWorldSpaceAnchor = bodyA->m_position;
		const Vec3 jointWorldSpaceAxis = Vec3(0, 0, 1).Normalize();
		Mat3 jointWorldSpaceMatrix;
		jointWorldSpaceMatrix.rows[0] = jointWorldSpaceAxis;
		jointWorldSpaceMatrix.rows[0].GetOrtho(jointWorldSpaceMatrix.rows[1], jointWorldSpaceMatrix.rows[2]);
		jointWorldSpaceMatrix.rows[2] = jointWorldSpaceAxis;
		jointWorldSpaceMatrix.rows[2].GetOrtho(jointWorldSpaceMatrix.rows[0], jointWorldSpaceMatrix.rows[1]);
		Vec3 jointWorldSpaceAxisLimited = Vec3(0, 1, -1);
		jointWorldSpaceAxisLimited.Normalize();

		ConstraintDistance* joint = new ConstraintDistance();

		const float pi = acosf(-1.0f);

		joint->m_bodyA = &bodies[bodies.size() - 1];
		joint->m_anchorA = joint->m_bodyA->WorldSpaceToBodySpace(jointWorldSpaceAnchor);
		joint->m_axisA = joint->m_bodyA->m_orientation.Inverse().RotatePoint(jointWorldSpaceAxis);


		body.m_position = joint->m_bodyA->m_position - jointWorldSpaceAxis * 1.0f;
		body.m_position = joint->m_bodyA->m_position + Vec3(1, 0, 0);
		body.m_orientation = Quat(0, 0, 0, 1);
		body.m_shape = new ShapeBox(g_boxSmall, sizeof(g_boxSmall) / sizeof(Vec3));
		body.m_invMass = 1.0f;
		body.m_elasticity = 1.0f;
		body.m_geometryName = geometryName;
		switch (i) {
		case 0: body.m_objectName = GeneralData::Geometry::RenderItemNames::Chain::BoxA; break;
		case 1: body.m_objectName = GeneralData::Geometry::RenderItemNames::Chain::BoxB; break;
		case 2: body.m_objectName = GeneralData::Geometry::RenderItemNames::Chain::BoxC; break;
		case 3: body.m_objectName = GeneralData::Geometry::RenderItemNames::Chain::BoxD; break;
		case 4: body.m_objectName = GeneralData::Geometry::RenderItemNames::Chain::BoxE; break;
		}
		body.m_materialName = GeneralData::Material::TextureNames::Brick;
		body.m_id = lastIndex;
		bodies.push_back(body);
		++lastIndex;

		joint->m_bodyB = &bodies[bodies.size() - 1];
		joint->m_anchorB = joint->m_bodyB->WorldSpaceToBodySpace(jointWorldSpaceAnchor);
		joint->m_axisB = joint->m_bodyB->m_orientation.Inverse().RotatePoint(jointWorldSpaceAxis);

		constraints.push_back(joint);
	}
	indices.emplace_back(startIndex, lastIndex);

	return lastIndex;
}


int AddHinge(std::vector< Body >& bodies, std::vector<Constraint*>& constraints
	, std::vector<std::pair<unsigned int, unsigned int>>& indices, const int startIndex) {
	Body body;
	std::string geometryName = GeneralData::Geometry::RenderObjectNames::Hinge;
	int lastIndex = startIndex;

	const float pi = acosf(-1.0f);

	body.m_position = Vec3(-2, -5, 6);
	body.m_linearVelocity = Vec3(0.0f, 0.0f, 0.0f);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_orientation = Quat(Vec3(1, 1, 1), pi * 0.25f);
	body.m_shape = new ShapeBox(g_boxSmall, sizeof(g_boxSmall) / sizeof(Vec3));
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.9f;
	body.m_friction = 0.5f;
	body.m_geometryName = geometryName;
	body.m_objectName = GeneralData::Geometry::RenderItemNames::Hinge::BoxA;
	body.m_materialName = GeneralData::Material::TextureNames::Brick;
	body.m_id = lastIndex;
	bodies.push_back(body);
	++lastIndex;

	body.m_position = Vec3(-2, -5, 5);
	body.m_linearVelocity = Vec3(0.0f, 0.0f, 0.0f);
	body.m_orientation = Quat(Vec3(0, 1, 1), pi * 0.5f);
	body.m_shape = new ShapeBox(g_boxSmall, sizeof(g_boxSmall) / sizeof(Vec3));
	body.m_invMass = 1.0f;
	body.m_elasticity = 1.0f;
	body.m_friction = 0.5f;
	body.m_geometryName = geometryName;
	body.m_objectName = GeneralData::Geometry::RenderItemNames::Hinge::BoxB;
	body.m_materialName = GeneralData::Material::TextureNames::Brick;
	body.m_id = lastIndex;
	bodies.push_back(body);
	++lastIndex;
	{
		ConstraintHingeLimited* joint = new ConstraintHingeLimited();
		joint->m_bodyA = &bodies[bodies.size() - 2];
		joint->m_bodyB = &bodies[bodies.size() - 1];

		const Vec3 jointWorldSpaceAnchor = joint->m_bodyA->m_position;
		joint->m_anchorA = joint->m_bodyA->WorldSpaceToBodySpace(jointWorldSpaceAnchor);
		joint->m_anchorB = joint->m_bodyB->WorldSpaceToBodySpace(jointWorldSpaceAnchor);

		joint->m_axisA = joint->m_bodyA->m_orientation.Inverse().RotatePoint(Vec3(1, 0, 0));

		// Set the initial relative orientation
		joint->m_targetRelativeOrientation = joint->m_bodyA->m_orientation.Inverse() * joint->m_bodyB->m_orientation;

		constraints.push_back(joint);
	}

	indices.emplace_back(startIndex, lastIndex);

	return lastIndex;
}
int AddVelocity(std::vector< Body >& bodies, std::vector<Constraint*>& constraints
	, std::vector<std::pair<unsigned int, unsigned int>>& indices, const int startIndex) {
	Body body;
	std::string geometryName = GeneralData::Geometry::RenderObjectNames::Velocity;
	int lastIndex = startIndex;

	const float pi = acosf(-1.0f);

	body.m_position = Vec3(2, -5, 6);
	body.m_linearVelocity = Vec3(0.0f, 0.0f, 0.0f);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_orientation = Quat(Vec3(1, 1, 1), pi * 0.5f);
	body.m_shape = new ShapeBox(g_boxSmall, sizeof(g_boxSmall) / sizeof(Vec3));
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.9f;
	body.m_friction = 0.5f;
	body.m_geometryName = geometryName;
	body.m_objectName = GeneralData::Geometry::RenderItemNames::Velocity::BoxA;
	body.m_materialName = GeneralData::Material::TextureNames::Brick;
	body.m_id = lastIndex;
	bodies.push_back(body);
	++lastIndex;

	body.m_position = Vec3(2, -5, 5);
	body.m_linearVelocity = Vec3(0.0f, 0.0f, 0.0f);
	body.m_orientation = Quat(Vec3(0, 1, 1), pi * 0.5f);
	body.m_shape = new ShapeBox(g_boxSmall, sizeof(g_boxSmall) / sizeof(Vec3));
	body.m_invMass = 1.0f;
	body.m_elasticity = 1.0f;
	body.m_friction = 0.5f;
	body.m_geometryName = geometryName;
	body.m_objectName = GeneralData::Geometry::RenderItemNames::Velocity::BoxB;
	body.m_materialName = GeneralData::Material::TextureNames::Brick;
	body.m_id = lastIndex;
	bodies.push_back(body);
	++lastIndex;
	{
		ConstraintConstantVelocityLimited* joint = new ConstraintConstantVelocityLimited();
		joint->m_bodyA = &bodies[bodies.size() - 2];
		joint->m_bodyB = &bodies[bodies.size() - 1];

		const Vec3 jointWorldSpaceAnchor = joint->m_bodyA->m_position;
		joint->m_anchorA = joint->m_bodyA->WorldSpaceToBodySpace(jointWorldSpaceAnchor);
		joint->m_anchorB = joint->m_bodyB->WorldSpaceToBodySpace(jointWorldSpaceAnchor);

		joint->m_axisA = joint->m_bodyA->m_orientation.Inverse().RotatePoint(Vec3(0, 0, 1));

		// Set the initial relative orientation
		joint->m_targetRelativeOrientation = joint->m_bodyA->m_orientation.Inverse() * joint->m_bodyB->m_orientation;

		constraints.push_back(joint);
	}

	indices.emplace_back(startIndex, lastIndex);

	return lastIndex;
}
int AddOrientation(std::vector< Body >& bodies, std::vector<Constraint*>& constraints
	, std::vector<std::pair<unsigned int, unsigned int>>& indices, const int startIndex) {
	Body body;
	std::string geometryName = GeneralData::Geometry::RenderObjectNames::Orientation;
	int lastIndex = startIndex;

	body.m_position = Vec3(5, 0, 5);
	body.m_linearVelocity = Vec3(0.0f, 0.0f, 0.0f);
	body.m_angularVelocity = Vec3(0.0f, 0.0f, 0.0f);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_shape = new ShapeBox(g_boxSmall, sizeof(g_boxSmall) / sizeof(Vec3));
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.9f;
	body.m_friction = 0.5f;
	body.m_geometryName = geometryName;
	body.m_objectName = GeneralData::Geometry::RenderItemNames::Orientation::BoxA;
	body.m_materialName = GeneralData::Material::TextureNames::Brick;
	body.m_id = lastIndex;
	bodies.push_back(body);
	++lastIndex;

	body.m_position = Vec3(6, 0, 5);
	body.m_linearVelocity = Vec3(0.0f, 0.0f, 0.0f);
	body.m_angularVelocity = Vec3(0.0f, 0.0f, 0.0f);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_shape = new ShapeBox(g_boxSmall, sizeof(g_boxSmall) / sizeof(Vec3));
	body.m_invMass = 0.001f;
	body.m_elasticity = 1.0f;
	body.m_friction = 0.5f;
	body.m_geometryName = geometryName;
	body.m_objectName = GeneralData::Geometry::RenderItemNames::Orientation::BoxB;
	body.m_materialName = GeneralData::Material::TextureNames::Brick;
	body.m_id = lastIndex;
	bodies.push_back(body);
	++lastIndex;
	{
		ConstraintOrientation* joint = new ConstraintOrientation();
		joint->m_bodyA = &bodies[bodies.size() - 2];
		joint->m_bodyB = &bodies[bodies.size() - 1];

		const Vec3 jointWorldSpaceAnchor = joint->m_bodyA->m_position;
		joint->m_anchorA = joint->m_bodyA->WorldSpaceToBodySpace(jointWorldSpaceAnchor);
		joint->m_anchorB = joint->m_bodyB->WorldSpaceToBodySpace(jointWorldSpaceAnchor);

		// Set the initial relative orientation
		joint->m_targetRelativeOrientation = joint->m_bodyA->m_orientation.Inverse() * joint->m_bodyB->m_orientation;

		constraints.push_back(joint);
	}

	indices.emplace_back(startIndex, lastIndex);

	return lastIndex;
}

int AddSpinner(std::vector< Body >& bodies, std::vector<Constraint*>& constraints
	, std::vector<std::pair<unsigned int, unsigned int>>& indices, const int startIndex) {
	Body body;
	std::string geometryName = GeneralData::Geometry::RenderObjectNames::Spinner;
	int lastIndex = startIndex;

	Vec3 motorPos = Vec3(5, 0, 2);
	Vec3 motorAxis = Vec3(0, 0, 1).Normalize();
	Quat motorOrient = Quat(1, 0, 0, 0);

	body.m_position = motorPos;
	body.m_linearVelocity = Vec3(0.0f, 0.0f, 0.0f);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_shape = new ShapeBox(g_boxSmall, sizeof(g_boxSmall) / sizeof(Vec3));
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.9f;
	body.m_friction = 0.5f;
	body.m_geometryName = geometryName;
	body.m_objectName = GeneralData::Geometry::RenderItemNames::Spinner::Pivot;
	body.m_materialName = GeneralData::Material::TextureNames::Brick;
	body.m_id = lastIndex;
	bodies.push_back(body);
	++lastIndex;

	body.m_position = motorPos - motorAxis;
	body.m_linearVelocity = Vec3(0.0f, 0.0f, 0.0f);
	body.m_orientation = motorOrient;
	body.m_shape = new ShapeBox(g_boxBeam, sizeof(g_boxBeam) / sizeof(Vec3));
	body.m_invMass = 0.01f;
	body.m_elasticity = 1.0f;
	body.m_friction = 0.5f;
	body.m_geometryName = geometryName;
	body.m_objectName = GeneralData::Geometry::RenderItemNames::Spinner::Beam;
	body.m_materialName = GeneralData::Material::TextureNames::Brick;
	body.m_id = lastIndex;
	bodies.push_back(body);
	++lastIndex;
	{
		ConstraintSpinner* joint = new ConstraintSpinner();
		joint->m_bodyA = &bodies[bodies.size() - 2];
		joint->m_bodyB = &bodies[bodies.size() - 1];

		const Vec3 jointWorldSpaceAnchor = joint->m_bodyA->m_position;
		joint->m_anchorA = joint->m_bodyA->WorldSpaceToBodySpace(jointWorldSpaceAnchor);
		joint->m_anchorB = joint->m_bodyB->WorldSpaceToBodySpace(jointWorldSpaceAnchor);

		joint->m_motorTargetSpeed = 2.0f;
		joint->m_motorAxis = joint->m_bodyA->m_orientation.Inverse().RotatePoint(motorAxis);

		// Set the initial relative orientation (in bodyA's space)
		joint->m_targetRelativeOrientation = joint->m_bodyA->m_orientation.Inverse() * joint->m_bodyB->m_orientation;

		constraints.push_back(joint);
	}

	indices.emplace_back(startIndex, lastIndex);	

	return lastIndex;
}
int AddRagdoll(std::vector< Body >& bodies, std::vector<Constraint*>& constraints
	, std::vector<std::pair<unsigned int, unsigned int>>& indices, const int startIndex) {
	Body body;
	std::string geometryName = GeneralData::Geometry::RenderObjectNames::Ragdoll;
	int lastIndex = startIndex;
	Vec3 offset = Vec3(-5, 0, 0);

	// head
	body.m_position = Vec3(0, 0, 5.5f) + offset;
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_shape = new ShapeBox(g_boxSmall, sizeof(g_boxSmall) / sizeof(Vec3));
	body.m_invMass = 2.0f;
	body.m_elasticity = 1.0f;
	body.m_friction = 1.0f;
	body.m_geometryName = geometryName;
	body.m_objectName = GeneralData::Geometry::RenderItemNames::Ragdoll::Head;
	body.m_materialName = GeneralData::Material::MaterialNames::Brick;
	body.m_id = lastIndex;
	bodies.push_back(body);
	++lastIndex;

	// torso
	body.m_position = Vec3(0, 0, 4) + offset;
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_shape = new ShapeBox(g_boxBody, sizeof(g_boxBody) / sizeof(Vec3));
	body.m_invMass = 0.5f;
	body.m_elasticity = 1.0f;
	body.m_friction = 1.0f;
	body.m_geometryName = geometryName;
	body.m_objectName = GeneralData::Geometry::RenderItemNames::Ragdoll::Torso;
	body.m_materialName = GeneralData::Material::MaterialNames::Brick;
	body.m_id = lastIndex;
	bodies.push_back(body);
	++lastIndex;

	// left arm
	body.m_position = Vec3(0.0f, 2.0f, 4.75f) + offset;
	body.m_orientation = Quat(Vec3(0, 0, 1), -3.1415f / 2.0f);
	body.m_shape = new ShapeBox(g_boxLimb, sizeof(g_boxLimb) / sizeof(Vec3));
	body.m_invMass = 1.0f;
	body.m_elasticity = 1.0f;
	body.m_friction = 1.0f;
	body.m_geometryName = geometryName;
	body.m_objectName = GeneralData::Geometry::RenderItemNames::Ragdoll::LeftArm;
	body.m_materialName = GeneralData::Material::MaterialNames::Brick;
	body.m_id = lastIndex;
	bodies.push_back(body);
	++lastIndex;

	// right arm
	body.m_position = Vec3(0.0f, -2.0f, 4.75f) + offset;
	body.m_orientation = Quat(Vec3(0, 0, 1), 3.1415f / 2.0f);
	body.m_shape = new ShapeBox(g_boxLimb, sizeof(g_boxLimb) / sizeof(Vec3));
	body.m_invMass = 1.0f;
	body.m_elasticity = 1.0f;
	body.m_friction = 1.0f;
	body.m_geometryName = geometryName;
	body.m_objectName = GeneralData::Geometry::RenderItemNames::Ragdoll::RightArm;
	body.m_materialName = GeneralData::Material::MaterialNames::Brick;
	body.m_id = lastIndex;
	bodies.push_back(body);
	++lastIndex;

	// left leg
	body.m_position = Vec3(0.0f, 1.0f, 2.5f) + offset;
	body.m_orientation = Quat(Vec3(0, 1, 0), 3.1415f / 2.0f);
	body.m_shape = new ShapeBox(g_boxLimb, sizeof(g_boxLimb) / sizeof(Vec3));
	body.m_invMass = 1.0f;
	body.m_elasticity = 1.0f;
	body.m_friction = 1.0f;
	body.m_geometryName = geometryName;
	body.m_objectName = GeneralData::Geometry::RenderItemNames::Ragdoll::LeftLeg;
	body.m_materialName = GeneralData::Material::MaterialNames::Brick;
	body.m_id = lastIndex;
	bodies.push_back(body);
	++lastIndex;

	// right leg
	body.m_position = Vec3(0.0f, -1.0f, 2.5f) + offset;
	body.m_orientation = Quat(Vec3(0, 1, 0), 3.1415f / 2.0f);
	body.m_shape = new ShapeBox(g_boxLimb, sizeof(g_boxLimb) / sizeof(Vec3));
	body.m_invMass = 1.0f;
	body.m_elasticity = 1.0f;
	body.m_friction = 1.0f;
	body.m_geometryName = geometryName;
	body.m_objectName = GeneralData::Geometry::RenderItemNames::Ragdoll::RightLeg;
	body.m_materialName = GeneralData::Material::MaterialNames::Brick;
	body.m_id = lastIndex;
	bodies.push_back(body);
	++lastIndex;

	indices.emplace_back(startIndex, lastIndex);


	const int idxHead = startIndex;
	const int idxTorso = startIndex + 1;
	const int idxArmLeft = startIndex + 2;
	const int idxArmRight = startIndex + 3;
	const int idxLegLeft = startIndex + 4;
	const int idxLegRight = startIndex + 5;

	// Neck
	{
		ConstraintHingeLimited* joint = new ConstraintHingeLimited();
		joint->m_bodyA = &bodies[idxHead];
		joint->m_bodyB = &bodies[idxTorso];

		const Vec3 jointWorldSpaceAnchor = joint->m_bodyA->m_position + Vec3(0, 0, -0.5f);
		joint->m_anchorA = joint->m_bodyA->WorldSpaceToBodySpace(jointWorldSpaceAnchor);
		joint->m_anchorB = joint->m_bodyB->WorldSpaceToBodySpace(jointWorldSpaceAnchor);

		joint->m_axisA = joint->m_bodyA->m_orientation.Inverse().RotatePoint(Vec3(0, 1, 0));

		// Set the initial relative orientation
		joint->m_targetRelativeOrientation = joint->m_bodyA->m_orientation.Inverse() * joint->m_bodyB->m_orientation;

		constraints.push_back(joint);
	}

	// Shoulder Left
	{
		ConstraintConstantVelocityLimited* joint = new ConstraintConstantVelocityLimited();
		joint->m_bodyB = &bodies[idxArmLeft];
		joint->m_bodyA = &bodies[idxTorso];

		const Vec3 jointWorldSpaceAnchor = joint->m_bodyB->m_position + Vec3(0, -1.0f, 0.0f);
		joint->m_anchorA = joint->m_bodyA->WorldSpaceToBodySpace(jointWorldSpaceAnchor);
		joint->m_anchorB = joint->m_bodyB->WorldSpaceToBodySpace(jointWorldSpaceAnchor);

		joint->m_axisA = joint->m_bodyA->m_orientation.Inverse().RotatePoint(Vec3(0, 1, 0));

		// Set the initial relative orientation
		joint->m_targetRelativeOrientation = joint->m_bodyA->m_orientation.Inverse() * joint->m_bodyB->m_orientation;

		constraints.push_back(joint);
	}

	// Shoulder Right
	{
		ConstraintConstantVelocityLimited* joint = new ConstraintConstantVelocityLimited();
		joint->m_bodyB = &bodies[idxArmRight];
		joint->m_bodyA = &bodies[idxTorso];

		const Vec3 jointWorldSpaceAnchor = joint->m_bodyB->m_position + Vec3(0, 1.0f, 0.0f);
		joint->m_anchorA = joint->m_bodyA->WorldSpaceToBodySpace(jointWorldSpaceAnchor);
		joint->m_anchorB = joint->m_bodyB->WorldSpaceToBodySpace(jointWorldSpaceAnchor);

		joint->m_axisA = joint->m_bodyA->m_orientation.Inverse().RotatePoint(Vec3(0, -1, 0));

		// Set the initial relative orientation
		joint->m_targetRelativeOrientation = joint->m_bodyA->m_orientation.Inverse() * joint->m_bodyB->m_orientation;

		constraints.push_back(joint);
	}

	// Hip Left
	{
		ConstraintHingeLimited* joint = new ConstraintHingeLimited();
		joint->m_bodyB = &bodies[idxLegLeft];
		joint->m_bodyA = &bodies[idxTorso];

		const Vec3 jointWorldSpaceAnchor = joint->m_bodyB->m_position + Vec3(0, 0, 0.5f);
		joint->m_anchorA = joint->m_bodyA->WorldSpaceToBodySpace(jointWorldSpaceAnchor);
		joint->m_anchorB = joint->m_bodyB->WorldSpaceToBodySpace(jointWorldSpaceAnchor);

		joint->m_axisA = joint->m_bodyA->m_orientation.Inverse().RotatePoint(Vec3(0, 1, 0));

		// Set the initial relative orientation
		joint->m_targetRelativeOrientation = joint->m_bodyA->m_orientation.Inverse() * joint->m_bodyB->m_orientation;

		constraints.push_back(joint);
	}

	// Hip Right
	{
		ConstraintHingeLimited* joint = new ConstraintHingeLimited();
		joint->m_bodyB = &bodies[idxLegRight];
		joint->m_bodyA = &bodies[idxTorso];

		const Vec3 jointWorldSpaceAnchor = joint->m_bodyB->m_position + Vec3(0, 0, 0.5f);
		joint->m_anchorA = joint->m_bodyA->WorldSpaceToBodySpace(jointWorldSpaceAnchor);
		joint->m_anchorB = joint->m_bodyB->WorldSpaceToBodySpace(jointWorldSpaceAnchor);

		joint->m_axisA = joint->m_bodyA->m_orientation.Inverse().RotatePoint(Vec3(0, 1, 0));

		// Set the initial relative orientation
		joint->m_targetRelativeOrientation = joint->m_bodyA->m_orientation.Inverse() * joint->m_bodyB->m_orientation;

		constraints.push_back(joint);
	}

	return lastIndex;
}



int AddConvex(std::vector< Body >& bodies, std::vector<std::pair<unsigned int, unsigned int>>& indices, const int startIndex) {
	Body body;
	std::string geometryName = GeneralData::Geometry::RenderObjectNames::Convex;
	int lastIndex = startIndex;

	body.m_position = Vec3(-10.0f, 0.0f, 5.0f);
	body.m_linearVelocity = Vec3(0.0f, 0.0f, 0.0f);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_shape = new ShapeSphere(1.0f);
	body.m_invMass = 1.0f;
	body.m_elasticity = 0.9f;
	body.m_friction = 0.5f;
	body.m_geometryName = geometryName;
	body.m_objectName = GeneralData::Geometry::RenderItemNames::Convex::Sphere;
	body.m_materialName = GeneralData::Material::TextureNames::Brick;
	body.m_id = lastIndex;
	bodies.push_back(body);
	++lastIndex;

	body.m_position = Vec3(-10.0f, 0.0f, 10.0f);
	body.m_linearVelocity = Vec3(0.0f, 0.0f, 0.0f);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_shape = new ShapeConvex(g_diamond, sizeof(g_diamond) / sizeof(Vec3));
	body.m_invMass = 1.0f;
	body.m_elasticity = 1.0f;
	body.m_friction = 0.5f;
	body.m_geometryName = geometryName;
	body.m_objectName = GeneralData::Geometry::RenderItemNames::Convex::Diamond;
	body.m_materialName = GeneralData::Material::TextureNames::Brick;
	body.m_id = lastIndex;
	bodies.push_back(body);
	++lastIndex;


	indices.emplace_back(startIndex, lastIndex);

	return lastIndex;
}
/*
// use this function to extend
int AddChain(std::vector< Body >& bodies, std::vector<std::pair<unsigned int, unsigned int>>& indices, const int startIndex) {
	Body body;
	std::string geometryName = GeneralData::Geometry::RenderObjectNames::Ragdoll;
	int lastIndex = startIndex;


		body.m_geometryName = geometryName;
	body.m_objectName = GeneralData::Geometry::RenderItemNames::Ragdoll::RightLeg;
	
	++lastIndex;


	indices.emplace_back(startIndex, lastIndex);

	return lastIndex;

}
*/

int AddSandbox(std::vector< Body >& bodies, std::vector<std::pair<unsigned int, unsigned int>> &indices, const int startIndex) {
	Body body;
	std::string geometryName = GeneralData::Geometry::RenderObjectNames::Sandbox;
	int lastIndex = startIndex;
	body.m_position = Vec3(0, 0, 0);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity.Zero();
	body.m_angularVelocity.Zero();
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.5f;
	body.m_shape = new ShapeBox(g_boxGround, sizeof(g_boxGround) / sizeof(Vec3));
	body.m_geometryName = geometryName;
	body.m_objectName = GeneralData::Geometry::RenderItemNames::Sandbox::Ground;
	body.m_materialName = GeneralData::Material::MaterialNames::Brick;
	body.m_id = lastIndex;
	bodies.push_back(body);
	++lastIndex;

	body.m_position = Vec3(50, 0, 0);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity.Zero();
	body.m_angularVelocity.Zero();
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.0f;
	body.m_shape = new ShapeBox(g_boxWall0, sizeof(g_boxWall0) / sizeof(Vec3));
	body.m_geometryName = geometryName;
	body.m_objectName = GeneralData::Geometry::RenderItemNames::Sandbox::WallHorizontal;
	body.m_materialName = GeneralData::Material::MaterialNames::Brick;
	body.m_id = lastIndex;
	bodies.push_back(body);
	++lastIndex;

	body.m_position = Vec3(-50, 0, 0);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity.Zero();
	body.m_angularVelocity.Zero();
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.0f;
	body.m_shape = new ShapeBox(g_boxWall0, sizeof(g_boxWall0) / sizeof(Vec3));
	body.m_geometryName = geometryName;
	body.m_objectName = GeneralData::Geometry::RenderItemNames::Sandbox::WallHorizontal;
	body.m_materialName = GeneralData::Material::MaterialNames::Brick;
	body.m_id = lastIndex;
	bodies.push_back(body);
	++lastIndex;

	body.m_position = Vec3(0, 25, 0);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity.Zero();
	body.m_angularVelocity.Zero();
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.0f;
	body.m_shape = new ShapeBox(g_boxWall1, sizeof(g_boxWall1) / sizeof(Vec3));
	body.m_geometryName = geometryName;
	body.m_objectName = GeneralData::Geometry::RenderItemNames::Sandbox::WallVertical;
	body.m_materialName = GeneralData::Material::MaterialNames::Brick;
	body.m_id = lastIndex;
	bodies.push_back(body);
	++lastIndex;

	body.m_position = Vec3(0, -25, 0);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_linearVelocity.Zero();
	body.m_angularVelocity.Zero();
	body.m_invMass = 0.0f;
	body.m_elasticity = 0.5f;
	body.m_friction = 0.0f;
	body.m_shape = new ShapeBox(g_boxWall1, sizeof(g_boxWall1) / sizeof(Vec3));
	body.m_geometryName = geometryName;
	body.m_objectName = GeneralData::Geometry::RenderItemNames::Sandbox::WallVertical;
	body.m_materialName = GeneralData::Material::MaterialNames::Brick;
	body.m_id = lastIndex;
	bodies.push_back(body);
	++lastIndex;


	indices.emplace_back(startIndex, lastIndex);

	return lastIndex;
}
