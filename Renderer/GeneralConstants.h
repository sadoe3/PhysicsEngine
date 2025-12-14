#pragma once
#include <string>

namespace GeneralData {
const int NumFrameResources = 3;
const int NumTextures = 14;
const int NumSandboxMaxBodyCount = 100;
const int NumSandboxGroundBodies = 6;
const int NumInstanceBufferSrvIndex = 3;
const int NumMaxInstanceCapacity = 5000;	
const int NumFloorSphereCount = 9;

const float LockTime = 1.5f;
const float CameraSpeed = 0.9f;
const float TargetFrameTime = 1.0f / 60.0f;
const float FixedDeltaTime = 1.0f / 30.0f;
const float ContactPointWidth = 0.1f;

namespace GUI {
	const int NumSrvDescriptorsImGui = 64;
	const int HistorySize = 60; // fps
	const int MaxFrameHistorySize = 120;
	const int PanelWidth = 250;
	const int PanelHeightStressTest = 477;
	const int PanelHeightSandbox = 500;
}


namespace Geometry {
	namespace RenderObjectNames {
		const std::string Spheres = "Spheres";
		const std::string Diamonds = "Diamonds";
		const std::string Floor = "Floor";
		const std::string ContactPoints = "ContactPoints";

		const std::string Ragdoll = "Ragdoll";
		const std::string Chain = "Chain";
		const std::string Stack = "Stack";
		const std::string Spinner = "Spinner";
		const std::string Mover = "Mover";
		const std::string Hinge = "Hinge";
		const std::string Velocity = "Velocity";
		const std::string Orientation = "Orientation";
		const std::string Convex = "Convex";
		const std::string Sandbox = "Sandbox";
	};

	namespace RenderItemNames {
		namespace Ragdoll {
			const std::string Head = "Head";
			const std::string Torso = "Torso";
			const std::string LeftArm = "LeftArm";
			const std::string RightArm = "RightArm";
			const std::string LeftLeg = "LeftLeg";
			const std::string RightLeg = "RightLeg";
		};
		namespace Chain {
			const std::string TopBox = "Top";
			const std::string BoxA = "A";
			const std::string BoxB = "B";
			const std::string BoxC = "C";
			const std::string BoxD = "D";
			const std::string BoxE = "E";
		};

		namespace Stack {
			const std::string BoxA = "A";
			const std::string BoxB = "B";
			const std::string BoxC = "C";
			const std::string BoxD = "D";
			const std::string BoxE = "E";
		};

		namespace Spinner {
			const std::string Pivot = "Pivot";
			const std::string Beam = "Beam";
		};

		namespace Mover {
			const std::string Platform = "Platform";
			const std::string Unit = "Unit";
		};

		namespace Hinge {
			const std::string BoxA = "A";
			const std::string BoxB = "B";
		};

		namespace Velocity {
			const std::string BoxA = "A";
			const std::string BoxB = "B";
		};

		namespace Orientation {
			const std::string BoxA = "A";
			const std::string BoxB = "B";
		};

		namespace Convex {
			const std::string Sphere = "Sphere";
			const std::string Diamond = "Diamond";
		};

		namespace Sandbox {
			const std::string Ground = "Ground";
			const std::string WallHorizontal = "WallHorizontal";
			const std::string WallVertical = "WallVertical";
		};
	};
};

namespace Material {
	namespace Indices {
		const int MaterialIndexStart = 0;
		const int MaterialIndexBrick = 0;
		const int MaterialIndexStone = 1;
		const int MaterialIndexTile = 2;
		const int MaterialIndexEnd = 3;
	}

	namespace MaterialNames {
		const std::string Brick = "bricks0";
		const std::string Tile  = "tile0";
	};

	namespace TextureNames {
		const std::string Checker = "checkerTex";
		const std::string Brick = "bricks0";
		const std::string Tile = "tile0";
		const std::string Mirror = "mirror0";
		const std::string SkullMat = "skullMat";
		const std::string Sky = "sky";
		const std::string Highlight = "highlight0";
	}
};

};


