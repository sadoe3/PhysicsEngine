#pragma once

// renderer
#include "../Renderer/Common/d3dApp.h"
#include "../Renderer/Common/MathHelper.h"
#include "../Renderer/Common/UploadBuffer.h"
#include "../Renderer/Common/GeometryGenerator.h"
#include "../Renderer/Common/Camera.h"
#include "../Renderer/FrameResource.h"
#include "../Renderer/GeneralConstants.h"
#include "../Renderer/ShadowMap.h"

// physics
#include "../Physics/Body.h"
#include "../Physics/Broadphase.h"
#include "../Physics/Constraints.h"
#include "../Physics/Contact.h"
#include "../Physics/GJK.h"
#include "../Physics/Intersections.h"
#include "../Physics/Manifold.h"

// scene management
#include "../Renderer/StateData.h"
#include "../Main/SceneConfiguration.h"


using Microsoft::WRL::ComPtr;
using namespace DirectX;
using namespace DirectX::PackedVector;


// A structure to hold an object's complete state
struct ObjectState {
    DirectX::XMFLOAT3 position;
    DirectX::XMFLOAT4 orientation; // Quaternion for rotation
    DirectX::XMFLOAT3 linearVelocity;
    DirectX::XMFLOAT3 angularVelocity;
    // Include other state like forces, torques, etc. if needed
};
// A structure to hold the state of the entire scene for one frame
struct FrameState {
    std::unordered_map<int, ObjectState> objectStates;
};

// imgui
struct ImguiDescriptorHeapAllocator
{
    ID3D12DescriptorHeap* Heap = nullptr;
    D3D12_DESCRIPTOR_HEAP_TYPE  HeapType = D3D12_DESCRIPTOR_HEAP_TYPE_NUM_TYPES;
    D3D12_CPU_DESCRIPTOR_HANDLE HeapStartCpu;
    D3D12_GPU_DESCRIPTOR_HANDLE HeapStartGpu;
    UINT                        HeapHandleIncrement;
    ImVector<int>               FreeIndices;

    // This method now takes a sub-range of the heap to manage.
    void Create(ID3D12Device* device, ID3D12DescriptorHeap* heap, UINT startOffset, UINT numDescriptors)
    {
        IM_ASSERT(Heap == nullptr && FreeIndices.empty());
        IM_ASSERT(heap != nullptr);

        Heap = heap;
        D3D12_DESCRIPTOR_HEAP_DESC desc = heap->GetDesc();
        HeapType = desc.Type;
        HeapHandleIncrement = device->GetDescriptorHandleIncrementSize(HeapType);

        // Calculate the starting handles for the sub-range
        HeapStartCpu.ptr = heap->GetCPUDescriptorHandleForHeapStart().ptr + (static_cast<SIZE_T>(startOffset) * HeapHandleIncrement);
        HeapStartGpu.ptr = heap->GetGPUDescriptorHandleForHeapStart().ptr + (static_cast<SIZE_T>(startOffset) * HeapHandleIncrement);

        // Populate the free indices list for this specific sub-range
        FreeIndices.reserve(numDescriptors);
        for (int n = numDescriptors - 1; n >= 0; --n)
            FreeIndices.push_back(n);
    }

    // The rest of the methods remain correct for managing a sub-range.
    void Destroy()
    {
        Heap = nullptr;
        FreeIndices.clear();
    }

    void Alloc(D3D12_CPU_DESCRIPTOR_HANDLE* out_cpu_desc_handle, D3D12_GPU_DESCRIPTOR_HANDLE* out_gpu_desc_handle)
    {
        IM_ASSERT(FreeIndices.Size > 0);
        int idx = FreeIndices.back();
        FreeIndices.pop_back();
        out_cpu_desc_handle->ptr = HeapStartCpu.ptr + (static_cast<SIZE_T>(idx) * HeapHandleIncrement);
        out_gpu_desc_handle->ptr = HeapStartGpu.ptr + (static_cast<SIZE_T>(idx) * HeapHandleIncrement);
    }

    void Free(D3D12_CPU_DESCRIPTOR_HANDLE out_cpu_desc_handle, D3D12_GPU_DESCRIPTOR_HANDLE out_gpu_desc_handle)
    {
        IM_ASSERT(out_cpu_desc_handle.ptr >= HeapStartCpu.ptr);
        int cpu_idx = (int)((out_cpu_desc_handle.ptr - HeapStartCpu.ptr) / HeapHandleIncrement);
        int gpu_idx = (int)((out_gpu_desc_handle.ptr - HeapStartGpu.ptr) / HeapHandleIncrement);
        IM_ASSERT(cpu_idx == gpu_idx);
        FreeIndices.push_back(cpu_idx);
    }
};

// Lightweight structure stores parameters to draw a shape.  This will
// vary from app-to-app.
struct RenderItem
{
    RenderItem() = default;
    RenderItem(const RenderItem& rhs) = delete;

    bool Visible = true;

    BoundingBox Bounds;

    // World matrix of the shape that describes the object's local space
    // relative to the world space, which defines the position, orientation,
    // and scale of the object in the world.
    XMFLOAT4X4 World = MathHelper::Identity4x4();

    XMFLOAT4X4 TexTransform = MathHelper::Identity4x4();

    // Dirty flag indicating the object data has changed and we need to update the constant buffer.
    // Because we have an object cbuffer for each FrameResource, we have to apply the
    // update to each FrameResource.  Thus, when we modify obect data we should set 
    // NumFramesDirty = gNumFrameResources so that each frame resource gets the update.
    int NumFramesDirty = gNumFrameResources;

    // Index into GPU constant buffer corresponding to the ObjectCB for this render item.
    UINT ObjCBIndex = -1;

    Material* Mat = nullptr;
    MeshGeometry* Geo = nullptr;

    // Primitive topology.
    D3D12_PRIMITIVE_TOPOLOGY PrimitiveType = D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;

    // DrawIndexedInstanced parameters.
    UINT IndexCount = 0;
    UINT StartIndexLocation = 0;
    int BaseVertexLocation = 0;
};

enum class RenderLayer : int
{
    Opaque = 0,
    Count
};

class PhysicsApplication : public D3DApp
{
public:
    PhysicsApplication(HINSTANCE hInstance);
    PhysicsApplication(const PhysicsApplication& rhs) = delete;
    PhysicsApplication& operator=(const PhysicsApplication& rhs) = delete;
    ~PhysicsApplication();

    virtual bool Initialize()override;

private:
    // main loop
    virtual void Update(const GameTimer& gt)override;
    virtual void Draw(const GameTimer& gt)override;

    // event handler
    virtual void OnResize()override;
    virtual void OnMouseDown(WPARAM btnState, int x, int y)override;
    virtual void OnMouseUp(WPARAM btnState, int x, int y)override;
    virtual void OnMouseMove(WPARAM btnState, int x, int y)override;
    virtual void OnKeyboardInput(WPARAM btnState)override;   // for general shortcut
    void OnKeyboardInput(const GameTimer& gt);               // for camera movement

    // picking
    void Pick(int sx, int sy);
    void ResetPickedItem();

    void ApplyPhysicsOnPickedItem();
    void PauseEngine();
    void ResumeEngine();

    // initialization
    void LoadTextures();
    void BuildRootSignature();
    void BuildMaterials();
    void BuildFrameResources();
    void BuildDescriptorHeaps();
    void BuildShadersAndInputLayout();
    void BuildPSOs();

    // sampler
    std::array<const CD3DX12_STATIC_SAMPLER_DESC, 7> GetStaticSamplers();
    // camera
    void ResetCamera();

    // update
    void CleanupSceneResources();

    void UpdatePositionAndOrientation(const float deltaSecond);
    void UpdateInstanceData(const GameTimer& gt);
    void UpdateObjectCBs();
    void UpdateMaterialBuffer(const GameTimer& gt);
    void UpdateShadowTransform(const GameTimer& gt);
    void UpdateMainPassCB(const GameTimer& gt);
    void UpdateShadowPassCB(const GameTimer& gt);

    // render GUI
    void RenderMainMenuUI();
    void RenderDemoUICommon();
    void RenderDemoUILoading();
    void RenderDemoUIGizmo();
    void RenderDemoUIVisualDebugger();
    void RenderDemoUIStressTest();
    void RenderDemoUISandBox();

    // frame controller
    void SaveCurrentFrameState();
    void LoadFrameState(const int targetIndex);

    // FPS
    void CalculateAndDisplayFPS(float deltaTime);

    // setup demo scenes
    // -----
    void SetupDemoSceneStressTest(GeometryGenerator& geometryGenerator);
    void SetupDemoSceneSandbox(GeometryGenerator& geometryGenerator);

    // stress test
    void SetupDemoSceneSpheres(GeometryGenerator& geometryGenerator);
    void SetupDemoSceneDiamonds(GeometryGenerator& geometryGenerator);

    void BuildRenderItemsStressTest(GeometryGenerator& geometryGenerator, int lastIndex);

    // sandbox
    void SetupDemoSceneStack(GeometryGenerator& geometryGenerator);
    void SetupDemoSceneMover(GeometryGenerator& geometryGenerator);
    void SetupDemoSceneChain(GeometryGenerator& geometryGenerator);

    void SetupDemoSceneHinge(GeometryGenerator& geometryGenerator);
    void SetupDemoSceneVelocity(GeometryGenerator& geometryGenerator);
    void SetupDemoSceneOrientation(GeometryGenerator& geometryGenerator);

    void SetupDemoSceneSpinner(GeometryGenerator& geometryGenerator);
    void SetupDemoSceneRagdoll(GeometryGenerator& geometryGenerator);

    void SetupDemoSceneConvex(GeometryGenerator& geometryGenerator);

    void SetupDemoSceneAll(GeometryGenerator& geometryGenerator);    

    void BuildRenderItemsSandbox(GeometryGenerator& geometryGenerator, int lastIndex);

    // contact points
    void BuildRenderItemsContactPoints();
    // -----

    // draw
    void DrawRenderItems(ID3D12GraphicsCommandList* cmdList, const std::vector<RenderItem*>& ritems, const std::unordered_set<Body*>& collisionBodies, bool isWireframePass);
    std::vector<Vec3> GetIncidentFaceVertices(Body* targetBody, const Vec3& contactNormalWorld, const Vec3& contactPointWorld);
    void DrawContactPoints(ID3D12GraphicsCommandList* cmdList, Body* selectedBody);
    void DrawSceneToShadowMap();

    // build geometry
    // -----
    // stress test
    void BuildGeometrySpheres(GeometryGenerator& geoGen);
    void BuildGeometryDiamonds(GeometryGenerator& geoGen);
    void BuildGeometryFloor(GeometryGenerator& geoGen);

    // sandbox
    void BuildGeometryStack(GeometryGenerator& geoGen, int lastIndex);
    void BuildGeometryMover(GeometryGenerator& geoGen);
    void BuildGeometryChain(GeometryGenerator& geoGen, int lastIndex);

    void BuildGeometryHinge(GeometryGenerator& geoGen, int lastIndex);
    void BuildGeometryVelocity(GeometryGenerator& geoGen, int lastIndex);
    void BuildGeometryOrientation(GeometryGenerator& geoGen, int lastIndex);

    void BuildGeometrySpinner(GeometryGenerator& geoGen, int lastIndex);
    void BuildGeometryRagdoll(GeometryGenerator& geoGen, int lastIndex);

    void BuildGeometryConvex(GeometryGenerator& geoGen, int lastIndex);

    void BuildGeometrySandbox(GeometryGenerator& geoGen);

    // contact points
    void BuildGeometryContactPoints(GeometryGenerator& geoGen);
    // -----
private:
    std::vector<std::unique_ptr<FrameResource>> mFrameResources;
    FrameResource* mCurrFrameResource = nullptr;
    int mCurrFrameResourceIndex = 0;

    ComPtr<ID3D12RootSignature> mRootSignature = nullptr;
    ComPtr<ID3D12DescriptorHeap> mSrvDescriptorHeap = nullptr;
    UINT mCbvSrvDescriptorSize = 0;

    std::unordered_map<std::string, std::unique_ptr<MeshGeometry>> mGeometries;
    std::unordered_map<std::string, std::unique_ptr<Material>> mMaterials;
    std::unordered_map<std::string, std::unique_ptr<Texture>> mTextures;
    std::unordered_map<std::string, ComPtr<ID3DBlob>> mShaders;
    std::unordered_map<std::string, ComPtr<ID3D12PipelineState>> mPSOs;

    std::vector<D3D12_INPUT_ELEMENT_DESC> mInputLayout;

    // List of all the render items.
    std::vector<std::unique_ptr<RenderItem>> mAllRitems;

    // Render items divided by PSO.
    std::vector<RenderItem*> mRitemLayer[(int)RenderLayer::Count];

    UINT mSkyTexHeapIndex = 0;
    UINT mShadowMapHeapIndex = 0;
    UINT mNullTexSrvIndex = 0;

    CD3DX12_GPU_DESCRIPTOR_HANDLE mNullSrv;

    PassConstants mMainPassCB;  // index 0 of pass cbuffer.
    PassConstants mShadowPassCB;// index 1 of pass cbuffer.

    Camera mCamera;
    const XMFLOAT3 INIT_CAMERA_POSITION = { 0.0f, 13.0f, -30.0f };

    std::unique_ptr<ShadowMap> mShadowMap;

    DirectX::BoundingSphere mSceneBounds;

    float mLightNearZ = 0.0f;
    float mLightFarZ = 0.0f;
    XMFLOAT3 mLightPosW;
    XMFLOAT4X4 mLightView = MathHelper::Identity4x4();
    XMFLOAT4X4 mLightProj = MathHelper::Identity4x4();
    XMFLOAT4X4 mShadowTransform = MathHelper::Identity4x4();

    XMFLOAT3 mBaseLightDirections[3] = {
        XMFLOAT3(0.57735f, -0.57735f, 0.57735f),
        XMFLOAT3(-0.57735f, -0.57735f, 0.57735f),
        XMFLOAT3(0.0f, -0.707f, -0.707f)
    };

    POINT mLastMousePos;



    // physics
    std::vector<Body> mBodies;
    std::vector<Constraint*> mConstraints;
    ManifoldCollector m_manifolds;
    std::vector<std::pair<unsigned int, unsigned int>> geometryStartEndIndices;

    // scene state
    SceneState mRequestedSceneState = SceneState::NONE;
    SceneState mCurrentSceneState = SceneState::NONE;

    // picking
    RenderItem* mPickedRenderItem = nullptr;
    int mPickedContactPointIndex = -1;
    std::vector<contact_t> mContactPoints;
    std::vector<std::unique_ptr<RenderItem>> mRenderItemsContactPoints;
    bool mIsHitResultVisible = true;

    // instancing
    std::vector<InstanceData> mInstanceDataCache;

    // frame controller
    std::deque<FrameState> mFrameHistory;
    int mCurrentAvailableFrameHistoryIndex = 0;
    int mCurrentFrameHistoryIndex = 0;

    // wireframe
    bool mIsWireframeDebugEnabled = false;
    std::unordered_set<Body*> mCollisionBodies;

    // imgui
    ImFont* mFontMainMenu;
    ImFont* mFontDemo;
    const float mFontSizeMainMenu = 24.0f;
    const float mFontSizeDemo = 13.0f;
    bool mIsRestartNeeded = false;
    float mLockTimer = -1.0f;

    bool mIsSliderMoving = false;
    bool mIsGuizmoMoving = false;
    ImGuizmo::OPERATION mCurrentGizmoOperation = ImGuizmo::TRANSLATE;

    // fps
    float mElapsedTime = 0.0f;
    int mFrameCount = 0;
    float mAverageFPS = 0.0f;
    float mHistoryFPS[GeneralData::GUI::HistorySize] = { 0.0f };
    int mHistoryIndex = 0;
public:
    // scene related values
    SandboxState mSandboxState = SandboxState::STACK;
    bool mIsBroadNarrowOn = true;

    bool mIsStressTestShapeSphere = true;
    int mStressLevel = 6;
    float mStressStartHeight = 10.0f;
};