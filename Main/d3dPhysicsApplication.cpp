#include "PCH.h"
#include "d3dPhysicsApplication.h"


const int gNumFrameResources = GeneralData::NumFrameResources;

// imgui
static ImguiDescriptorHeapAllocator g_pd3dSrvDescHeapAlloc;


PhysicsApplication::PhysicsApplication(HINSTANCE hInstance)
    : D3DApp(hInstance)
{
    // Estimate the scene bounding sphere manually since we know how the scene was constructed.
    // The grid is the "widest object" with a width of 20 and depth of 30.0f, and centered at
    // the world space origin.  In general, you need to loop over every world space vertex
    // position and compute the bounding sphere.
    mSceneBounds.Center = XMFLOAT3(0.0f, 0.0f, 0.0f);
    mSceneBounds.Radius = sqrtf(2900.0f);
}
PhysicsApplication::~PhysicsApplication()
{
    if (md3dDevice != nullptr)
        FlushCommandQueue();
}

// initialize
bool PhysicsApplication::Initialize()
{
    if (!D3DApp::Initialize())
        return false;

    mCbvSrvDescriptorSize = md3dDevice->GetDescriptorHandleIncrementSize(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);

    // Reset the command list to prep for initialization commands.
    ThrowIfFailed(mCommandList->Reset(mDirectCmdListAlloc.Get(), nullptr));

    ResetCamera();

    mShadowMap = std::make_unique<ShadowMap>(
        md3dDevice.Get(), 2048, 2048);

    LoadTextures();
    BuildRootSignature();
    BuildMaterials();
    BuildFrameResources();
    BuildDescriptorHeaps();     // imgui
    BuildShadersAndInputLayout();
    BuildPSOs();


    mRequestedSceneState = SceneState::MAIN_MENU;

    // imgui
    // ----------------------------------------------
    // Initialize ImGui backends
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;         // Enable Docking

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();

    // Setup scaling
    ImGuiStyle& style = ImGui::GetStyle();
    float main_scale = ImGui_ImplWin32_GetDpiScaleForMonitor(::MonitorFromPoint(POINT{ 0, 0 }, MONITOR_DEFAULTTOPRIMARY));
    style.ScaleAllSizes(main_scale);        // Bake a fixed style scale. (until we have a solution for dynamic style scaling, changing this requires resetting Style + calling this again)
    style.FontScaleDpi = main_scale;        // Set initial font scale. (using io.ConfigDpiScaleFonts=true makes this unnecessary. We leave both here for documentation purpose)

    // Setup Platform/Renderer backends
    ImGui_ImplWin32_Init(mhMainWnd);

    ImGui_ImplDX12_InitInfo init_info = {};
    init_info.Device = md3dDevice.Get();
    init_info.CommandQueue = mCommandQueue.Get();
    init_info.NumFramesInFlight = GeneralData::NumFrameResources;
    init_info.RTVFormat = DXGI_FORMAT_R8G8B8A8_UNORM;
    init_info.DSVFormat = DXGI_FORMAT_UNKNOWN;
    // Allocating SRV descriptors (for textures) is up to the application, so we provide callbacks.
    // (current version of the backend will only allocate one descriptor, future versions will need to allocate more)
    init_info.SrvDescriptorHeap = mSrvDescriptorHeap.Get();
    init_info.SrvDescriptorAllocFn = [](ImGui_ImplDX12_InitInfo*, D3D12_CPU_DESCRIPTOR_HANDLE* out_cpu_handle, D3D12_GPU_DESCRIPTOR_HANDLE* out_gpu_handle) { return g_pd3dSrvDescHeapAlloc.Alloc(out_cpu_handle, out_gpu_handle); };
    init_info.SrvDescriptorFreeFn = [](ImGui_ImplDX12_InitInfo*, D3D12_CPU_DESCRIPTOR_HANDLE cpu_handle, D3D12_GPU_DESCRIPTOR_HANDLE gpu_handle) { return g_pd3dSrvDescHeapAlloc.Free(cpu_handle, gpu_handle); };
    ImGui_ImplDX12_Init(&init_info);

    ImFontConfig fontConfig;
    fontConfig.SizePixels = mFontSizeMainMenu;
    mFontMainMenu = io.Fonts->AddFontDefault(&fontConfig);
    fontConfig.SizePixels = mFontSizeDemo;
    mFontDemo = io.Fonts->AddFontDefault(&fontConfig);
    // ----------------------------------------------

    // Execute the initialization commands.
    ThrowIfFailed(mCommandList->Close());
    ID3D12CommandList* cmdsLists[] = { mCommandList.Get() };
    mCommandQueue->ExecuteCommandLists(_countof(cmdsLists), cmdsLists);
    // Wait until initialization is complete.
    FlushCommandQueue();


    // convex
    FillDiamond();

    return true;
}

void PhysicsApplication::LoadTextures()
{
    std::vector<std::string> texNames =
    {
        "skyCubeMap"
    };

    std::vector<std::wstring> texFilenames =
    {
        L"Textures\\desertcube1024.dds"
    };

    for (int i = 0; i < (int)texNames.size(); ++i)
    {
        auto texMap = std::make_unique<Texture>();
        texMap->Name = texNames[i];
        texMap->Filename = texFilenames[i];
        ThrowIfFailed(DirectX::CreateDDSTextureFromFile12(md3dDevice.Get(),
            mCommandList.Get(), texMap->Filename.c_str(),
            texMap->Resource, texMap->UploadHeap));

        mTextures[texMap->Name] = std::move(texMap);
    }
}
void PhysicsApplication::BuildRootSignature()
{
    CD3DX12_DESCRIPTOR_RANGE texTable0;
    texTable0.Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 2, 0, 0);
    CD3DX12_DESCRIPTOR_RANGE texTable1;
    texTable1.Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 2, 0);

    // Descriptor Range for Instancing Data (t3, space1)
    CD3DX12_DESCRIPTOR_RANGE instTable;
    instTable.Init(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 3, 1);

    // Root parameter can be a table, root descriptor or root constants.
    // expand slot to 6 for instancing
    CD3DX12_ROOT_PARAMETER slotRootParameter[7];

    // Perfomance TIP: Order from most frequent to least frequent.
    slotRootParameter[0].InitAsConstantBufferView(0);
    slotRootParameter[1].InitAsConstantBufferView(1);
    slotRootParameter[2].InitAsShaderResourceView(0, 1);
    slotRootParameter[3].InitAsDescriptorTable(1, &texTable0, D3D12_SHADER_VISIBILITY_PIXEL);
    slotRootParameter[4].InitAsDescriptorTable(1, &texTable1, D3D12_SHADER_VISIBILITY_PIXEL);

    // new slot for instancing
    slotRootParameter[5].InitAsDescriptorTable(1, &instTable, D3D12_SHADER_VISIBILITY_VERTEX);
    

    auto staticSamplers = GetStaticSamplers();

    // A root signature is an array of root parameters.
    CD3DX12_ROOT_SIGNATURE_DESC rootSigDesc(6, slotRootParameter,
        (UINT)staticSamplers.size(), staticSamplers.data(),
        D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT);

    // create a root signature with a single slot which points to a descriptor range consisting of a single constant buffer
    ComPtr<ID3DBlob> serializedRootSig = nullptr;
    ComPtr<ID3DBlob> errorBlob = nullptr;
    HRESULT hr = D3D12SerializeRootSignature(&rootSigDesc, D3D_ROOT_SIGNATURE_VERSION_1,
        serializedRootSig.GetAddressOf(), errorBlob.GetAddressOf());

    if (errorBlob != nullptr)
    {
        ::OutputDebugStringA((char*)errorBlob->GetBufferPointer());
    }
    ThrowIfFailed(hr);

    ThrowIfFailed(md3dDevice->CreateRootSignature(
        0,
        serializedRootSig->GetBufferPointer(),
        serializedRootSig->GetBufferSize(),
        IID_PPV_ARGS(mRootSignature.GetAddressOf())));
}
void PhysicsApplication::BuildMaterials()
{
    auto sky = std::make_unique<Material>();
    sky->Name = "sky";
    sky->MatCBIndex = 0;
    sky->DiffuseSrvHeapIndex = 0;
    // since there's no normal map, use Null 2D SRV index (2) instead
    sky->NormalSrvHeapIndex = 2;
    sky->DiffuseAlbedo = XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f);
    sky->FresnelR0 = XMFLOAT3(0.1f, 0.1f, 0.1f);
    sky->Roughness = 1.0f;



    // custom material
    auto custom = std::make_unique<Material>();
    custom->Name = GeneralData::Material::TextureNames::Brick;
    custom->MatCBIndex = 1;
    custom->DiffuseSrvHeapIndex = 2;
    custom->NormalSrvHeapIndex = 2;
    custom->DiffuseAlbedo = XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f);
    custom->FresnelR0 = XMFLOAT3(0.01f, 0.01f, 0.01f);
    custom->Roughness = 0.0f;

    mMaterials["sky"] = std::move(sky);
    mMaterials[GeneralData::Material::TextureNames::Brick] = std::move(custom);
}
void PhysicsApplication::BuildFrameResources()
{
    const UINT MAX_OBJECTS_CAPACITY = 30000;

    for (int i = 0; i < gNumFrameResources; ++i)
    {
        mFrameResources.push_back(std::make_unique<FrameResource>(md3dDevice.Get(),
            2,
            MAX_OBJECTS_CAPACITY, // <--- Use the max capacity instead of initial size!
            (UINT)mMaterials.size(), GeneralData::NumMaxInstanceCapacity));
    }
}
void PhysicsApplication::BuildDescriptorHeaps()
{
    //
    // Create the SRV heap.
    //
    D3D12_DESCRIPTOR_HEAP_DESC srvHeapDesc = {};
    // Sky(0), Shadow(1), Diffuse/Null(2), Instancing(3,4,5) + Imgui
    srvHeapDesc.NumDescriptors = 3 + gNumFrameResources + GeneralData::GUI::NumSrvDescriptorsImGui;
    srvHeapDesc.Type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
    srvHeapDesc.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE;
    ThrowIfFailed(md3dDevice->CreateDescriptorHeap(&srvHeapDesc, IID_PPV_ARGS(&mSrvDescriptorHeap)));
    g_pd3dSrvDescHeapAlloc.Create(md3dDevice.Get(), mSrvDescriptorHeap.Get(), 0, srvHeapDesc.NumDescriptors);


    //
    // Fill out the heap with actual descriptors.
    //
    CD3DX12_CPU_DESCRIPTOR_HANDLE hDescriptor(mSrvDescriptorHeap->GetCPUDescriptorHandleForHeapStart());
    auto skyCubeMap = mTextures["skyCubeMap"]->Resource;

    D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
    srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
    srvDesc.ViewDimension = D3D12_SRV_DIMENSION_TEXTURECUBE;
    srvDesc.TextureCube.MostDetailedMip = 0;
    srvDesc.TextureCube.MipLevels = skyCubeMap->GetDesc().MipLevels;
    srvDesc.TextureCube.ResourceMinLODClamp = 0.0f;
    srvDesc.Format = skyCubeMap->GetDesc().Format;
    md3dDevice->CreateShaderResourceView(skyCubeMap.Get(), &srvDesc, hDescriptor);

    mSkyTexHeapIndex = 0;
    mShadowMapHeapIndex = mSkyTexHeapIndex + 1;
    mNullTexSrvIndex = mShadowMapHeapIndex + 1;

    auto srvCpuStart = mSrvDescriptorHeap->GetCPUDescriptorHandleForHeapStart();
    auto srvGpuStart = mSrvDescriptorHeap->GetGPUDescriptorHandleForHeapStart();
    auto dsvCpuStart = mDsvHeap->GetCPUDescriptorHandleForHeapStart();


    auto nullSrv = CD3DX12_CPU_DESCRIPTOR_HANDLE(srvCpuStart, mNullTexSrvIndex, mCbvSrvUavDescriptorSize);
    mNullSrv = CD3DX12_GPU_DESCRIPTOR_HANDLE(srvGpuStart, mNullTexSrvIndex, mCbvSrvUavDescriptorSize);

    srvDesc.ViewDimension = D3D12_SRV_DIMENSION_TEXTURE2D;
    srvDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
    srvDesc.Texture2D.MostDetailedMip = 0;
    srvDesc.Texture2D.MipLevels = 1;
    srvDesc.Texture2D.ResourceMinLODClamp = 0.0f;
    md3dDevice->CreateShaderResourceView(nullptr, &srvDesc, nullSrv);

    mShadowMap->BuildDescriptors(
        CD3DX12_CPU_DESCRIPTOR_HANDLE(srvCpuStart, mShadowMapHeapIndex, mCbvSrvUavDescriptorSize),
        CD3DX12_GPU_DESCRIPTOR_HANDLE(srvGpuStart, mShadowMapHeapIndex, mCbvSrvUavDescriptorSize),
        CD3DX12_CPU_DESCRIPTOR_HANDLE(dsvCpuStart, 1, mDsvDescriptorSize));


    //
    // Create Instancing Buffer SRV 
    //
    UINT nextSrvIndex = mNullTexSrvIndex + 1; // start frome Index 3

    for (int i = 0; i < gNumFrameResources; ++i)
    {
        FrameResource* fr = mFrameResources[i].get();

        // 1. Calculate SRV handle location
        CD3DX12_CPU_DESCRIPTOR_HANDLE hDescriptor(mSrvDescriptorHeap->GetCPUDescriptorHandleForHeapStart());
        hDescriptor.Offset(nextSrvIndex, mCbvSrvUavDescriptorSize);

        // 2. Store Index : this value will be utilized in Draw()
        fr->InstanceBufferSrvHeapIndex = nextSrvIndex;

        // 3. Setup SRV Descriptor
        D3D12_SHADER_RESOURCE_VIEW_DESC srvDesc = {};
        srvDesc.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING;
        srvDesc.ViewDimension = D3D12_SRV_DIMENSION_BUFFER;
        srvDesc.Format = DXGI_FORMAT_UNKNOWN; // Structured Buffer
        srvDesc.Buffer.FirstElement = 0;
        srvDesc.Buffer.NumElements = GeneralData::NumMaxInstanceCapacity; // MaxInstances
        srvDesc.Buffer.StructureByteStride = sizeof(InstanceData); // 160 bytes
        srvDesc.Buffer.Flags = D3D12_BUFFER_SRV_FLAG_NONE;

        // 4. Create SRV
        md3dDevice->CreateShaderResourceView(fr->InstanceBuffer->Resource(), &srvDesc, hDescriptor);

        ++nextSrvIndex;
    }
}
void PhysicsApplication::BuildShadersAndInputLayout()
{
    mShaders["standardVS"] = d3dUtil::CompileShader(L"Renderer\\Shaders\\Default.hlsl", nullptr, "VS", "vs_5_1");
    mShaders["opaquePS"] = d3dUtil::CompileShader(L"Renderer\\Shaders\\Default.hlsl", nullptr, "PS", "ps_5_1");

    mShaders["contactPointsVS"] = d3dUtil::CompileShader(L"Renderer\\Shaders\\ContactPoints.hlsl", nullptr, "VS", "vs_5_1");
    mShaders["contactPointsPS"] = d3dUtil::CompileShader(L"Renderer\\Shaders\\ContactPoints.hlsl", nullptr, "PS", "ps_5_1");

    mShaders["instancedVS"] = d3dUtil::CompileShader(L"Renderer\\Shaders\\Instancing.hlsl", nullptr, "VS_Instanced", "vs_5_1");
    mShaders["instancedGS"] = d3dUtil::CompileShader(L"Renderer\\Shaders\\Instancing.hlsl", nullptr, "GS_Culler", "gs_5_1");
    mShaders["instancedPS"] = d3dUtil::CompileShader(L"Renderer\\Shaders\\Instancing.hlsl", nullptr, "PS_Instanced", "ps_5_1");

    mInputLayout =
    {
        { "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
        { "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
        { "TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, 24, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
        { "TANGENT", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 32, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
    };
}
void PhysicsApplication::BuildPSOs()
{
    D3D12_GRAPHICS_PIPELINE_STATE_DESC opaquePsoDesc;
    //
    // PSO for opaque objects.
    //
    ZeroMemory(&opaquePsoDesc, sizeof(D3D12_GRAPHICS_PIPELINE_STATE_DESC));
    opaquePsoDesc.InputLayout = { mInputLayout.data(), (UINT)mInputLayout.size() };
    opaquePsoDesc.pRootSignature = mRootSignature.Get();
    opaquePsoDesc.VS =
    {
        reinterpret_cast<BYTE*>(mShaders["standardVS"]->GetBufferPointer()),
        mShaders["standardVS"]->GetBufferSize()
    };
    opaquePsoDesc.PS =
    {
        reinterpret_cast<BYTE*>(mShaders["opaquePS"]->GetBufferPointer()),
        mShaders["opaquePS"]->GetBufferSize()
    };
    opaquePsoDesc.RasterizerState = CD3DX12_RASTERIZER_DESC(D3D12_DEFAULT);
    opaquePsoDesc.BlendState = CD3DX12_BLEND_DESC(D3D12_DEFAULT);
    opaquePsoDesc.DepthStencilState = CD3DX12_DEPTH_STENCIL_DESC(D3D12_DEFAULT);
    opaquePsoDesc.SampleMask = UINT_MAX;
    opaquePsoDesc.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    opaquePsoDesc.NumRenderTargets = 1;
    opaquePsoDesc.RTVFormats[0] = mBackBufferFormat;
    opaquePsoDesc.SampleDesc.Count = m4xMsaaState ? 4 : 1;
    opaquePsoDesc.SampleDesc.Quality = m4xMsaaState ? (m4xMsaaQuality - 1) : 0;
    opaquePsoDesc.DSVFormat = mDepthStencilFormat;
    ThrowIfFailed(md3dDevice->CreateGraphicsPipelineState(&opaquePsoDesc, IID_PPV_ARGS(&mPSOs["opaque"])));



    // 
    // PSO for Contact Points
    // 
    D3D12_GRAPHICS_PIPELINE_STATE_DESC contactPointPsoDesc = opaquePsoDesc;
    contactPointPsoDesc.VS =
    {
        reinterpret_cast<BYTE*>(mShaders["contactPointsVS"]->GetBufferPointer()),
        mShaders["contactPointsVS"]->GetBufferSize()
    };
    contactPointPsoDesc.PS =
    {
        reinterpret_cast<BYTE*>(mShaders["contactPointsPS"]->GetBufferPointer()),
        mShaders["contactPointsPS"]->GetBufferSize()
    };
    // Since contact points are small, drawing only the front faces is usually fine, 
    // but disabling culling ensures visibility regardless of camera angle.
    contactPointPsoDesc.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;
    // Ensure it uses the same Input Layout, Root Signature, and Depth/Stencil setup as opaque objects
    // which it inherits from opaquePsoDesc.
    ThrowIfFailed(md3dDevice->CreateGraphicsPipelineState(&contactPointPsoDesc, IID_PPV_ARGS(&mPSOs["contactpoints"])));



    //
    // PSO for Wireframe Rendering (used for selected object)
    //
    // Copy the opaque PSO description first.
    D3D12_GRAPHICS_PIPELINE_STATE_DESC wireframePsoDesc = opaquePsoDesc;
    // Wireframe PSO with Culling Disabled (Recommended for better visibility of internal lines)
    D3D12_GRAPHICS_PIPELINE_STATE_DESC wireframeNoCullPsoDesc = wireframePsoDesc;
    wireframeNoCullPsoDesc.RasterizerState.FillMode = D3D12_FILL_MODE_WIREFRAME;
    wireframeNoCullPsoDesc.RasterizerState.CullMode = D3D12_CULL_MODE_NONE;
    ThrowIfFailed(md3dDevice->CreateGraphicsPipelineState(&wireframeNoCullPsoDesc, IID_PPV_ARGS(&mPSOs["wireframe"])));


    //
    // PSO for stress test (instancing applied)
    //
    // Copy the opaque PSO description first.
    D3D12_GRAPHICS_PIPELINE_STATE_DESC stressTestPsoDesc = opaquePsoDesc;
    stressTestPsoDesc.VS =
    {
        mShaders["instancedVS"]->GetBufferPointer(),
        mShaders["instancedVS"]->GetBufferSize()
    };
    stressTestPsoDesc.GS =
    {
        mShaders["instancedGS"]->GetBufferPointer(),
        mShaders["instancedGS"]->GetBufferSize()
    };
    stressTestPsoDesc.PS =
    {
        mShaders["instancedPS"]->GetBufferPointer(),
        mShaders["instancedPS"]->GetBufferSize()
    };
    ThrowIfFailed(md3dDevice->CreateGraphicsPipelineState(&stressTestPsoDesc, IID_PPV_ARGS(&mPSOs["instancing"])));




    //
    // PSO for Shadow Pass for Stress Test (Instancing)
    // -------------------------------------------------------------------
    // Copy the existing instancing PSO description as a base.
    D3D12_GRAPHICS_PIPELINE_STATE_DESC shadowStressPsoDesc = stressTestPsoDesc;

    // CRITICAL FIX: Match the state used in DrawSceneToShadowMap (OMSetRenderTargets(0, nullptr, ...))
    // AMD/ATI drivers strictly require the PSO's NumRenderTargets to match the command list state.
    shadowStressPsoDesc.NumRenderTargets = 0;
    shadowStressPsoDesc.RTVFormats[0] = DXGI_FORMAT_UNKNOWN;

    // OPTIMIZATION: Shadow pass only needs depth. We can disable the Pixel Shader for performance.
    // If your instanced GS/VS already handles position output, PS is not required for depth-only pass.
    shadowStressPsoDesc.PS = { nullptr, 0 };

    // Create the specialized PSO for Shadow Instancing.
    ThrowIfFailed(md3dDevice->CreateGraphicsPipelineState(&shadowStressPsoDesc, IID_PPV_ARGS(&mPSOs["shadow_instancing"])));
}

// sampler preset
std::array<const CD3DX12_STATIC_SAMPLER_DESC, 7> PhysicsApplication::GetStaticSamplers()
{
    // Applications usually only need a handful of samplers.  So just define them all up front
    // and keep them available as part of the root signature.  

    const CD3DX12_STATIC_SAMPLER_DESC pointWrap(
        0, // shaderRegister
        D3D12_FILTER_MIN_MAG_MIP_POINT, // filter
        D3D12_TEXTURE_ADDRESS_MODE_WRAP,  // addressU
        D3D12_TEXTURE_ADDRESS_MODE_WRAP,  // addressV
        D3D12_TEXTURE_ADDRESS_MODE_WRAP); // addressW

    const CD3DX12_STATIC_SAMPLER_DESC pointClamp(
        1, // shaderRegister
        D3D12_FILTER_MIN_MAG_MIP_POINT, // filter
        D3D12_TEXTURE_ADDRESS_MODE_CLAMP,  // addressU
        D3D12_TEXTURE_ADDRESS_MODE_CLAMP,  // addressV
        D3D12_TEXTURE_ADDRESS_MODE_CLAMP); // addressW

    const CD3DX12_STATIC_SAMPLER_DESC linearWrap(
        2, // shaderRegister
        D3D12_FILTER_MIN_MAG_MIP_LINEAR, // filter
        D3D12_TEXTURE_ADDRESS_MODE_WRAP,  // addressU
        D3D12_TEXTURE_ADDRESS_MODE_WRAP,  // addressV
        D3D12_TEXTURE_ADDRESS_MODE_WRAP); // addressW

    const CD3DX12_STATIC_SAMPLER_DESC linearClamp(
        3, // shaderRegister
        D3D12_FILTER_MIN_MAG_MIP_LINEAR, // filter
        D3D12_TEXTURE_ADDRESS_MODE_CLAMP,  // addressU
        D3D12_TEXTURE_ADDRESS_MODE_CLAMP,  // addressV
        D3D12_TEXTURE_ADDRESS_MODE_CLAMP); // addressW

    const CD3DX12_STATIC_SAMPLER_DESC anisotropicWrap(
        4, // shaderRegister
        D3D12_FILTER_ANISOTROPIC, // filter
        D3D12_TEXTURE_ADDRESS_MODE_WRAP,  // addressU
        D3D12_TEXTURE_ADDRESS_MODE_WRAP,  // addressV
        D3D12_TEXTURE_ADDRESS_MODE_WRAP,  // addressW
        0.0f,                             // mipLODBias
        8);                               // maxAnisotropy

    const CD3DX12_STATIC_SAMPLER_DESC anisotropicClamp(
        5, // shaderRegister
        D3D12_FILTER_ANISOTROPIC, // filter
        D3D12_TEXTURE_ADDRESS_MODE_CLAMP,  // addressU
        D3D12_TEXTURE_ADDRESS_MODE_CLAMP,  // addressV
        D3D12_TEXTURE_ADDRESS_MODE_CLAMP,  // addressW
        0.0f,                              // mipLODBias
        8);                                // maxAnisotropy

    const CD3DX12_STATIC_SAMPLER_DESC shadow(
        6, // shaderRegister
        D3D12_FILTER_COMPARISON_MIN_MAG_LINEAR_MIP_POINT, // filter
        D3D12_TEXTURE_ADDRESS_MODE_BORDER,  // addressU
        D3D12_TEXTURE_ADDRESS_MODE_BORDER,  // addressV
        D3D12_TEXTURE_ADDRESS_MODE_BORDER,  // addressW
        0.0f,                               // mipLODBias
        16,                                 // maxAnisotropy
        D3D12_COMPARISON_FUNC_LESS_EQUAL,
        D3D12_STATIC_BORDER_COLOR_OPAQUE_BLACK);

    return {
        pointWrap, pointClamp,
        linearWrap, linearClamp,
        anisotropicWrap, anisotropicClamp,
        shadow
    };
}

// camera
void PhysicsApplication::ResetCamera() {
    mCamera.SetPosition(INIT_CAMERA_POSITION);
    mCamera.ResetViewDirection();
    mCamera.UpdateViewMatrix();
}

// update
void PhysicsApplication::Update(const GameTimer& gt)
{
    // PIX - 1. Top-Level Marker: Measures the entire CPU frame time for logic
    PIXBeginEvent(PIX_COLOR(255, 255, 255), "Update_Total_Frame");

    // ------------
    // keyboard event
    OnKeyboardInput(gt);
    mCamera.UpdateViewMatrix();

    // Check if a new state has been requested from a button click
    if (mRequestedSceneState != mCurrentSceneState || mIsRestartNeeded == true) {
        // Cleanup old scene resources
        CleanupSceneResources();

        // Update the current application state
        mCurrentSceneState = mRequestedSceneState;

        // Set up the new scene
        // Reset command list for new resource upload
        if (mCurrentSceneState != SceneState::MAIN_MENU) {
            mAppPaused = true;

            GeometryGenerator geometryGenerator;
            ThrowIfFailed(mCommandList->Reset(mDirectCmdListAlloc.Get(), nullptr));
            switch (mCurrentSceneState) {
            case SceneState::STRESS_TEST:
                SetupDemoSceneStressTest(geometryGenerator);
                break;
            case SceneState::SANDBOX:
                SetupDemoSceneSandbox(geometryGenerator);
                break;
            }
            SaveCurrentFrameState();
            mCurrentFrameHistoryIndex = 1;
            // setup drawing contact points
            BuildGeometryContactPoints(geometryGenerator);

            // Execute the commands to upload the new geometry
            ThrowIfFailed(mCommandList->Close());
            ID3D12CommandList* cmdsLists[] = { mCommandList.Get() };
            mCommandQueue->ExecuteCommandLists(_countof(cmdsLists), cmdsLists);
            FlushCommandQueue();
        }
    }
    // ------------


    // Synchronize with GPU
    // ------------
    // Cycle through the circular frame resource array.
    // PIX - 2. Synchronization Marker: Checks how long CPU is waiting for GPU
    PIXBeginEvent(PIX_COLOR(128, 128, 128), "GPU_Sync_Wait");
    {
        mCurrFrameResourceIndex = (mCurrFrameResourceIndex + 1) % gNumFrameResources;
        mCurrFrameResource = mFrameResources[mCurrFrameResourceIndex].get();

        // Wait for the GPU if the current frame resource is in use.
        if (mCurrFrameResource->Fence != 0 && mFence->GetCompletedValue() < mCurrFrameResource->Fence)
        {
            HANDLE eventHandle = CreateEventEx(nullptr, false, false, EVENT_ALL_ACCESS);
            ThrowIfFailed(mFence->SetEventOnCompletion(mCurrFrameResource->Fence, eventHandle));
            WaitForSingleObject(eventHandle, INFINITE);
            CloseHandle(eventHandle);
        }
    }
    PIXEndEvent(); // End GPU_Sync_Wait
    // ------------



    // ------------
    // Update main data with ImGui
    // Begin a new ImGui frame. This is crucial for CPU-side UI data generation.
    ImGui_ImplDX12_NewFrame();
    ImGui_ImplWin32_NewFrame();
    ImGui::NewFrame();

    // Perform scene-specific updates and render the appropriate UI
    if (mCurrentSceneState == SceneState::MAIN_MENU) {
        // Render the Main Menu UI to check whether a button was clicked
        if (mLockTimer < 0.0f)
            RenderMainMenuUI();
        else {
            RenderDemoUILoading();
            mLockTimer -= mTimer.DeltaTime();
        }
    }
    else {
        // Render the UI for the demo scene
        if (mIsRestartNeeded) {
            mIsRestartNeeded = false;
            mLockTimer = GeneralData::LockTime;
        }

        if (mLockTimer < 0.0f) {
            RenderDemoUICommon();
            RenderDemoUIVisualDebugger();

            switch (mCurrentSceneState) {
            case SceneState::STRESS_TEST: RenderDemoUIStressTest(); break;
            case SceneState::SANDBOX: RenderDemoUISandBox(); break;
            }
        }
        else {
            RenderDemoUILoading();
            mLockTimer -= mTimer.DeltaTime();
        }


        // Update physics and scene data for the active demo only if the engine is not paused or OneFrame button clicked
        if (mAppPaused == false)
            UpdatePositionAndOrientation(gt.DeltaTime());

        // draw gizmo if an object is picked
        RenderDemoUIGizmo();

        // apply physics on picked item
        if (mPickedRenderItem != nullptr)
            ApplyPhysicsOnPickedItem();
    }

    // Finalize the ImGui frame data. It is now ready for rendering on the GPU.
    ImGui::Render();
    // ------------



    // update collision bodies
    mCollisionBodies.clear();
    if (mIsWireframeDebugEnabled)
    {
        for (const auto& contact : mContactPoints)
        {
            mCollisionBodies.insert(contact.bodyA);
            mCollisionBodies.insert(contact.bodyB);
        }
    }

    // Update GPU stuff
    // ------------
    // PIX - 3. ConstantBuffer-Update Marker: Checks how long it takes to update constant buffers
    PIXBeginEvent(PIX_COLOR(0, 128, 0), "Update_ConstantBuffers");
    {
        if (mCurrentSceneState == SceneState::STRESS_TEST)
            UpdateInstanceData(gt);
        UpdateObjectCBs();
        UpdateMaterialBuffer(gt);
        UpdateShadowTransform(gt);
        UpdateMainPassCB(gt);
        UpdateShadowPassCB(gt);
    }
    PIXEndEvent(); // End Update_ConstantBuffers
    // ------------

    PIXEndEvent(); // End Update_Total_Frame
}

// update gpu stuff
void PhysicsApplication::UpdateInstanceData(const GameTimer& gt)
{
    FrameResource* currFrame = mCurrFrameResource;

    // --- OPTIMIZATION: Use the persistent member variable ---
    mInstanceDataCache.clear();
    size_t numInstances = mAllRitems.size() - GeneralData::NumFloorSphereCount;
    if (mInstanceDataCache.capacity() < numInstances)
    {
        mInstanceDataCache.reserve(numInstances); // Resize only if needed.
    }

    // --- 1. Gather Data from Render Items/Physics Objects ---
    // iterate all objects except for floor objects
    for (size_t i = 0; i < numInstances; ++i)
    {
        const auto& ritem = mAllRitems[i];
        InstanceData data;

        XMMATRIX world = XMLoadFloat4x4(&ritem->World);
        XMStoreFloat4x4(&data.World, XMMatrixTranspose(world));
        XMStoreFloat4x4(&data.TexTransform, XMLoadFloat4x4(&ritem->TexTransform));

        // Color and Material Index (for use in the shader)
        data.Color = ritem->Mat->DiffuseAlbedo;
        data.MaterialIndex = ritem->Mat->MatCBIndex;

        // flag for wireframe
        if (mCollisionBodies.count(&mBodies[i]) > 0)
            data.IsCollisionBody = 1;
        else
            data.IsCollisionBody = 0;


        mInstanceDataCache.push_back(data);
    }
    UINT actualInstanceCount = (UINT)mInstanceDataCache.size();

    // --- 2. Copy Data to the Upload Buffer ---
    // Copy all collected instance data to the mapped GPU memory.
    // The InstanceBuffer (UploadBuffer) is assumed to be correctly mapped (mMappedData is valid).
    for (size_t i = 0; i < actualInstanceCount; ++i)
    {
        // CopyData handles the offset: mMappedData[i * mElementByteSize]
        currFrame->InstanceBuffer->CopyData((int)i, mInstanceDataCache[i]);
    }
}
void PhysicsApplication::UpdateObjectCBs()
{
    auto currObjectCB = mCurrFrameResource->ObjectCB.get();

    if (mCurrentSceneState == SceneState::STRESS_TEST) 
    {
        // update floor
        if (mAllRitems[mAllRitems.size() - 1].get()->NumFramesDirty > 0)
        {
            for (size_t currentIndex = mAllRitems.size() - GeneralData::NumFloorSphereCount, endIndex = mAllRitems.size(); currentIndex < endIndex; ++currentIndex)
            {
                auto e = mAllRitems[currentIndex].get();
                XMMATRIX world = XMLoadFloat4x4(&e->World);
                XMMATRIX texTransform = XMLoadFloat4x4(&e->TexTransform);

                ObjectConstants objConstants;
                XMStoreFloat4x4(&objConstants.World, XMMatrixTranspose(world));
                XMStoreFloat4x4(&objConstants.TexTransform, XMMatrixTranspose(texTransform));
                objConstants.MaterialIndex = e->Mat->MatCBIndex;

                currObjectCB->CopyData(e->ObjCBIndex, objConstants);

                // Next FrameResource need to be updated too.
                e->NumFramesDirty--;
            }
        }
        
        // update collision bodies
        for (auto currentBody : mCollisionBodies)
        {
            auto e = mAllRitems[currentBody->m_id].get();
            XMMATRIX world = XMLoadFloat4x4(&e->World);
            XMMATRIX texTransform = XMLoadFloat4x4(&e->TexTransform);

            ObjectConstants objConstants;
            XMStoreFloat4x4(&objConstants.World, XMMatrixTranspose(world));
            XMStoreFloat4x4(&objConstants.TexTransform, XMMatrixTranspose(texTransform));
            objConstants.MaterialIndex = e->Mat->MatCBIndex;

            currObjectCB->CopyData(e->ObjCBIndex, objConstants);

            // Next FrameResource need to be updated too.
            e->NumFramesDirty--;
        }

        return;
    }

    for (auto& e : mAllRitems)
    {
        // Only update the cbuffer data if the constants have changed.  
        // This needs to be tracked per frame resource.
        if (e->NumFramesDirty > 0)
        {
            XMMATRIX world = XMLoadFloat4x4(&e->World);
            XMMATRIX texTransform = XMLoadFloat4x4(&e->TexTransform);

            ObjectConstants objConstants;
            XMStoreFloat4x4(&objConstants.World, XMMatrixTranspose(world));
            XMStoreFloat4x4(&objConstants.TexTransform, XMMatrixTranspose(texTransform));
            objConstants.MaterialIndex = e->Mat->MatCBIndex;

            currObjectCB->CopyData(e->ObjCBIndex, objConstants);

            // Next FrameResource need to be updated too.
            e->NumFramesDirty--;
        }
    }
}
void PhysicsApplication::UpdateMaterialBuffer(const GameTimer& gt)
{
    auto currMaterialBuffer = mCurrFrameResource->MaterialBuffer.get();
    for (auto& e : mMaterials)
    {
        // Only update the cbuffer data if the constants have changed.  If the cbuffer
        // data changes, it needs to be updated for each FrameResource.
        Material* mat = e.second.get();
        if (mat->NumFramesDirty > 0)
        {
            XMMATRIX matTransform = XMLoadFloat4x4(&mat->MatTransform);

            MaterialData matData;
            matData.DiffuseAlbedo = mat->DiffuseAlbedo;
            matData.FresnelR0 = mat->FresnelR0;
            matData.Roughness = mat->Roughness;
            XMStoreFloat4x4(&matData.MatTransform, XMMatrixTranspose(matTransform));
            matData.DiffuseMapIndex = mat->DiffuseSrvHeapIndex;
            matData.NormalMapIndex = mat->NormalSrvHeapIndex;

            currMaterialBuffer->CopyData(mat->MatCBIndex, matData);

            // Next FrameResource need to be updated too.
            mat->NumFramesDirty--;
        }
    }
}
void PhysicsApplication::UpdateShadowTransform(const GameTimer& gt)
{
    // Only the first "main" light casts a shadow.
    XMVECTOR lightDir = XMLoadFloat3(&mBaseLightDirections[1]);
    XMVECTOR lightPos = -1.0f * mSceneBounds.Radius / 2.0f * lightDir;
    XMVECTOR targetPos = XMLoadFloat3(&mSceneBounds.Center);
    XMVECTOR lightUp = XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);
    XMMATRIX lightView = XMMatrixLookAtLH(lightPos, targetPos, lightUp);

    XMStoreFloat3(&mLightPosW, lightPos);

    // Transform bounding sphere to light space.
    XMFLOAT3 sphereCenterLS;
    XMStoreFloat3(&sphereCenterLS, XMVector3TransformCoord(targetPos, lightView));

    // Ortho frustum in light space encloses scene.
    float l = sphereCenterLS.x - mSceneBounds.Radius;
    float b = sphereCenterLS.y - mSceneBounds.Radius;
    float n = sphereCenterLS.z - mSceneBounds.Radius;
    float r = sphereCenterLS.x + mSceneBounds.Radius;
    float t = sphereCenterLS.y + mSceneBounds.Radius;
    float f = sphereCenterLS.z + mSceneBounds.Radius;

    mLightNearZ = n;
    mLightFarZ = f;
    XMMATRIX lightProj = XMMatrixOrthographicOffCenterLH(l, r, b, t, n, f);

    // Transform NDC space [-1,+1]^2 to texture space [0,1]^2
    XMMATRIX T(
        0.5f, 0.0f, 0.0f, 0.0f,
        0.0f, -0.5f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.5f, 0.5f, 0.0f, 1.0f);

    XMMATRIX S = lightView * lightProj * T;
    XMStoreFloat4x4(&mLightView, lightView);
    XMStoreFloat4x4(&mLightProj, lightProj);
    XMStoreFloat4x4(&mShadowTransform, S);
}
void PhysicsApplication::UpdateMainPassCB(const GameTimer& gt)
{
    XMMATRIX view = mCamera.GetView();
    XMMATRIX proj = mCamera.GetProj();

    XMMATRIX viewProj = XMMatrixMultiply(view, proj);
    XMMATRIX invView = XMMatrixInverse(&XMMatrixDeterminant(view), view);
    XMMATRIX invProj = XMMatrixInverse(&XMMatrixDeterminant(proj), proj);
    XMMATRIX invViewProj = XMMatrixInverse(&XMMatrixDeterminant(viewProj), viewProj);

    XMMATRIX shadowTransform = XMLoadFloat4x4(&mShadowTransform);

    XMStoreFloat4x4(&mMainPassCB.View, XMMatrixTranspose(view));
    XMStoreFloat4x4(&mMainPassCB.InvView, XMMatrixTranspose(invView));
    XMStoreFloat4x4(&mMainPassCB.Proj, XMMatrixTranspose(proj));
    XMStoreFloat4x4(&mMainPassCB.InvProj, XMMatrixTranspose(invProj));
    XMStoreFloat4x4(&mMainPassCB.ViewProj, XMMatrixTranspose(viewProj));
    XMStoreFloat4x4(&mMainPassCB.InvViewProj, XMMatrixTranspose(invViewProj));
    XMStoreFloat4x4(&mMainPassCB.ShadowTransform, XMMatrixTranspose(shadowTransform));
    mMainPassCB.EyePosW = mCamera.GetPosition3f();
    mMainPassCB.RenderTargetSize = XMFLOAT2((float)mClientWidth, (float)mClientHeight);
    mMainPassCB.InvRenderTargetSize = XMFLOAT2(1.0f / mClientWidth, 1.0f / mClientHeight);
    mMainPassCB.NearZ = 1.0f;
    mMainPassCB.FarZ = 1000.0f;
    mMainPassCB.TotalTime = gt.TotalTime();
    mMainPassCB.DeltaTime = gt.DeltaTime();
    mMainPassCB.AmbientLight = { 0.25f, 0.25f, 0.35f, 1.0f };
    mMainPassCB.Lights[0].Direction = mBaseLightDirections[0];
    mMainPassCB.Lights[0].Strength = { 0.9f, 0.8f, 0.7f };
    mMainPassCB.Lights[1].Direction = mBaseLightDirections[1];
    mMainPassCB.Lights[1].Strength = { 0.4f, 0.4f, 0.4f };
    mMainPassCB.Lights[2].Direction = mBaseLightDirections[2];
    mMainPassCB.Lights[2].Strength = { 0.2f, 0.2f, 0.2f };

    auto currPassCB = mCurrFrameResource->PassCB.get();
    currPassCB->CopyData(0, mMainPassCB);
}
void PhysicsApplication::UpdateShadowPassCB(const GameTimer& gt)
{
    XMMATRIX view = XMLoadFloat4x4(&mLightView);
    XMMATRIX proj = XMLoadFloat4x4(&mLightProj);

    XMMATRIX viewProj = XMMatrixMultiply(view, proj);
    XMMATRIX invView = XMMatrixInverse(&XMMatrixDeterminant(view), view);
    XMMATRIX invProj = XMMatrixInverse(&XMMatrixDeterminant(proj), proj);
    XMMATRIX invViewProj = XMMatrixInverse(&XMMatrixDeterminant(viewProj), viewProj);

    UINT w = mShadowMap->Width();
    UINT h = mShadowMap->Height();

    XMStoreFloat4x4(&mShadowPassCB.View, XMMatrixTranspose(view));
    XMStoreFloat4x4(&mShadowPassCB.InvView, XMMatrixTranspose(invView));
    XMStoreFloat4x4(&mShadowPassCB.Proj, XMMatrixTranspose(proj));
    XMStoreFloat4x4(&mShadowPassCB.InvProj, XMMatrixTranspose(invProj));
    XMStoreFloat4x4(&mShadowPassCB.ViewProj, XMMatrixTranspose(viewProj));
    XMStoreFloat4x4(&mShadowPassCB.InvViewProj, XMMatrixTranspose(invViewProj));
    mShadowPassCB.EyePosW = mLightPosW;
    mShadowPassCB.RenderTargetSize = XMFLOAT2((float)w, (float)h);
    mShadowPassCB.InvRenderTargetSize = XMFLOAT2(1.0f / w, 1.0f / h);
    mShadowPassCB.NearZ = mLightNearZ;
    mShadowPassCB.FarZ = mLightFarZ;

    auto currPassCB = mCurrFrameResource->PassCB.get();
    currPassCB->CopyData(1, mShadowPassCB);
}


// physics main loop
void PhysicsApplication::UpdatePositionAndOrientation(const float deltaSecond) {
    // PIX - 0. Main-Loop Marker
    PIXBeginEvent(PIX_COLOR(255, 0, 0), "Physics_Main_Loop");

    m_manifolds.RemoveExpired();

    // PIX - 1. Gravity Marker
    PIXBeginEvent(PIX_COLOR_DEFAULT, "Physics_Step1_Gravity");
    for (int currentBodyIndex = 0; currentBodyIndex < mBodies.size(); ++currentBodyIndex) {
        Body* currentBody = &mBodies[currentBodyIndex];
        if (currentBody->m_invMass == 0.0f) continue; // Optimization: Skip statics

        float mass = 1.0f / currentBody->m_invMass;
        Vec3 impulseGravity = Vec3(0, 0, -10) * mass * deltaSecond;
        currentBody->ApplyImpulseLinear(impulseGravity);
    }
    PIXEndEvent();  // End Physics_Step1_Gravity

    // Use vector to prevent Stack Overflow during stress tests
    std::vector<contact_t> contacts;

    // PIX - 2. Collision-Detection Marker (Moved OUTSIDE if/else)
    PIXBeginEvent(PIX_COLOR_DEFAULT, "Physics_Step2_BroadNarrowPhase");
    {
        if (mIsBroadNarrowOn == true) {
            // BroadPhase
            std::vector<collisionPair_t> collisionPairs;
            BroadPhase(mBodies.data(), static_cast<int>(mBodies.size()), collisionPairs, deltaSecond);

            // Reserve memory to avoid reallocations
            contacts.reserve(collisionPairs.size());

            // NarrowPhase
            for (const auto& currentPair : collisionPairs) {
                Body* bodyA = &mBodies[currentPair.a];
                Body* bodyB = &mBodies[currentPair.b];

                if (0.0f == bodyA->m_invMass && 0.0f == bodyB->m_invMass)
                    continue;

                contact_t contact;
                if (DoesIntersect(bodyA, bodyB, deltaSecond, contact)) {
                    if (contact.timeOfImpact == 0.0f) {
                        // static contact
                        m_manifolds.AddContact(contact);
                    }
                    else {
                        // dynamic contact
                        contacts.push_back(contact);
                    }
                }
            }
        }
        else {
            // Brute force
            contacts.reserve(mBodies.size()); // Approximation

            for (int currentBodyA = 0; currentBodyA < mBodies.size(); ++currentBodyA) {
                for (int currentBodyB = currentBodyA + 1; currentBodyB < mBodies.size(); currentBodyB++) {
                    Body* bodyA = &mBodies[currentBodyA];
                    Body* bodyB = &mBodies[currentBodyB];

                    if (0.0f == bodyA->m_invMass && 0.0f == bodyB->m_invMass)
                        continue;

                    contact_t contact;
                    if (DoesIntersect(bodyA, bodyB, deltaSecond, contact)) {
                        if (contact.timeOfImpact == 0.0f) {
                            // static contact
                            m_manifolds.AddContact(contact);
                        }
                        else {
                            // dynamic contact
                            contacts.push_back(contact);
                        }
                    }
                }
            }
        }
    }
    PIXEndEvent();  // End Physics_Step2_BroadNarrowPhase


    // PIX - 3. Solver Marker
    PIXBeginEvent(PIX_COLOR_DEFAULT, "Physics_Step3_Solver");
    float accumulatedTime = 0.0f;
    {
        // sort times of impact
        if (contacts.size() > 1)
            std::sort(contacts.begin(), contacts.end(), [](const contact_t& a, const contact_t& b) {
            return a.timeOfImpact < b.timeOfImpact;
                });

        // resolve constraints
        for (auto* currentConstraint : mConstraints) 
            currentConstraint->PreSolve(deltaSecond);
        m_manifolds.PreSolve(deltaSecond);

        const int maxIterationCount = 5;
        // apply iterative approach
        for (int i = 0; i < maxIterationCount; ++i) {
            for (auto* currentConstraint : mConstraints)
                currentConstraint->Solve();
            m_manifolds.Solve();
        }

        for (auto* currentConstraint : mConstraints)
            currentConstraint->PostSolve();
        m_manifolds.PostSolve();

        // resolve collisions
        // note that there's no recalculation of earlier collisions for later ones to improve performance.
        // thus, while the first collision is handled correctly, later collisions may be processed improperly if they are related to the earlier collisions.
        for (int currentContactIndex = 0; currentContactIndex < contacts.size(); ++currentContactIndex) {
            contact_t& contact = contacts[currentContactIndex];
            const float deltaTime = contact.timeOfImpact - accumulatedTime;

            // PERFORMANCE HIT: Updating ALL bodies for EVERY contact
            for (auto& currentBody : mBodies) {
                // position update
                const auto previousPosition = currentBody.m_position;
                currentBody.Update(deltaTime);

                // Flagging dirty frames
                auto difVector = previousPosition - currentBody.m_position;
                if (difVector.GetLengthSqr() > 1e-9f) // Optimized float check
                    mAllRitems[currentBody.m_id].get()->NumFramesDirty = gNumFrameResources;
            }

            ResolveContact(contact);
            accumulatedTime += deltaTime;
        }
    }
    PIXEndEvent();  // End Physics_Step3_Solver


    // PIX - 4. State-Update Marker
    PIXBeginEvent(PIX_COLOR_DEFAULT, "Physics_Step4_StateUpdate");
    // update the positions for the rest of this frame's time
    {
        const float timeRemaining = deltaSecond - accumulatedTime;
        if (timeRemaining > 0.0f) {
            for (auto& currentBody : mBodies) {
                const auto previousPosition = currentBody.m_position;
                currentBody.Update(timeRemaining);

                auto difVector = previousPosition - currentBody.m_position;
                if (difVector.GetLengthSqr() > 1e-9f)
                    mAllRitems[currentBody.m_id].get()->NumFramesDirty = gNumFrameResources;
            }
        }
    }
    PIXEndEvent();  // End Physics_Step4_StateUpdate


    // PIX - 5. Render-Sync Marker
    PIXBeginEvent(PIX_COLOR_DEFAULT, "Physics_Step5_RenderSync");
    {
        // redraw items that are related to constraints or manifolds
        for (auto* currentConstraint : mConstraints) {
            mAllRitems[currentConstraint->m_bodyA->m_id]->NumFramesDirty = gNumFrameResources;
            mAllRitems[currentConstraint->m_bodyB->m_id]->NumFramesDirty = gNumFrameResources;
        }
        for (const auto& currentManifold : m_manifolds.m_manifolds) {
            mAllRitems[currentManifold.m_bodyA->m_id]->NumFramesDirty = gNumFrameResources;
            mAllRitems[currentManifold.m_bodyB->m_id]->NumFramesDirty = gNumFrameResources;
        }

        // Rebuild matrices
        // Using direct loop logic for better readability
        const size_t END_INDEX = mBodies.size() - GeneralData::NumSandboxGroundBodies;
        for (size_t currentBodyIndex = 0; currentBodyIndex < mBodies.size(); ++currentBodyIndex) {
            // Check condition for Sandbox/Stress Test
            bool shouldRender = (mRequestedSceneState == SceneState::STRESS_TEST) ||
                (mRequestedSceneState == SceneState::SANDBOX && currentBodyIndex <= END_INDEX);

            if (mAllRitems[currentBodyIndex]->NumFramesDirty > 0 && shouldRender) {
                // SIMD Optimization: Load directly into registers if possible in future
                Body& currentBody = mBodies[currentBodyIndex];
                XMVECTOR orientationVector = XMVectorSet(currentBody.m_orientation.x, currentBody.m_orientation.z, -currentBody.m_orientation.y, currentBody.m_orientation.w);
                XMVECTOR positionVector = XMVectorSet(currentBody.m_position.x, currentBody.m_position.z, -currentBody.m_position.y, 0.0f);

                XMMATRIX world = XMMatrixRotationQuaternion(orientationVector) * XMMatrixTranslationFromVector(positionVector);
                XMStoreFloat4x4(&mAllRitems[currentBodyIndex]->World, world);
            }
        }
        SaveCurrentFrameState();
    }
    PIXEndEvent();  // End Physics_Step5_RenderSync

    PIXEndEvent(); // End Physics_Main_Loop
}

// another physics method for picked item
void PhysicsApplication::ApplyPhysicsOnPickedItem() {
    ManifoldCollector manifoldCollector;

    const auto maxContacts = mBodies.size() * mBodies.size();
    contact_t* contacts = reinterpret_cast<contact_t*>(alloca(sizeof(contact_t) * maxContacts));

    int numContacts = 0;
    mContactPoints.clear();
    mRenderItemsContactPoints.clear();
    for (int i = 0; i < mBodies.size(); i++) {
        for (int j = i + 1; j < mBodies.size(); j++) {
            Body* bodyA = &mBodies[i];
            Body* bodyB = &mBodies[j];

            // skip body pairs with infinite mass
            if (0.0f == bodyA->m_invMass && 0.0f == bodyB->m_invMass)
                continue;

            // check for intersection
            contact_t contact;
            if (DoesIntersect(bodyA, bodyB, GeneralData::FixedDeltaTime, contact)) {
                if (bodyA->m_id == mPickedRenderItem->ObjCBIndex || bodyB->m_id == mPickedRenderItem->ObjCBIndex)
                    mContactPoints.push_back(contact);

                if (contact.timeOfImpact == 0.0f) {
                    // static contact
                    // handled by the penetration constraint
                    manifoldCollector.AddContact(contact);
                }
                else {
                    // dynamic contact
                    contacts[numContacts] = contact;
                    ++numContacts;
                }
            }
        }
    }
    BuildRenderItemsContactPoints();


    // sort times of impact from earliest to latest
    if (numContacts > 1)
    qsort(contacts, numContacts, sizeof(contact_t), CompareContacts);


    manifoldCollector.PreSolve(GeneralData::FixedDeltaTime);
    const int maxIterationCount = 5;
    for (int currentIterativeCount = 0; currentIterativeCount < maxIterationCount; ++currentIterativeCount) {
        manifoldCollector.Solve();
    }
    manifoldCollector.PostSolve();


    // resolve collisions
    // note that there's no recalculation of earlier collisions for later ones to improve performance.
    // thus, while the first collision is handled correctly, later collisions may be processed improperly if they are related to the earlier collisions.
    float accumulatedTime = 0.0f;
    for (int currentContactIndex = 0; currentContactIndex < numContacts; ++currentContactIndex) {
        contact_t& contact = contacts[currentContactIndex];
        const float deltaTime = contact.timeOfImpact - accumulatedTime;

        // position update
        for (auto& currentBody : mBodies) {
            const auto previousPosition = currentBody.m_position;
            currentBody.Update(deltaTime);
            auto difVector = previousPosition - currentBody.m_position;
            if (difVector.GetLengthSqr() > 0.00000000001f)
                mAllRitems[currentBody.m_id].get()->NumFramesDirty = gNumFrameResources;
        }

        ResolveContact(contact);
        accumulatedTime += deltaTime;
    }

    // update the positions for the rest of this frame's time
    const float timeRemaining = GeneralData::FixedDeltaTime - accumulatedTime;
    if (timeRemaining > 0.0f) {
        for (auto& currentBody : mBodies) {
            const auto previousPosition = currentBody.m_position;
            currentBody.Update(timeRemaining);
            auto difVector = previousPosition - currentBody.m_position;
            if (difVector.GetLengthSqr() > 0.00000000001f)
                mAllRitems[currentBody.m_id].get()->NumFramesDirty = gNumFrameResources;
        }
    }

    // reset velocities again
    for (auto& currentBody : mBodies) {
        currentBody.m_linearVelocity.x = 0.0f;
        currentBody.m_linearVelocity.y = 0.0f;
        currentBody.m_linearVelocity.z = 0.0f;

        currentBody.m_angularVelocity.x = 0.0f;
        currentBody.m_angularVelocity.y = 0.0f;
        currentBody.m_angularVelocity.z = 0.0f;
    }

    // rebuild the render items if it moves
    unsigned currentIndex = 0;
    for (auto& currentRenderItem : mAllRitems) {
        if (currentRenderItem->NumFramesDirty > 0) {
            // Convert the custom Quat to an XMVECTOR.
            XMVECTOR orientationVector = XMVectorSet(mBodies[currentIndex].m_orientation.x,
                mBodies[currentIndex].m_orientation.z,
                -mBodies[currentIndex].m_orientation.y,
                mBodies[currentIndex].m_orientation.w);
            // Create the rotation matrix from the orientation vector.
            XMMATRIX rotationMatrix = XMMatrixRotationQuaternion(orientationVector);

            XMVECTOR positionVector = XMVectorSet(mBodies[currentIndex].m_position.x,
                mBodies[currentIndex].m_position.z,
                -mBodies[currentIndex].m_position.y,
                0.0f);
            XMMATRIX translationMatrix = XMMatrixTranslationFromVector(positionVector);

            // Combine the rotation and translation matrices.
            XMMATRIX currentWorldMatrix = rotationMatrix * translationMatrix;

            XMStoreFloat4x4(&mAllRitems[currentIndex].get()->World, currentWorldMatrix);
        }
        ++currentIndex;
    }

}


// pause/resume
void PhysicsApplication::PauseEngine() {
    mAppPaused = true;
    //mTimer.Stop();    // we don't actually stop, since paused scene should be interactive

    mCurrentFrameHistoryIndex = mCurrentAvailableFrameHistoryIndex;
    
    // and set them as 0
    for (size_t currentIndex = 0, endIndex = mBodies.size(); currentIndex < endIndex; ++currentIndex) {
        mBodies[currentIndex].m_linearVelocity.x = 0.0f;
        mBodies[currentIndex].m_linearVelocity.y = 0.0f;
        mBodies[currentIndex].m_linearVelocity.z = 0.0f;

        mBodies[currentIndex].m_angularVelocity.x = 0.0f;
        mBodies[currentIndex].m_angularVelocity.y = 0.0f;
        mBodies[currentIndex].m_angularVelocity.z = 0.0f;
    }
}
void PhysicsApplication::ResumeEngine() {
    mAppPaused = false;
    ResetPickedItem();
    mIsWireframeDebugEnabled = false;

    // restore original velocities
    LoadFrameState(mCurrentAvailableFrameHistoryIndex - 1);
}


// reset
void PhysicsApplication::CleanupSceneResources() {
    // Wait for the GPU to finish all commands before deleting resources.
    FlushCommandQueue();

    // Release physics data
    for (auto& body : mBodies) {
        delete body.m_shape;
    }
    mBodies.clear();
    for (auto& constraint : mConstraints) {
        delete constraint;
    }
    mConstraints.clear();

    // Release DX12 resources
    mGeometries.clear();
    mAllRitems.clear();
    mRitemLayer[(int)RenderLayer::Opaque].clear();
    ResetPickedItem();
    geometryStartEndIndices.clear();


    // reset other data
    m_manifolds.Clear();

    mCurrentAvailableFrameHistoryIndex = 0;
    mFrameHistory.clear();

    mContactPoints.clear();
    mRenderItemsContactPoints.clear();

    mPickedContactPointIndex = -1;

    // reset flags
    mIsHitResultVisible = true;
    mIsBroadNarrowOn = true;
}

// draw
void PhysicsApplication::Draw(const GameTimer& gt)
{
    // PIX Start
    PIXBeginEvent(mCommandQueue.Get(), PIX_COLOR(255, 0, 0), "Draw");

    auto cmdListAlloc = mCurrFrameResource->CmdListAlloc;
    ThrowIfFailed(cmdListAlloc->Reset());
    ThrowIfFailed(mCommandList->Reset(cmdListAlloc.Get(), mPSOs["opaque"].Get()));

    ID3D12DescriptorHeap* descriptorHeaps[] = { mSrvDescriptorHeap.Get() };
    mCommandList->SetDescriptorHeaps(_countof(descriptorHeaps), descriptorHeaps);
    mCommandList->SetGraphicsRootSignature(mRootSignature.Get());

    // --- REMOVED: InstanceBuffer Barriers ---
    // Since InstanceBuffer is an UploadBuffer, we do NOT transition it. 
    // It is already in the required GENERIC_READ state.

    // --- COMMON BUFFER BINDING ---
    auto matBuffer = mCurrFrameResource->MaterialBuffer->Resource();
    mCommandList->SetGraphicsRootShaderResourceView(2, matBuffer->GetGPUVirtualAddress());
    mCommandList->SetGraphicsRootDescriptorTable(3, mNullSrv);
    mCommandList->SetGraphicsRootDescriptorTable(4, mSrvDescriptorHeap->GetGPUDescriptorHandleForHeapStart());

    // --- SHADOW MAP PASS ---
    DrawSceneToShadowMap();

    mCommandList->RSSetViewports(1, &mScreenViewport);
    mCommandList->RSSetScissorRects(1, &mScissorRect);

    // Transition Back Buffer (Default Heap) - THIS IS VALID
    mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(CurrentBackBuffer(),
        D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_RENDER_TARGET));

    mCommandList->ClearRenderTargetView(CurrentBackBufferView(), Colors::LightSteelBlue, 0, nullptr);
    mCommandList->ClearDepthStencilView(DepthStencilView(), D3D12_CLEAR_FLAG_DEPTH | D3D12_CLEAR_FLAG_STENCIL, 1.0f, 0, 0, nullptr);
    mCommandList->OMSetRenderTargets(1, &CurrentBackBufferView(), true, &DepthStencilView());

    auto passCB = mCurrFrameResource->PassCB->Resource();
    mCommandList->SetGraphicsRootConstantBufferView(1, passCB->GetGPUVirtualAddress());

    CD3DX12_GPU_DESCRIPTOR_HANDLE skyTexDescriptor(mSrvDescriptorHeap->GetGPUDescriptorHandleForHeapStart());
    skyTexDescriptor.Offset(mSkyTexHeapIndex, mCbvSrvUavDescriptorSize);
    mCommandList->SetGraphicsRootDescriptorTable(3, skyTexDescriptor);

    // --- TWO-PASS RENDERING LOGIC ---
    DrawRenderItems(mCommandList.Get(), mRitemLayer[(int)RenderLayer::Opaque], mCollisionBodies, false);

    if (mIsWireframeDebugEnabled && !mCollisionBodies.empty())
    {
        mCommandList->SetPipelineState(mPSOs["wireframe"].Get());
        DrawRenderItems(mCommandList.Get(), mRitemLayer[(int)RenderLayer::Opaque], mCollisionBodies, true);
    }

    if (mPickedRenderItem != nullptr && mIsHitResultVisible == true)
        DrawContactPoints(mCommandList.Get(), &mBodies[mPickedRenderItem->ObjCBIndex]);

    // --- IMGUI ---
    ImGui_ImplDX12_RenderDrawData(ImGui::GetDrawData(), mCommandList.Get());

    // Transition Back Buffer: RT -> PRESENT
    mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(CurrentBackBuffer(),
        D3D12_RESOURCE_STATE_RENDER_TARGET, D3D12_RESOURCE_STATE_PRESENT));

    ThrowIfFailed(mCommandList->Close());
    ID3D12CommandList* cmdsLists[] = { mCommandList.Get() };
    mCommandQueue->ExecuteCommandLists(_countof(cmdsLists), cmdsLists);

    PIXEndEvent(mCommandQueue.Get());

    ThrowIfFailed(mSwapChain->Present(0, 0));
    mCurrBackBuffer = (mCurrBackBuffer + 1) % SwapChainBufferCount;
    mCurrFrameResource->Fence = ++mCurrentFence;
    mCommandQueue->Signal(mFence.Get(), mCurrentFence);
}
void PhysicsApplication::DrawSceneToShadowMap()
{
    mCommandList->RSSetViewports(1, &mShadowMap->Viewport());
    mCommandList->RSSetScissorRects(1, &mShadowMap->ScissorRect());

    // Transition Shadow Map (Default Heap) - THIS IS VALID
    mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(mShadowMap->Resource(),
        D3D12_RESOURCE_STATE_GENERIC_READ, D3D12_RESOURCE_STATE_DEPTH_WRITE));

    UINT passCBByteSize = d3dUtil::CalcConstantBufferByteSize(sizeof(PassConstants));

    mCommandList->ClearDepthStencilView(mShadowMap->Dsv(),
        D3D12_CLEAR_FLAG_DEPTH | D3D12_CLEAR_FLAG_STENCIL, 1.0f, 0, 0, nullptr);

    mCommandList->OMSetRenderTargets(0, nullptr, false, &mShadowMap->Dsv());

    auto passCB = mCurrFrameResource->PassCB->Resource();
    D3D12_GPU_VIRTUAL_ADDRESS passCBAddress = passCB->GetGPUVirtualAddress() + 1 * passCBByteSize;
    mCommandList->SetGraphicsRootConstantBufferView(1, passCBAddress);

    // Ensure correct PSO for Instancing in Stress Test mode
    if (mCurrentSceneState == SceneState::STRESS_TEST)
    {
        // NOTE: The "instancing" PSO must be compatible with 0 Render Targets
        mCommandList->SetPipelineState(mPSOs["shadow_instancing"].Get());
    }

    std::unordered_set<Body*> emptySet;
    DrawRenderItems(mCommandList.Get(), mRitemLayer[(int)RenderLayer::Opaque], emptySet, false);

    mCommandList->ResourceBarrier(1, &CD3DX12_RESOURCE_BARRIER::Transition(mShadowMap->Resource(),
        D3D12_RESOURCE_STATE_DEPTH_WRITE, D3D12_RESOURCE_STATE_GENERIC_READ));
}
void PhysicsApplication::DrawRenderItems(
    ID3D12GraphicsCommandList* cmdList,
    const std::vector<RenderItem*>& ritems,
    const std::unordered_set<Body*>& collisionBodies,
    bool isWireframePass)
{
    // If the list is empty, exit the function.
    if (ritems.empty()) return;

    // Set up necessary buffers for both instancing and standard draws.
    UINT objCBByteSize = d3dUtil::CalcConstantBufferByteSize(sizeof(ObjectConstants));
    auto objectCB = mCurrFrameResource->ObjectCB->Resource();

    // ====================================================================
    // --- STRESS TEST MODE: SOLID PASS (Instancing & Per-Object Floor) ---
    // ====================================================================

    if (mCurrentSceneState == SceneState::STRESS_TEST && !isWireframePass)
    {
        // This block executes ONLY for the Solid Pass in STRESS_TEST mode.
        UINT instanceCount = (UINT)ritems.size() - GeneralData::NumFloorSphereCount;

        // 1. DRAW FLOOR OBJECTS (9 individual Draw Calls)
        // Floor objects use the standard opaque PSO.
        cmdList->SetPipelineState(mPSOs["opaque"].Get());

        // Iterate through the last 9 items (Floor objects)
        for (size_t i = ritems.size() - GeneralData::NumFloorSphereCount; i < ritems.size(); ++i)
        {
            auto ri = ritems[i];

            // 1-1. Bind Floor Geometry
            cmdList->IASetVertexBuffers(0, 1, &ri->Geo->VertexBufferView());
            cmdList->IASetIndexBuffer(&ri->Geo->IndexBufferView());

            // 1-2. Set Floor Object CB (Root Slot 0)
            D3D12_GPU_VIRTUAL_ADDRESS objCBAddress = objectCB->GetGPUVirtualAddress() + ri->ObjCBIndex * objCBByteSize;
            cmdList->SetGraphicsRootConstantBufferView(0, objCBAddress);

            // 1-3. Execute Standard Draw Call (Instance Count = 1)
            cmdList->DrawIndexedInstanced(ri->IndexCount, 1, ri->StartIndexLocation, ri->BaseVertexLocation, 0);
        }

        // 2. DRAW SPHERE/DIAMOND OBJECTS (N^2 Instanced Draw Call)
        if (instanceCount > 0)
        {
            // Set the Instancing PSO
            cmdList->SetPipelineState(mPSOs["instancing"].Get());

            // 2-1. Use the prototype geometry from the first render item (ritems[0])
            const auto& ritem = ritems[0];

            // 2-2. Bind Geometry (Slot 0)
            cmdList->IASetVertexBuffers(0, 1, &ritem->Geo->VertexBufferView());
            cmdList->IASetIndexBuffer(&ritem->Geo->IndexBufferView());


            // 2-3--- INSTANCING SRV BINDING (ROOT PARAMETER INDEX 5) ---
            // Bind the Instance Buffer Structured Buffer once per-frame. This is required
            // for Instancing mode, but the SRV remains bound for all draws.

            // 1. Get the GPU handle to the start of the SRV heap.
            CD3DX12_GPU_DESCRIPTOR_HANDLE instancingSrvHandle(mSrvDescriptorHeap->GetGPUDescriptorHandleForHeapStart());

            // 2. Offset the handle to the correct index (Index 3, GeneralData::NumInstanceBufferSrvIndex)
            instancingSrvHandle.Offset(
                mCurrFrameResource->InstanceBufferSrvHeapIndex,
                mCbvSrvUavDescriptorSize
            );

            // 3. Set the Descriptor Table at Root Slot 5
            mCommandList->SetGraphicsRootDescriptorTable(
                5, // Root Parameter Index 5
                instancingSrvHandle
            );
            // --- END INSTANCING SRV BINDING ---

            // 2-4. Execute the Instanced Draw Call for N^2 objects.
            cmdList->DrawIndexedInstanced(
                ritem->IndexCount,
                instanceCount, // N^2 objects
                ritem->StartIndexLocation,
                ritem->BaseVertexLocation,
                0);
        }

        // After finishing the Solid Pass in STRESS_TEST mode, we exit.
        return;
    }

    // ====================================================================
    // --- STANDARD/WIREFRAME PASSES (SANDBOX MODE or WIREFRAME OVERLAY) ---
    // ====================================================================

    // This block handles:
    // 1. All standard per-object drawing in SANDBOX mode (isWireframePass == false).
    // 2. The Wireframe Overlay pass (isWireframePass == true), regardless of mode.

    // 1. Set PSO for SANDBOX mode (if applicable)
    if (!isWireframePass)
    {
        // SANDBOX: Ensure Opaque PSO is set for individual draws.
        cmdList->SetPipelineState(mPSOs["opaque"].Get());
    }
    // Note: If it's a wireframe pass, the PSO ("wireframe") was set in the calling Draw() function.

    // 2. Per-Object Draw Loop
    for (size_t i = 0; i < ritems.size(); ++i)
    {
        auto ri = ritems[i];

        Body* currentBody = &mBodies[ri->ObjCBIndex];
        bool isCollisionBody = collisionBodies.count(currentBody) > 0;

        // --- Culling Logic (Skip) ---
        if (isWireframePass)
        {
            // PASS 2 (Wireframe Overlay): Draw ONLY collision bodies.
            if (!isCollisionBody) continue;
        }
        else // Solid Pass (SANDBOX Mode)
        {
            // PASS 1 (Solid): Skip collision bodies if wireframe debug is enabled.
            if (mIsWireframeDebugEnabled && isCollisionBody) continue;
        }
        // -----------------------

        // 3. Draw Call
        cmdList->IASetVertexBuffers(0, 1, &ri->Geo->VertexBufferView());
        cmdList->IASetIndexBuffer(&ri->Geo->IndexBufferView());

        D3D12_GPU_VIRTUAL_ADDRESS objCBAddress = objectCB->GetGPUVirtualAddress() + ri->ObjCBIndex * objCBByteSize;
        cmdList->SetGraphicsRootConstantBufferView(0, objCBAddress);

        // Standard Draw Call (1 instance per call).
        cmdList->DrawIndexedInstanced(ri->IndexCount, 1, ri->StartIndexLocation, ri->BaseVertexLocation, 0);
    }
}


// Standard box local normals (used to identify the 6 faces/planes for SHAPE_BOX)
const Vec3 g_StandardBoxNormals[6] = {
    Vec3(1.0f, 0.0f, 0.0f), Vec3(-1.0f, 0.0f, 0.0f),
    Vec3(0.0f, 1.0f, 0.0f), Vec3(0.0f, -1.0f, 0.0f),
    Vec3(0.0f, 0.0f, 1.0f), Vec3(0.0f, 0.0f, -1.0f)
};
// Helper function to check if a 3D point P is inside the triangle defined by A, B, C
// using Barycentric Coordinates (3D cross-product method).
bool IsPointInTriangle(const Vec3& P, const Vec3& A, const Vec3& B, const Vec3& C)
{
    Vec3 N = (B - A).Cross(C - A);
    if (N.GetLengthSqr() < 1e-6f) return false;
    const float EPSILON = 1e-4f;
    Vec3 cross_AB = (B - A).Cross(P - A);
    Vec3 cross_BC = (C - B).Cross(P - B);
    Vec3 cross_CA = (A - C).Cross(P - C);
    float dot_AB = N.Dot(cross_AB);
    float dot_BC = N.Dot(cross_BC);
    float dot_CA = N.Dot(cross_CA);
    // Point P is inside if P is on the same side of all three edges relative to the plane normal N.
    return (dot_AB >= -EPSILON) && (dot_BC >= -EPSILON) && (dot_CA >= -EPSILON);
}
std::vector<Vec3> PhysicsApplication::GetIncidentFaceVertices(Body* targetBody, const Vec3& contactNormalWorld, const Vec3& contactPointWorld)
{
    Shape::shapeType_t shapeType = targetBody->m_shape->GetType();

    // Safety check for supported shapes
    if (shapeType != Shape::SHAPE_CONVEX && shapeType != Shape::SHAPE_BOX)
        return {};

    const std::vector<Vec3>* verticesPtr = nullptr;
    const std::vector<tri_t>* trianglesPtr = nullptr;

    // Get shape-specific data pointers
    if (shapeType == Shape::SHAPE_BOX)
    {
        const ShapeBox* boxShape = static_cast<const ShapeBox*>(targetBody->m_shape);
        verticesPtr = &boxShape->m_points;
    }
    else // SHAPE_CONVEX
    {
        const ShapeConvex* convexShape = static_cast<const ShapeConvex*>(targetBody->m_shape);
        verticesPtr = &convexShape->m_points;
        trianglesPtr = &convexShape->m_triangles;
    }

    const auto& vertices = *verticesPtr;

    // 1. Transform World Data to Local Space.
    Quat inverseOrientation = targetBody->m_orientation;
    inverseOrientation.Invert();

    Vec3 localNormal = inverseOrientation.RotatePoint(contactNormalWorld);
    localNormal.Normalize();

    Vec3 localContactPoint = inverseOrientation.RotatePoint(contactPointWorld - targetBody->m_position);

    // 2. Search for the best aligned face/triangle.
    float bestMetric = -std::numeric_limits<float>::max();
    int bestIncidentIndex = -1; // Index refers to standard face index (Box) or triangle index (Convex)
    bool wasBestFaceFlippedInternally = false;

    // Store the 4 vertices of the best quad face found for BOX only
    std::vector<Vec3> bestQuadVertices;

    if (shapeType == Shape::SHAPE_BOX)
    {
        // --- BOX: Dynamic Face Selection (Robust against custom vertex order) ---
        for (int i = 0; i < 6; ++i)
        {
            Vec3 faceNormal = g_StandardBoxNormals[i];

            float dot = faceNormal.Dot(localNormal);
            float absDot = std::abs(dot);

            // Find the vertex furthest along the face normal direction (reference point on the plane)
            float maxDot = -std::numeric_limits<float>::max();
            Vec3 planeRefVertex;

            for (const auto& v : vertices) {
                float currentDot = v.Dot(faceNormal);
                if (currentDot > maxDot) {
                    maxDot = currentDot;
                    planeRefVertex = v;
                }
            }

            // Calculate Plane Distance using the reference point
            float planeDistance = std::abs(faceNormal.Dot(localContactPoint - planeRefVertex));

            // Use the aggressive metric (1000.0f) for BOX to ensure closest face is selected
            float currentMetric = absDot - (planeDistance * 1000.0f);

            if (currentMetric > bestMetric)
            {
                bestMetric = currentMetric;
                bestIncidentIndex = i;
                wasBestFaceFlippedInternally = dot < 0.0f;
            }
        }

        // --- Phase 2.5: Retrieve the 4 vertices of the BEST Quad Face ---
        if (bestIncidentIndex != -1)
        {
            Vec3 bestFaceNormal = g_StandardBoxNormals[bestIncidentIndex];

            // Find the 4 vertices defining the best face plane (max extent)
            float maxDot = -std::numeric_limits<float>::max();
            for (const auto& v : vertices) {
                float currentDot = v.Dot(bestFaceNormal);
                if (currentDot > maxDot) {
                    maxDot = currentDot;
                }
            }

            const float PLANE_EPSILON = 1e-4f;
            for (const auto& v : vertices) {
                // If the vertex is on the selected face plane
                if (std::abs(v.Dot(bestFaceNormal) - maxDot) < PLANE_EPSILON) {
                    bestQuadVertices.push_back(v);
                }
            }

            // Safety check
            if (bestQuadVertices.size() != 4) {
                return {};
            }
        }
    }
    else // SHAPE_CONVEX (User's provided logic)
    {
        const auto& triangles = *trianglesPtr;
        for (size_t i = 0; i < triangles.size(); ++i)
        {
            const tri_t& tri = triangles[i];
            const Vec3& v0 = vertices[tri.a];
            const Vec3& v1 = vertices[tri.b];
            const Vec3& v2 = vertices[tri.c];

            Vec3 edge1 = v1 - v0;
            Vec3 edge2 = v2 - v0;
            Vec3 faceNormal = edge1.Cross(edge2);

            // Skip degenerate face
            if (faceNormal.GetLengthSqr() < 1e-6f) continue;
            faceNormal.Normalize();

            // --- Pre-Metric Flip: Ensure faceNormal aligns with localNormal for stable metric ---
            float initialDot = faceNormal.Dot(localNormal);
            bool internallyFlipped = false;

            if (initialDot < 0.0f)
            {
                faceNormal = faceNormal * -1.0f;
                initialDot *= -1.0f;
                internallyFlipped = true;
            }

            float absDot = initialDot;

            // Proximity 1: Distance to the face plane (D)
            float planeDistance = std::abs(faceNormal.Dot(localContactPoint - v0));

            // Proximity 2: Distance to the face centroid (C)
            Vec3 faceCentroid = (v0 + v1 + v2) * (1.0f / 3.0f);
            float centroidDistance = (localContactPoint - faceCentroid).GetMagnitude();

            // Maximize AbsDot (alignment), Minimize Plane Distance (small penalty), Minimize Centroid Distance (main tie-breaker)
            float currentMetric = absDot
                - (planeDistance * 0.01f)
                - (centroidDistance * 1.0f);

            if (currentMetric > bestMetric)
            {
                bestMetric = currentMetric;
                bestIncidentIndex = (int)i;
                wasBestFaceFlippedInternally = internallyFlipped;
            }
        }
    }


    // 3. Extract and refine incident vertices (must be exactly 3).
    std::vector<Vec3> faceVertices;

    if (bestIncidentIndex != -1)
    {
        if (shapeType == Shape::SHAPE_BOX)
        {
            // BOX: Refine the best Quad face (4 vertices) to the single best Triangle (3 vertices)
            if (!bestQuadVertices.empty() && bestQuadVertices.size() == 4)
            {
                const Vec3& V0 = bestQuadVertices[0];
                const Vec3& V1 = bestQuadVertices[1];
                const Vec3& V2 = bestQuadVertices[2];
                const Vec3& V3 = bestQuadVertices[3];

                // T1: V0, V1, V2 
                bool inT1 = IsPointInTriangle(localContactPoint, V0, V1, V2);
                Vec3 C1 = (V0 + V1 + V2) * (1.0f / 3.0f);
                float distT1Sqr = (localContactPoint - C1).GetLengthSqr();

                // T2: V2, V3, V0 
                bool inT2 = IsPointInTriangle(localContactPoint, V2, V3, V0);
                Vec3 C2 = (V2 + V3 + V0) * (1.0f / 3.0f);
                float distT2Sqr = (localContactPoint - C2).GetLengthSqr();


                // Selection Logic: Prioritize strict inclusion, use centroid distance for ties/fallbacks.
                if (inT1 && !inT2)
                {
                    faceVertices.push_back(V0); faceVertices.push_back(V1); faceVertices.push_back(V2);
                }
                else if (!inT1 && inT2)
                {
                    faceVertices.push_back(V2); faceVertices.push_back(V3); faceVertices.push_back(V0);
                }
                else // Both T1 and T2 contain the point (tie) OR Neither contains the point (fallback)
                {
                    if (distT1Sqr <= distT2Sqr)
                    {
                        faceVertices.push_back(V0); faceVertices.push_back(V1); faceVertices.push_back(V2);
                    }
                    else
                    {
                        faceVertices.push_back(V2); faceVertices.push_back(V3); faceVertices.push_back(V0);
                    }
                }
            }
        }
        else // SHAPE_CONVEX
        {
            // CONVEX HULL: 1 Triangle (3 vertices)
            const tri_t& tri = (*trianglesPtr)[bestIncidentIndex];

            faceVertices.push_back(vertices[tri.a]);
            faceVertices.push_back(vertices[tri.b]);
            faceVertices.push_back(vertices[tri.c]);
        }
    }

    // 4. Transform the winning vertices back to Physics World Space and apply flip.
    if (!faceVertices.empty())
    {
        bool needsWindingFlip = wasBestFaceFlippedInternally;

        const Quat& orientation = targetBody->m_orientation;
        const Vec3& position = targetBody->m_position;

        for (Vec3& v : faceVertices)
        {
            // PURE RIGID BODY TRANSFORMATION (Physics Coords)
            v = orientation.RotatePoint(v) + position;
        }

        // Apply Winding Flip (Visual Correction)
        if (needsWindingFlip)
        {
            // Swap V1 and V2 to flip the winding order
            std::swap(faceVertices[1], faceVertices[2]);
        }
    }

    return faceVertices;
}
// Helper function to calculate Rotation Matrix that aligns the Box's local Y-axis 
DirectX::XMMATRIX CalculateRotationMatrix(const Vec3& direction, DirectX::XMVECTOR& axis, float& angle)
{
    // Define the default local axis (e.g., +Y for the Box's height/length)
    DirectX::XMVECTOR V_default = DirectX::XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);

    // Target direction, with Z-up coordinate swap (X_w, Z_w, -Y_w)
    DirectX::XMFLOAT3 dir_float3(direction.x, direction.z, -direction.y);
    DirectX::XMVECTOR V_target = DirectX::XMLoadFloat3(&dir_float3);
    V_target = DirectX::XMVector3Normalize(V_target);

    axis = DirectX::XMVector3Cross(V_default, V_target);
    float dot_val = DirectX::XMVectorGetX(DirectX::XMVector3Dot(V_default, V_target));

    // Clamp for safety
    float clamped_dot = std::max(-1.0f, std::min(1.0f, dot_val));
    angle = acos(clamped_dot);

    const float AXIS_TOLERANCE = 1e-4f;
    if (angle < AXIS_TOLERANCE)
    {
        return DirectX::XMMatrixIdentity();
    }
    else
    {
        // Handle 180 degree rotation case (axis is zero)
        if (DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(axis)) < AXIS_TOLERANCE * AXIS_TOLERANCE)
        {
            // If the vectors are opposite, rotate 180 degrees around a perpendicular axis (e.g., X-axis)
            return DirectX::XMMatrixRotationX(XM_PI);
        }
        return DirectX::XMMatrixRotationAxis(axis, angle);
    }
}
void PhysicsApplication::DrawContactPoints(ID3D12GraphicsCommandList* cmdList, Body* selectedBody)
{
    if (selectedBody == nullptr || mContactPoints.empty())
        return;

    const auto& contactPointsGeometry = mGeometries[GeneralData::Geometry::RenderObjectNames::ContactPoints].get();
    if (contactPointsGeometry == nullptr)
        return;
    const auto& contactPointsDrawArgs = contactPointsGeometry->DrawArgs.at(GeneralData::Geometry::RenderObjectNames::ContactPoints);

    UINT objCBByteSize = d3dUtil::CalcConstantBufferByteSize(sizeof(ObjectConstants));
    auto objectCB_GPU = mCurrFrameResource->ObjectCB->Resource();

    // Set PSO for the Contact Points and Gizmos (Boxes)
    cmdList->SetPipelineState(mPSOs["contactpoints"].Get());
    cmdList->IASetVertexBuffers(0, 1, &contactPointsGeometry->VertexBufferView());
    cmdList->IASetIndexBuffer(&contactPointsGeometry->IndexBufferView());
    cmdList->IASetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

    DirectX::XMFLOAT4 colorRed(1.0f, 0.0f, 0.0f, 1.0f);
    DirectX::XMFLOAT4 colorBlue(0.0f, 0.0f, 1.0f, 1.0f);
    DirectX::XMFLOAT4 colorYellow(1.0f, 1.0f, 0.0f, 1.0f);

    // Constants for Normal Arrow Visualization
    const float NORMAL_ARROW_LENGTH_BODY = 0.4f;
    const float NORMAL_ARROW_THICKNESS_BODY = 0.01f;
    const float NORMAL_ARROW_LENGTH_HEAD = 0.1f;
    const float NORMAL_ARROW_THICKNESS_HEAD = 0.03f;
    const float BLUE_LINE_THICKNESS = 0.02f; // Used for the line segments

    int currentGizmoCBIndex = static_cast<int>(mAllRitems.size());

    ObjectConstants gizmoObjCB;
    gizmoObjCB.TexTransform = MathHelper::Identity4x4();

    for (const auto& contact : mContactPoints)
    {
        // Only draw contacts involving the selected body.
        if (contact.bodyA != selectedBody && contact.bodyB != selectedBody) continue;

        // --------------------------------------------------------
        // DRAW 1 & 2: Contact Point (RED CUBE) and Normal Arrow (YELLOW BOXES) - UNCHANGED
        // --------------------------------------------------------

        Vec3 contactPos = contact.ptOnA_WorldSpace;
        Vec3 normal = contact.normal;

        // --- DRAW 1: Contact Point Gizmo (RED CUBE) ---
        XMMATRIX scaleM = XMMatrixScaling(0.05f, 0.05f, 0.05f);
        XMMATRIX transM = XMMatrixTranslation(contactPos.x, contactPos.z, -contactPos.y);
        XMStoreFloat4x4(&gizmoObjCB.World, DirectX::XMMatrixTranspose(scaleM * transM));
        gizmoObjCB.Color = colorRed;
        mCurrFrameResource->ObjectCB->CopyData(currentGizmoCBIndex, gizmoObjCB);
        D3D12_GPU_VIRTUAL_ADDRESS objCBAddress = objectCB_GPU->GetGPUVirtualAddress() + currentGizmoCBIndex * objCBByteSize;
        cmdList->SetGraphicsRootConstantBufferView(0, objCBAddress);
        cmdList->DrawIndexedInstanced(contactPointsDrawArgs.IndexCount, 1, contactPointsDrawArgs.StartIndexLocation, contactPointsDrawArgs.BaseVertexLocation, 0);
        currentGizmoCBIndex++;

        // --- DRAW 2A: Normal Arrow Body (YELLOW) ---
        {
            Vec3 bodyCenter = contactPos + normal * (NORMAL_ARROW_LENGTH_BODY / 2.0f);

            // Reusing the existing rotation calculation function placeholder
            DirectX::XMVECTOR axis;
            float angle;
            XMMATRIX rotationM = CalculateRotationMatrix(normal, axis, angle);

            scaleM = XMMatrixScaling(NORMAL_ARROW_THICKNESS_BODY, NORMAL_ARROW_LENGTH_BODY, NORMAL_ARROW_THICKNESS_BODY);
            transM = XMMatrixTranslation(bodyCenter.x, bodyCenter.z, -bodyCenter.y);

            XMStoreFloat4x4(&gizmoObjCB.World, DirectX::XMMatrixTranspose(scaleM * rotationM * transM));
            gizmoObjCB.Color = colorYellow;
            mCurrFrameResource->ObjectCB->CopyData(currentGizmoCBIndex, gizmoObjCB);
            objCBAddress = objectCB_GPU->GetGPUVirtualAddress() + currentGizmoCBIndex * objCBByteSize;
            cmdList->SetGraphicsRootConstantBufferView(0, objCBAddress);
            cmdList->DrawIndexedInstanced(contactPointsDrawArgs.IndexCount, 1, contactPointsDrawArgs.StartIndexLocation, contactPointsDrawArgs.BaseVertexLocation, 0);
            currentGizmoCBIndex++;
        }

        // --- DRAW 2B: Normal Arrow Head (YELLOW) ---
        {
            Vec3 headCenter = contactPos + normal * (NORMAL_ARROW_LENGTH_BODY + (NORMAL_ARROW_LENGTH_HEAD / 2.0f));

            DirectX::XMVECTOR axis;
            float angle;
            XMMATRIX rotationM = CalculateRotationMatrix(normal, axis, angle);

            scaleM = XMMatrixScaling(NORMAL_ARROW_THICKNESS_HEAD, NORMAL_ARROW_LENGTH_HEAD, NORMAL_ARROW_THICKNESS_HEAD);
            transM = XMMatrixTranslation(headCenter.x, headCenter.z, -headCenter.y);

            XMStoreFloat4x4(&gizmoObjCB.World, DirectX::XMMatrixTranspose(scaleM * rotationM * transM));
            gizmoObjCB.Color = colorYellow;
            mCurrFrameResource->ObjectCB->CopyData(currentGizmoCBIndex, gizmoObjCB);
            objCBAddress = objectCB_GPU->GetGPUVirtualAddress() + currentGizmoCBIndex * objCBByteSize;
            cmdList->SetGraphicsRootConstantBufferView(0, objCBAddress);
            cmdList->DrawIndexedInstanced(contactPointsDrawArgs.IndexCount, 1, contactPointsDrawArgs.StartIndexLocation, contactPointsDrawArgs.BaseVertexLocation, 0);
            currentGizmoCBIndex++;
        }


        // --------------------------------------------------------
        // DRAW 3: Incident Face Edges (BLUE LINES) - MODIFIED FOR BOTH BODIES
        // --------------------------------------------------------

        // Process both Body A and Body B
        Body* bodiesToDraw[] = { contact.bodyA, contact.bodyB };

        for (Body* bodyToDraw : bodiesToDraw)
        {
            // Use GetIncidentFaceVertices with the body and contact info.
            // We rely on GetIncidentFaceVertices to correctly determine the incident face
            // on the given 'bodyToDraw' using the 'contact.normal' and 'contact.ptOnA_WorldSpace'.
            std::vector<Vec3> faceVerts = GetIncidentFaceVertices(bodyToDraw, contact.normal, contact.ptOnA_WorldSpace);

            if (faceVerts.size() == 3)
            {
                int numSegments = 3;

                for (int i = 0; i < numSegments; ++i)
                {
                    Vec3 start = faceVerts[i];
                    Vec3 end = faceVerts[(i + 1) % 3];

                    // 1. Midpoint and Length calculations
                    Vec3 mid = (start + end) * 0.5f;
                    float len = (start - end).GetMagnitude();

                    if (len < 1e-4f) continue;

                    // --- 2. Orientation: Calculate the Rotation Matrix (alignRotation) ---
                    // This is the complex, stable rotation logic from the original code.
                    DirectX::XMVECTOR V_start = DirectX::XMVectorSet(1.0f, 0.0f, 0.0f, 0.0f);
                    Vec3 dir_vec3 = end - start;

                    // RE-APPLY COORDINATE SWAP FOR DIRECTION VECTOR (Z-up rendering space)
                    DirectX::XMFLOAT3 dir_float3(dir_vec3.x, dir_vec3.z, -dir_vec3.y);
                    DirectX::XMVECTOR V_target = DirectX::XMLoadFloat3(&dir_float3);

                    V_target = DirectX::XMVector3Normalize(V_target);

                    // Manual Rotation Logic (Stable)
                    DirectX::XMVECTOR axis = DirectX::XMVector3Cross(V_start, V_target);
                    float dot_val = DirectX::XMVectorGetX(DirectX::XMVector3Dot(V_start, V_target));

                    float clamped_dot = std::max(-1.0f, std::min(1.0f, dot_val));
                    float angle = acos(clamped_dot);
                    XMMATRIX alignRotation;

                    const float AXIS_TOLERANCE = 1e-2f;

                    if (angle < AXIS_TOLERANCE) {
                        alignRotation = XMMatrixIdentity();
                    }
                    else {
                        if (DirectX::XMVectorGetX(DirectX::XMVector3LengthSq(axis)) < AXIS_TOLERANCE * AXIS_TOLERANCE)
                        {
                            alignRotation = XMMatrixRotationY(XM_PI);
                        }
                        else
                        {
                            alignRotation = DirectX::XMMatrixRotationAxis(axis, angle);
                        }
                    }

                    // 3. Combine matrices (Scale * Rotate * Translate)
                    // The X axis of the box aligns with the line segment.
                    scaleM = XMMatrixScaling(len, BLUE_LINE_THICKNESS, BLUE_LINE_THICKNESS);

                    // Translation Swap (X_w, Z_w, -Y_w) for correct positioning
                    transM = XMMatrixTranslation(mid.x, mid.z, -mid.y);

                    // World Matrix = Scale * Rotate * Translate
                    XMStoreFloat4x4(&gizmoObjCB.World, DirectX::XMMatrixTranspose(scaleM * alignRotation * transM));
                    gizmoObjCB.Color = colorBlue;

                    mCurrFrameResource->ObjectCB->CopyData(currentGizmoCBIndex, gizmoObjCB);

                    objCBAddress = objectCB_GPU->GetGPUVirtualAddress() + currentGizmoCBIndex * objCBByteSize;
                    cmdList->SetGraphicsRootConstantBufferView(0, objCBAddress);
                    cmdList->DrawIndexedInstanced(contactPointsDrawArgs.IndexCount, 1, contactPointsDrawArgs.StartIndexLocation, contactPointsDrawArgs.BaseVertexLocation, 0);

                    currentGizmoCBIndex++;
                }
            }
        }
    }
}


// ImGUI
void PhysicsApplication::RenderMainMenuUI() {
    static const float MAIN_MENU_UI_WIDTH = mClientWidth * 0.5f;
    ImGui::SetNextWindowSize(ImVec2(MAIN_MENU_UI_WIDTH + 16.0f, mClientHeight * 0.22f));
    ImGui::SetNextWindowPos(ImVec2(mClientWidth * 0.25f - 8.0f, mClientHeight * 0.30f));
    ImGui::PushFont(mFontMainMenu);
    ImGui::Begin("Main Menu");

    ImGui::Text("Select a demo scene to load:");
    ImGui::Separator();

    if (ImGui::Button("Stress Test", ImVec2(MAIN_MENU_UI_WIDTH, 0)))
        mRequestedSceneState = SceneState::STRESS_TEST;
    if (ImGui::Button("Sandbox", ImVec2(MAIN_MENU_UI_WIDTH, 0)))
        mRequestedSceneState = SceneState::SANDBOX;

    ImGui::Separator();
    if (ImGui::Button("Quit Application", ImVec2(MAIN_MENU_UI_WIDTH, 0)))
        PostQuitMessage(0);

    ImGui::PopFont();
    ImGui::End();
}
void PhysicsApplication::RenderDemoUICommon() {
    // Begin a new ImGui window for the demo controls
    ImGui::SetNextWindowPos(ImVec2(10, 10));
    ImGui::SetNextWindowSize(ImVec2(170, 110));
    ImGui::PushFont(mFontDemo);
    ImGui::Begin("Controls");

    // --- Common Controls for All Demo Scenes ---

    // Button to pause/play the simulation
    std::string buttonName;
    if (mAppPaused)
        buttonName = "Resume";
    else
        buttonName = "Pause";

    if (ImGui::Button(buttonName.c_str())) {
        if (mAppPaused == true)
            ResumeEngine();
        else
            PauseEngine();
    }

    ImGui::Separator();

    // Button to restart the demo scene
    if (ImGui::Button("Restart this demo")) {
        ResetCamera();
        mIsRestartNeeded = true;
    }

    ImGui::Separator();

    // Button to return to the main menu
    if (ImGui::Button("Return to Main Menu")) {
        mRequestedSceneState = SceneState::MAIN_MENU;
        mLockTimer = GeneralData::LockTime;
        ResetCamera();
    }

    ImGui::PopFont();
    ImGui::End();
}
void PhysicsApplication::RenderDemoUILoading() {
    ImGui::PushFont(mFontDemo);

    // Static variables to keep track of animation state
    static float progress_animated = 0.0f;
    static float speed = 0.5f; // Adjust this value to change the animation speed

    // Update the progress value based on the frame time
    // Get the time elapsed since the last frame
    float delta_time = ImGui::GetIO().DeltaTime;
    progress_animated += speed * delta_time;

    // Wrap the value back to 0.0f to create a seamless loop
    if (progress_animated > 1.0f) {
        progress_animated -= 1.0f;
    }

    // Set up the window for the progress bar
    ImGuiIO& io = ImGui::GetIO();
    ImVec2 window_size(300, 50);
    ImVec2 window_pos(io.DisplaySize.x - window_size.x - 10, io.DisplaySize.y - window_size.y - 10);
    ImGui::SetNextWindowPos(window_pos);
    ImGui::SetNextWindowSize(window_size);

    // Begin the non-intrusive window
    ImGui::Begin("LoadingStatus", nullptr,
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBackground |
        ImGuiWindowFlags_NoBringToFrontOnFocus);

    // Display the text and the animated progress bar
    ImGui::Text("Please wait...");
    ImGui::ProgressBar(progress_animated, ImVec2(-1.0f, 0.0f), "");
    ImGui::PopFont();
    ImGui::End();
}
void PhysicsApplication::RenderDemoUIGizmo() {
    if (mPickedRenderItem != nullptr) {
        // Set the rect where the gizmo should be drawn.
        ImGuizmo::SetRect(0, 0, static_cast<float>(mClientWidth), static_cast<float>(mClientHeight));

        // You must call ImGuizmo::BeginFrame() every frame.
        ImGuizmo::BeginFrame();

        // Set the current operation and mode.
        static ImGuizmo::MODE currentGizmoMode = ImGuizmo::WORLD;

        // Call the Manipulate function. This is the core of ImGuizmo.
        // It draws the gizmo and handles user input.
        XMFLOAT4X4 viewMatrix;
        XMFLOAT4X4 projMatrix;

        // Inside your rendering loop or GUI update function:
        // Store the XMMATRIX data into the temporary XMFLOAT4X4 structs.
        XMStoreFloat4x4(&viewMatrix, mCamera.GetView());
        XMStoreFloat4x4(&projMatrix, mCamera.GetProj());
        ImGuizmo::Manipulate(
            (const float*)viewMatrix.m,       // camera's view matrix
            (const float*)projMatrix.m, // camera's projection matrix
            mCurrentGizmoOperation,  // The current operation (TRANSLATE, ROTATE, SCALE)
            currentGizmoMode,       // The current mode (WORLD or LOCAL)
            (float*)mPickedRenderItem->World.m       // The model matrix of the object you're manipulating
        );

        // redraw the picked item
        mPickedRenderItem->NumFramesDirty = gNumFrameResources;

        // Load the updated World matrix from XMFLOAT4X4 into XMMATRIX
        XMMATRIX newWorld = XMLoadFloat4x4(&mPickedRenderItem->World);

        // Separate the matrix components
        XMVECTOR scale, rotationQuat, translation;
        XMMatrixDecompose(&scale, &rotationQuat, &translation, newWorld);

        mIsGuizmoMoving = false;

        // Now, update your object's data members based on the operation
        if (ImGuizmo::IsUsing()) { // Only update the physics body if the gizmo was actively manipulated
            mIsGuizmoMoving = true;

            // --- TRANSLATE/POSITION UPDATE (Corrected for clarity) ---
            if (mCurrentGizmoOperation == ImGuizmo::OPERATION::TRANSLATE) {
                // Extract the new position components
                float newX = XMVectorGetX(translation);
                float newY = XMVectorGetY(translation);
                float newZ = XMVectorGetZ(translation);

                // Update physics body position (using your specific coordinate system mapping)
                mBodies[mPickedRenderItem->ObjCBIndex].m_position.x = newX;
                mBodies[mPickedRenderItem->ObjCBIndex].m_position.y = -newZ; // Assuming Y and Z are swapped/inverted
                mBodies[mPickedRenderItem->ObjCBIndex].m_position.z = newY;
            }
            // --- ROTATION/ORIENTATION UPDATE ---
            else if (mCurrentGizmoOperation == ImGuizmo::OPERATION::ROTATE) {
                // Extract the components of the quaternion
                XMFLOAT4 quat;
                XMStoreFloat4(&quat, rotationQuat);

                // Update physics body orientation (Quaternion includes W component)
                mBodies[mPickedRenderItem->ObjCBIndex].m_orientation.x = quat.x;
                mBodies[mPickedRenderItem->ObjCBIndex].m_orientation.y = -quat.z; // Assuming Y and Z are swapped/inverted
                mBodies[mPickedRenderItem->ObjCBIndex].m_orientation.z = quat.y;
                mBodies[mPickedRenderItem->ObjCBIndex].m_orientation.w = quat.w;
            }
        }
    }
}

void PhysicsApplication::RenderDemoUIVisualDebugger() 
{
    ImGui::SetNextWindowPos(ImVec2(10, 130));
    ImGui::SetNextWindowSize(ImVec2(280, 470));
    ImGui::PushFont(mFontDemo);
    ImGui::Begin("Visual Debugger");
    if (mPickedRenderItem != nullptr) {
        std::string nameIsOnOff = (mCurrentGizmoOperation == ImGuizmo::TRANSLATE ? "On" : "Off");
        if (ImGui::Button(("Translate: " + nameIsOnOff).c_str())) {
            mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
        }
        ImGui::SameLine();
        nameIsOnOff = (mCurrentGizmoOperation == ImGuizmo::ROTATE ? "On" : "Off");
        if (ImGui::Button(("Rotate: " + nameIsOnOff).c_str())) {
            mCurrentGizmoOperation = ImGuizmo::ROTATE;
        }

        ImGui::Text("Object ID: %d", mPickedRenderItem->ObjCBIndex);
        ImGui::Text("Position\nX: %.2f, Y: %.2f, Z: %.2f",
            mBodies[mPickedRenderItem->ObjCBIndex].m_position.x,
            mBodies[mPickedRenderItem->ObjCBIndex].m_position.z,
            -mBodies[mPickedRenderItem->ObjCBIndex].m_position.y);
        ImGui::Text("Orientation\nX: %.2f, Y: %.2f, Z: %.2f\n, W: %.2f",
            mBodies[mPickedRenderItem->ObjCBIndex].m_orientation.x,
            mBodies[mPickedRenderItem->ObjCBIndex].m_orientation.z,
            -mBodies[mPickedRenderItem->ObjCBIndex].m_orientation.y,
            mBodies[mPickedRenderItem->ObjCBIndex].m_orientation.w);
        ImGui::Text("Linear Velocity\nX: %.2f, Y: %.2f, Z: %.2f",
            mFrameHistory[mCurrentFrameHistoryIndex - 1].objectStates[mPickedRenderItem->ObjCBIndex].linearVelocity.x,
            mFrameHistory[mCurrentFrameHistoryIndex - 1].objectStates[mPickedRenderItem->ObjCBIndex].linearVelocity.z,
            -mFrameHistory[mCurrentFrameHistoryIndex - 1].objectStates[mPickedRenderItem->ObjCBIndex].linearVelocity.y);
        ImGui::Text("Angular Velocity\nX: %.2f, Y: %.2f, Z: %.2f",
            mFrameHistory[mCurrentFrameHistoryIndex - 1].objectStates[mPickedRenderItem->ObjCBIndex].angularVelocity.x,
            mFrameHistory[mCurrentFrameHistoryIndex - 1].objectStates[mPickedRenderItem->ObjCBIndex].angularVelocity.z,
            -mFrameHistory[mCurrentFrameHistoryIndex - 1].objectStates[mPickedRenderItem->ObjCBIndex].angularVelocity.y);

        ImGui::Separator();

        nameIsOnOff = (mIsHitResultVisible ? "Hide" : "Show");
        if (ImGui::Button((nameIsOnOff + " Hit Results").c_str())) {
            mIsHitResultVisible = !mIsHitResultVisible;
            mPickedContactPointIndex = -1;
        }
        if (mIsHitResultVisible == true) {
            if(ImGui::TreeNode("Hit Results")) {
                int hitResultIndex = 0;
                ImGuiTreeNodeFlags flag;
                for (auto& currentContactPoint : mContactPoints) {
                    flag = (hitResultIndex == mPickedContactPointIndex) ? ImGuiTreeNodeFlags_Selected : 0;
                    if (ImGui::TreeNodeEx((std::string("Hit Result ") + std::to_string(hitResultIndex)).c_str(), flag)) {
                        ImGui::Text("Body A: %d, Body B: %d",
                            mContactPoints[hitResultIndex].bodyA->m_id,
                            mContactPoints[hitResultIndex].bodyB->m_id);

                        ImGui::Text("Collision Point on A (local)\nX: %.2f, Y: %.2f, Z: %.2f",
                            mContactPoints[hitResultIndex].ptOnA_LocalSpace.x,
                            mContactPoints[hitResultIndex].ptOnA_LocalSpace.z,
                            -mContactPoints[hitResultIndex].ptOnA_LocalSpace.y);
                        ImGui::Text("Collision Point on A (global)\nX: %.2f, Y: %.2f, Z: %.2f",
                            mContactPoints[hitResultIndex].ptOnA_WorldSpace.x,
                            mContactPoints[hitResultIndex].ptOnA_WorldSpace.z,
                            -mContactPoints[hitResultIndex].ptOnA_WorldSpace.y);

                        ImGui::Text("Collision Normal\nX: %.2f, Y: %.2f, Z: %.2f",
                            mContactPoints[hitResultIndex].normal.x,
                            mContactPoints[hitResultIndex].normal.z,
                            -mContactPoints[hitResultIndex].normal.y);
                        ImGui::TreePop();
                    }
                    ++hitResultIndex;
                }

                ImGui::TreePop();
            }

            nameIsOnOff = (mIsWireframeDebugEnabled ? "Off" : "On");
            if (ImGui::Button(("Wireframe " + nameIsOnOff).c_str())) {
                mIsWireframeDebugEnabled = !mIsWireframeDebugEnabled;
            }
        }
        else 
            ImGui::Text("Contact Point is not picked.");
    }
    else 
        ImGui::Text("Object is not picked.");
    

    ImGui::PopFont();
    ImGui::End();
}

std::string ConvertVec3ToString(const Vec3& inputValue) {
    const float epsilon = 0.1f;

    std::stringstream output;
    output << std::fixed << std::setprecision(1);

    auto FormatFloat = [&](float val) -> float {
        return (std::abs(val) < epsilon) ? 0.0f : val;
        };

    output << FormatFloat(inputValue.x) << ", "
        << FormatFloat(inputValue.z) << ", "
        << FormatFloat(inputValue.y);

    return output.str();
}
void PhysicsApplication::CalculateAndDisplayFPS(float deltaTime) {
    mElapsedTime += deltaTime;
    ++mFrameCount;

    if (mElapsedTime >= 0.5f) {
        mAverageFPS = static_cast<float>(mFrameCount) / mElapsedTime;

        mElapsedTime = 0.0f;
        mFrameCount = 0;
        mHistoryFPS[mHistoryIndex] = mAverageFPS;
        mHistoryIndex = (mHistoryIndex + 1) % GeneralData::GUI::HistorySize;
    }
}

void PhysicsApplication::RenderDemoUIStressTest() {
    ImGui::PushFont(mFontDemo);
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImVec2 panelSize = ImVec2(GeneralData::GUI::PanelWidth, GeneralData::GUI::PanelHeightStressTest);
    ImVec2 panelPosition = ImVec2(viewport->WorkPos.x + viewport->WorkSize.x - panelSize.x, viewport->WorkPos.y);

    ImGui::SetNextWindowPos(panelPosition, ImGuiCond_Always);
    ImGui::SetNextWindowSize(panelSize, ImGuiCond_Always);
    ImGui::SetNextWindowViewport(viewport->ID);

    ImGuiWindowFlags mainPanelFlags = ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
    mainPanelFlags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;

    ImGui::Begin("Information", nullptr, mainPanelFlags);

    ImGuiID myDockspaceId = ImGui::GetID("MyDockSpace");
    ImGui::DockSpace(myDockspaceId, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_NoResize);

    // This is the correct place for the DockBuilder code.
    // It runs only after the dockspace has been created.
    static bool isFirstRun = true;
    if (isFirstRun) {
        isFirstRun = false;

        ImGui::DockBuilderRemoveNode(myDockspaceId);
        ImGui::DockBuilderAddNode(myDockspaceId, ImGuiDockNodeFlags_NoResize);
        ImGui::DockBuilderSetNodeSize(myDockspaceId, panelSize);

        ImGuiID dockIdA, dockIdB, dockIdC, dockIdD, dockIdE, dockIdF;
        ImGui::DockBuilderSplitNode(myDockspaceId, ImGuiDir_Up, 0.30f, &dockIdA, &myDockspaceId);
        ImGui::DockBuilderSplitNode(myDockspaceId, ImGuiDir_Up, 0.18f, &dockIdB, &myDockspaceId);
        ImGui::DockBuilderSplitNode(myDockspaceId, ImGuiDir_Up, 0.22f, &dockIdC, &myDockspaceId);
        ImGui::DockBuilderSplitNode(myDockspaceId, ImGuiDir_Up, 0.40f, &dockIdD, &myDockspaceId);
        ImGui::DockBuilderSplitNode(myDockspaceId, ImGuiDir_Up, 0.53f, &dockIdE, &myDockspaceId);
        ImGui::DockBuilderSplitNode(myDockspaceId, ImGuiDir_Up, 0.40f, &dockIdF, &myDockspaceId);

        ImGui::DockBuilderDockWindow("Performance Monitor", dockIdA);
        ImGui::DockBuilderDockWindow("Frame Controller", dockIdB);
        ImGui::DockBuilderDockWindow("Optimization", dockIdC);
        ImGui::DockBuilderDockWindow("Shape", dockIdD);
        ImGui::DockBuilderDockWindow("Stress Level", dockIdE);
        ImGui::DockBuilderDockWindow("Height", dockIdF);

        ImGui::DockBuilderFinish(myDockspaceId);
    }

    ImGui::End();


    // Now, create the actual windows. They will automatically be docked.
    mIsSliderMoving = false;

    // Performance monitor
    ImGui::Begin("Performance Monitor");
    CalculateAndDisplayFPS(mTimer.DeltaTime());
    ImGui::Text("FPS: %.0f", mAverageFPS);
    ImGui::PlotLines("##frametime", mHistoryFPS, GeneralData::GUI::HistorySize, mHistoryIndex, "FPS", 10.0f, 60.0f, ImVec2(0, 80));
    ImGui::End();

    // Frame Controller
    ImGui::Begin("Frame Controller");
    if (mAppPaused == true) {
        if (ImGui::SliderInt("##Controller", &mCurrentFrameHistoryIndex, 1, GeneralData::GUI::MaxFrameHistorySize > static_cast<int>(mFrameHistory.size())
            ? static_cast<int>(mFrameHistory.size()) : GeneralData::GUI::MaxFrameHistorySize)) {
            LoadFrameState(mCurrentFrameHistoryIndex - 1);
        }
        if (ImGui::IsItemActive())
            mIsSliderMoving = true;
    }
    ImGui::End();


    // Shape
    ImGui::Begin("Shape");
    std::string nameIsOnOff = (mIsStressTestShapeSphere ? "On" : "Off");
    if (ImGui::Button(("Sphere: " + nameIsOnOff).c_str())) {
        mIsStressTestShapeSphere = true;
        mIsRestartNeeded = true;
    }
    nameIsOnOff = (mIsStressTestShapeSphere == false ? "On" : "Off");
    if (ImGui::Button(("Diamond: " + nameIsOnOff).c_str())) {
        mIsStressTestShapeSphere = false;
        mIsRestartNeeded = true;
    }
    ImGui::End();

    // Stress Level
    ImGui::Begin("Stress Level");
    ImGui::SliderInt("##Stress Level", &mStressLevel, 1, 10);
    if (ImGui::IsItemActive())
        mIsSliderMoving = true;
    if (ImGui::IsItemDeactivatedAfterEdit())
        mIsRestartNeeded = true;
    ImGui::End();

    // Start Height
    if (mAppPaused == true) {
        ImGui::Begin("Height");
        ImGui::SliderFloat("##Height", &mStressStartHeight, 5.0f, 20.0f);
        if (ImGui::IsItemActive())
            mIsSliderMoving = true;
        if (ImGui::IsItemDeactivatedAfterEdit())
            mIsRestartNeeded = true;
        ImGui::End();
    }

    // Optimization
    ImGui::Begin("Optimization");
    nameIsOnOff = (mIsBroadNarrowOn ? "On" : "Off");
    if (ImGui::Button(("Broad/Narrow: " + nameIsOnOff).c_str())) {
        mIsBroadNarrowOn = !mIsBroadNarrowOn;
        mIsRestartNeeded = true;
    }
    ImGui::End();


    ImGui::PopFont();
}
void PhysicsApplication::RenderDemoUISandBox() {
    ImGui::PushFont(mFontDemo);
    const ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImVec2 panelSize = ImVec2(GeneralData::GUI::PanelWidth, GeneralData::GUI::PanelHeightSandbox);
    ImVec2 panelPosition = ImVec2(viewport->WorkPos.x + viewport->WorkSize.x - panelSize.x, viewport->WorkPos.y);

    ImGui::SetNextWindowPos(panelPosition, ImGuiCond_Always);
    ImGui::SetNextWindowSize(panelSize, ImGuiCond_Always);
    ImGui::SetNextWindowViewport(viewport->ID);

    ImGuiWindowFlags mainPanelFlags = ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
    mainPanelFlags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;

    ImGui::Begin("Information", nullptr, mainPanelFlags);

    ImGuiID myDockspaceId = ImGui::GetID("MyDockSpace");
    ImGui::DockSpace(myDockspaceId, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_NoResize);

    // This is the correct place for the DockBuilder code.
    // It runs only after the dockspace has been created.
    static bool isFirstRun = true;
    if (isFirstRun) {
        isFirstRun = false;

        ImGui::DockBuilderRemoveNode(myDockspaceId);
        ImGui::DockBuilderAddNode(myDockspaceId, ImGuiDockNodeFlags_NoResize);
        ImGui::DockBuilderSetNodeSize(myDockspaceId, panelSize);

        ImGuiID dockIdA, dockIdB, dockIdC;
        ImGui::DockBuilderSplitNode(myDockspaceId, ImGuiDir_Up, 0.30f, &dockIdA, &myDockspaceId);
        ImGui::DockBuilderSplitNode(myDockspaceId, ImGuiDir_Up, 0.20f, &dockIdB, &myDockspaceId);
        ImGui::DockBuilderSplitNode(myDockspaceId, ImGuiDir_Up, 0.25f, &dockIdC, &myDockspaceId);

        ImGui::DockBuilderDockWindow("Performance Monitor", dockIdA);
        ImGui::DockBuilderDockWindow("Frame Controller", dockIdB);
        ImGui::DockBuilderDockWindow("Scene Loader", dockIdC);

        ImGui::DockBuilderFinish(myDockspaceId);
    }

    ImGui::End();


    // Now, create the actual windows. They will automatically be docked.
    mIsSliderMoving = false;

    // Performance monitor
    ImGui::Begin("Performance Monitor");
    CalculateAndDisplayFPS(mTimer.DeltaTime());
    ImGui::Text("FPS: %.0f", mAverageFPS);
    ImGui::PlotLines("##frametime", mHistoryFPS, GeneralData::GUI::HistorySize, mHistoryIndex, "FPS", 10.0f, 60.0f, ImVec2(0, 80));
    ImGui::End();

    // Frame Controller
    ImGui::Begin("Frame Controller");
    if (mAppPaused == true) {
        if (ImGui::SliderInt("##Controller", &mCurrentFrameHistoryIndex, 1, GeneralData::GUI::MaxFrameHistorySize > static_cast<int>(mFrameHistory.size())
            ? static_cast<int>(mFrameHistory.size()) : GeneralData::GUI::MaxFrameHistorySize)) {
            LoadFrameState(mCurrentFrameHistoryIndex - 1);
        }
        if (ImGui::IsItemActive())
            mIsSliderMoving = true;
    }
    ImGui::End();

    // Scene loader
    ImGui::Begin("Scene Loader");
    std::string nameIsOnOff = (mSandboxState == SandboxState::ALL ? "On" : "Off");
    if (ImGui::Button(("All: " + nameIsOnOff).c_str())) {
        mSandboxState = SandboxState::ALL;
        mIsRestartNeeded = true;
    }
    nameIsOnOff = (mSandboxState == SandboxState::STACK ? "On" : "Off");
    if (ImGui::Button(("Stack: " + nameIsOnOff).c_str())) {
        mSandboxState = SandboxState::STACK;
        mIsRestartNeeded = true;
    }
    nameIsOnOff = (mSandboxState == SandboxState::MOVER ? "On" : "Off");
    if (ImGui::Button(("Mover: " + nameIsOnOff).c_str())) {
        mSandboxState = SandboxState::MOVER;
        mIsRestartNeeded = true;
    }
    nameIsOnOff = (mSandboxState == SandboxState::CHAIN ? "On" : "Off");
    if (ImGui::Button(("Chain: " + nameIsOnOff).c_str())) {
        mSandboxState = SandboxState::CHAIN;
        mIsRestartNeeded = true;
    }

    /*
    // todo : skip this part for now // temp
    nameIsOnOff = (mSandboxState == SandboxState::HINGE ? "On" : "Off");
    if (ImGui::Button(("Hinge: " + nameIsOnOff).c_str())) {
        mSandboxState = SandboxState::HINGE;
        mIsRestartNeeded = true;
    }
    nameIsOnOff = (mSandboxState == SandboxState::VELOCITY ? "On" : "Off");
    if (ImGui::Button(("Velocity: " + nameIsOnOff).c_str())) {
        mSandboxState = SandboxState::VELOCITY;
        mIsRestartNeeded = true;
    }
    nameIsOnOff = (mSandboxState == SandboxState::ORIENTATION ? "On" : "Off");
    if (ImGui::Button(("Orientation: " + nameIsOnOff).c_str())) {
        mSandboxState = SandboxState::ORIENTATION;
        mIsRestartNeeded = true;
    }
    */

    nameIsOnOff = (mSandboxState == SandboxState::SPINNER ? "On" : "Off");
    if (ImGui::Button(("Spinner: " + nameIsOnOff).c_str())) {
        mSandboxState = SandboxState::SPINNER;
        mIsRestartNeeded = true;
    }
    nameIsOnOff = (mSandboxState == SandboxState::RAGDOLL ? "On" : "Off");
    if (ImGui::Button(("Ragdoll: " + nameIsOnOff).c_str())) {
        mSandboxState = SandboxState::RAGDOLL;
        mIsRestartNeeded = true;
    }

    nameIsOnOff = (mSandboxState == SandboxState::CONVEX ? "On" : "Off");
    if (ImGui::Button(("Convex: " + nameIsOnOff).c_str())) {
        mSandboxState = SandboxState::CONVEX;
        mIsRestartNeeded = true;
    }





    ImGui::End();
    
    /*
    // Optimization
    ImGui::Begin("Optimization");
    nameIsOnOff = (mIsBroadNarrowOn ? "On" : "Off");
    if (ImGui::Button(("Broad/Narrow: " + nameIsOnOff).c_str())) {
        mIsBroadNarrowOn = !mIsBroadNarrowOn;
        mIsRestartNeeded = true;
    }
    ImGui::End();
    */

    ImGui::PopFont();
}

// frame contoller
void PhysicsApplication::SaveCurrentFrameState() {
    FrameState currentFrameState;

    // Iterate through all objects in your physics world
    for (const auto& currentBody : mBodies) {
        ObjectState state;
        state.position = DirectX::XMFLOAT3(currentBody.m_position.x, currentBody.m_position.y, currentBody.m_position.z);
        state.orientation = DirectX::XMFLOAT4(currentBody.m_orientation.x, currentBody.m_orientation.y, currentBody.m_orientation.z, currentBody.m_orientation.w);
        state.linearVelocity = DirectX::XMFLOAT3(currentBody.m_linearVelocity.x, currentBody.m_linearVelocity.y, currentBody.m_linearVelocity.z);
        state.angularVelocity = DirectX::XMFLOAT3(currentBody.m_angularVelocity.x, currentBody.m_angularVelocity.y, currentBody.m_angularVelocity.z);
        currentFrameState.objectStates[currentBody.m_id] = state;
    }

    for (size_t currentCount = mCurrentAvailableFrameHistoryIndex + 1, endCount = mFrameHistory.size(); currentCount < mFrameHistory.size(); ++currentCount)
        mFrameHistory.pop_back();
    mFrameHistory.push_back(currentFrameState);
    ++mCurrentAvailableFrameHistoryIndex;
    if (mCurrentAvailableFrameHistoryIndex > GeneralData::GUI::MaxFrameHistorySize) {
        --mCurrentAvailableFrameHistoryIndex;
        mFrameHistory.pop_front();
    }
}
void PhysicsApplication::LoadFrameState(const int targetIndex) {
    if (targetIndex > -1 && targetIndex < GeneralData::GUI::MaxFrameHistorySize) {
        const FrameState& targetState = mFrameHistory[targetIndex];

        // Restore the state of each object
        const size_t END_INDEX = mBodies.size() - GeneralData::NumSandboxGroundBodies;
        for (const auto& currentObjectStatePair : targetState.objectStates) {
            int objectID = currentObjectStatePair.first;
            const ObjectState& state = currentObjectStatePair.second;

            // restore physical properties
            mBodies[objectID].m_position = Vec3(state.position.x, state.position.y, state.position.z);
            mBodies[objectID].m_orientation = Quat(state.orientation.x, state.orientation.y, state.orientation.z, state.orientation.w);
            mBodies[objectID].m_linearVelocity = Vec3(state.linearVelocity.x, state.linearVelocity.y, state.linearVelocity.z);
            mBodies[objectID].m_angularVelocity = Vec3(state.angularVelocity.x, state.angularVelocity.y, state.angularVelocity.z);

            // redraw
            mAllRitems[objectID].get()->NumFramesDirty = gNumFrameResources;
            XMVECTOR orientationVector = XMVectorSet(mBodies[objectID].m_orientation.x,
                mBodies[objectID].m_orientation.z,
                -mBodies[objectID].m_orientation.y,
                mBodies[objectID].m_orientation.w);
            XMMATRIX rotationMatrix = XMMatrixRotationQuaternion(orientationVector);
            XMVECTOR positionVector = XMVectorSet(mBodies[objectID].m_position.x,
                mBodies[objectID].m_position.z,
                -mBodies[objectID].m_position.y,
                0.0f);
            if (mRequestedSceneState == SceneState::SANDBOX) {
                if (objectID > END_INDEX)
                    continue;
            }
            XMMATRIX translationMatrix = XMMatrixTranslationFromVector(positionVector);
            XMMATRIX currentWorldMatrix = rotationMatrix * translationMatrix;
            XMStoreFloat4x4(&mAllRitems[objectID].get()->World, currentWorldMatrix);
        }
    }
}

// scene setup
void PhysicsApplication::SetupDemoSceneStressTest(GeometryGenerator& geometryGenerator) {
    if (mIsStressTestShapeSphere)
        SetupDemoSceneSpheres(geometryGenerator);
    else
        SetupDemoSceneDiamonds(geometryGenerator);
}
void PhysicsApplication::SetupDemoSceneSandbox(GeometryGenerator& geometryGenerator) {
    switch (mSandboxState) {
        case SandboxState::STACK:       SetupDemoSceneStack(geometryGenerator); break;
        case SandboxState::MOVER:       SetupDemoSceneMover(geometryGenerator); break;
        case SandboxState::CHAIN:       SetupDemoSceneChain(geometryGenerator); break;

        case SandboxState::HINGE:       SetupDemoSceneHinge(geometryGenerator); break;
        case SandboxState::VELOCITY:    SetupDemoSceneVelocity(geometryGenerator); break;
        case SandboxState::ORIENTATION: SetupDemoSceneOrientation(geometryGenerator); break;
        case SandboxState::SPINNER:     SetupDemoSceneSpinner(geometryGenerator); break;
        case SandboxState::RAGDOLL:     SetupDemoSceneRagdoll(geometryGenerator); break;

        case SandboxState::CONVEX:      SetupDemoSceneConvex(geometryGenerator); break;

        case SandboxState::ALL:         SetupDemoSceneAll(geometryGenerator); break;
    }
}


// specific setup : stress test
void PhysicsApplication::SetupDemoSceneSpheres(GeometryGenerator& geometryGenerator) {
    int lastIndex = AddSpheres(mBodies, geometryStartEndIndices, 0, mStressLevel, mStressStartHeight);
    BuildGeometrySpheres(geometryGenerator);

    BuildRenderItemsStressTest(geometryGenerator, lastIndex);
}
void PhysicsApplication::SetupDemoSceneDiamonds(GeometryGenerator& geometryGenerator) {
    int lastIndex = AddDiamonds(mBodies, geometryStartEndIndices, 0, mStressLevel, mStressStartHeight);
    BuildGeometryDiamonds(geometryGenerator);

    BuildRenderItemsStressTest(geometryGenerator, lastIndex);
}

void PhysicsApplication::BuildRenderItemsStressTest(GeometryGenerator& geometryGenerator, int lastIndex) {
    lastIndex = AddFloor(mBodies, geometryStartEndIndices, lastIndex);
    BuildGeometryFloor(geometryGenerator);

    XMMATRIX textureTransform = XMMatrixScaling(1.0f, 1.0f, 1.0f);
    for (const auto& currentBody : mBodies)
    {
        auto currentRenderItem = std::make_unique<RenderItem>();
        XMMATRIX translationMatrix = XMMatrixTranslation(currentBody.m_position.x, currentBody.m_position.z, -currentBody.m_position.y);
        XMStoreFloat4x4(&currentRenderItem->World, translationMatrix);
        XMStoreFloat4x4(&currentRenderItem->TexTransform, textureTransform);
        currentRenderItem->ObjCBIndex = currentBody.m_id;
        currentRenderItem->Mat = mMaterials[currentBody.m_materialName].get();
        currentRenderItem->Geo = mGeometries[currentBody.m_geometryName].get();
        currentRenderItem->PrimitiveType = D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST;

        // CRITICAL FIX: Update DrawArgs lookup key.
        // Since BuildGeometry methods now create only one submesh and keys it by the Geometry Name (geo->Name), 
        // we must use that key here instead of the Body Object Name (currentBody.m_objectName, which was the index).
        const std::string& geoName = currentRenderItem->Geo->Name;
        currentRenderItem->IndexCount = currentRenderItem->Geo->DrawArgs[geoName].IndexCount;
        currentRenderItem->StartIndexLocation = currentRenderItem->Geo->DrawArgs[geoName].StartIndexLocation;
        currentRenderItem->BaseVertexLocation = currentRenderItem->Geo->DrawArgs[geoName].BaseVertexLocation;

        mRitemLayer[(int)RenderLayer::Opaque].push_back(currentRenderItem.get());
        mAllRitems.push_back(std::move(currentRenderItem));
    }
}


// specific setup : sandbox
void PhysicsApplication::SetupDemoSceneStack(GeometryGenerator& geometryGenerator) {
    int lastIndex = AddStack(mBodies, geometryStartEndIndices, 0);
    BuildGeometryStack(geometryGenerator, 0);

    BuildRenderItemsSandbox(geometryGenerator, lastIndex);
}
void PhysicsApplication::SetupDemoSceneMover(GeometryGenerator& geometryGenerator) {
    int lastIndex = AddMover(mBodies, mConstraints, geometryStartEndIndices, 0);
    BuildGeometryMover(geometryGenerator);

    BuildRenderItemsSandbox(geometryGenerator, lastIndex);
}
void PhysicsApplication::SetupDemoSceneChain(GeometryGenerator& geometryGenerator) {
    int lastIndex = AddChain(mBodies, mConstraints, geometryStartEndIndices, 0);
    BuildGeometryChain(geometryGenerator, 0);

    BuildRenderItemsSandbox(geometryGenerator, lastIndex);
}
void PhysicsApplication::SetupDemoSceneHinge(GeometryGenerator& geometryGenerator) {
    int lastIndex = AddHinge(mBodies, mConstraints, geometryStartEndIndices, 0);
    BuildGeometryHinge(geometryGenerator, 0);

    BuildRenderItemsSandbox(geometryGenerator, lastIndex);
}
void PhysicsApplication::SetupDemoSceneVelocity(GeometryGenerator& geometryGenerator) {
    int lastIndex = AddVelocity(mBodies, mConstraints, geometryStartEndIndices, 0);
    BuildGeometryVelocity(geometryGenerator, 0);

    BuildRenderItemsSandbox(geometryGenerator, lastIndex);
}
void PhysicsApplication::SetupDemoSceneOrientation(GeometryGenerator& geometryGenerator) {
    int lastIndex = AddOrientation(mBodies, mConstraints, geometryStartEndIndices, 0);
    BuildGeometryOrientation(geometryGenerator, 0);

    BuildRenderItemsSandbox(geometryGenerator, lastIndex);
}
void PhysicsApplication::SetupDemoSceneSpinner(GeometryGenerator& geometryGenerator) {
    int lastIndex = AddSpinner(mBodies, mConstraints, geometryStartEndIndices, 0);
    BuildGeometrySpinner(geometryGenerator, 0);

    BuildRenderItemsSandbox(geometryGenerator, lastIndex);
}
void PhysicsApplication::SetupDemoSceneRagdoll(GeometryGenerator& geometryGenerator) {
    int lastIndex = AddRagdoll(mBodies, mConstraints, geometryStartEndIndices, 0);
    BuildGeometryRagdoll(geometryGenerator, 0);

    BuildRenderItemsSandbox(geometryGenerator, lastIndex);
}
void PhysicsApplication::SetupDemoSceneConvex(GeometryGenerator& geometryGenerator) {
    int lastIndex = AddConvex(mBodies, geometryStartEndIndices, 0);
    BuildGeometryConvex(geometryGenerator, 0);

    BuildRenderItemsSandbox(geometryGenerator, lastIndex);
}
void PhysicsApplication::SetupDemoSceneAll(GeometryGenerator& geometryGenerator) {
    int lastIndex = 0, currentIndex = 0;
    std::vector<int> indices;

    mBodies.reserve(GeneralData::NumSandboxMaxBodyCount);

    lastIndex = AddStack(mBodies, geometryStartEndIndices, 0);
    lastIndex = AddMover(mBodies, mConstraints, geometryStartEndIndices, lastIndex);
    indices.push_back(lastIndex);
    lastIndex = AddChain(mBodies, mConstraints, geometryStartEndIndices, lastIndex);
    indices.push_back(lastIndex);
    /*
    // todo : skip this part for now // temp
    lastIndex = AddHinge(mBodies, mConstraints, geometryStartEndIndices, lastIndex);
    indices.push_back(lastIndex);
    lastIndex = AddVelocity(mBodies, mConstraints, geometryStartEndIndices, lastIndex);
    indices.push_back(lastIndex);
    lastIndex = AddOrientation(mBodies, mConstraints, geometryStartEndIndices, lastIndex);
    indices.push_back(lastIndex);
    */
    lastIndex = AddSpinner(mBodies, mConstraints, geometryStartEndIndices, lastIndex);
    indices.push_back(lastIndex);
    lastIndex = AddRagdoll(mBodies, mConstraints, geometryStartEndIndices, lastIndex);
    indices.push_back(lastIndex);
    lastIndex = AddConvex(mBodies, geometryStartEndIndices, lastIndex);

    BuildGeometryStack(geometryGenerator, 0);
    BuildGeometryMover(geometryGenerator);
    BuildGeometryChain(geometryGenerator, indices[currentIndex++]);
    /*
    // todo : skip this part for now // temp
    BuildGeometryHinge(geometryGenerator, indices[currentIndex++]);
    BuildGeometryVelocity(geometryGenerator, indices[currentIndex++]);
    BuildGeometryOrientation(geometryGenerator, indices[currentIndex++]);
    */
    BuildGeometrySpinner(geometryGenerator, indices[currentIndex++]);
    BuildGeometryRagdoll(geometryGenerator, indices[currentIndex++]);
    BuildGeometryConvex(geometryGenerator, indices[currentIndex++]);

    BuildRenderItemsSandbox(geometryGenerator, lastIndex);
}

void PhysicsApplication::BuildRenderItemsSandbox(GeometryGenerator& geometryGenerator, int lastIndex) {
    lastIndex = AddSandbox(mBodies, geometryStartEndIndices, lastIndex);
    BuildGeometrySandbox(geometryGenerator);
    unsigned sandboxIndex = lastIndex - GeneralData::NumSandboxGroundBodies;

    XMMATRIX textureTransform = XMMatrixScaling(1.0f, 1.0f, 1.0f);
    for (const auto& currentBody : mBodies)
    {
        auto currentRenderItem = std::make_unique<RenderItem>();

        if (currentBody.m_id > sandboxIndex) {
            XMVECTOR positionVector = XMVectorSet(currentBody.m_position.x,
                currentBody.m_position.z - 0.5f,
                -currentBody.m_position.y,
                0.0f);
            XMStoreFloat4x4(&currentRenderItem->World, XMMatrixTranslationFromVector(positionVector));
        }
        else {
            XMMATRIX translationMatrix = XMMatrixTranslation(currentBody.m_position.x, currentBody.m_position.z, -currentBody.m_position.y);
            XMStoreFloat4x4(&currentRenderItem->World, translationMatrix);
        }

        XMStoreFloat4x4(&currentRenderItem->TexTransform, textureTransform);
        currentRenderItem->ObjCBIndex = currentBody.m_id;
        currentRenderItem->Mat = mMaterials[currentBody.m_materialName].get();
        currentRenderItem->Geo = mGeometries[currentBody.m_geometryName].get();
        currentRenderItem->PrimitiveType = D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST;

        currentRenderItem->IndexCount = currentRenderItem->Geo->DrawArgs[currentBody.m_objectName].IndexCount;
        currentRenderItem->StartIndexLocation = currentRenderItem->Geo->DrawArgs[currentBody.m_objectName].StartIndexLocation;
        currentRenderItem->BaseVertexLocation = currentRenderItem->Geo->DrawArgs[currentBody.m_objectName].BaseVertexLocation;

        mRitemLayer[(int)RenderLayer::Opaque].push_back(currentRenderItem.get());
        mAllRitems.push_back(std::move(currentRenderItem));
    }
}

// construct render items for contact points
void PhysicsApplication::BuildRenderItemsContactPoints() 
{
    XMMATRIX textureTransform = XMMatrixScaling(1.0f, 1.0f, 1.0f);
    for (size_t currentIndex = 0, endIndex = mContactPoints.size(); currentIndex < endIndex; ++currentIndex)
    {
        auto currentRenderItem = std::make_unique<RenderItem>();

        XMMATRIX translationMatrix = XMMatrixTranslation(mContactPoints[currentIndex].ptOnA_LocalSpace.x, 
                                                         mContactPoints[currentIndex].ptOnA_LocalSpace.z, 
                                                        -mContactPoints[currentIndex].ptOnA_LocalSpace.y);
        XMStoreFloat4x4(&currentRenderItem->World, translationMatrix);

        XMStoreFloat4x4(&currentRenderItem->TexTransform, textureTransform);
        currentRenderItem->ObjCBIndex = static_cast<UINT>(currentIndex);

        mRenderItemsContactPoints.push_back(std::move(currentRenderItem));
    }
}

// build geometry : Stress Test
void PhysicsApplication::BuildGeometrySpheres(GeometryGenerator& geoGen) {
    // 1. Create a single Geosphere mesh (the base geometry for all instances).
    GeometryGenerator::MeshData sphere = geoGen.CreateGeosphere(mBodies[0].m_shape->GetBounds().WidthX() / 2, 2);

    // 2. Define the single Submesh Geometry for this Mesh.
    // BaseVertexLocation and StartIndexLocation are both 0 as there is only one mesh.
    SubmeshGeometry submesh;
    submesh.IndexCount = (UINT)sphere.Indices32.size();
    submesh.StartIndexLocation = 0;
    submesh.BaseVertexLocation = 0;

    // 3. Consolidate Vertex and Index Data
    std::vector<Vertex> vertices((size_t)sphere.Vertices.size());
    for (size_t i = 0; i < sphere.Vertices.size(); ++i)
    {
        vertices[i].Pos = sphere.Vertices[i].Position;
        vertices[i].Normal = sphere.Vertices[i].Normal;
        vertices[i].TexC = sphere.Vertices[i].TexC;
        vertices[i].TangentU = sphere.Vertices[i].TangentU;
    }
    std::vector<std::uint16_t> indices = sphere.GetIndices16();


    // 4. Create MeshGeometry and Copy to GPU Buffers
    const UINT vbByteSize = (UINT)vertices.size() * sizeof(Vertex);
    const UINT ibByteSize = (UINT)indices.size() * sizeof(std::uint16_t);

    auto geo = std::make_unique<MeshGeometry>();
    geo->Name = GeneralData::Geometry::RenderObjectNames::Spheres;

    ThrowIfFailed(D3DCreateBlob(vbByteSize, &geo->VertexBufferCPU));
    CopyMemory(geo->VertexBufferCPU->GetBufferPointer(), vertices.data(), vbByteSize);

    ThrowIfFailed(D3DCreateBlob(ibByteSize, &geo->IndexBufferCPU));
    CopyMemory(geo->IndexBufferCPU->GetBufferPointer(), indices.data(), ibByteSize);

    geo->VertexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
        mCommandList.Get(), vertices.data(), vbByteSize, geo->VertexBufferUploader);

    geo->IndexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
        mCommandList.Get(), indices.data(), ibByteSize, geo->IndexBufferUploader);

    geo->VertexByteStride = sizeof(Vertex);
    geo->VertexBufferByteSize = vbByteSize;
    geo->IndexFormat = DXGI_FORMAT_R16_UINT;
    geo->IndexBufferByteSize = ibByteSize;


    // 5. CRITICAL: Register the single Submesh using the Geometry Name as the key.
    // This allows DrawRenderItems to easily fetch the DrawArgs for the single instanced draw call.
    geo->DrawArgs[geo->Name] = submesh;

    geo->Vertices = vertices;
    geo->Indices = indices;
    mGeometries[geo->Name] = std::move(geo);
}
void PhysicsApplication::BuildGeometryDiamonds(GeometryGenerator& geoGen) {
    // The Instancing approach requires defining the geometry only once.
    
    // 1. Create a single Convex mesh.
    ShapeConvex* diamondShape = dynamic_cast<ShapeConvex*>(mBodies[0].m_shape);
    GeometryGenerator::MeshData diamond = geoGen.CreateConvex(diamondShape);

    // 2. Define the single Submesh Geometry for this Mesh.
    SubmeshGeometry submesh;
    submesh.IndexCount = (UINT)diamond.Indices32.size();
    submesh.StartIndexLocation = 0;
    submesh.BaseVertexLocation = 0;

    // 3. Consolidate Vertex and Index Data
    std::vector<Vertex> vertices((size_t)diamond.Vertices.size());
    for (size_t i = 0; i < diamond.Vertices.size(); ++i)
    {
        vertices[i].Pos = diamond.Vertices[i].Position;
        vertices[i].Normal = diamond.Vertices[i].Normal;
        vertices[i].TexC = diamond.Vertices[i].TexC;
        vertices[i].TangentU = diamond.Vertices[i].TangentU;
    }
    std::vector<std::uint16_t> indices = diamond.GetIndices16();


    // 4. Create MeshGeometry and Copy to GPU Buffers
    const UINT vbByteSize = (UINT)vertices.size() * sizeof(Vertex);
    const UINT ibByteSize = (UINT)indices.size() * sizeof(std::uint16_t);

    auto geo = std::make_unique<MeshGeometry>();
    geo->Name = GeneralData::Geometry::RenderObjectNames::Diamonds;

    ThrowIfFailed(D3DCreateBlob(vbByteSize, &geo->VertexBufferCPU));
    CopyMemory(geo->VertexBufferCPU->GetBufferPointer(), vertices.data(), vbByteSize);

    ThrowIfFailed(D3DCreateBlob(ibByteSize, &geo->IndexBufferCPU));
    CopyMemory(geo->IndexBufferCPU->GetBufferPointer(), indices.data(), ibByteSize);

    geo->VertexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
        mCommandList.Get(), vertices.data(), vbByteSize, geo->VertexBufferUploader);

    geo->IndexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
        mCommandList.Get(), indices.data(), ibByteSize, geo->IndexBufferUploader);

    geo->VertexByteStride = sizeof(Vertex);
    geo->VertexBufferByteSize = vbByteSize;
    geo->IndexFormat = DXGI_FORMAT_R16_UINT;
    geo->IndexBufferByteSize = ibByteSize;

    // 5. CRITICAL: Register the single Submesh using the Geometry Name as the key.
    // This allows DrawRenderItems to use this single definition for Instancing.
    geo->DrawArgs[geo->Name] = submesh;

    geo->Vertices = vertices;
    geo->Indices = indices;
    mGeometries[geo->Name] = std::move(geo);
}

void PhysicsApplication::BuildGeometryFloor(GeometryGenerator& geoGen) {
    const unsigned stressSqaured = mStressLevel * mStressLevel;

    Shape* prototypeFloorShape = mBodies[stressSqaured].m_shape;
    float radius = prototypeFloorShape->GetBounds().WidthX() / 2;
    GeometryGenerator::MeshData floorMesh = geoGen.CreateGeosphere(radius, 3);

    // 1. Define the single Submesh Geometry for this Mesh.
    SubmeshGeometry submesh;
    submesh.IndexCount = (UINT)floorMesh.Indices32.size();
    submesh.StartIndexLocation = 0;
    submesh.BaseVertexLocation = 0;

    // 2. Consolidate Vertex and Index Data (Only for the single prototype mesh)
    std::vector<Vertex> vertices((size_t)floorMesh.Vertices.size());
    for (size_t i = 0; i < floorMesh.Vertices.size(); ++i)
    {
        vertices[i].Pos = floorMesh.Vertices[i].Position;
        vertices[i].Normal = floorMesh.Vertices[i].Normal;
        vertices[i].TexC = floorMesh.Vertices[i].TexC;
        vertices[i].TangentU = floorMesh.Vertices[i].TangentU;
    }
    std::vector<std::uint16_t> indices = floorMesh.GetIndices16();


    // 3. Create MeshGeometry and Copy to GPU Buffers
    const UINT vbByteSize = (UINT)vertices.size() * sizeof(Vertex);
    const UINT ibByteSize = (UINT)indices.size() * sizeof(std::uint16_t);

    auto geo = std::make_unique<MeshGeometry>();
    geo->Name = GeneralData::Geometry::RenderObjectNames::Floor; // "Floor"

    // CPU Buffer creation
    ThrowIfFailed(D3DCreateBlob(vbByteSize, &geo->VertexBufferCPU));
    CopyMemory(geo->VertexBufferCPU->GetBufferPointer(), vertices.data(), vbByteSize);

    ThrowIfFailed(D3DCreateBlob(ibByteSize, &geo->IndexBufferCPU));
    CopyMemory(geo->IndexBufferCPU->GetBufferPointer(), indices.data(), ibByteSize);

    // GPU Buffer creation
    geo->VertexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
        mCommandList.Get(), vertices.data(), vbByteSize, geo->VertexBufferUploader);

    geo->IndexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
        mCommandList.Get(), indices.data(), ibByteSize, geo->IndexBufferUploader);

    geo->VertexByteStride = sizeof(Vertex);
    geo->VertexBufferByteSize = vbByteSize;
    geo->IndexFormat = DXGI_FORMAT_R16_UINT;
    geo->IndexBufferByteSize = ibByteSize;


    // 4. Register the single Submesh using the Geometry Name as the key.
    geo->DrawArgs[geo->Name] = submesh;

    geo->Vertices = vertices;
    geo->Indices = indices;
    mGeometries[geo->Name] = std::move(geo);
}


// build geometry : Sandbox
void PhysicsApplication::BuildGeometryStack(GeometryGenerator& geoGen, int lastIndex) {
    int currentBodyIndex = lastIndex;
    Shape* tempBox = mBodies[currentBodyIndex].m_shape;
    GeometryGenerator::MeshData boxA = geoGen.CreateBox(tempBox->GetBounds().WidthX(), tempBox->GetBounds().WidthZ(), tempBox->GetBounds().WidthY(), 2);
    ++currentBodyIndex;
    tempBox = mBodies[currentBodyIndex].m_shape;
    GeometryGenerator::MeshData boxB = geoGen.CreateBox(tempBox->GetBounds().WidthX(), tempBox->GetBounds().WidthZ(), tempBox->GetBounds().WidthY(), 2);
    ++currentBodyIndex;
    tempBox = mBodies[currentBodyIndex].m_shape;
    GeometryGenerator::MeshData boxC = geoGen.CreateBox(tempBox->GetBounds().WidthX(), tempBox->GetBounds().WidthZ(), tempBox->GetBounds().WidthY(), 2);
    ++currentBodyIndex;
    tempBox = mBodies[currentBodyIndex].m_shape;
    GeometryGenerator::MeshData boxD = geoGen.CreateBox(tempBox->GetBounds().WidthX(), tempBox->GetBounds().WidthZ(), tempBox->GetBounds().WidthY(), 2);
    ++currentBodyIndex;
    tempBox = mBodies[currentBodyIndex].m_shape;
    GeometryGenerator::MeshData boxE = geoGen.CreateBox(tempBox->GetBounds().WidthX(), tempBox->GetBounds().WidthZ(), tempBox->GetBounds().WidthY(), 2);
    //
    // We are concatenating all the geometry into one big vertex/index buffer.  So
    // define the regions in the buffer each submesh covers.
    //

    // Cache the vertex offsets to each object in the concatenated vertex buffer.
    UINT boxAVertexOffset = 0;
    UINT boxBVertexOffset = (UINT)boxA.Vertices.size();
    UINT boxCVertexOffset = boxBVertexOffset + (UINT)boxB.Vertices.size();
    UINT boxDVertexOffset = boxCVertexOffset + (UINT)boxC.Vertices.size();
    UINT boxEVertexOffset = boxDVertexOffset + (UINT)boxD.Vertices.size();

    // Cache the starting index for each object in the concatenated index buffer.
    UINT boxAIndexOffset = 0;
    UINT boxBIndexOffset = (UINT)boxA.Indices32.size();
    UINT boxCIndexOffset = boxBIndexOffset + (UINT)boxB.Indices32.size();
    UINT boxDIndexOffset = boxCIndexOffset + (UINT)boxC.Indices32.size();
    UINT boxEIndexOffset = boxDIndexOffset + (UINT)boxD.Indices32.size();

    // Define the SubmeshGeometry that cover different 
    // regions of the vertex/index buffers.

    SubmeshGeometry boxASubmesh;
    boxASubmesh.IndexCount = (UINT)boxA.Indices32.size();
    boxASubmesh.StartIndexLocation = boxAIndexOffset;
    boxASubmesh.BaseVertexLocation = boxAVertexOffset;

    SubmeshGeometry boxBSubmesh;
    boxBSubmesh.IndexCount = (UINT)boxB.Indices32.size();
    boxBSubmesh.StartIndexLocation = boxBIndexOffset;
    boxBSubmesh.BaseVertexLocation = boxBVertexOffset;

    SubmeshGeometry boxCSubmesh;
    boxCSubmesh.IndexCount = (UINT)boxC.Indices32.size();
    boxCSubmesh.StartIndexLocation = boxCIndexOffset;
    boxCSubmesh.BaseVertexLocation = boxCVertexOffset;

    SubmeshGeometry boxDSubmesh;
    boxDSubmesh.IndexCount = (UINT)boxD.Indices32.size();
    boxDSubmesh.StartIndexLocation = boxDIndexOffset;
    boxDSubmesh.BaseVertexLocation = boxDVertexOffset;

    SubmeshGeometry boxESubmesh;
    boxESubmesh.IndexCount = (UINT)boxE.Indices32.size();
    boxESubmesh.StartIndexLocation = boxEIndexOffset;
    boxESubmesh.BaseVertexLocation = boxEVertexOffset;

    //
    // Extract the vertex elements we are interested in and pack the
    // vertices of all the meshes into one vertex buffer.
    //

    auto totalVertexCount =
        boxA.Vertices.size() +
        boxB.Vertices.size() +
        boxC.Vertices.size() +
        boxD.Vertices.size() +
        boxE.Vertices.size();

    std::vector<Vertex> vertices(totalVertexCount);

    std::vector<GeometryGenerator::MeshData*> items;
    items.emplace_back(&boxA);
    items.emplace_back(&boxB);
    items.emplace_back(&boxC);
    items.emplace_back(&boxD);
    items.emplace_back(&boxE);

    UINT k = 0;
    for (auto currentItem : items) {
        for (size_t i = 0; i < currentItem->Vertices.size(); ++i, ++k)
        {
            vertices[k].Pos = currentItem->Vertices[i].Position;
            vertices[k].Normal = currentItem->Vertices[i].Normal;
            vertices[k].TexC = currentItem->Vertices[i].TexC;
            vertices[k].TangentU = currentItem->Vertices[i].TangentU;
        }
    }


    std::vector<std::uint16_t> indices;
    indices.insert(indices.end(), std::begin(boxA.GetIndices16()), std::end(boxA.GetIndices16()));
    indices.insert(indices.end(), std::begin(boxB.GetIndices16()), std::end(boxB.GetIndices16()));
    indices.insert(indices.end(), std::begin(boxC.GetIndices16()), std::end(boxC.GetIndices16()));
    indices.insert(indices.end(), std::begin(boxD.GetIndices16()), std::end(boxD.GetIndices16()));
    indices.insert(indices.end(), std::begin(boxE.GetIndices16()), std::end(boxE.GetIndices16()));

    const UINT vbByteSize = (UINT)vertices.size() * sizeof(Vertex);
    const UINT ibByteSize = (UINT)indices.size() * sizeof(std::uint16_t);

    auto geo = std::make_unique<MeshGeometry>();
    geo->Name = GeneralData::Geometry::RenderObjectNames::Stack;

    ThrowIfFailed(D3DCreateBlob(vbByteSize, &geo->VertexBufferCPU));
    CopyMemory(geo->VertexBufferCPU->GetBufferPointer(), vertices.data(), vbByteSize);

    ThrowIfFailed(D3DCreateBlob(ibByteSize, &geo->IndexBufferCPU));
    CopyMemory(geo->IndexBufferCPU->GetBufferPointer(), indices.data(), ibByteSize);

    geo->VertexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
        mCommandList.Get(), vertices.data(), vbByteSize, geo->VertexBufferUploader);

    geo->IndexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
        mCommandList.Get(), indices.data(), ibByteSize, geo->IndexBufferUploader);

    geo->VertexByteStride = sizeof(Vertex);
    geo->VertexBufferByteSize = vbByteSize;
    geo->IndexFormat = DXGI_FORMAT_R16_UINT;
    geo->IndexBufferByteSize = ibByteSize;

    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Stack::BoxA] = boxASubmesh;
    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Stack::BoxB] = boxBSubmesh;
    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Stack::BoxC] = boxCSubmesh;
    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Stack::BoxD] = boxDSubmesh;
    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Stack::BoxE] = boxESubmesh;


    geo->Vertices = vertices;
    geo->Indices = indices;
    mGeometries[geo->Name] = std::move(geo);
}
void PhysicsApplication::BuildGeometryMover(GeometryGenerator& geoGen) {
    ShapeBox tempBox(g_boxPlatform, sizeof(g_boxPlatform) / sizeof(Vec3));
    GeometryGenerator::MeshData boxPlatform = geoGen.CreateBox(tempBox.m_bounds.WidthX(), tempBox.m_bounds.WidthZ(), tempBox.m_bounds.WidthY(), 2);
    tempBox = ShapeBox(g_boxUnit, sizeof(g_boxUnit) / sizeof(Vec3));
    GeometryGenerator::MeshData boxUnit = geoGen.CreateBox(tempBox.m_bounds.WidthX(), tempBox.m_bounds.WidthZ(), tempBox.m_bounds.WidthY(), 2);

    //
    // We are concatenating all the geometry into one big vertex/index buffer.  So
    // define the regions in the buffer each submesh covers.
    //

    // Cache the vertex offsets to each object in the concatenated vertex buffer.
    UINT boxPlatformVertexOffset = 0;
    UINT boxUnitVertexOffset = (UINT)boxPlatform.Vertices.size();

    // Cache the starting index for each object in the concatenated index buffer.
    UINT boxPlatformIndexOffset = 0;
    UINT boxUnitIndexOffset = (UINT)boxPlatform.Indices32.size();

    // Define the SubmeshGeometry that cover different 
    // regions of the vertex/index buffers.

    SubmeshGeometry boxPlatformSubmesh;
    boxPlatformSubmesh.IndexCount = (UINT)boxPlatform.Indices32.size();
    boxPlatformSubmesh.StartIndexLocation = boxPlatformIndexOffset;
    boxPlatformSubmesh.BaseVertexLocation = boxPlatformVertexOffset;

    SubmeshGeometry boxUnitSubmesh;
    boxUnitSubmesh.IndexCount = (UINT)boxUnit.Indices32.size();
    boxUnitSubmesh.StartIndexLocation = boxUnitIndexOffset;
    boxUnitSubmesh.BaseVertexLocation = boxUnitVertexOffset;


    //
    // Extract the vertex elements we are interested in and pack the
    // vertices of all the meshes into one vertex buffer.
    //

    auto totalVertexCount =
        boxPlatform.Vertices.size() +
        boxUnit.Vertices.size();

    std::vector<Vertex> vertices(totalVertexCount);

    std::vector<GeometryGenerator::MeshData*> items;
    items.emplace_back(&boxPlatform);
    items.emplace_back(&boxUnit);

    UINT k = 0;
    for (auto currentItem : items) {
        for (size_t i = 0; i < currentItem->Vertices.size(); ++i, ++k)
        {
            vertices[k].Pos = currentItem->Vertices[i].Position;
            vertices[k].Normal = currentItem->Vertices[i].Normal;
            vertices[k].TexC = currentItem->Vertices[i].TexC;
            vertices[k].TangentU = currentItem->Vertices[i].TangentU;
        }
    }


    std::vector<std::uint16_t> indices;
    indices.insert(indices.end(), std::begin(boxPlatform.GetIndices16()), std::end(boxPlatform.GetIndices16()));
    indices.insert(indices.end(), std::begin(boxUnit.GetIndices16()), std::end(boxUnit.GetIndices16()));

    const UINT vbByteSize = (UINT)vertices.size() * sizeof(Vertex);
    const UINT ibByteSize = (UINT)indices.size() * sizeof(std::uint16_t);

    auto geo = std::make_unique<MeshGeometry>();
    geo->Name = GeneralData::Geometry::RenderObjectNames::Mover;

    ThrowIfFailed(D3DCreateBlob(vbByteSize, &geo->VertexBufferCPU));
    CopyMemory(geo->VertexBufferCPU->GetBufferPointer(), vertices.data(), vbByteSize);

    ThrowIfFailed(D3DCreateBlob(ibByteSize, &geo->IndexBufferCPU));
    CopyMemory(geo->IndexBufferCPU->GetBufferPointer(), indices.data(), ibByteSize);

    geo->VertexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
        mCommandList.Get(), vertices.data(), vbByteSize, geo->VertexBufferUploader);

    geo->IndexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
        mCommandList.Get(), indices.data(), ibByteSize, geo->IndexBufferUploader);

    geo->VertexByteStride = sizeof(Vertex);
    geo->VertexBufferByteSize = vbByteSize;
    geo->IndexFormat = DXGI_FORMAT_R16_UINT;
    geo->IndexBufferByteSize = ibByteSize;

    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Mover::Platform] = boxPlatformSubmesh;
    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Mover::Unit] = boxUnitSubmesh;

    geo->Vertices = vertices;
    geo->Indices = indices;
    mGeometries[geo->Name] = std::move(geo);
}
void PhysicsApplication::BuildGeometryChain(GeometryGenerator& geoGen, int lastIndex) {
    int currentBodyIndex = lastIndex;
    Shape* tempBox = mBodies[currentBodyIndex].m_shape;
    GeometryGenerator::MeshData boxTop = geoGen.CreateBox(tempBox->GetBounds().WidthX(), tempBox->GetBounds().WidthZ(), tempBox->GetBounds().WidthY(), 2);
    ++currentBodyIndex;
    tempBox = mBodies[currentBodyIndex].m_shape;
    GeometryGenerator::MeshData boxA = geoGen.CreateBox(tempBox->GetBounds().WidthX(), tempBox->GetBounds().WidthZ(), tempBox->GetBounds().WidthY(), 2);
    ++currentBodyIndex;
    tempBox = mBodies[currentBodyIndex].m_shape;
    GeometryGenerator::MeshData boxB = geoGen.CreateBox(tempBox->GetBounds().WidthX(), tempBox->GetBounds().WidthZ(), tempBox->GetBounds().WidthY(), 2);
    ++currentBodyIndex;
    tempBox = mBodies[currentBodyIndex].m_shape;
    GeometryGenerator::MeshData boxC = geoGen.CreateBox(tempBox->GetBounds().WidthX(), tempBox->GetBounds().WidthZ(), tempBox->GetBounds().WidthY(), 2);
    ++currentBodyIndex;
    tempBox = mBodies[currentBodyIndex].m_shape;
    GeometryGenerator::MeshData boxD = geoGen.CreateBox(tempBox->GetBounds().WidthX(), tempBox->GetBounds().WidthZ(), tempBox->GetBounds().WidthY(), 2);
    ++currentBodyIndex;
    tempBox = mBodies[currentBodyIndex].m_shape;
    GeometryGenerator::MeshData boxE = geoGen.CreateBox(tempBox->GetBounds().WidthX(), tempBox->GetBounds().WidthZ(), tempBox->GetBounds().WidthY(), 2);
    //
    // We are concatenating all the geometry into one big vertex/index buffer.  So
    // define the regions in the buffer each submesh covers.
    //

    // Cache the vertex offsets to each object in the concatenated vertex buffer.
    UINT boxTopVertexOffset = 0;
    UINT boxAVertexOffset = (UINT)boxTop.Vertices.size();
    UINT boxBVertexOffset = boxAVertexOffset + (UINT)boxA.Vertices.size();
    UINT boxCVertexOffset = boxBVertexOffset + (UINT)boxB.Vertices.size();
    UINT boxDVertexOffset = boxCVertexOffset + (UINT)boxC.Vertices.size();
    UINT boxEVertexOffset = boxDVertexOffset + (UINT)boxD.Vertices.size();

    // Cache the starting index for each object in the concatenated index buffer.
    UINT boxTopIndexOffset = 0;
    UINT boxAIndexOffset = (UINT)boxTop.Indices32.size();
    UINT boxBIndexOffset = boxAIndexOffset + (UINT)boxA.Indices32.size();
    UINT boxCIndexOffset = boxBIndexOffset + (UINT)boxB.Indices32.size();
    UINT boxDIndexOffset = boxCIndexOffset + (UINT)boxC.Indices32.size();
    UINT boxEIndexOffset = boxDIndexOffset + (UINT)boxD.Indices32.size();

    // Define the SubmeshGeometry that cover different 
    // regions of the vertex/index buffers.

    SubmeshGeometry boxTopSubmesh;
    boxTopSubmesh.IndexCount = (UINT)boxTop.Indices32.size();
    boxTopSubmesh.StartIndexLocation = boxTopIndexOffset;
    boxTopSubmesh.BaseVertexLocation = boxTopVertexOffset;

    SubmeshGeometry boxASubmesh;
    boxASubmesh.IndexCount = (UINT)boxA.Indices32.size();
    boxASubmesh.StartIndexLocation = boxAIndexOffset;
    boxASubmesh.BaseVertexLocation = boxAVertexOffset;

    SubmeshGeometry boxBSubmesh;
    boxBSubmesh.IndexCount = (UINT)boxB.Indices32.size();
    boxBSubmesh.StartIndexLocation = boxBIndexOffset;
    boxBSubmesh.BaseVertexLocation = boxBVertexOffset;

    SubmeshGeometry boxCSubmesh;
    boxCSubmesh.IndexCount = (UINT)boxC.Indices32.size();
    boxCSubmesh.StartIndexLocation = boxCIndexOffset;
    boxCSubmesh.BaseVertexLocation = boxCVertexOffset;

    SubmeshGeometry boxDSubmesh;
    boxDSubmesh.IndexCount = (UINT)boxD.Indices32.size();
    boxDSubmesh.StartIndexLocation = boxDIndexOffset;
    boxDSubmesh.BaseVertexLocation = boxDVertexOffset;

    SubmeshGeometry boxESubmesh;
    boxESubmesh.IndexCount = (UINT)boxE.Indices32.size();
    boxESubmesh.StartIndexLocation = boxEIndexOffset;
    boxESubmesh.BaseVertexLocation = boxEVertexOffset;

    //
    // Extract the vertex elements we are interested in and pack the
    // vertices of all the meshes into one vertex buffer.
    //

    auto totalVertexCount =
        boxTop.Vertices.size() +
        boxA.Vertices.size() +
        boxB.Vertices.size() +
        boxC.Vertices.size() +
        boxD.Vertices.size() +
        boxE.Vertices.size();

    std::vector<Vertex> vertices(totalVertexCount);

    std::vector<GeometryGenerator::MeshData*> items;
    items.emplace_back(&boxTop);
    items.emplace_back(&boxA);
    items.emplace_back(&boxB);
    items.emplace_back(&boxC);
    items.emplace_back(&boxD);
    items.emplace_back(&boxE);

    UINT k = 0;
    for (auto currentItem : items) {
        for (size_t i = 0; i < currentItem->Vertices.size(); ++i, ++k)
        {
            vertices[k].Pos = currentItem->Vertices[i].Position;
            vertices[k].Normal = currentItem->Vertices[i].Normal;
            vertices[k].TexC = currentItem->Vertices[i].TexC;
            vertices[k].TangentU = currentItem->Vertices[i].TangentU;
        }
    }


    std::vector<std::uint16_t> indices;
    indices.insert(indices.end(), std::begin(boxTop.GetIndices16()), std::end(boxTop.GetIndices16()));
    indices.insert(indices.end(), std::begin(boxA.GetIndices16()), std::end(boxA.GetIndices16()));
    indices.insert(indices.end(), std::begin(boxB.GetIndices16()), std::end(boxB.GetIndices16()));
    indices.insert(indices.end(), std::begin(boxC.GetIndices16()), std::end(boxC.GetIndices16()));
    indices.insert(indices.end(), std::begin(boxD.GetIndices16()), std::end(boxD.GetIndices16()));
    indices.insert(indices.end(), std::begin(boxE.GetIndices16()), std::end(boxE.GetIndices16()));

    const UINT vbByteSize = (UINT)vertices.size() * sizeof(Vertex);
    const UINT ibByteSize = (UINT)indices.size() * sizeof(std::uint16_t);

    auto geo = std::make_unique<MeshGeometry>();
    geo->Name = GeneralData::Geometry::RenderObjectNames::Chain;

    ThrowIfFailed(D3DCreateBlob(vbByteSize, &geo->VertexBufferCPU));
    CopyMemory(geo->VertexBufferCPU->GetBufferPointer(), vertices.data(), vbByteSize);

    ThrowIfFailed(D3DCreateBlob(ibByteSize, &geo->IndexBufferCPU));
    CopyMemory(geo->IndexBufferCPU->GetBufferPointer(), indices.data(), ibByteSize);

    geo->VertexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
        mCommandList.Get(), vertices.data(), vbByteSize, geo->VertexBufferUploader);

    geo->IndexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
        mCommandList.Get(), indices.data(), ibByteSize, geo->IndexBufferUploader);

    geo->VertexByteStride = sizeof(Vertex);
    geo->VertexBufferByteSize = vbByteSize;
    geo->IndexFormat = DXGI_FORMAT_R16_UINT;
    geo->IndexBufferByteSize = ibByteSize;

    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Chain::TopBox] = boxTopSubmesh;
    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Chain::BoxA] = boxASubmesh;
    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Chain::BoxB] = boxBSubmesh;
    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Chain::BoxC] = boxCSubmesh;
    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Chain::BoxD] = boxDSubmesh;
    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Chain::BoxE] = boxDSubmesh;


    geo->Vertices = vertices;
    geo->Indices = indices;
    mGeometries[geo->Name] = std::move(geo);
}
void PhysicsApplication::BuildGeometryHinge(GeometryGenerator& geoGen, int lastIndex) {
    int currentBodyIndex = lastIndex;
    Shape* tempBox = mBodies[currentBodyIndex].m_shape;
    GeometryGenerator::MeshData boxA = geoGen.CreateBox(tempBox->GetBounds().WidthX(), tempBox->GetBounds().WidthZ(), tempBox->GetBounds().WidthY(), 2);
    ++currentBodyIndex;
    tempBox = mBodies[currentBodyIndex].m_shape;
    GeometryGenerator::MeshData boxB = geoGen.CreateBox(tempBox->GetBounds().WidthX(), tempBox->GetBounds().WidthZ(), tempBox->GetBounds().WidthY(), 2);
    //
    // We are concatenating all the geometry into one big vertex/index buffer.  So
    // define the regions in the buffer each submesh covers.
    //

    // Cache the vertex offsets to each object in the concatenated vertex buffer.
    UINT boxAVertexOffset = 0;
    UINT boxBVertexOffset = (UINT)boxA.Vertices.size();

    // Cache the starting index for each object in the concatenated index buffer.
    UINT boxAIndexOffset = 0;
    UINT boxBIndexOffset = (UINT)boxA.Indices32.size();

    // Define the SubmeshGeometry that cover different 
    // regions of the vertex/index buffers.

    SubmeshGeometry boxASubmesh;
    boxASubmesh.IndexCount = (UINT)boxA.Indices32.size();
    boxASubmesh.StartIndexLocation = boxAIndexOffset;
    boxASubmesh.BaseVertexLocation = boxAVertexOffset;

    SubmeshGeometry boxBSubmesh;
    boxBSubmesh.IndexCount = (UINT)boxB.Indices32.size();
    boxBSubmesh.StartIndexLocation = boxBIndexOffset;
    boxBSubmesh.BaseVertexLocation = boxBVertexOffset;

    //
    // Extract the vertex elements we are interested in and pack the
    // vertices of all the meshes into one vertex buffer.
    //

    auto totalVertexCount =
        boxA.Vertices.size() +
        boxB.Vertices.size();

    std::vector<Vertex> vertices(totalVertexCount);

    std::vector<GeometryGenerator::MeshData*> items;
    items.emplace_back(&boxA);
    items.emplace_back(&boxB);

    UINT k = 0;
    for (auto currentItem : items) {
        for (size_t i = 0; i < currentItem->Vertices.size(); ++i, ++k)
        {
            vertices[k].Pos = currentItem->Vertices[i].Position;
            vertices[k].Normal = currentItem->Vertices[i].Normal;
            vertices[k].TexC = currentItem->Vertices[i].TexC;
            vertices[k].TangentU = currentItem->Vertices[i].TangentU;
        }
    }

    std::vector<std::uint16_t> indices;
    indices.insert(indices.end(), std::begin(boxA.GetIndices16()), std::end(boxA.GetIndices16()));
    indices.insert(indices.end(), std::begin(boxB.GetIndices16()), std::end(boxB.GetIndices16()));

    const UINT vbByteSize = (UINT)vertices.size() * sizeof(Vertex);
    const UINT ibByteSize = (UINT)indices.size() * sizeof(std::uint16_t);

    auto geo = std::make_unique<MeshGeometry>();
    geo->Name = GeneralData::Geometry::RenderObjectNames::Hinge;

    ThrowIfFailed(D3DCreateBlob(vbByteSize, &geo->VertexBufferCPU));
    CopyMemory(geo->VertexBufferCPU->GetBufferPointer(), vertices.data(), vbByteSize);

    ThrowIfFailed(D3DCreateBlob(ibByteSize, &geo->IndexBufferCPU));
    CopyMemory(geo->IndexBufferCPU->GetBufferPointer(), indices.data(), ibByteSize);

    geo->VertexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
        mCommandList.Get(), vertices.data(), vbByteSize, geo->VertexBufferUploader);

    geo->IndexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
        mCommandList.Get(), indices.data(), ibByteSize, geo->IndexBufferUploader);

    geo->VertexByteStride = sizeof(Vertex);
    geo->VertexBufferByteSize = vbByteSize;
    geo->IndexFormat = DXGI_FORMAT_R16_UINT;
    geo->IndexBufferByteSize = ibByteSize;

    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Hinge::BoxA] = boxASubmesh;
    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Hinge::BoxB] = boxBSubmesh;


    geo->Vertices = vertices;
    geo->Indices = indices;
    mGeometries[geo->Name] = std::move(geo);
}
void PhysicsApplication::BuildGeometryVelocity(GeometryGenerator& geoGen, int lastIndex) {
    int currentBodyIndex = lastIndex;
    Shape* tempBox = mBodies[currentBodyIndex].m_shape;
    GeometryGenerator::MeshData boxA = geoGen.CreateBox(tempBox->GetBounds().WidthX(), tempBox->GetBounds().WidthZ(), tempBox->GetBounds().WidthY(), 2);
    ++currentBodyIndex;
    tempBox = mBodies[currentBodyIndex].m_shape;
    GeometryGenerator::MeshData boxB = geoGen.CreateBox(tempBox->GetBounds().WidthX(), tempBox->GetBounds().WidthZ(), tempBox->GetBounds().WidthY(), 2);
    //
    // We are concatenating all the geometry into one big vertex/index buffer.  So
    // define the regions in the buffer each submesh covers.
    //

    // Cache the vertex offsets to each object in the concatenated vertex buffer.
    UINT boxAVertexOffset = 0;
    UINT boxBVertexOffset = (UINT)boxA.Vertices.size();

    // Cache the starting index for each object in the concatenated index buffer.
    UINT boxAIndexOffset = 0;
    UINT boxBIndexOffset = (UINT)boxA.Indices32.size();

    // Define the SubmeshGeometry that cover different 
    // regions of the vertex/index buffers.

    SubmeshGeometry boxASubmesh;
    boxASubmesh.IndexCount = (UINT)boxA.Indices32.size();
    boxASubmesh.StartIndexLocation = boxAIndexOffset;
    boxASubmesh.BaseVertexLocation = boxAVertexOffset;

    SubmeshGeometry boxBSubmesh;
    boxBSubmesh.IndexCount = (UINT)boxB.Indices32.size();
    boxBSubmesh.StartIndexLocation = boxBIndexOffset;
    boxBSubmesh.BaseVertexLocation = boxBVertexOffset;

    //
    // Extract the vertex elements we are interested in and pack the
    // vertices of all the meshes into one vertex buffer.
    //

    auto totalVertexCount =
        boxA.Vertices.size() +
        boxB.Vertices.size();

    std::vector<Vertex> vertices(totalVertexCount);

    std::vector<GeometryGenerator::MeshData*> items;
    items.emplace_back(&boxA);
    items.emplace_back(&boxB);

    UINT k = 0;
    for (auto currentItem : items) {
        for (size_t i = 0; i < currentItem->Vertices.size(); ++i, ++k)
        {
            vertices[k].Pos = currentItem->Vertices[i].Position;
            vertices[k].Normal = currentItem->Vertices[i].Normal;
            vertices[k].TexC = currentItem->Vertices[i].TexC;
            vertices[k].TangentU = currentItem->Vertices[i].TangentU;
        }
    }

    std::vector<std::uint16_t> indices;
    indices.insert(indices.end(), std::begin(boxA.GetIndices16()), std::end(boxA.GetIndices16()));
    indices.insert(indices.end(), std::begin(boxB.GetIndices16()), std::end(boxB.GetIndices16()));

    const UINT vbByteSize = (UINT)vertices.size() * sizeof(Vertex);
    const UINT ibByteSize = (UINT)indices.size() * sizeof(std::uint16_t);

    auto geo = std::make_unique<MeshGeometry>();
    geo->Name = GeneralData::Geometry::RenderObjectNames::Velocity;

    ThrowIfFailed(D3DCreateBlob(vbByteSize, &geo->VertexBufferCPU));
    CopyMemory(geo->VertexBufferCPU->GetBufferPointer(), vertices.data(), vbByteSize);

    ThrowIfFailed(D3DCreateBlob(ibByteSize, &geo->IndexBufferCPU));
    CopyMemory(geo->IndexBufferCPU->GetBufferPointer(), indices.data(), ibByteSize);

    geo->VertexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
        mCommandList.Get(), vertices.data(), vbByteSize, geo->VertexBufferUploader);

    geo->IndexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
        mCommandList.Get(), indices.data(), ibByteSize, geo->IndexBufferUploader);

    geo->VertexByteStride = sizeof(Vertex);
    geo->VertexBufferByteSize = vbByteSize;
    geo->IndexFormat = DXGI_FORMAT_R16_UINT;
    geo->IndexBufferByteSize = ibByteSize;

    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Velocity::BoxA] = boxASubmesh;
    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Velocity::BoxB] = boxBSubmesh;


    geo->Vertices = vertices;
    geo->Indices = indices;
    mGeometries[geo->Name] = std::move(geo);
}
void PhysicsApplication::BuildGeometryOrientation(GeometryGenerator& geoGen, int lastIndex) {
    int currentBodyIndex = lastIndex;
    Shape* tempBox = mBodies[currentBodyIndex].m_shape;
    GeometryGenerator::MeshData boxA = geoGen.CreateBox(tempBox->GetBounds().WidthX(), tempBox->GetBounds().WidthZ(), tempBox->GetBounds().WidthY(), 2);
    ++currentBodyIndex;
    tempBox = mBodies[currentBodyIndex].m_shape;
    GeometryGenerator::MeshData boxB = geoGen.CreateBox(tempBox->GetBounds().WidthX(), tempBox->GetBounds().WidthZ(), tempBox->GetBounds().WidthY(), 2);
    //
    // We are concatenating all the geometry into one big vertex/index buffer.  So
    // define the regions in the buffer each submesh covers.
    //

    // Cache the vertex offsets to each object in the concatenated vertex buffer.
    UINT boxAVertexOffset = 0;
    UINT boxBVertexOffset = (UINT)boxA.Vertices.size();

    // Cache the starting index for each object in the concatenated index buffer.
    UINT boxAIndexOffset = 0;
    UINT boxBIndexOffset = (UINT)boxA.Indices32.size();

    // Define the SubmeshGeometry that cover different 
    // regions of the vertex/index buffers.

    SubmeshGeometry boxASubmesh;
    boxASubmesh.IndexCount = (UINT)boxA.Indices32.size();
    boxASubmesh.StartIndexLocation = boxAIndexOffset;
    boxASubmesh.BaseVertexLocation = boxAVertexOffset;

    SubmeshGeometry boxBSubmesh;
    boxBSubmesh.IndexCount = (UINT)boxB.Indices32.size();
    boxBSubmesh.StartIndexLocation = boxBIndexOffset;
    boxBSubmesh.BaseVertexLocation = boxBVertexOffset;

    //
    // Extract the vertex elements we are interested in and pack the
    // vertices of all the meshes into one vertex buffer.
    //

    auto totalVertexCount =
        boxA.Vertices.size() +
        boxB.Vertices.size();

    std::vector<Vertex> vertices(totalVertexCount);

    std::vector<GeometryGenerator::MeshData*> items;
    items.emplace_back(&boxA);
    items.emplace_back(&boxB);

    UINT k = 0;
    for (auto currentItem : items) {
        for (size_t i = 0; i < currentItem->Vertices.size(); ++i, ++k)
        {
            vertices[k].Pos = currentItem->Vertices[i].Position;
            vertices[k].Normal = currentItem->Vertices[i].Normal;
            vertices[k].TexC = currentItem->Vertices[i].TexC;
            vertices[k].TangentU = currentItem->Vertices[i].TangentU;
        }
    }

    std::vector<std::uint16_t> indices;
    indices.insert(indices.end(), std::begin(boxA.GetIndices16()), std::end(boxA.GetIndices16()));
    indices.insert(indices.end(), std::begin(boxB.GetIndices16()), std::end(boxB.GetIndices16()));

    const UINT vbByteSize = (UINT)vertices.size() * sizeof(Vertex);
    const UINT ibByteSize = (UINT)indices.size() * sizeof(std::uint16_t);

    auto geo = std::make_unique<MeshGeometry>();
    geo->Name = GeneralData::Geometry::RenderObjectNames::Orientation;

    ThrowIfFailed(D3DCreateBlob(vbByteSize, &geo->VertexBufferCPU));
    CopyMemory(geo->VertexBufferCPU->GetBufferPointer(), vertices.data(), vbByteSize);

    ThrowIfFailed(D3DCreateBlob(ibByteSize, &geo->IndexBufferCPU));
    CopyMemory(geo->IndexBufferCPU->GetBufferPointer(), indices.data(), ibByteSize);

    geo->VertexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
        mCommandList.Get(), vertices.data(), vbByteSize, geo->VertexBufferUploader);

    geo->IndexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
        mCommandList.Get(), indices.data(), ibByteSize, geo->IndexBufferUploader);

    geo->VertexByteStride = sizeof(Vertex);
    geo->VertexBufferByteSize = vbByteSize;
    geo->IndexFormat = DXGI_FORMAT_R16_UINT;
    geo->IndexBufferByteSize = ibByteSize;

    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Orientation::BoxA] = boxASubmesh;
    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Orientation::BoxB] = boxBSubmesh;


    geo->Vertices = vertices;
    geo->Indices = indices;
    mGeometries[geo->Name] = std::move(geo);
}
void PhysicsApplication::BuildGeometrySpinner(GeometryGenerator& geoGen, int lastIndex) {
    int currentBodyIndex = lastIndex;
    Shape* tempBox = mBodies[currentBodyIndex].m_shape;
    GeometryGenerator::MeshData boxPivot = geoGen.CreateBox(tempBox->GetBounds().WidthX(), tempBox->GetBounds().WidthZ(), tempBox->GetBounds().WidthY(), 2);
    ++currentBodyIndex;
    tempBox = mBodies[currentBodyIndex].m_shape;
    GeometryGenerator::MeshData boxBeam = geoGen.CreateBox(tempBox->GetBounds().WidthX(), tempBox->GetBounds().WidthZ(), tempBox->GetBounds().WidthY(), 2);
    //
    // We are concatenating all the geometry into one big vertex/index buffer.  So
    // define the regions in the buffer each submesh covers.
    //

    // Cache the vertex offsets to each object in the concatenated vertex buffer.
    UINT boxPivotVertexOffset = 0;
    UINT boxBeamVertexOffset = (UINT)boxPivot.Vertices.size();

    // Cache the starting index for each object in the concatenated index buffer.
    UINT boxPivotIndexOffset = 0;
    UINT boxBeamIndexOffset = (UINT)boxPivot.Indices32.size();

    // Define the SubmeshGeometry that cover different 
    // regions of the vertex/index buffers.

    SubmeshGeometry boxPivotSubmesh;
    boxPivotSubmesh.IndexCount = (UINT)boxPivot.Indices32.size();
    boxPivotSubmesh.StartIndexLocation = boxPivotIndexOffset;
    boxPivotSubmesh.BaseVertexLocation = boxPivotVertexOffset;

    SubmeshGeometry boxBeamSubmesh;
    boxBeamSubmesh.IndexCount = (UINT)boxBeam.Indices32.size();
    boxBeamSubmesh.StartIndexLocation = boxBeamIndexOffset;
    boxBeamSubmesh.BaseVertexLocation = boxBeamVertexOffset;

    //
    // Extract the vertex elements we are interested in and pack the
    // vertices of all the meshes into one vertex buffer.
    //

    auto totalVertexCount =
        boxPivot.Vertices.size() +
        boxBeam.Vertices.size();

    std::vector<Vertex> vertices(totalVertexCount);
    std::vector<GeometryGenerator::MeshData*> items;
    items.emplace_back(&boxPivot);
    items.emplace_back(&boxBeam);

    UINT k = 0;
    for (auto& currentItem : items) {
        for (size_t i = 0; i < currentItem->Vertices.size(); ++i, ++k)
        {
            vertices[k].Pos = currentItem->Vertices[i].Position;
            vertices[k].Normal = currentItem->Vertices[i].Normal;
            vertices[k].TexC = currentItem->Vertices[i].TexC;
            vertices[k].TangentU = currentItem->Vertices[i].TangentU;
        }
    }

    std::vector<std::uint16_t> indices;
    indices.insert(indices.end(), std::begin(boxPivot.GetIndices16()), std::end(boxPivot.GetIndices16()));
    indices.insert(indices.end(), std::begin(boxBeam.GetIndices16()), std::end(boxBeam.GetIndices16()));

    const UINT vbByteSize = (UINT)vertices.size() * sizeof(Vertex);
    const UINT ibByteSize = (UINT)indices.size() * sizeof(std::uint16_t);

    auto geo = std::make_unique<MeshGeometry>();
    geo->Name = GeneralData::Geometry::RenderObjectNames::Spinner;

    ThrowIfFailed(D3DCreateBlob(vbByteSize, &geo->VertexBufferCPU));
    CopyMemory(geo->VertexBufferCPU->GetBufferPointer(), vertices.data(), vbByteSize);

    ThrowIfFailed(D3DCreateBlob(ibByteSize, &geo->IndexBufferCPU));
    CopyMemory(geo->IndexBufferCPU->GetBufferPointer(), indices.data(), ibByteSize);

    geo->VertexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
        mCommandList.Get(), vertices.data(), vbByteSize, geo->VertexBufferUploader);

    geo->IndexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
        mCommandList.Get(), indices.data(), ibByteSize, geo->IndexBufferUploader);

    geo->VertexByteStride = sizeof(Vertex);
    geo->VertexBufferByteSize = vbByteSize;
    geo->IndexFormat = DXGI_FORMAT_R16_UINT;
    geo->IndexBufferByteSize = ibByteSize;

    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Spinner::Pivot] = boxPivotSubmesh;
    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Spinner::Beam] = boxBeamSubmesh;


    geo->Vertices = vertices;
    geo->Indices = indices;
    mGeometries[geo->Name] = std::move(geo);
}
void PhysicsApplication::BuildGeometryRagdoll(GeometryGenerator& geoGen, int lastIndex) {
    int currentBodyIndex = lastIndex;
    Shape* tempBox = mBodies[currentBodyIndex].m_shape;
    GeometryGenerator::MeshData boxHead = geoGen.CreateBox(tempBox->GetBounds().WidthX(), tempBox->GetBounds().WidthZ(), tempBox->GetBounds().WidthY(), 2);
    ++currentBodyIndex;
    tempBox = mBodies[currentBodyIndex].m_shape;
    GeometryGenerator::MeshData boxTorso = geoGen.CreateBox(tempBox->GetBounds().WidthX(), tempBox->GetBounds().WidthZ(), tempBox->GetBounds().WidthY(), 2);
    ++currentBodyIndex;
    tempBox = mBodies[currentBodyIndex].m_shape;
    GeometryGenerator::MeshData boxLeftArm = geoGen.CreateBox(tempBox->GetBounds().WidthX(), tempBox->GetBounds().WidthZ(), tempBox->GetBounds().WidthY(), 2);
    ++currentBodyIndex;
    tempBox = mBodies[currentBodyIndex].m_shape;
    GeometryGenerator::MeshData boxRightArm = geoGen.CreateBox(tempBox->GetBounds().WidthX(), tempBox->GetBounds().WidthZ(), tempBox->GetBounds().WidthY(), 2);
    ++currentBodyIndex;
    tempBox = mBodies[currentBodyIndex].m_shape;
    GeometryGenerator::MeshData boxLeftLeg = geoGen.CreateBox(tempBox->GetBounds().WidthX(), tempBox->GetBounds().WidthZ(), tempBox->GetBounds().WidthY(), 2);
    ++currentBodyIndex;
    tempBox = mBodies[currentBodyIndex].m_shape;
    GeometryGenerator::MeshData boxRightLeg = geoGen.CreateBox(tempBox->GetBounds().WidthX(), tempBox->GetBounds().WidthZ(), tempBox->GetBounds().WidthY(), 2);
    //
    // We are concatenating all the geometry into one big vertex/index buffer.  So
    // define the regions in the buffer each submesh covers.
    //

    // Cache the vertex offsets to each object in the concatenated vertex buffer.
    UINT boxHeadVertexOffset = 0;
    UINT boxTorsoVertexOffset = (UINT)boxHead.Vertices.size();
    UINT boxLeftArmVertexOffset = boxTorsoVertexOffset + (UINT)boxTorso.Vertices.size();
    UINT boxRightArmVertexOffset = boxLeftArmVertexOffset + (UINT)boxLeftArm.Vertices.size();
    UINT boxLeftLegVertexOffset = boxRightArmVertexOffset + (UINT)boxRightArm.Vertices.size();
    UINT boxRightLegVertexOffset = boxLeftLegVertexOffset + (UINT)boxLeftLeg.Vertices.size();

    // Cache the starting index for each object in the concatenated index buffer.
    UINT boxHeadIndexOffset = 0;
    UINT boxTorsoIndexOffset = (UINT)boxHead.Indices32.size();
    UINT boxLeftArmIndexOffset = boxTorsoIndexOffset + (UINT)boxTorso.Indices32.size();
    UINT boxRightArmIndexOffset = boxLeftArmIndexOffset + (UINT)boxLeftArm.Indices32.size();
    UINT boxLeftLegIndexOffset = boxRightArmIndexOffset + (UINT)boxRightArm.Indices32.size();
    UINT boxRightLegIndexOffset = boxLeftLegIndexOffset + (UINT)boxLeftLeg.Indices32.size();

    // Define the SubmeshGeometry that cover different 
    // regions of the vertex/index buffers.

    SubmeshGeometry boxHeadSubmesh;
    boxHeadSubmesh.IndexCount = (UINT)boxHead.Indices32.size();
    boxHeadSubmesh.StartIndexLocation = boxHeadIndexOffset;
    boxHeadSubmesh.BaseVertexLocation = boxHeadVertexOffset;

    SubmeshGeometry boxTorsoSubmesh;
    boxTorsoSubmesh.IndexCount = (UINT)boxTorso.Indices32.size();
    boxTorsoSubmesh.StartIndexLocation = boxTorsoIndexOffset;
    boxTorsoSubmesh.BaseVertexLocation = boxTorsoVertexOffset;

    SubmeshGeometry boxLeftArmSubmesh;
    boxLeftArmSubmesh.IndexCount = (UINT)boxLeftArm.Indices32.size();
    boxLeftArmSubmesh.StartIndexLocation = boxLeftArmIndexOffset;
    boxLeftArmSubmesh.BaseVertexLocation = boxLeftArmVertexOffset;

    SubmeshGeometry boxRightArmSubmesh;
    boxRightArmSubmesh.IndexCount = (UINT)boxRightArm.Indices32.size();
    boxRightArmSubmesh.StartIndexLocation = boxRightArmIndexOffset;
    boxRightArmSubmesh.BaseVertexLocation = boxRightArmVertexOffset;

    SubmeshGeometry boxLeftLegSubmesh;
    boxLeftLegSubmesh.IndexCount = (UINT)boxLeftLeg.Indices32.size();
    boxLeftLegSubmesh.StartIndexLocation = boxLeftLegIndexOffset;
    boxLeftLegSubmesh.BaseVertexLocation = boxLeftLegVertexOffset;

    SubmeshGeometry boxRightLegSubmesh;
    boxRightLegSubmesh.IndexCount = (UINT)boxRightLeg.Indices32.size();
    boxRightLegSubmesh.StartIndexLocation = boxRightLegIndexOffset;
    boxRightLegSubmesh.BaseVertexLocation = boxRightLegVertexOffset;

    //
    // Extract the vertex elements we are interested in and pack the
    // vertices of all the meshes into one vertex buffer.
    //

    auto totalVertexCount =
        boxHead.Vertices.size() +
        boxTorso.Vertices.size() +
        boxLeftArm.Vertices.size() +
        boxRightArm.Vertices.size() +
        boxLeftLeg.Vertices.size() +
        boxRightLeg.Vertices.size();

    std::vector<Vertex> vertices(totalVertexCount);

    std::vector<GeometryGenerator::MeshData *> items;
    items.emplace_back(&boxHead);
    items.emplace_back(&boxTorso);
    items.emplace_back(&boxLeftArm);
    items.emplace_back(&boxRightArm);
    items.emplace_back(&boxLeftLeg);
    items.emplace_back(&boxRightLeg);

    UINT k = 0;
    for (auto currentItem : items) {
        for (size_t i = 0; i < currentItem->Vertices.size(); ++i, ++k)
        {
            vertices[k].Pos = currentItem->Vertices[i].Position;
            vertices[k].Normal = currentItem->Vertices[i].Normal;
            vertices[k].TexC = currentItem->Vertices[i].TexC;
            vertices[k].TangentU = currentItem->Vertices[i].TangentU;
        }
    }

    std::vector<std::uint16_t> indices;
    indices.insert(indices.end(), std::begin(boxHead.GetIndices16()), std::end(boxHead.GetIndices16()));
    indices.insert(indices.end(), std::begin(boxTorso.GetIndices16()), std::end(boxTorso.GetIndices16()));
    indices.insert(indices.end(), std::begin(boxLeftArm.GetIndices16()), std::end(boxLeftArm.GetIndices16()));
    indices.insert(indices.end(), std::begin(boxRightArm.GetIndices16()), std::end(boxRightArm.GetIndices16()));
    indices.insert(indices.end(), std::begin(boxLeftLeg.GetIndices16()), std::end(boxLeftLeg.GetIndices16()));
    indices.insert(indices.end(), std::begin(boxRightLeg.GetIndices16()), std::end(boxRightLeg.GetIndices16()));

    const UINT vbByteSize = (UINT)vertices.size() * sizeof(Vertex);
    const UINT ibByteSize = (UINT)indices.size() * sizeof(std::uint16_t);

    auto geo = std::make_unique<MeshGeometry>();
    geo->Name = GeneralData::Geometry::RenderObjectNames::Ragdoll;

    ThrowIfFailed(D3DCreateBlob(vbByteSize, &geo->VertexBufferCPU));
    CopyMemory(geo->VertexBufferCPU->GetBufferPointer(), vertices.data(), vbByteSize);

    ThrowIfFailed(D3DCreateBlob(ibByteSize, &geo->IndexBufferCPU));
    CopyMemory(geo->IndexBufferCPU->GetBufferPointer(), indices.data(), ibByteSize);

    geo->VertexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
        mCommandList.Get(), vertices.data(), vbByteSize, geo->VertexBufferUploader);

    geo->IndexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
        mCommandList.Get(), indices.data(), ibByteSize, geo->IndexBufferUploader);

    geo->VertexByteStride = sizeof(Vertex);
    geo->VertexBufferByteSize = vbByteSize;
    geo->IndexFormat = DXGI_FORMAT_R16_UINT;
    geo->IndexBufferByteSize = ibByteSize;

    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Ragdoll::Head] = boxHeadSubmesh;
    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Ragdoll::Torso] = boxTorsoSubmesh;
    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Ragdoll::LeftArm] = boxLeftArmSubmesh;
    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Ragdoll::RightArm] = boxRightArmSubmesh;
    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Ragdoll::LeftLeg] = boxLeftLegSubmesh;
    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Ragdoll::RightLeg] = boxRightLegSubmesh;

    geo->Vertices = vertices;
    geo->Indices = indices;
    mGeometries[geo->Name] = std::move(geo);
}
void PhysicsApplication::BuildGeometryConvex(GeometryGenerator& geoGen, int lastIndex) {
    int currentBodyIndex = lastIndex;
    Shape* tempBox = mBodies[currentBodyIndex].m_shape;
    GeometryGenerator::MeshData sphere = geoGen.CreateGeosphere(tempBox->GetBounds().WidthX() / 2.0f, 2);
    ++currentBodyIndex;
    ShapeConvex* tempConvex = dynamic_cast<ShapeConvex*>(mBodies[currentBodyIndex].m_shape);
    GeometryGenerator::MeshData diamond = geoGen.CreateConvex(tempConvex);
    //
    // We are concatenating all the geometry into one big vertex/index buffer.  So
    // define the regions in the buffer each submesh covers.
    //

    // Cache the vertex offsets to each object in the concatenated vertex buffer.
    UINT sphereVertexOffset = 0;
    UINT diamondVertexOffset = (UINT)sphere.Vertices.size();

    // Cache the starting index for each object in the concatenated index buffer.
    UINT sphereIndexOffset = 0;
    UINT diamondIndexOffset = (UINT)sphere.Indices32.size();

    // Define the SubmeshGeometry that cover different 
    // regions of the vertex/index buffers.

    SubmeshGeometry sphereSubmesh;
    sphereSubmesh.IndexCount = (UINT)sphere.Indices32.size();
    sphereSubmesh.StartIndexLocation = sphereIndexOffset;
    sphereSubmesh.BaseVertexLocation = sphereVertexOffset;

    SubmeshGeometry diamondSubmesh;
    diamondSubmesh.IndexCount = (UINT)diamond.Indices32.size();
    diamondSubmesh.StartIndexLocation = diamondIndexOffset;
    diamondSubmesh.BaseVertexLocation = diamondVertexOffset;

    //
    // Extract the vertex elements we are interested in and pack the
    // vertices of all the meshes into one vertex buffer.
    //

    auto totalVertexCount =
        sphere.Vertices.size() +
        diamond.Vertices.size();

    std::vector<Vertex> vertices(totalVertexCount);

    std::vector<GeometryGenerator::MeshData*> items;
    items.emplace_back(&sphere);
    items.emplace_back(&diamond);

    UINT k = 0;
    for (auto currentItem : items) {
        for (size_t i = 0; i < currentItem->Vertices.size(); ++i, ++k)
        {
            vertices[k].Pos = currentItem->Vertices[i].Position;
            vertices[k].Normal = currentItem->Vertices[i].Normal;
            vertices[k].TexC = currentItem->Vertices[i].TexC;
            vertices[k].TangentU = currentItem->Vertices[i].TangentU;
        }
    }

    std::vector<std::uint16_t> indices;
    indices.insert(indices.end(), std::begin(sphere.GetIndices16()), std::end(sphere.GetIndices16()));
    indices.insert(indices.end(), std::begin(diamond.GetIndices16()), std::end(diamond.GetIndices16()));

    const UINT vbByteSize = (UINT)vertices.size() * sizeof(Vertex);
    const UINT ibByteSize = (UINT)indices.size() * sizeof(std::uint16_t);

    auto geo = std::make_unique<MeshGeometry>();
    geo->Name = GeneralData::Geometry::RenderObjectNames::Convex;

    ThrowIfFailed(D3DCreateBlob(vbByteSize, &geo->VertexBufferCPU));
    CopyMemory(geo->VertexBufferCPU->GetBufferPointer(), vertices.data(), vbByteSize);

    ThrowIfFailed(D3DCreateBlob(ibByteSize, &geo->IndexBufferCPU));
    CopyMemory(geo->IndexBufferCPU->GetBufferPointer(), indices.data(), ibByteSize);

    geo->VertexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
        mCommandList.Get(), vertices.data(), vbByteSize, geo->VertexBufferUploader);

    geo->IndexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
        mCommandList.Get(), indices.data(), ibByteSize, geo->IndexBufferUploader);

    geo->VertexByteStride = sizeof(Vertex);
    geo->VertexBufferByteSize = vbByteSize;
    geo->IndexFormat = DXGI_FORMAT_R16_UINT;
    geo->IndexBufferByteSize = ibByteSize;

    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Convex::Sphere] = sphereSubmesh;
    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Convex::Diamond] = diamondSubmesh;

    geo->Vertices = vertices;
    geo->Indices = indices;
    mGeometries[geo->Name] = std::move(geo);
}

void PhysicsApplication::BuildGeometrySandbox(GeometryGenerator& geoGen) {
    ShapeBox tempBox(g_boxGround, sizeof(g_boxGround) / sizeof(Vec3));
    GeometryGenerator::MeshData boxGround = geoGen.CreateBox(tempBox.m_bounds.WidthX(), tempBox.m_bounds.WidthZ(), tempBox.m_bounds.WidthY(), 2);
    tempBox = ShapeBox(g_boxWall0, sizeof(g_boxWall0) / sizeof(Vec3));
    GeometryGenerator::MeshData boxWallHorizontal = geoGen.CreateBox(tempBox.m_bounds.WidthX(), tempBox.m_bounds.WidthZ(), tempBox.m_bounds.WidthY(), 2);
    tempBox = ShapeBox(g_boxWall1, sizeof(g_boxWall1) / sizeof(Vec3));
    GeometryGenerator::MeshData boxWallVertical = geoGen.CreateBox(tempBox.m_bounds.WidthX(), tempBox.m_bounds.WidthZ(), tempBox.m_bounds.WidthY(), 2);

    //
    // We are concatenating all the geometry into one big vertex/index buffer.  So
    // define the regions in the buffer each submesh covers.
    //

    // Cache the vertex offsets to each object in the concatenated vertex buffer.
    UINT boxGroundVertexOffset = 0;
    UINT boxWallHorizontalVertexOffset = (UINT)boxGround.Vertices.size();
    UINT boxWallVerticalVertexOffset = boxWallHorizontalVertexOffset + (UINT)boxWallHorizontal.Vertices.size();

    // Cache the starting index for each object in the concatenated index buffer.
    UINT boxGroundIndexOffset = 0;
    UINT boxWallHorizontalIndexOffset = (UINT)boxGround.Indices32.size();
    UINT boxWallVerticalIndexOffset = boxWallHorizontalIndexOffset + (UINT)boxWallHorizontal.Indices32.size();

    // Define the SubmeshGeometry that cover different 
    // regions of the vertex/index buffers.

    SubmeshGeometry boxGroundSubmesh;
    boxGroundSubmesh.IndexCount = (UINT)boxGround.Indices32.size();
    boxGroundSubmesh.StartIndexLocation = boxGroundIndexOffset;
    boxGroundSubmesh.BaseVertexLocation = boxGroundVertexOffset;

    SubmeshGeometry boxWallHorizontalSubmesh;
    boxWallHorizontalSubmesh.IndexCount = (UINT)boxWallHorizontal.Indices32.size();
    boxWallHorizontalSubmesh.StartIndexLocation = boxWallHorizontalIndexOffset;
    boxWallHorizontalSubmesh.BaseVertexLocation = boxWallHorizontalVertexOffset;

    SubmeshGeometry boxWallVerticalSubmesh;
    boxWallVerticalSubmesh.IndexCount = (UINT)boxWallVertical.Indices32.size();
    boxWallVerticalSubmesh.StartIndexLocation = boxWallVerticalIndexOffset;
    boxWallVerticalSubmesh.BaseVertexLocation = boxWallVerticalVertexOffset;


    //
    // Extract the vertex elements we are interested in and pack the
    // vertices of all the meshes into one vertex buffer.
    //

    auto totalVertexCount =
        boxGround.Vertices.size() +
        boxWallHorizontal.Vertices.size() +
        boxWallVertical.Vertices.size();

    std::vector<Vertex> vertices(totalVertexCount);

    std::vector<GeometryGenerator::MeshData*> items;
    items.emplace_back(&boxGround);
    items.emplace_back(&boxWallHorizontal);
    items.emplace_back(&boxWallVertical);

    UINT k = 0;
    for (auto currentItem : items) {
        for (size_t i = 0; i < currentItem->Vertices.size(); ++i, ++k)
        {
            vertices[k].Pos = currentItem->Vertices[i].Position;
            vertices[k].Normal = currentItem->Vertices[i].Normal;
            vertices[k].TexC = currentItem->Vertices[i].TexC;
            vertices[k].TangentU = currentItem->Vertices[i].TangentU;
        }
    }


    std::vector<std::uint16_t> indices;
    indices.insert(indices.end(), std::begin(boxGround.GetIndices16()), std::end(boxGround.GetIndices16()));
    indices.insert(indices.end(), std::begin(boxWallHorizontal.GetIndices16()), std::end(boxWallHorizontal.GetIndices16()));
    indices.insert(indices.end(), std::begin(boxWallVertical.GetIndices16()), std::end(boxWallVertical.GetIndices16()));

    const UINT vbByteSize = (UINT)vertices.size() * sizeof(Vertex);
    const UINT ibByteSize = (UINT)indices.size() * sizeof(std::uint16_t);

    auto geo = std::make_unique<MeshGeometry>();
    geo->Name = GeneralData::Geometry::RenderObjectNames::Sandbox;

    ThrowIfFailed(D3DCreateBlob(vbByteSize, &geo->VertexBufferCPU));
    CopyMemory(geo->VertexBufferCPU->GetBufferPointer(), vertices.data(), vbByteSize);

    ThrowIfFailed(D3DCreateBlob(ibByteSize, &geo->IndexBufferCPU));
    CopyMemory(geo->IndexBufferCPU->GetBufferPointer(), indices.data(), ibByteSize);

    geo->VertexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
        mCommandList.Get(), vertices.data(), vbByteSize, geo->VertexBufferUploader);

    geo->IndexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
        mCommandList.Get(), indices.data(), ibByteSize, geo->IndexBufferUploader);

    geo->VertexByteStride = sizeof(Vertex);
    geo->VertexBufferByteSize = vbByteSize;
    geo->IndexFormat = DXGI_FORMAT_R16_UINT;
    geo->IndexBufferByteSize = ibByteSize;

    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Sandbox::Ground] = boxGroundSubmesh;
    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Sandbox::WallHorizontal] = boxWallHorizontalSubmesh;
    geo->DrawArgs[GeneralData::Geometry::RenderItemNames::Sandbox::WallVertical] = boxWallVerticalSubmesh;

    geo->Vertices = vertices;
    geo->Indices = indices;
    mGeometries[geo->Name] = std::move(geo);
}

// geometry for contact points
void PhysicsApplication::BuildGeometryContactPoints(GeometryGenerator& geoGen) 
{
    // Generate the base mesh data (centered at 0,0,0)
    GeometryGenerator::MeshData gizmoMesh = geoGen.CreateBox(1, 1, 1, 2);

    const UINT vbByteSize = (UINT)gizmoMesh.Vertices.size() * sizeof(Vertex);
    const UINT ibByteSize = (UINT)gizmoMesh.Indices32.size() * sizeof(std::uint16_t);

    auto geo = std::make_unique<MeshGeometry>();
    geo->Name = GeneralData::Geometry::RenderObjectNames::ContactPoints;

    // --- CPU Buffer Creation ---
    ThrowIfFailed(D3DCreateBlob(vbByteSize, &geo->VertexBufferCPU));
    CopyMemory(geo->VertexBufferCPU->GetBufferPointer(), gizmoMesh.Vertices.data(), vbByteSize);

    ThrowIfFailed(D3DCreateBlob(ibByteSize, &geo->IndexBufferCPU));
    CopyMemory(geo->IndexBufferCPU->GetBufferPointer(), gizmoMesh.GetIndices16().data(), ibByteSize);

    // --- GPU Buffer Upload (using your existing utility) ---
    geo->VertexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
        mCommandList.Get(), gizmoMesh.Vertices.data(), vbByteSize, geo->VertexBufferUploader);

    geo->IndexBufferGPU = d3dUtil::CreateDefaultBuffer(md3dDevice.Get(),
        mCommandList.Get(), gizmoMesh.GetIndices16().data(), ibByteSize, geo->IndexBufferUploader);

    // --- Final MeshGeometry Setup ---
    geo->VertexByteStride = sizeof(Vertex);
    geo->VertexBufferByteSize = vbByteSize;
    geo->IndexFormat = DXGI_FORMAT_R16_UINT;
    geo->IndexBufferByteSize = ibByteSize;

    // The entire gizmo mesh is one 'submesh'
    SubmeshGeometry submesh;
    submesh.IndexCount = (UINT)gizmoMesh.Indices32.size();
    submesh.StartIndexLocation = 0;
    submesh.BaseVertexLocation = 0;

    // Store the drawing arguments in the map for easy lookup
    geo->DrawArgs[geo->Name] = submesh;

    mGeometries[geo->Name] = std::move(geo);
}



// update
void PhysicsApplication::OnResize()
{
    D3DApp::OnResize();

    mCamera.SetLens(0.25f * MathHelper::Pi, AspectRatio(), 1.0f, 1000.0f);
}
void PhysicsApplication::OnMouseDown(WPARAM btnState, int x, int y)
{
    if ((btnState & MK_LBUTTON) != 0)
    {
        mLastMousePos.x = x;
        mLastMousePos.y = y;

        SetCapture(mhMainWnd);
    }
    else if ((btnState & MK_RBUTTON) != 0)
    {
        if (mAppPaused == true)
            Pick(x, y);
    }
}
void PhysicsApplication::OnMouseUp(WPARAM btnState, int x, int y)
{
    ReleaseCapture();
}
void PhysicsApplication::OnMouseMove(WPARAM btnState, int x, int y)
{
    if ((btnState & MK_LBUTTON) != 0 && mIsSliderMoving == false && mIsGuizmoMoving == false)
    {
        // Make each pixel correspond to a quarter of a degree.
        float dx = XMConvertToRadians(0.25f * static_cast<float>(x - mLastMousePos.x));
        float dy = XMConvertToRadians(0.25f * static_cast<float>(y - mLastMousePos.y));

        mCamera.Pitch(dy);
        mCamera.RotateY(dx);
    }

    mLastMousePos.x = x;
    mLastMousePos.y = y;
}

void PhysicsApplication::OnKeyboardInput(WPARAM btnState) 
{
    switch (btnState) {
    case 'P':
        if (mAppPaused == true)
            ResumeEngine();
        else
            PauseEngine();
        break;
    case 'R':
        ResetCamera();
        mIsRestartNeeded = true;
        break;
    case VK_ESCAPE:
        mRequestedSceneState = SceneState::MAIN_MENU;
        mLockTimer = GeneralData::LockTime;
        ResetCamera();
        break;
    }

    //if (GetAsyncKeyState('N') & 0x8000) {
    //    if (mAppPaused == true)
    //        mIsOneFrame = true;
    //}
}
void PhysicsApplication::OnKeyboardInput(const GameTimer& gt)
{
    if (mActivated == true) {
        const float dt = gt.DeltaTime();

        if (GetAsyncKeyState('W') & 0x8000)
            mCamera.Walk(10.0f * dt);
        if (GetAsyncKeyState('S') & 0x8000)
            mCamera.Walk(-10.0f * dt);
        if (GetAsyncKeyState('A') & 0x8000)
            mCamera.Strafe(-10.0f * dt);
        if (GetAsyncKeyState('D') & 0x8000)
            mCamera.Strafe(10.0f * dt);
    }
}


void PhysicsApplication::Pick(int sx, int sy)
{
    XMFLOAT4X4 P = mCamera.GetProj4x4f();

    // Compute picking ray in view space.
    float vx = (+2.0f * sx / mClientWidth - 1.0f) / P(0, 0);
    float vy = (-2.0f * sy / mClientHeight + 1.0f) / P(1, 1);
     
    // Ray definition in view space.
    XMVECTOR rayOriginView = XMVectorSet(0.0f, 0.0f, 0.0f, 1.0f);
    XMVECTOR rayDirView = XMVectorSet(vx, vy, 1.0f, 0.0f);

    XMMATRIX V = mCamera.GetView();
    XMMATRIX invView = XMMatrixInverse(&XMMatrixDeterminant(V), V);

    // FIX: Transform ray to world space BEFORE the loop
    XMVECTOR rayOriginWorld = XMVector3TransformCoord(rayOriginView, invView);
    XMVECTOR rayDirWorld = XMVector3TransformNormal(rayDirView, invView);

    // Make the ray direction unit length for the intersection tests.
    rayDirWorld = XMVector3Normalize(rayDirWorld);

    float closest_tmin = MathHelper::Infinity;
    RenderItem* pickedItem = nullptr;
    UINT finalPickedTriangle = 0;

    // check contact points first
    if (mIsHitResultVisible == true) {
        for (size_t currentIndex = 0, endIndex = mRenderItemsContactPoints.size(); currentIndex < endIndex; ++currentIndex)
        {
            RenderItem* ri = mRenderItemsContactPoints[currentIndex].get();

            XMMATRIX W = XMLoadFloat4x4(&ri->World);
            XMMATRIX invWorld = XMMatrixInverse(&XMMatrixDeterminant(W), W);

            XMFLOAT3 sphereCenterWorld = { mContactPoints[ri->ObjCBIndex].ptOnA_WorldSpace.x,
                mContactPoints[ri->ObjCBIndex].ptOnA_WorldSpace.z,
                -mContactPoints[ri->ObjCBIndex].ptOnA_WorldSpace.y };
            // Perform the Ray-Sphere Intersection Test
            // XNA/DirectX Math has a built-in helper for this (DirectX::IntersectRaySphere)
            DirectX::BoundingSphere contactSphere(sphereCenterWorld, GeneralData::ContactPointWidth);

            // 2. Call the instance method: BoundingSphere::Intersects(rayOrigin, rayDirection, distance)
            //    This method returns true/false and outputs the distance (tmin).
            float tmin;
            bool hit = contactSphere.Intersects(rayOriginWorld, rayDirWorld, tmin);

            // 3. Find the Closest Valid Hit
            if (hit)
            {
                // Intersection must be in front of the ray origin and closer than the current closest hit.
                if (tmin > 0.0f && tmin < closest_tmin)
                {
                    closest_tmin = tmin;
                    pickedItem = ri;
                }
            }
        }
        if (pickedItem != nullptr) {
            mPickedContactPointIndex = pickedItem->ObjCBIndex;
            return;
        }
    }


    // check normal objects
    pickedItem = nullptr;
    closest_tmin = MathHelper::Infinity;
    for (auto ri : mRitemLayer[(int)RenderLayer::Opaque])
    {
        if (ri->Visible == false)
            continue;

        XMMATRIX W = XMLoadFloat4x4(&ri->World);
        XMMATRIX invWorld = XMMatrixInverse(&XMMatrixDeterminant(W), W);

        // transform world space ray to local space for this specific item
        XMVECTOR rayOriginLocal = XMVector3TransformCoord(rayOriginWorld, invWorld);
        XMVECTOR rayDirLocal = XMVector3TransformNormal(rayDirWorld, invWorld);
        rayDirLocal = XMVector3Normalize(rayDirLocal);

        float tmin = 0.0f;
        if (ri->Bounds.Intersects(rayOriginLocal, rayDirLocal, tmin))
        {
            auto vertices = ri->Geo->Vertices;
            auto indices = ri->Geo->Indices;
            UINT triCount = ri->IndexCount / 3;

            for (UINT i = 0; i < triCount; ++i)
            {
                UINT i0 = indices[ri->StartIndexLocation + i * 3 + 0];
                UINT i1 = indices[ri->StartIndexLocation + i * 3 + 1];
                UINT i2 = indices[ri->StartIndexLocation + i * 3 + 2];

                XMVECTOR v0 = XMLoadFloat3(&vertices[ri->BaseVertexLocation + i0].Pos);
                XMVECTOR v1 = XMLoadFloat3(&vertices[ri->BaseVertexLocation + i1].Pos);
                XMVECTOR v2 = XMLoadFloat3(&vertices[ri->BaseVertexLocation + i2].Pos);

                float t = 0.0f;
                if (TriangleTests::Intersects(rayOriginLocal, rayDirLocal, v0, v1, v2, t))
                {
                    if (t < closest_tmin)
                    {
                        // This is the new nearest picked triangle over all objects.
                        closest_tmin = t;
                        pickedItem = ri; // Store the render item that was hit.
                        finalPickedTriangle = i; // Store which triangle in that item was hit.
                    }
                }
            }
        }
    }
    if (pickedItem != nullptr) {
        mPickedRenderItem = pickedItem;
        mPickedContactPointIndex = -1;
    }
}

void PhysicsApplication::ResetPickedItem() {
    if (mPickedRenderItem != nullptr)
        mPickedRenderItem = nullptr;

    mPickedContactPointIndex = -1;
}