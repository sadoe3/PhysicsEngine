//======================================================================================
// Instancing.hlsl - FINAL VERSION (Aligned with Root Signature)
// This shader implements instanced drawing using a Structured Buffer to fetch
// per-instance World and Texture transformation matrices.
//======================================================================================

// --- 1. Constants and Structures (Standard Definitions) ---

#ifndef MaxLights
    #define MaxLights 16
#endif

// Constant Buffer for Per-Object data (Root Slot 0)
// register(b0)
cbuffer cbObject : register(b0)
{
    float4x4 gWorld;
    float4x4 gTexTransform;
    uint gMaterialIndex; 
    uint3 gObjPad;
};

// Light structure
struct Light
{
    float3 Strength;
    float FalloffStart;
    float3 Direction;
    float FalloffEnd;
    float3 Position;
    float SpotPower;
    int Type; 
    float Pad;
};

// Material Data structure
struct MaterialData
{
    float4 DiffuseAlbedo;
    float3 FresnelR0;
    float  Roughness;
    float4x4 MatTransform;
    uint  DiffuseMapIndex;
    uint  NormalMapIndex;
    uint  MatPad1;
    uint  MatPad2;
};

// --- 2. Per-Instance Data Structure ---
struct InstanceData
{
    float4x4 World;        // Per-instance World Matrix
    float4x4 TexTransform; // Per-instance Texture Transform
    float4   Color;        // Per-instance color override
    uint     MaterialIndex; // Index to gMaterialData for this instance
    uint     IsCollisionBody; // 1 if this object is a collision body, 0 otherwise
    uint2    Pad;          
};

// --- 3. Resource Bindings (Aligned with BuildRootSignature) ---

// Root Slot 3: Descriptor Table (texTable0: t0, t1, space0)
TextureCube gCubeMap : register(t0);
Texture2D gShadowMap : register(t1);

// Root Slot 4: Descriptor Table (texTable1: t2, space0)
Texture2D gTextureMap : register(t2);

// Root Slot 2: Material SRV (Root Descriptor, register(t0, space1))
StructuredBuffer<MaterialData> gMaterialData : register(t0, space1); 

// Root Slot 5: Instancing SRV (Descriptor Table, register(t3, space1))
StructuredBuffer<InstanceData> gInstanceData : register(t3, space1); 

// Samplers (Static Samplers)
SamplerState gsamPointWrap      : register(s0);
SamplerState gsamPointClamp     : register(s1);
SamplerState gsamLinearWrap     : register(s2);
SamplerState gsamLinearClamp    : register(s3);
SamplerState gsamAnisotropicWrap : register(s4);
SamplerState gsamAnisotropicClamp : register(s5);
SamplerComparisonState gsamShadow : register(s6);


// --- 4. Constant Buffers (Root Slot 1) ---
// register(b1)
cbuffer cbPass : register(b1)
{
    float4x4 gView;
    float4x4 gInvView;
    float4x4 gProj;
    float4x4 gInvProj;
    float4x4 gViewProj; 
    float4x4 gInvViewProj;
    float4x4 gShadowTransform; 
    float3 gEyePosW;
    float cbPerObjectPad1;
    float2 gRenderTargetSize;
    float2 gInvRenderTargetSize;
    float gNearZ;
    float gFarZ;
    float gTotalTime;
    float gDeltaTime;
    float4 gAmbientLight;

    Light gLights[MaxLights];
};

// --- 5. Utility Functions (CalcShadowFactor) ---
float CalcShadowFactor(float4 shadowPosH)
{
    shadowPosH.xyz /= shadowPosH.w;
    float depth = shadowPosH.z;

    // Apply a depth bias to combat Shadow Acne.
    const float depthBias = 0.005f; 
    depth -= depthBias;
    depth = saturate(depth);
    
    uint width, height, numMips;
    gShadowMap.GetDimensions(0, width, height, numMips);

    float dx = 1.0f / (float)width;

    float percentLit = 0.0f;
    const float2 offsets[9] =
    {
        float2(-dx, -dx), float2(0.0f, -dx), float2(dx, -dx),
        float2(-dx, 0.0f), float2(0.0f, 0.0f), float2(dx, 0.0f),
        float2(-dx, +dx), float2(0.0f, +dx), float2(dx, +dx)
    };

    [unroll]
    for(int i = 0; i < 9; ++i)
    {
        percentLit += gShadowMap.SampleCmpLevelZero(gsamShadow,
            shadowPosH.xy + offsets[i], depth).r;
    }
    
    return percentLit / 9.0f;
}


// --- 6. Input/Output Structs ---
struct VertexIn
{
    float3 PosL      : POSITION;
    float3 NormalL   : NORMAL;
    float2 TexC      : TEXCOORD;
    float3 TangentU  : TANGENT;
};

struct VertexOut
{
    float4 PosH        : SV_POSITION;

    float4 ShadowPosH  : TEXCOORD0;
    float4 PosW        : TEXCOORD1;
    float3 NormalW     : TEXCOORD2;
    float3 ModelNormal : TEXCOORD3;
    float3 PosL        : TEXCOORD4;
    
    // Per-instance outputs
    uint MaterialIndex : TEXCOORD5; 
    float2 TexC        : TEXCOORD6; 
    uint IsCollisionBody : TEXCOORD7;
};


// --- 7. Procedural Pattern ---
float3 GetColorFromPositionAndNormal(in float3 localPosition, in float3 normal)
{
    const float pi = 3.14159265359;
    
    // **NEW LOD FACTOR:** Increase this value to make the pattern fade out faster with distance/size.
    const float LOD_FACTOR = 5.0f; // Increased from 1.0f to 5.0f

    // --- High-Frequency Pattern (t) ---
    float3 scaledPos = localPosition.xyz * pi * 2.0;
    float t_base = cos(scaledPos.x) * cos(scaledPos.y) * cos(scaledPos.z);
    
    // Calculate the procedural anti-aliasing (fwidth)
    float t_halfWidth = fwidth(t_base) * 0.5f; 
    
    // Apply stronger smoothing by multiplying t_halfWidth by LOD_FACTOR
    float t_smooth = lerp(t_base, 0.0f, saturate(t_halfWidth * LOD_FACTOR)); // <-- CHANGE HERE

    float t = ceil(t_smooth * 0.9); 

    // --- Low-Frequency Pattern (s) ---
    float3 scaledPos2 = localPosition.xyz * pi * 2.0 / 10.0 + float3(pi / 4.0, pi / 4.0, pi / 4.0);
    float s_base = cos(scaledPos2.x) * cos(scaledPos2.y) * cos(scaledPos2.z);
    
    // Apply stronger smoothing for s as well
    float s_halfWidth = fwidth(s_base) * 0.5f;
    float s_smooth = lerp(s_base, 0.0f, saturate(s_halfWidth * LOD_FACTOR)); // <-- CHANGE HERE
    
    float s = (ceil(s_smooth * 0.9) + 3.0) * 0.25;

    // --- Normal-based color multiplier (Remains the same) ---
    float3 colorMultiplier = float3(0.5, 0.5, 1.0);
    
    if (abs(normal.x) > abs(normal.y) && abs(normal.x) > abs(normal.z)) {
        colorMultiplier = float3(1.0, 0.5, 0.5);
    } else if (abs(normal.y) > abs(normal.x) && abs(normal.y) > abs(normal.z)) {
        colorMultiplier = float3(0.5, 1.0, 0.5);
    }

    // --- Final Color Blend ---
    float3 colorB = float3(0.85, 0.85, 0.85); 
    float3 colorA = float3(1.0, 1.0, 1.0);
    
    float3 finalColor = lerp(colorA, colorB, t) * s; 

    return colorMultiplier * finalColor;
}



// --- 8. Vertex Shader (Instancing) ---
VertexOut VS_Instanced(VertexIn vin, uint instanceID : SV_InstanceID)
{
    VertexOut vout = (VertexOut)0.0f;

    // 1. Fetch Per-Instance Data using the instance index (Uses register(t3, space1))
    InstanceData inst = gInstanceData[instanceID]; 

    // 2. Local Space Data Pass-through
    vout.PosL = vin.PosL;
    vout.ModelNormal = vin.NormalL;

    // 3. World Transform: Use the per-instance World matrix
    float4 posW = mul(float4(vin.PosL, 1.0f), inst.World);
    vout.PosW = posW;

    // 4. Normal Transform (Using only the rotational part of inst.World)
    vout.NormalW = mul(vin.NormalL, (float3x3)inst.World);

    // 5. Clip Space Position and Shadow Position
    vout.PosH = mul(posW, gViewProj);
    vout.ShadowPosH = mul(posW, gShadowTransform);

    // 6. Texture Coordinate Transform: Use the per-instance texture transform
    vout.TexC = mul(float4(vin.TexC, 0.0f, 1.0f), inst.TexTransform).xy;

    // 7. Pass Per-Instance Material Index
    vout.MaterialIndex = inst.MaterialIndex;

    // 8. Pass Collision Flag to GS
    vout.IsCollisionBody = inst.IsCollisionBody;

    return vout;
}

// --- 9. Geometry Shader (Instancing) ---
[maxvertexcount(3)]
void GS_Culler(triangle VertexOut input[3], inout TriangleStream<VertexOut> output)
{
    // Culling Logic:
    // If IsCollisionBody is 1, discard the primitive.
    // We only need to check one vertex, as all vertices belong to the same instance.
    
    // If it is NOT a collision body (IsCollisionBody == 0) -> Draw it normally (Opaque Pass).
    if (input[0].IsCollisionBody == 0)
    {
        output.Append(input[0]);
        output.Append(input[1]);
        output.Append(input[2]);
        output.RestartStrip(); // Finalize the triangle strip
    }
    
    // If it is a collision body (IsCollisionBody == 1) -> Output nothing, effectively culling it.
}


// --- 10. Pixel Shader (Instancing) ---
float4 PS_Instanced(VertexOut pin, float4 screenPos : SV_Position) : SV_Target
{
    // Fetch Material Data (Material Data is still needed for DiffuseAlbedo's alpha value)
    MaterialData mat = gMaterialData[pin.MaterialIndex];

    //
    // 1. Normalization and Lighting Setup
    //
    // Interpolating normal can unnormalize it, so renormalize it.
    pin.NormalW.xyz = normalize(pin.NormalW.xyz);
    
    // NOTE: pin.ModelNormal (TEXCOORD3) is also being passed through VS_Instanced
    // But we will use pin.NormalW (World Normal) for the lighting calculation and 
    // pin.ModelNormal (Model Normal) for the color multiplier logic within GetColorFromPositionAndNormal.
    
    float3 dirToLight = normalize(float3(1.0f, 1.0f, 1.0f)); 

    //
    // 2. Final Checkerboard Color Calculation (Using PosL)
    //
    // The pattern is now object-relative using pin.PosL (Model Space Position, TEXCOORD4).
    // The ModelNormal is used inside the function to determine the color multiplier based on face orientation.
    float3 colorFinalRGB = GetColorFromPositionAndNormal(pin.PosL, pin.ModelNormal.xyz); 
    
    float4 finalColor;
    // Apply the material's albedo RGB (optional, usually 1,1,1) and use the Material's Alpha (needed for blending).
    finalColor.rgb = colorFinalRGB * mat.DiffuseAlbedo.rgb; 
    finalColor.a   = mat.DiffuseAlbedo.a; 

    //
    // 3. Shadow Factor (Uses CalcShadowFactor)
    //
    float shadowFactor = CalcShadowFactor(pin.ShadowPosH);
    
    //
    // 4. Final Lighting Calculation (Simplified Lambertian)
    //
    float ambient = 0.5f;
    float NdotL = saturate(dot(pin.NormalW.xyz, dirToLight.xyz));

    float diffuseTerm = clamp(NdotL, 0.0f, 1.0f - ambient); 
    
    float flux = diffuseTerm * shadowFactor + ambient;

    finalColor.rgb *= flux; // Apply the flux (ambient + diffuse * shadow)

    return finalColor;
}