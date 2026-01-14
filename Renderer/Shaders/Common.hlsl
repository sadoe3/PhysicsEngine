//***************************************************************************************
// Common.hlsl by Frank Luna (C) 2015 All Rights Reserved.
// (REVISED with CalcShadowFactor depth bias fix)
//***************************************************************************************

// Defaults for number of lights.
#ifndef NUM_DIR_LIGHTS
    #define NUM_DIR_LIGHTS 3
#endif

#ifndef NUM_POINT_LIGHTS
    #define NUM_POINT_LIGHTS 0
#endif

#ifndef NUM_SPOT_LIGHTS
    #define NUM_SPOT_LIGHTS 0
#endif

// Max number of lights for the array size. Assumes MaxLights is defined in C++ or LightUtil.hlsl.
#ifndef MaxLights
    #define MaxLights 16
#endif

// Include structures and functions for lighting.
#include "LightingUtil.hlsl" // Assumed to define struct Light

struct MaterialData
{
    float4  DiffuseAlbedo;
    float3  FresnelR0;
    float   Roughness;
    float4x4 MatTransform;
    uint    DiffuseMapIndex;
    uint    NormalMapIndex;
    uint    MatPad1;
    uint    MatPad2;
};

// Resource Bindings
TextureCube gCubeMap : register(t0);
Texture2D gShadowMap : register(t1);
Texture2D gTextureMap : register(t2);
StructuredBuffer<MaterialData> gMaterialData : register(t0, space1);

// Sampler Bindings
SamplerState gsamPointWrap       : register(s0);
SamplerState gsamPointClamp      : register(s1);
SamplerState gsamLinearWrap      : register(s2);
SamplerState gsamLinearClamp     : register(s3);
SamplerState gsamAnisotropicWrap : register(s4);
SamplerState gsamAnisotropicClamp : register(s5);
SamplerComparisonState gsamShadow : register(s6);

// Constant data that varies per object.
cbuffer cbPerObject : register(b0)
{
    float4x4 gWorld;        // Pack 0-3
    float4x4 gTexTransform; // Pack 4-7
    float4   gColor;        // Pack 8 <--- Your new color
    uint     gMaterialIndex;// Pack 9.x
    uint3    gObjPad;       // Pack 9.y, 9.z, 9.w (Padding)
};

// Constant data that varies per frame/pass.
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

//---------------------------------------------------------------------------------------
// Transforms a normal map sample to world space.
//---------------------------------------------------------------------------------------
float3 NormalSampleToWorldSpace(float3 normalMapSample, float3 unitNormalW, float3 tangentW)
{
    float3 normalT = 2.0f*normalMapSample - 1.0f;

    float3 N = unitNormalW;
    float3 T = normalize(tangentW - dot(tangentW, N)*N);
    float3 B = cross(N, T);

    float3x3 TBN = float3x3(T, B, N);
    float3 bumpedNormalW = mul(normalT, TBN);

    return bumpedNormalW;
}

//---------------------------------------------------------------------------------------
// PCF for shadow mapping. (Revised with Tuned Bias)
//---------------------------------------------------------------------------------------
float CalcShadowFactor(float4 shadowPosH)
{
    // Complete projection by doing division by w.
    shadowPosH.xyz /= shadowPosH.w;

    // Depth in NDC space.
    float depth = shadowPosH.z;

    // ------------------------------------------------------------------
    // FIX 1: Texture Bounds Check
    // If the pixel is outside the shadow map's view volume (UVs < 0 or > 1),
    // we should consider it 'Lit' (1.0) instead of 'Shadowed'.
    // This fixes the dark artifacts on the far sides of your large floor.
    // ------------------------------------------------------------------
    if (shadowPosH.x < 0.0f || shadowPosH.x > 1.0f ||
        shadowPosH.y < 0.0f || shadowPosH.y > 1.0f ||
        depth < 0.0f || depth > 1.0f)
    {
        return 1.0f;
    }

    // ------------------------------------------------------------------
    // FIX 2: Tuned Slope-Scaled Bias
    // We calculate how fast the depth changes in screen space (ddx/ddy).
    // This automatically creates a larger bias for steep slopes (the sphere edges)
    // and a smaller bias for flat surfaces, fixing the grid/stripe acne.
    // ------------------------------------------------------------------
    float3 d1 = ddx(shadowPosH.xyz);
    float3 d2 = ddy(shadowPosH.xyz);
    float z_slope = sqrt(d1.z * d1.z + d2.z * d2.z);
    
    // --- TWEAKABLE VALUES ---
    // If stripes remain, slightly increase 'baseBias'.
    // If the shadow creates a gap (floating/detached), decrease 'maxBias'.
    const float baseBias   = 0.002f; // Previously 0.0001f (Too low)
    const float slopeScale = 4.0f;   // Strength of slope reaction
    const float maxBias    = 0.01f;  // Limit the maximum bias
    
    // Calculate final bias with limits
    float depthBias = baseBias + (z_slope * slopeScale);
    depthBias = min(depthBias, maxBias); 

    depth -= depthBias;
    depth = saturate(depth);
    
    uint width, height, numMips;
    gShadowMap.GetDimensions(0, width, height, numMips);
    float dx = 1.0f / (float)width;

    float percentLit = 0.0f;
    const float2 offsets[9] =
    {
        float2(-dx,  -dx), float2(0.0f,  -dx), float2(dx,  -dx),
        float2(-dx, 0.0f), float2(0.0f, 0.0f), float2(dx, 0.0f),
        float2(-dx,  +dx), float2(0.0f,  +dx), float2(dx,  +dx)
    };

    [unroll]
    for(int i = 0; i < 9; ++i)
    {
        percentLit += gShadowMap.SampleCmpLevelZero(gsamShadow,
            shadowPosH.xy + offsets[i], depth).r;
    }
    
    return percentLit / 9.0f;
}