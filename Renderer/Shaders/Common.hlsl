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
// PCF for shadow mapping. (With Depth Bias fix)
//---------------------------------------------------------------------------------------
float CalcShadowFactor(float4 shadowPosH)
{
    // Complete projection by doing division by w.
    shadowPosH.xyz /= shadowPosH.w;

    // Depth in NDC space.
    float depth = shadowPosH.z;

    // FIX: Apply a depth bias to combat Shadow Acne (self-shadowing).
    // Adjust this value (0.001f to 0.005f) based on scene scale if artifacts persist.
    const float depthBias = 0.005f; 
    depth -= depthBias;
    depth = saturate(depth);
    
    uint width, height, numMips;
    gShadowMap.GetDimensions(0, width, height, numMips);

    // Texel size.
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