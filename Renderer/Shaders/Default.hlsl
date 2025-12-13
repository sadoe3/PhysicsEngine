//***************************************************************************************
// Default.hlsl by Frank Luna (C) 2015 All Rights Reserved.
// (REVISED to use Model Space Position for size-relative checkerboard and fwidth AA)
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

// Include common HLSL code (contains resources and CalcShadowFactor with bias fix).
#include "Common.hlsl"

struct VertexIn
{
    float3 PosL    : POSITION;
    float3 NormalL : NORMAL;
    float2 TexC    : TEXCOORD;
    float3 TangentU : TANGENT;
};

struct VertexOut
{
    float4 PosH        : SV_POSITION; // MUST be SV_POSITION (clip-space)
    
    // Corrected Semantics for Interpolation (Start at TEXCOORD0)
    float4 ShadowPosH  : TEXCOORD0; // Corresponds to shadowPos (Clip/Projective space)
    float4 PosW        : TEXCOORD1; // Corresponds to modelPos (World Position)
    float3 NormalW     : TEXCOORD2; // Corresponds to worldNormal (World Space Normal)
    float3 ModelNormal : TEXCOORD3; // Corresponds to modelNormal (Model Space Normal)
    float3 PosL        : TEXCOORD4; // <--- ADDED: Interpolated Model Space Position
};

/*
==========================================
GetColorFromPositionAndNormal (Now uses Local Space for pattern)
==========================================
*/
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


/*
==========================================
VS (Vertex Shader) (UPDATED to pass PosL)
==========================================
*/
VertexOut VS(VertexIn vin) 
{
    VertexOut vout = (VertexOut)0.0f;
    
    // NEW: Pass Model Space Position (Local Position)
    vout.PosL = vin.PosL; 

    // Model-space normal
    float3 modelSpaceNormal = vin.NormalL;
    vout.ModelNormal = modelSpaceNormal;

    // World Position (modelPos)
    float4 posW = mul(float4(vin.PosL, 1.0f), gWorld);
    vout.PosW = posW; 
    
    // World Normal (worldNormal)
    vout.NormalW = mul(modelSpaceNormal, (float3x3)gWorld); 
    
    // Clip Position (gl_Position)
    vout.PosH = mul(posW, gViewProj);
    
    // Shadow Position (shadowPos)
    vout.ShadowPosH = mul(posW, gShadowTransform);
    
    return vout;
}

/*
==========================================
PS (Pixel Shader) (UPDATED to use PosL for color)
==========================================
*/
float4 PS(VertexOut pin) : SV_Target
{
    // Interpolating normal can unnormalize it, so renormalize it.
    pin.NormalW.xyz = normalize(pin.NormalW.xyz);
    
    float3 dirToLight = normalize(float3(1.0f, 1.0f, 1.0f)); 

    //
    // 1. Final Checkerboard Color Calculation (Using PosL)
    //
    // The pattern is now object-relative.
    float3 colorFinalRGB = GetColorFromPositionAndNormal(pin.PosL, pin.ModelNormal.xyz); 
    
    float4 finalColor;
    finalColor.rgb = colorFinalRGB;
    finalColor.a = 1.0f;

    //
    // 2. Shadow Factor (Uses CalcShadowFactor from Common.hlsl with bias fix)
    //
    float shadowFactor = CalcShadowFactor(pin.ShadowPosH);
    
    //
    // 3. Final Lighting Calculation
    //
    float ambient = 0.5f;
    float NdotL = saturate(dot(pin.NormalW.xyz, dirToLight.xyz));
    
    float diffuseTerm = clamp(NdotL, 0.0f, 1.0f - ambient); 
    
    float flux = diffuseTerm * shadowFactor + ambient;

    finalColor.rgb *= flux; // Apply the flux (ambient + diffuse * shadow)

    return finalColor;
}