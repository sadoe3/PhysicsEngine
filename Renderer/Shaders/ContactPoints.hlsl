// Include common HLSL code (to access gWorld, gViewProj, etc.)
#include "Common.hlsl" 

struct VertexIn
{
    float3 PosL : POSITION;    // Local-space position of the gizmo vertex
    float3 NormalL : NORMAL;
    float2 TexC : TEXCOORD;
};

struct VertexOut
{
    float4 PosH : SV_POSITION; // Homogeneous clip-space position
};
 
//-----------------------------------------------------------------------------
// Vertex Shader (VS)
// Performs standard Model-View-Projection transformation.
//-----------------------------------------------------------------------------
VertexOut VS(VertexIn vin)
{
    VertexOut vout;

    // 1. Transform to World Space using the object's World Matrix (gWorld).
    // The gWorld matrix is crucial here; the CPU sets it to a Translation matrix 
    // to position the gizmo at the contact point.
    float4 posW = mul(float4(vin.PosL, 1.0f), gWorld);
    
    // 2. Transform to Homogeneous Clip Space (Screen Space)
    // The gizmo is depth-tested normally.
    vout.PosH = mul(posW, gViewProj);
    
    return vout;
}

//-----------------------------------------------------------------------------
// Pixel Shader (PS)
// Outputs a fixed, bright color for clear debug visualization.
//-----------------------------------------------------------------------------
float4 PS(VertexOut pin) : SV_Target
{
    // The CPU has set gColor to either Red (for points) or Blue (for lines)
    // via the Object Constant Buffer (cbPerObject).
    return gColor; 
}