#pragma once

// ==========================================================
// 1. Global Macros
// ==========================================================
#if defined(DEBUG) || defined(_DEBUG)
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#endif

#if !defined(NO_D3D11_DEBUG_NAME) && ( defined(_DEBUG) || defined(PROFILE) )
#pragma comment(lib,"dxguid.lib")
#endif

// exclude unnecessary API
#define WIN32_LEAN_AND_MEAN
// handle conflict between min/max macro in Windows and std::min/max
#define NOMINMAX 

// use pix in release mode
#ifndef SHIPPING
#define USE_PIX 
#endif



// ==========================================================
// 2. OS/DirectX Headers
// ==========================================================
#include "Windows.h"
#include "windowsx.h"
#include <wrl.h>

#include <dxgi1_4.h>
#include <d3d11_1.h>
#include <d3d12.h>
#include <D3Dcompiler.h>
#include <DirectXMath.h>
#include <DirectXPackedVector.h>
#include <DirectXColors.h>
#include <DirectXCollision.h>
#include "Renderer/Common/d3dx12.h"


// ==========================================================
// 3. Standard Libaries
// ==========================================================
#include <unordered_map>
#include <deque>
#include <unordered_set>
#include <string>
#include <memory>
#include <algorithm>
#include <vector>
#include <array>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <map>
#include <random>

#include <cassert>
#include <cstdint>
#include <cstdarg>

#include <math.h>
#include <stdio.h>
#include <comdef.h>
#include <float.h>


// ==========================================================
// 4. Third-Party Libraries
// ==========================================================
// ImGui
#include "Renderer/imgui/imgui.h"
#include "Renderer/imgui/imgui_impl_win32.h"
#include "Renderer/imgui/imgui_impl_dx12.h"
// ImGuizmo
#include "Renderer/imgui/ImGuizmo.h"
#include "Renderer/imgui/imgui_internal.h"
// PIX
#include <pix3.h>


// ==========================================================
// 5. Math/Physics Headers
// ==========================================================
#include "Math/Vector.h"
#include "Math/Quat.h"
#include "Math/Matrix.h"
#include "Math/Bounds.h"
#include "Math/LCP.h"

#include "Physics/Shapes.h"


// ==========================================================
// 6. Libary Auto Links
// ==========================================================
#pragma comment(lib, "d3dcompiler.lib")
#pragma comment(lib, "D3D12.lib")
#pragma comment(lib, "dxgi.lib")


// ==========================================================
// 7. FScopedCPUStat for PIX profiling
// ==========================================================

#ifndef SHIPPING
struct FScopedCPUStat {
	FScopedCPUStat(UINT32 color, PCSTR formattedTitle, ...) {
		char buffer[256];

		va_list arguments;
		va_start(arguments, formattedTitle);

		vsnprintf(buffer, sizeof(buffer), formattedTitle, arguments);

		va_end(arguments);
		PIXBeginEvent(color, buffer);
	}
	~FScopedCPUStat() {
		PIXEndEvent();
	}
};
#define CONCAT_IMPL(x, y) x##y
#define MACRO_CONCAT(x, y) CONCAT_IMPL(x, y)

#define SetCPUStat(Color, ...) \
        FScopedCPUStat MACRO_CONCAT(ScopedStat, __LINE__)(Color, __VA_ARGS__)

#else
#define SetCPUStat(Color, ...)

#endif
