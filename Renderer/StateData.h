#pragma once
// todo
enum class SceneState {
    NONE,         // A neutral or uninitialized state
    MAIN_MENU,
    STRESS_TEST,
    SANDBOX
};

enum class SandboxState {
    ALL,
    STACK,
    MOVER,
    CHAIN,

    HINGE,
    SPINNER,
    VELOCITY,
    ORIENTATION,

    RAGDOLL,

    CONVEX
};