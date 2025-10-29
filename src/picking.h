#pragma once
#include <glm/glm.hpp>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

// SSBO for GPU picking data
extern unsigned int pickingSSBO;

// Mouse state
extern double lastMouseX;
extern double lastMouseY;
extern bool hasClickedVoxel;

// GPU picking data structure (matches shader layout)
struct PickingData
{
    glm::ivec4 hoveredVoxel; // xyz = voxel coords, w = valid flag
    int hoveredNeighborCount;
    int padding[3];                   // Align to 16 bytes
    glm::ivec4 hoveredNeighbors[343]; // MAX_NEIGHBORS, using ivec4 for alignment

    glm::ivec4 clickedVoxel; // xyz = voxel coords, w = valid flag
    int clickedNeighborCount;
    int padding2[3];                  // Align to 16 bytes
    glm::ivec4 clickedNeighbors[343]; // MAX_NEIGHBORS
};

// Setup/cleanup
void setupPickingSSBO();
void cleanupPickingSSBO();

// Update mouse position (sends to GPU)
void updateMousePosition(int mouseX, int mouseY, int windowWidth, int windowHeight);

// Mouse callbacks
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);

// Optional: read back picking data for debugging/UI
PickingData readPickingData();

// Get the currently hovered voxel
glm::ivec3 getHoveredVoxel();
