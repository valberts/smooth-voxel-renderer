#include "picking.h"
#include "config.h"
#include <iostream>

// SSBO handle
unsigned int pickingSSBO = 0;

// Mouse state
double lastMouseX = 0.0;
double lastMouseY = 0.0;
bool hasClickedVoxel = false;

// Screen dimensions (for coordinate conversion)
static int screenWidth = 0;
static int screenHeight = 0;

void setupPickingSSBO()
{
    glGenBuffers(1, &pickingSSBO);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, pickingSSBO);

    // Allocate buffer for PickingData structure
    glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(PickingData), nullptr, GL_DYNAMIC_COPY);

    // Initialize with empty data
    PickingData emptyData = {};
    emptyData.hoveredVoxel = glm::ivec4(-1, -1, -1, 0);
    emptyData.clickedVoxel = glm::ivec4(-1, -1, -1, 0);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, sizeof(PickingData), &emptyData);

    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, pickingSSBO);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
}

void cleanupPickingSSBO()
{
    if (pickingSSBO != 0)
    {
        glDeleteBuffers(1, &pickingSSBO);
        pickingSSBO = 0;
    }
}

void updateMousePosition(int mouseX, int mouseY, int windowWidth, int windowHeight)
{
    screenWidth = windowWidth;
    screenHeight = windowHeight;
    lastMouseX = mouseX;
    lastMouseY = mouseY;
}

void mouse_callback(GLFWwindow *window, double xpos, double ypos)
{
    int width, height;
    glfwGetWindowSize(window, &width, &height);
    updateMousePosition(static_cast<int>(xpos), static_cast<int>(ypos), width, height);
}

void mouse_button_callback(GLFWwindow *window, int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS)
    {
        // Left click - the shader will handle copying hovered data to clicked data
        hasClickedVoxel = true;
    }
    else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS)
    {
        // Right click - clear clicked voxel
        hasClickedVoxel = false;

        glBindBuffer(GL_SHADER_STORAGE_BUFFER, pickingSSBO);
        PickingData data;
        glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, sizeof(PickingData), &data);

        // Clear clicked data
        data.clickedVoxel = glm::ivec4(-1, -1, -1, 0);
        data.clickedNeighborCount = 0;

        glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, sizeof(PickingData), &data);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    }
}

// Get the currently hovered voxel coordinates (returns ivec3(-1) if none)
glm::ivec3 getHoveredVoxel() {
    PickingData data = readPickingData();
    if (data.hoveredVoxel.w > 0) {
        return glm::ivec3(data.hoveredVoxel.x, data.hoveredVoxel.y, data.hoveredVoxel.z);
    }
    return glm::ivec3(-1, -1, -1);
}

PickingData readPickingData() {
    PickingData data;
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, pickingSSBO);
    glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, sizeof(PickingData), &data);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
    return data;
}
