cmake_minimum_required(VERSION 3.10)
project(smooth_voxel_renderer VERSION 1.0.0)

find_package(OpenGL REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)

add_executable(smooth_voxel_renderer src/config.h src/main.cpp  "src/Camera.h")

target_link_libraries(smooth_voxel_renderer
    PRIVATE
        glfw
        glad::glad
        OpenGL::GL
)
