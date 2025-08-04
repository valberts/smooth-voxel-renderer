#version 330 core

out vec4 FragColor;

// Data coming from the C++ side
uniform sampler3D voxelData;
uniform mat4 invProjection;
uniform mat4 invView;
uniform vec3 cameraPos;

const int MAX_STEPS = 256;
const float T_MIN = 0.1; // Don't hit things right at the camera
const int GRID_SIZE = 64;

void main()
{
    // --- 1. Ray Generation ---
    float x = (gl_FragCoord.x / 640.0) * 2.0 - 1.0;
    float y = (gl_FragCoord.y / 480.0) * 2.0 - 1.0;

    vec4 target = invProjection * vec4(x, y, 1.0, 1.0);
    vec3 rayDir = vec3(invView * vec4(normalize(vec3(target) / target.w), 0.0));
    
    // Use the explicit camera position uniform as the ray origin
    vec3 rayOrigin = cameraPos; 

    // --- 2. Ray Traversal Setup (Amanatides & Woo) ---
    vec3 voxelGridMin = vec3(0.0);
    vec3 voxelGridMax = vec3(GRID_SIZE);

    // Check for intersection with the grid's bounding box first
    vec3 invDir = 1.0 / rayDir;
    vec3 tMin = (voxelGridMin - rayOrigin) * invDir;
    vec3 tMax = (voxelGridMax - rayOrigin) * invDir;
    vec3 t1 = min(tMin, tMax);
    vec3 t2 = max(tMin, tMax);
    float tNear = max(max(t1.x, t1.y), t1.z);
    float tFar = min(min(t2.x, t2.y), t2.z);

    if (tNear > tFar || tFar < 0.0) {
        // Ray doesn't hit the grid at all
        FragColor = vec4(0.1, 0.1, 0.2, 1.0); // Background color
        return;
    }
    
    // Start marching from the entry point of the grid
    float t = max(tNear, T_MIN);
    vec3 currentPos = rayOrigin + t * rayDir;
    ivec3 currentVoxel = ivec3(floor(currentPos));

    // Traversal variables
    vec3 tDelta = abs(vec3(GRID_SIZE) * invDir) / GRID_SIZE;
    ivec3 rayStep = ivec3(sign(rayDir));
    vec3 nextBoundary = (vec3(currentVoxel) + (vec3(rayStep) * 0.5 + 0.5)) * vec3(GRID_SIZE);
    vec3 tMaxVoxel = ( (vec3(currentVoxel) + max(vec3(0.0), vec3(rayStep)) ) - currentPos) * invDir;


    // --- 3. The Main Traversal Loop ---
    vec3 hitNormal = vec3(0.0);
    for(int i = 0; i < MAX_STEPS; ++i) {

        // Check if the current voxel is solid
        // We sample the texture at the center of the voxel
        vec3 samplePos = (vec3(currentVoxel) + 0.5) / GRID_SIZE;
        if (texture(voxelData, samplePos).r > 0.0) {
            // HIT! Use the calculated normal for shading
            FragColor = vec4(abs(hitNormal) * 0.7 + 0.3, 1.0);
            return;
        }

        // Advance to the next voxel
        if (tMaxVoxel.x < tMaxVoxel.y) {
            if (tMaxVoxel.x < tMaxVoxel.z) {
                currentVoxel.x += rayStep.x;
                hitNormal = vec3(-rayStep.x, 0, 0);
            } else {
                currentVoxel.z += rayStep.z;
                hitNormal = vec3(0, 0, -rayStep.z);
            }
        } else {
            if (tMaxVoxel.y < tMaxVoxel.z) {
                currentVoxel.y += rayStep.y;
                hitNormal = vec3(0, -rayStep.y, 0);
            } else {
                currentVoxel.z += rayStep.z;
                hitNormal = vec3(0, 0, -rayStep.z);
            }
        }
        
        if (tMaxVoxel.x < tMaxVoxel.y) {
            if (tMaxVoxel.x < tMaxVoxel.z) {
                tMaxVoxel.x += tDelta.x;
            } else {
                tMaxVoxel.z += tDelta.z;
            }
        } else {
            if (tMaxVoxel.y < tMaxVoxel.z) {
                tMaxVoxel.y += tDelta.y;
            } else {
                tMaxVoxel.z += tDelta.z;
            }
        }


        // Check if we have exited the grid
        if (currentVoxel.x < 0 || currentVoxel.x >= GRID_SIZE ||
            currentVoxel.y < 0 || currentVoxel.y >= GRID_SIZE ||
            currentVoxel.z < 0 || currentVoxel.z >= GRID_SIZE) {
            break; // Exited grid
        }
    }

    // If the loop finishes without a hit
    FragColor = vec4(0.1, 0.1, 0.2, 1.0); // Background color
}