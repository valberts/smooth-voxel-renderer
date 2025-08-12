#version 330 core

out vec4 FragColor;

uniform sampler3D voxelData;
uniform mat4 invProjection;
uniform mat4 invView;
uniform vec3 cameraPos;

const int MAX_STEPS = 256;
const float T_MIN = 0.1;
const int GRID_SIZE = 64;

/**
 * @brief Ray vs AABB intersection test
 * @param ro Ray origin
 * @param rd Ray direction (normalized)
 * @param grid_min The minimum corner of our grid.
 * @param grid_max The maximum corner of our grid.
 * @return Ray entry time, 1e30 if no intersection was found.
 */
float ray_aabb(const in vec3 ro, const in vec3 rd, const in vec3 grid_min, const in vec3 grid_max) {
    vec3 inv_dir = 1.0 / rd;
    vec3 t1 = (grid_min - ro) * inv_dir;
    vec3 t2 = (grid_max - ro) * inv_dir;

    vec3 tmin_v = min(t1, t2);
    vec3 tmax_v = max(t1, t2);

    float tmin = max(max(tmin_v.x, tmin_v.y), tmin_v.z);
    float tmax = min(min(tmax_v.x, tmax_v.y), tmax_v.z);

    if (tmax >= tmin) {
      return tmin;
    }

    return 1e30; // miss
}

void main()
{
    // --- Ray Generation ---
    // First, we figure out our ray's direction from the screen coordinates.
    float x = (gl_FragCoord.x / 640.0) * 2.0 - 1.0;
    float y = (gl_FragCoord.y / 480.0) * 2.0 - 1.0;

    vec4 target = invProjection * vec4(x, y, 1.0, 1.0);
    vec3 rd = vec3(invView * vec4(normalize(vec3(target) / target.w), 0.0));

    // The ray starts at our camera's position.
    vec3 ro = cameraPos;

    // --- Traversal Setup ---
    // Define the world-space boundaries of our voxel grid.
    vec3 grid_min = vec3(0.0);
    vec3 grid_max = vec3(GRID_SIZE);

    // We need to check if our ray even hits the grid's bounding box.
    float entry_t = ray_aabb(ro, rd, grid_min, grid_max);
    if (entry_t == 1e30) {
        FragColor = vec4(0.1, 0.1, 0.2, 1.0); // Background color
        return;
    }

    // We start marching from the grid entry point or the near plane, whichever is further.
    float t = max(entry_t, T_MIN);
    // This is our floating-point position where we enter the grid.
    vec3 entry_pos = ro + t * rd;
    // And this is the integer voxel coordinate we start in.
    ivec3 pos = ivec3(floor(entry_pos));

    // This variable was missing, it's needed for delta and tmax calculation.
    vec3 inv_dir = 1.0 / rd;

    // 'delta' is the time it takes for the ray to cross one voxel on each axis.
    vec3 delta = abs(vec3(GRID_SIZE) * inv_dir) / GRID_SIZE;
    // 'step' tells us which direction to step on each axis (1 or -1).
    ivec3 step = ivec3(sign(rd));
    // 'tmax' holds the time at which the ray crosses the next cell boundary on each axis.
    vec3 tmax = ( (vec3(pos) + max(vec3(0.0), vec3(step)) ) - entry_pos) * inv_dir;


    // --- Main Traversal Loop ---
    vec3 hit_normal = vec3(0.0);
    for(int i = 0; i < MAX_STEPS; ++i) {

        // Let's see if the voxel at our current position is solid.
        // We sample the texture at the center of the voxel.
        vec3 sample_pos = (vec3(pos) + 0.5) / GRID_SIZE;
        if (texture(voxelData, sample_pos).r > 0.0) {
            // We hit something! Let's use the normal for some basic shading.
            FragColor = vec4(abs(hit_normal) * 0.7 + 0.3, 1.0);
            return;
        }

        // Advance to the next voxel based on the smallest tmax.
        if (tmax.x < tmax.y) {
            if (tmax.x < tmax.z) {
                pos.x += step.x;
                hit_normal = vec3(-step.x, 0, 0);
            } else {
                pos.z += step.z;
                hit_normal = vec3(0, 0, -step.z);
            }
        } else {
            if (tmax.y < tmax.z) {
                pos.y += step.y;
                hit_normal = vec3(0, -step.y, 0);
            } else {
                pos.z += step.z;
                hit_normal = vec3(0, 0, -step.z);
            }
        }

        // Now we update the tmax value for the axis we just stepped on.
        if (tmax.x < tmax.y) {
            if (tmax.x < tmax.z) {
                tmax.x += delta.x;
            } else {
                tmax.z += delta.z;
            }
        } else {
            if (tmax.y < tmax.z) {
                tmax.y += delta.y;
            } else {
                tmax.z += delta.z;
            }
        }


        // Check if we've stepped outside the grid.
        if (pos.x < 0 || pos.x >= GRID_SIZE ||
            pos.y < 0 || pos.y >= GRID_SIZE ||
            pos.z < 0 || pos.z >= GRID_SIZE) {
            break; // Exited grid
        }
    }

    // If the loop finishes, it means we didn't hit anything.
    FragColor = vec4(0.1, 0.1, 0.2, 1.0); // Background color
}