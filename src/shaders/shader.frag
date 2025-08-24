#version 330 core
#define M_PI 3.1415926535897932384626433832795

out vec4 FragColor;

uniform sampler3D voxelData;
uniform sampler3D sdfCenters;
uniform sampler3D sdfNormals;
uniform bool usePlaneFitting;
uniform mat4 invProjection;
uniform mat4 invView;
uniform vec3 cameraPos;
uniform int neighborhoodRingSize; // 1=3x3, 2=5x5, 3=7x7

const int MAX_STEPS = 256;
const float T_MIN = 0.1;
const int GRID_SIZE = 64;
const int MAX_NEIGHBORS = 343;

int getVoxelNeighborhood(ivec3 coord, int ringSize, out vec3 neighbors[MAX_NEIGHBORS]) {
    int count = 0;
    for (int z = -ringSize; z <= ringSize; ++z) {
        for (int y = -ringSize; y <= ringSize; ++y) {
            for (int x = -ringSize; x <= ringSize; ++x) {
                if (count >= MAX_NEIGHBORS) break;

                ivec3 neighbor_coord = coord + ivec3(x, y, z);
                
                if (neighbor_coord.x < 0 || neighbor_coord.x >= GRID_SIZE ||
                    neighbor_coord.y < 0 || neighbor_coord.y >= GRID_SIZE ||
                    neighbor_coord.z < 0 || neighbor_coord.z >= GRID_SIZE) {
                    continue; 
                }

                float is_solid = texelFetch(voxelData, neighbor_coord, 0).r;

                if (is_solid > 0.0) {
                    neighbors[count] = vec3(neighbor_coord) + 0.5;
                    count++;
                }
            }
        }
    }
    return count;
}

vec3 calculateCentroid(vec3 neighbors[MAX_NEIGHBORS], int count) {
    if (count == 0) {
        return vec3(0.0);
    }

    vec3 sum = vec3(0.0);
    for (int i = 0; i < count; ++i) {
        sum += neighbors[i];
    }
    return sum / float(count);
}

mat3 calculateCovarianceMatrix(vec3 neighbors[MAX_NEIGHBORS], int count, vec3 mean) {
    mat3 covariance = mat3(0.0);

    for (int i = 0; i < count; ++i) {
        vec3 diff = neighbors[i] - mean;
        covariance += outerProduct(diff, diff);
    }

    if (count <=1) {
        return mat3(0.0);
    }

    covariance = covariance / float(count - 1);

    return covariance;
}

vec3 computeEigenvector(mat3 M, float eigenvalue) {
    vec3 row0 = vec3(M[0][0] - eigenvalue, M[0][1], M[0][2]);
    vec3 row1 = vec3(M[1][0], M[1][1] - eigenvalue, M[1][2]);
    vec3 row2 = vec3(M[2][0], M[2][1], M[2][2] - eigenvalue);

    vec3 r0xr1 = cross(row0, row1);
    vec3 r0xr2 = cross(row0, row2);
    vec3 r1xr2 = cross(row1, row2);

    float d0 = dot(r0xr1, r0xr1);
    float d1 = dot(r0xr2, r0xr2);
    float d2 = dot(r1xr2, r1xr2);

    if (d0 > d1 && d0 > d2) {
        return normalize(r0xr1);
    }
    if (d1 > d2) {
        return normalize(r0xr2);
    }
    return normalize(r1xr2);
}

void solveEigenSystem(mat3 M, out vec3 eigenvalues, out mat3 eigenvectors) {
    float m = (M[0][0] + M[1][1] + M[2][2]) / 3.0;
    float c1_sq = (M[0][0]-m)*(M[0][0]-m) + (M[1][1]-m)*(M[1][1]-m) + (M[2][2]-m)*(M[2][2]-m) + 2.0 * (M[0][1]*M[0][1] + M[0][2]*M[0][2] + M[1][2]*M[1][2]);
    float c0 = -((M[0][0]-m)*((M[1][1]-m)*(M[2][2]-m) - M[1][2]*M[1][2]) - M[0][1]*(M[0][1]*(M[2][2]-m) - M[0][2]*M[1][2]) + M[0][2]*(M[0][1]*M[1][2] - M[0][2]*(M[1][1]-m)));

    float c1_sq_safe = max(c1_sq, 0.0);
    float p = sqrt(c1_sq_safe/6.0);
    
    if (p < 1e-6) {
        eigenvalues = vec3(m);
        eigenvectors = mat3(1.0);
        return;
    }

    float inv_p = 1.0 / p;
    float b = 0.5 * c0 * inv_p * inv_p * inv_p;
    
    float phi = 0.0;
    if (b >= 1.0) phi = 0.0;
    else if (b <= -1.0) phi = M_PI/3.0;
    else phi = acos(b)/3.0;

    eigenvalues.x = m + 2.0 * p * cos(phi);
    eigenvalues.y = m + 2.0 * p * cos(phi + (2.0*M_PI/3.0));
    eigenvalues.z = 3.0 * m - eigenvalues.x - eigenvalues.y;

    eigenvectors[0] = computeEigenvector(M, eigenvalues.x);
    eigenvectors[1] = computeEigenvector(M, eigenvalues.y);
    eigenvectors[2] = cross(eigenvectors[0], eigenvectors[1]);
}

float ray_aabb(const in vec3 ro, const in vec3 rd, const in vec3 grid_min, const in vec3 grid_max) {
    vec3 inv_dir = 1.0 / rd;
    vec3 t1 = (grid_min - ro) * inv_dir;
    vec3 t2 = (grid_max - ro) * inv_dir;

    vec3 tmin_v = min(t1, t2);
    vec3 tmax_v = max(t1, t2);

    float tmin = max(max(tmin_v.x, tmin_v.y), tmin_v.z);
    float tmax = min(min(tmax_v.x, tmax_v.y), tmax_v.z);

    if (tmax >= tmin) return tmin;

    return 1e30; // miss
}

float map(vec3 p) {
    ivec3 voxel_coord = ivec3(floor(p));
    vec3 center = texelFetch(sdfCenters, voxel_coord, 0).rgb;
    vec3 normal = texelFetch(sdfNormals, voxel_coord, 0).rgb;

    if (length(normal) < 0.1) {
        vec3 voxel_center = vec3(voxel_coord) + 0.5;
        return max(0.5, distance(p, voxel_center));
    }

    return dot(p - center, normal);
}

void main()
{
    // --- Ray Generation ---
    float x = (gl_FragCoord.x / 640.0) * 2.0 - 1.0;
    float y = (gl_FragCoord.y / 480.0) * 2.0 - 1.0;
    vec4 target = invProjection * vec4(x, y, 1.0, 1.0);
    vec3 rd = vec3(invView * vec4(normalize(vec3(target) / target.w), 0.0));
    vec3 ro = cameraPos;

    // --- Bounding Box Intersection ---
    vec3 grid_min = vec3(0.0);
    vec3 grid_max = vec3(GRID_SIZE);
    float entry_t = ray_aabb(ro, rd, grid_min, grid_max);
    if (entry_t == 1e30) {
        FragColor = vec4(0.1, 0.1, 0.2, 1.0);
        return;
    }

    // --- setup ray traversal ---
    float t = max(entry_t, T_MIN);
    vec3 entry_pos = ro + t * rd;
    ivec3 pos = ivec3(floor(entry_pos));
    vec3 inv_dir = 1.0 / rd;
    vec3 delta = abs(vec3(GRID_SIZE) * inv_dir) / GRID_SIZE;
    ivec3 step = ivec3(sign(rd));
    vec3 tmax = ( (vec3(pos) + max(vec3(0.0), vec3(step)) ) - entry_pos) * inv_dir;

    // --- plane fitting ---
    if (usePlaneFitting) {
        for(int i = 0; i < MAX_STEPS; ++i) {
            vec3 sample_pos_uvw = (vec3(pos) + 0.5) / GRID_SIZE;
            if (texture(voxelData, sample_pos_uvw).r > 0.0) {
                vec3 neighbors[MAX_NEIGHBORS];
                int neighbor_count = getVoxelNeighborhood(pos, neighborhoodRingSize, neighbors);

                vec3 plane_center = calculateCentroid(neighbors, neighbor_count);

                mat3 covariance = calculateCovarianceMatrix(neighbors, neighbor_count, plane_center);

                
                vec3 eigenvalues;
                mat3 eigenvectors;
                solveEigenSystem(covariance, eigenvalues, eigenvectors);

                int smallest_idx = 0;
                if (eigenvalues.y < eigenvalues.x) smallest_idx = 1;
                if (eigenvalues.z < eigenvalues[smallest_idx]) smallest_idx = 2;

                vec3 normal = eigenvectors[smallest_idx];

                // --- consistent orientation---
                vec3 object_center = vec3(GRID_SIZE / 2.0);
                vec3 hit_voxel_center = vec3(pos) + 0.5;
                vec3 out_vector = hit_voxel_center - object_center;
                if (dot(normal, out_vector) < 0.0) {
                    normal = -normal;
                }

                // vec3 view_vector = ro - (vec3(pos) + 0.5);
                // if (dot(normal, view_vector) < 0.0) {
                //     normal = -normal;
                // }

                float denom = dot(rd, normal);

                if (abs(denom) > 1e-6) {
                    float t_intersect = dot(plane_center - ro, normal) / denom;
                    vec3 p = ro + t_intersect * rd;
                    if (all(equal(ivec3(floor(p)), pos))) {
                        vec4 clip_space_pos = invView * invProjection * vec4(p, 1.0);
                        gl_FragDepth = (clip_space_pos.z / clip_space_pos.w) * 0.5 + 0.5;
                        // 7. Visualize the now CONSISTENT normal vector.
                        vec3 normal_color = normal * 0.5 + 0.5;
                        
                        FragColor = vec4(normal_color, 1.0);
                        return;
                    }
                }

            }

            if (tmax.x < tmax.y) {
                if (tmax.x < tmax.z) { pos.x += step.x; tmax.x += delta.x; }
                else { pos.z += step.z; tmax.z += delta.z; }
            } else {
                if (tmax.y < tmax.z) { pos.y += step.y; tmax.y += delta.y; }
                else { pos.z += step.z; tmax.z += delta.z; }
            }
            if (pos.x < 0 || pos.x >= GRID_SIZE || pos.y < 0 || pos.y >= GRID_SIZE || pos.z < 0 || pos.z >= GRID_SIZE) break;
        }

        FragColor = vec4(0.1, 0.1, 0.2, 1.0);
        return;
    } else {
        // --- Main Traversal Loop ---
        vec3 hit_normal = vec3(0.0);
        for(int i = 0; i < MAX_STEPS; ++i) {

            // check if voxel at current position is solid
            // sample texture at center of voxel
            vec3 sample_pos = (vec3(pos) + 0.5) / GRID_SIZE;
            if (texture(voxelData, sample_pos).r > 0.0) { // hit
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

            // update the tmax value for the axis we just stepped on.
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


            // check if outside grid
            if (pos.x < 0 || pos.x >= GRID_SIZE ||
                pos.y < 0 || pos.y >= GRID_SIZE ||
                pos.z < 0 || pos.z >= GRID_SIZE) {
                break; // Exited grid
            }
        }
    }

    // no hit
    FragColor = vec4(0.1, 0.1, 0.2, 1.0); // Background color
}