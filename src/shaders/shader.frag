#version 330 core
#define M_PI 3.1415926535897932384626433832795

out vec4 FragColor;

uniform sampler3D voxelData;
uniform sampler3D sdfCenters;
uniform sampler3D sdfNormals;
uniform bool usePlaneFitting;
uniform mat4 invProjection;
uniform mat4 invView;
uniform mat4 projection;
uniform mat4 view;
uniform vec3 cameraPos;
uniform int neighborhoodRingSize; // 1=3x3, 2=5x5, 3=7x7
uniform bool checkBounds;
uniform int gridSize;
uniform bool usePhongLighting;
uniform bool useDistanceWeighting;
uniform float distanceWeightMultiplier;
uniform bool useVoxelCentricWeighting;
uniform float voxelCentricStepSize;
uniform float voxelCentricEpsilon;
uniform int voxelCentricMaxIterations;
uniform int useSphericalNeighborhood;
uniform float sphericalRadius;
uniform bool useVoxelCenterForSphere;
uniform int falloffMode; // 0 = linear, 1 = gaussian
uniform int surfaceType; // 0 = plane, 1 = sphere

const int MAX_NEIGHBORS = 343;

// seperable 1D gaussian: G(x) = exp(-x^2 / (2*sigma^2))
float gaussian1D(float offset, float sigma) {
    return exp(-(offset * offset) / (2.0 * sigma * sigma));
}

// 3D gaussian convolution weight using separable kernels
// weight = G(dx) * G(dy) * G(dz)
float gaussianConvolutionWeight(vec3 offset, float sigma) {
    return gaussian1D(offset.x, sigma) * gaussian1D(offset.y, sigma) * gaussian1D(offset.z, sigma);
}

// gaussian weight: using perpendicular distance approximation
float calculateGaussianDistanceWeight(vec3 point, vec3 rayOrigin, vec3 rayDirection, int ringSize) {
    // perpendicular distance
    vec3 toPoint = point - rayOrigin;
    vec3 projection = dot(toPoint, rayDirection) * rayDirection;
    vec3 perpendicular = toPoint - projection;
    
    // linear falloff has maxDist = sqrt(distanceWeightMultiplier) * ringSize
    // for Gaussian, we set sigma = maxDist / 2.5 so that at maxDist, weight ≈ 0.01
    float maxDist = sqrt(distanceWeightMultiplier) * float(ringSize);
    float sigma = maxDist / 2.5; // 2.5*sigma, gaussian ≈ 0.01
    
    return gaussianConvolutionWeight(perpendicular, sigma);
}

// gaussian weight: voxel-centric using 3D convolution
float calculateGaussianVoxelCentricWeight(vec3 neighborPos, vec3 currentRayPos, int ringSize) {
    vec3 offset = neighborPos - currentRayPos;
    
    float maxDist = sqrt(3.0) * float(ringSize);
    float sigma = maxDist / 2.5; // 2.5*sigma, gaussian ≈ 0.01
    
    // 3d gaussian convolution: G(x,y,z) = G(x) * G(y) * G(z)
    return gaussianConvolutionWeight(offset, sigma);
}

// ray-centric distance weight: perpendicular distance to ray
float calculateDistanceWeight(vec3 point, vec3 rayOrigin, vec3 rayDirection, int ringSize) {
    if (falloffMode == 1) { // gaussian
        return calculateGaussianDistanceWeight(point, rayOrigin, rayDirection, ringSize);
    }
    
    // linear falloff (falloffMode == 0)
    // perpendicular distance
    vec3 toPoint = point - rayOrigin;
    vec3 projection = dot(toPoint, rayDirection) * rayDirection;
    vec3 perpendicular = toPoint - projection;
    float perpDist = length(perpendicular);
    
    // lower is more tight falloff, higher is more gentle
    float maxDist = sqrt(distanceWeightMultiplier) * float(ringSize);
    
    // linear falloff, 1.0 at center to 0.0 at edge
    float weight = 1.0 - (perpDist / maxDist);
    return max(0.0, weight);
}

// voxel-centric distance weight: distance from center voxel
float calculateVoxelCentricWeight(vec3 neighborPos, vec3 currentRayPos, int ringSize) {
    if (falloffMode == 1) { // gaussian
        return calculateGaussianVoxelCentricWeight(neighborPos, currentRayPos, ringSize);
    }
    
    // Linear falloff (falloffMode == 0)
    float dist = distance(neighborPos, currentRayPos);
    float maxDist = sqrt(3.0) * float(ringSize); // diagonal of neighborhood
    
    // linear falloff, 1.0 at center to 0.0 at edge
    float weight = 1.0 - (dist / maxDist);
    return max(0.0, weight);
}

int getVoxelNeighborhood(ivec3 coord, int ringSize, out vec3 neighbors[MAX_NEIGHBORS]) {
    int count = 0;
    for (int z = -ringSize; z <= ringSize; ++z) {
        for (int y = -ringSize; y <= ringSize; ++y) {
            for (int x = -ringSize; x <= ringSize; ++x) {
                if (count >= MAX_NEIGHBORS) break;

                ivec3 neighbor_coord = coord + ivec3(x, y, z);
                
                if (neighbor_coord.x < 0 || neighbor_coord.x >= gridSize ||
                    neighbor_coord.y < 0 || neighbor_coord.y >= gridSize ||
                    neighbor_coord.z < 0 || neighbor_coord.z >= gridSize) {
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

int getSphericalNeighborhood(vec3 point, float radius, out vec3 neighbors[MAX_NEIGHBORS]) {
    int count = 0;
    ivec3 center_coord = ivec3(floor(point));
    int maxOffset = int(ceil(radius));
    
    for (int z = -maxOffset; z <= maxOffset; ++z) {
        for (int y = -maxOffset; y <= maxOffset; ++y) {
            for (int x = -maxOffset; x <= maxOffset; ++x) {
                if (count >= MAX_NEIGHBORS) break;
                
                ivec3 neighbor_coord = center_coord + ivec3(x, y, z);
                
                // Bounds check
                if (neighbor_coord.x < 0 || neighbor_coord.x >= gridSize ||
                    neighbor_coord.y < 0 || neighbor_coord.y >= gridSize ||
                    neighbor_coord.z < 0 || neighbor_coord.z >= gridSize) {
                    continue;
                }
                
                // Check if solid
                float is_solid = texelFetch(voxelData, neighbor_coord, 0).r;
                if (is_solid <= 0.0) {
                    continue;
                }
                
                // Check distance from center point to voxel center
                vec3 voxel_center = vec3(neighbor_coord) + 0.5;
                float dist = distance(point, voxel_center);
                
                if (dist <= radius) {
                    neighbors[count] = voxel_center;
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

vec3 calculateWeightedCentroid(vec3 neighbors[MAX_NEIGHBORS], int count, vec3 rayOrigin, vec3 rayDirection, int ringSize) {
    if (count == 0) {
        return vec3(0.0);
    }

    vec3 weightedSum = vec3(0.0);
    float totalWeight = 0.0;
    
    for (int i = 0; i < count; ++i) {
        float weight = calculateDistanceWeight(neighbors[i], rayOrigin, rayDirection, ringSize);
        weightedSum += neighbors[i] * weight;
        totalWeight += weight;
    }

    return weightedSum / totalWeight;
}

vec3 calculateVoxelCentricWeightedCentroid(vec3 neighbors[MAX_NEIGHBORS], int count, vec3 currentRayPos, int ringSize) {
    if (count == 0) {
        return vec3(0.0);
    }

    vec3 weightedSum = vec3(0.0);
    float totalWeight = 0.0;
    
    for (int i = 0; i < count; ++i) {
        float weight = calculateVoxelCentricWeight(neighbors[i], currentRayPos, ringSize);
        weightedSum += neighbors[i] * weight;
        totalWeight += weight;
    }
    
    return weightedSum / totalWeight;
}

mat3 calculateCovarianceMatrix (vec3 neighbors[MAX_NEIGHBORS], int count, vec3 mean) {
    if (count <= 1) {
        return mat3(0.0);
    }

    mat3 covariance = mat3(0.0);

    for (int i = 0; i < count; ++i) {
        vec3 diff = neighbors[i] - mean;
        covariance += outerProduct(diff, diff);
    }

    covariance = covariance / float(count - 1);

    return covariance;
}

mat3 calculateWeightedCovarianceMatrix(vec3 neighbors[MAX_NEIGHBORS], int count, vec3 mean, vec3 rayOrigin, vec3 rayDirection, int ringSize) {
    if (count <= 1) {
        return mat3(0.0);
    }

    mat3 covariance = mat3(0.0);
    float totalWeight = 0.0;

    for (int i = 0; i < count; ++i) {
        float weight = calculateDistanceWeight(neighbors[i], rayOrigin, rayDirection, ringSize);
        vec3 diff = neighbors[i] - mean;
        covariance += weight * outerProduct(diff, diff);
        totalWeight += weight;
    }

    covariance = covariance / totalWeight;

    return covariance;
}

mat3 calculateVoxelCentricWeightedCovarianceMatrix(vec3 neighbors[MAX_NEIGHBORS], int count, vec3 mean, vec3 currentRayPos, int ringSize) {
    if (count <= 1) {
        return mat3(0.0);
    }

    mat3 covariance = mat3(0.0);
    float totalWeight = 0.0;

    for (int i = 0; i < count; ++i) {
        float weight = calculateVoxelCentricWeight(neighbors[i], currentRayPos, ringSize);
        vec3 diff = neighbors[i] - mean;
        covariance += weight * outerProduct(diff, diff);
        totalWeight += weight;
    }

    return covariance / totalWeight;
}

vec3 computeEigenvector(mat3 M, float eigenvalue) {
    mat3 A = M - eigenvalue * mat3(1.0);
    
    vec3 row0 = vec3(A[0][0], A[0][1], A[0][2]);
    vec3 row1 = vec3(A[1][0], A[1][1], A[1][2]);
    vec3 row2 = vec3(A[2][0], A[2][1], A[2][2]);

    vec3 r0xr1 = cross(row0, row1);
    vec3 r0xr2 = cross(row0, row2);
    vec3 r1xr2 = cross(row1, row2);

    float d0 = dot(r0xr1, r0xr1);
    float d1 = dot(r0xr2, r0xr2);
    float d2 = dot(r1xr2, r1xr2);

    vec3 eigenvec;
    if (d0 > d1 && d0 > d2) {
        eigenvec = r0xr1;
    } else if (d1 > d2) {
        eigenvec = r0xr2;
    } else {
        eigenvec = r1xr2;
    }
    
    float len = length(eigenvec);
    if (len < 1e-6) {
        return vec3(0.0, 0.0, 1.0);
    }
    
    return eigenvec / len;
}

void solveEigenSystem(mat3 M, out vec3 eigenvalues, out mat3 eigenvectors) {
    
    float m = (M[0][0] + M[1][1] + M[2][2]) / 3.0;
    
    float c1_sq = (M[0][0]-m)*(M[0][0]-m) + (M[1][1]-m)*(M[1][1]-m) + (M[2][2]-m)*(M[2][2]-m) 
                  + 2.0 * (M[0][1]*M[0][1] + M[0][2]*M[0][2] + M[1][2]*M[1][2]);
    
    float det_M_minus_mI = (M[0][0]-m)*((M[1][1]-m)*(M[2][2]-m) - M[1][2]*M[1][2])
                         - M[0][1]*(M[0][1]*(M[2][2]-m) - M[0][2]*M[1][2])
                         + M[0][2]*(M[0][1]*M[1][2] - M[0][2]*(M[1][1]-m));
    float c0 = -det_M_minus_mI;

    float c1_sq_safe = max(c1_sq, 0.0);
    float p = sqrt(c1_sq_safe/6.0);
    
    if (p < 1e-8) {
        eigenvalues = vec3(m);
        eigenvectors = mat3(1.0, 0.0, 0.0,
                           0.0, 1.0, 0.0,
                           0.0, 0.0, 1.0);
        return;
    }

    float inv_p = 1.0 / p;
    float b = 0.5 * c0 * inv_p * inv_p * inv_p;
    float phi = 0.0;
    if (b >= 1.0) phi = 0.0;
    else if (b <= -1.0) phi = M_PI/3.0;
    else phi = acos(b)/3.0;

    eigenvalues.x = m + 2.0 * p * cos(phi);
    eigenvalues.z = m + 2.0 * p * cos(phi + (2.0*M_PI/3.0));
    eigenvalues.y = 3.0 * m - eigenvalues.x - eigenvalues.z;

    eigenvectors[0] = computeEigenvector(M, eigenvalues.x);
    eigenvectors[1] = computeEigenvector(M, eigenvalues.y);
    eigenvectors[2] = computeEigenvector(M, eigenvalues.z); // not taking cross
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

bool intersectPlane(vec3 ro, vec3 rd, vec3 plane_center, vec3 plane_normal, out float t) {
    float denom = dot(plane_normal, rd);
    if (abs(denom) > 1e-6) { 
        vec3 dist = plane_center - ro;
        t = dot(dist, plane_normal) / denom;
        return (t >= 0.0);
    }

    return false;
}

vec3 calculatePhongLighting(vec3 normal, vec3 intersection_point) {
    vec3 viewDir = normalize(cameraPos - intersection_point);
    vec3 lightDir = viewDir; // light comes from camera
    
    vec3 ambient = vec3(0.2);
    
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 diffuse = diff * vec3(0.8);
    
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32.0);
    vec3 specular = 0.5 * spec * vec3(1.0);
    
    return ambient + diffuse + specular;
}

void setFragmentDepth(vec3 worldPos) {
    vec4 clipPos = projection * view * vec4(worldPos, 1.0);
    float ndcDepth = clipPos.z / clipPos.w;
    gl_FragDepth = (ndcDepth + 1.0) * 0.5;
}

void main()
{
    // ray generation
    float x = (gl_FragCoord.x / 640.0) * 2.0 - 1.0;
    float y = (gl_FragCoord.y / 480.0) * 2.0 - 1.0;
    vec4 target = invProjection * vec4(x, y, 1.0, 1.0);
    vec3 rd = vec3(invView * vec4(normalize(vec3(target) / target.w), 0.0));
    vec3 ro = cameraPos;

    // bounding box intersection
    vec3 grid_min = vec3(0.0);
    vec3 grid_max = vec3(gridSize);
    float t = ray_aabb(ro, rd, grid_min, grid_max);
    if (t == 1e30) { // didnt hit bounding box
        FragColor = vec4(0.1, 0.1, 0.1, 1.0);
        return;
    }

    const int MAX_STEPS = 256;
    const float T_MIN = 0.001;

    t = max(t, T_MIN);
    vec3 start_pos = ro + rd * t;
    
    // DDA initialization
    ivec3 voxel = ivec3(floor(start_pos));
    vec3 delta_t = abs(1.0 / rd);
    ivec3 step = ivec3(sign(rd));
    
    // If starting outside grid, clamp to grid boundaries
    voxel = clamp(voxel, ivec3(0), ivec3(gridSize - 1));
    
    // Recalculate start position if we clamped the voxel
    vec3 grid_start_pos = vec3(voxel) + 0.5;
    
    vec3 t_max;
    for (int i = 0; i < 3; ++i) {
        if (step[i] > 0) {
            t_max[i] = t + (float(voxel[i] + 1) - start_pos[i]) * delta_t[i];
        } else {
            t_max[i] = t + (start_pos[i] - float(voxel[i])) * delta_t[i];
        }
        
        // Handle case where we're outside the grid
        if (start_pos[i] < 0.0 && step[i] > 0) {
            t_max[i] = t + (1.0 - start_pos[i]) * delta_t[i];
        } else if (start_pos[i] >= float(gridSize) && step[i] < 0) {
            t_max[i] = t + (start_pos[i] - float(gridSize - 1)) * delta_t[i];
        }
    }
    
    // DDA traversal
    for (int i = 0; i < MAX_STEPS; ++i) {
        if (voxel.x < 0 || voxel.x >= gridSize ||
            voxel.y < 0 || voxel.y >= gridSize ||
            voxel.z < 0 || voxel.z >= gridSize) {
            break;
        }
        
        if (texelFetch(voxelData, voxel, 0).r > 0.0) {
            vec3 hit_normal;
             
            if (usePlaneFitting) {
                if (useVoxelCentricWeighting) {
                    // voxel-centric with iterative refinement
                    float hit_t = min(min(t_max.x, t_max.y), t_max.z) - min(min(delta_t.x, delta_t.y), delta_t.z);
                    vec3 current_pos = ro + rd * max(0.0, hit_t);
                    bool found_surface = false;
                    vec3 final_normal;
                    vec3 final_intersection;
                    
                    for (int iter = 0; iter < voxelCentricMaxIterations; iter++) {
                        ivec3 current_voxel = ivec3(floor(current_pos));
                        
                        // Check if still in grid
                        if (current_voxel.x < 0 || current_voxel.x >= gridSize ||
                            current_voxel.y < 0 || current_voxel.y >= gridSize ||
                            current_voxel.z < 0 || current_voxel.z >= gridSize) {
                            break;
                        }
                        
                        // Check if current voxel is solid
                        if (texelFetch(voxelData, current_voxel, 0).r <= 0.0) {
                            // Step forward and continue
                            current_pos += rd * voxelCentricStepSize;
                            continue;
                        }
                        
                        // Get neighborhood around current position
                        vec3 neighbors[MAX_NEIGHBORS];
                        int neighbor_count;
                        
                        if (useSphericalNeighborhood == 1) {
                            neighbor_count = getSphericalNeighborhood(current_pos, sphericalRadius, neighbors);
                        } else {
                            neighbor_count = getVoxelNeighborhood(current_voxel, neighborhoodRingSize, neighbors);
                        }
                        
                        if (neighbor_count < 3) {
                            // Not enough neighbors, step forward
                            current_pos += rd * voxelCentricStepSize;
                            continue;
                        }
                        
                        // Compute plane with voxel-centric weighting
                        vec3 center_voxel_pos = vec3(current_voxel) + 0.5;
                        float effectiveRingSize = (useSphericalNeighborhood == 1) ? sphericalRadius : float(neighborhoodRingSize);
                        int ringSize = int(effectiveRingSize);
                        vec3 plane_center = calculateVoxelCentricWeightedCentroid(neighbors, neighbor_count, current_pos, ringSize);
                        mat3 covariance = calculateVoxelCentricWeightedCovarianceMatrix(neighbors, neighbor_count, plane_center, current_pos, ringSize);
                        
                        vec3 eigenvalues;
                        mat3 eigenvectors;
                        solveEigenSystem(covariance, eigenvalues, eigenvectors);
                        
                        int smallest_eigenvalue = 0;
                        if (eigenvalues[1] < eigenvalues[smallest_eigenvalue]) {
                            smallest_eigenvalue = 1;
                        }
                        if (eigenvalues[2] < eigenvalues[smallest_eigenvalue]) {
                            smallest_eigenvalue = 2;
                        }
                        vec3 plane_normal = eigenvectors[smallest_eigenvalue];
                        
                        // Orient normal toward camera
                        vec3 view_direction = normalize(ro - center_voxel_pos);
                        if (dot(plane_normal, view_direction) < 0.0) {
                            plane_normal = -plane_normal;
                        }
                        
                        // Intersect with plane
                        float plane_t;
                        bool plane_hit = intersectPlane(ro, rd, plane_center, plane_normal, plane_t);
                        
                        if (plane_hit && plane_t > 0.0) {
                            vec3 intersection_point = ro + rd * plane_t;
                            
                            // Check if intersection is within epsilon
                            vec3 diff = intersection_point - current_pos;
                            float dist = length(diff);

                            
                            if (dist < voxelCentricEpsilon) {
                                found_surface = true;
                                final_normal = normalize(plane_normal);
                                final_intersection = intersection_point;
                                break;
                            }
                        }
                        
                        // Step forward along ray
                        current_pos += rd * voxelCentricStepSize;
                    }
                    
                    if (found_surface) {
                        setFragmentDepth(final_intersection);
                        if (usePhongLighting) {
                            vec3 result = calculatePhongLighting(final_normal, final_intersection);
                            FragColor = vec4(result, 1.0);
                        } else {
                            FragColor = vec4(abs(final_normal) * 0.7 + 0.3, 1.0);
                        }
                        return;
                    }
                    
                } else {
                    // ray-centric mode
                    vec3 neighbors[MAX_NEIGHBORS];
                    int neighbor_count;
                    
                    if (useSphericalNeighborhood == 1) {
                        vec3 sample_point;
                        if (useVoxelCenterForSphere) {
                            sample_point = vec3(voxel) + 0.5;
                        } else {
                            float hit_t = min(min(t_max.x, t_max.y), t_max.z) - min(min(delta_t.x, delta_t.y), delta_t.z);
                            sample_point = ro + rd * hit_t;
                        }
                        neighbor_count = getSphericalNeighborhood(sample_point, sphericalRadius, neighbors);
                    } else {
                        neighbor_count = getVoxelNeighborhood(voxel, neighborhoodRingSize, neighbors);
                    }
                    
                    vec3 plane_center;
                    mat3 covariance;
                    
                    if (useDistanceWeighting) {
                        // Use sphericalRadius for weighting if spherical neighborhoods are enabled
                        float effectiveRingSize = (useSphericalNeighborhood == 1) ? sphericalRadius : float(neighborhoodRingSize);
                        int ringSize = int(effectiveRingSize);
                        plane_center = calculateWeightedCentroid(neighbors, neighbor_count, ro, rd, ringSize);
                        covariance = calculateWeightedCovarianceMatrix(neighbors, neighbor_count, plane_center, ro, rd, ringSize);
                    } else {
                        plane_center = calculateCentroid(neighbors, neighbor_count);
                        covariance = calculateCovarianceMatrix(neighbors, neighbor_count, plane_center);
                    }
                    
                    vec3 eigenvalues;
                    mat3 eigenvectors;
                    solveEigenSystem(covariance, eigenvalues, eigenvectors);

                    int smallest_eigenvalue = 0;
                    if (eigenvalues[1] < eigenvalues[smallest_eigenvalue]) {
                        smallest_eigenvalue = 1;
                    }
                    if (eigenvalues[2] < eigenvalues[smallest_eigenvalue]) {
                        smallest_eigenvalue = 2;
                    }
                    vec3 plane_normal = eigenvectors[smallest_eigenvalue];
                    
                    vec3 view_direction = normalize(ro - (vec3(voxel) + 0.5));
                    if (dot(plane_normal, view_direction) < 0.0) {
                        plane_normal = -plane_normal;
                    }
                    
                    float t_plane;
                    bool hit = intersectPlane(ro, rd, plane_center, plane_normal, t_plane);
                    
                    if (hit) {
                        vec3 intersection_point = ro + rd * t_plane;
                        ivec3 voxel_coords = ivec3(floor(intersection_point));

                        if (checkBounds) {
                            if (all(equal(voxel_coords, voxel))) {
                                plane_normal = normalize(plane_normal);
                                hit_normal = plane_normal;
                                
                                setFragmentDepth(intersection_point);
                                if (usePhongLighting) {
                                    vec3 result = calculatePhongLighting(hit_normal, intersection_point);
                                    FragColor = vec4(result, 1.0);
                                } else {
                                    FragColor = vec4(abs(hit_normal) * 0.7 + 0.3, 1.0);
                                }
                                return;
                            }
                        }
                        else {
                            plane_normal = normalize(plane_normal);
                            hit_normal = plane_normal;
                            
                            setFragmentDepth(intersection_point);
                            if (usePhongLighting) {
                                vec3 result = calculatePhongLighting(hit_normal, intersection_point);
                                FragColor = vec4(result, 1.0);
                            } else {
                                FragColor = vec4(abs(hit_normal) * 0.7 + 0.3, 1.0);
                            }
                            return;
                        }
                        // if the intersection point isnt in the current voxel, we continue voxel traversal
                    }
                }

            } else {
                if (t_max.x - delta_t.x > t_max.y - delta_t.y && t_max.x - delta_t.x > t_max.z - delta_t.z) {
                    hit_normal = vec3(-step.x, 0.0, 0.0);
                } else if (t_max.y - delta_t.y > t_max.z - delta_t.z) {
                    hit_normal = vec3(0.0, -step.y, 0.0);
                } else {
                    hit_normal = vec3(0.0, 0.0, -step.z);
                }
                
                // calculate intersection point for this voxel
                vec3 intersection_point = ro + rd * (min(min(t_max.x, t_max.y), t_max.z) - min(min(delta_t.x, delta_t.y), delta_t.z));
                setFragmentDepth(intersection_point);
                
                if (usePhongLighting) {
                    vec3 result = calculatePhongLighting(hit_normal, intersection_point);
                    FragColor = vec4(result, 1.0);
                } else {
                    FragColor = vec4(abs(hit_normal) * 0.7 + 0.3, 1.0);
                }
                return;
            }
        }
        
        // step to next voxel
        if (t_max.x < t_max.y) {
            if (t_max.x < t_max.z) {
                voxel.x += step.x;
                t_max.x += delta_t.x;
            } else {
                voxel.z += step.z;
                t_max.z += delta_t.z;
            }
        } else {
            if (t_max.y < t_max.z) {
                voxel.y += step.y;
                t_max.y += delta_t.y;
            } else {
                voxel.z += step.z;
                t_max.z += delta_t.z;
            }
        }
    }
    
    FragColor = vec4(0.1, 0.1, 0.1, 1.0); // didnt hit a voxel
}