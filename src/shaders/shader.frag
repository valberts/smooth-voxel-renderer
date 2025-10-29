#version 430 core
#define M_PI 3.1415926535897932384626433832795

layout(location = 0) out vec4 FragColor;

uniform sampler3D voxelData;
uniform bool useFitting;
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
uniform bool visualizeRadius;
uniform int falloffMode; // 0 = linear, 1 = gaussian
uniform int surfaceType; // 0 = plane, 1 = sphere

// Mouse picking
uniform vec2 mousePixel; // Mouse position in screen coordinates
uniform bool shouldUpdateClicked; // True when left mouse button clicked

const int MAX_NEIGHBORS = 343;

// GPU Picking Data - written by shader, read by CPU
layout(std430, binding = 0) buffer PickingData {
    ivec4 hoveredVoxel;               // xyz = voxel coords, w = valid flag
    int hoveredNeighborCount;
    int padding1[3];
    ivec4 hoveredNeighbors[MAX_NEIGHBORS];
    
    ivec4 clickedVoxel;                // xyz = voxel coords, w = valid flag
    int clickedNeighborCount;
    int padding2[3];
    ivec4 clickedNeighbors[MAX_NEIGHBORS];
} pickingData;

// Global arrays to avoid register pressure from multiple large local arrays
vec3 g_neighbors[MAX_NEIGHBORS];
float g_weights[MAX_NEIGHBORS];

// seperable 1D gaussian: G(x) = exp(-x^2 / (2*sigma^2))
float computeGaussian1D(float offset, float sigma) {
    return exp(-(offset * offset) / (2.0 * sigma * sigma));
}

// 3D gaussian convolution weight using separable kernels
// weight = G(dx) * G(dy) * G(dz)
float computeGaussianConvolutionWeight(vec3 offset, float sigma) {
    return computeGaussian1D(offset.x, sigma) * computeGaussian1D(offset.y, sigma) * computeGaussian1D(offset.z, sigma);
}

float computeGaussianDistanceFalloff(vec3 point, vec3 rayOrigin, vec3 rayDirection, int ringSize) {
    // perpendicular distance
    vec3 toPoint = point - rayOrigin;
    vec3 projection = dot(toPoint, rayDirection) * rayDirection;
    vec3 perpendicular = toPoint - projection;
    
    // linear falloff has maxDist = sqrt(distanceWeightMultiplier) * ringSize
    // for Gaussian, we set sigma = maxDist / 2.5 so that at maxDist, weight ≈ 0.01
    float maxDist = sqrt(distanceWeightMultiplier) * float(ringSize);
    float sigma = maxDist / 2.5; // 2.5*sigma, gaussian ≈ 0.01
    
    return computeGaussianConvolutionWeight(perpendicular, sigma);
}

float computeGaussianVoxelCentricFalloff(vec3 neighborPos, vec3 currentRayPos, int ringSize) {
    vec3 offset = neighborPos - currentRayPos;
    
    float maxDist = sqrt(3.0) * float(ringSize);
    float sigma = maxDist / 2.5; // 2.5*sigma, gaussian ≈ 0.01
    
    // 3d gaussian convolution: G(x,y,z) = G(x) * G(y) * G(z)
    return computeGaussianConvolutionWeight(offset, sigma);
}

// ray-centric distance weight: perpendicular distance to ray
float computeDistanceWeight(vec3 point, vec3 rayOrigin, vec3 rayDirection, int ringSize) {
    if (falloffMode == 1) { // gaussian
        return computeGaussianDistanceFalloff(point, rayOrigin, rayDirection, ringSize);
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
float computeVoxelCentricWeight(vec3 neighborPos, vec3 currentRayPos, int ringSize) {
    if (falloffMode == 1) { // gaussian
        return computeGaussianVoxelCentricFalloff(neighborPos, currentRayPos, ringSize);
    }
    
    // Linear falloff (falloffMode == 0)
    float dist = distance(neighborPos, currentRayPos);
    float maxDist = sqrt(3.0) * float(ringSize); // diagonal of neighborhood
    
    // linear falloff, 1.0 at center to 0.0 at edge
    float weight = 1.0 - (dist / maxDist);
    return max(0.0, weight);
}

int getNeighborsVoxel(ivec3 coord, int ringSize, out vec3 neighbors[MAX_NEIGHBORS]) {
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

int getNeighborsSpherical(vec3 point, float radius, out vec3 neighbors[MAX_NEIGHBORS]) {
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

vec3 computeCentroid(vec3 neighbors[MAX_NEIGHBORS], int count) {
    if (count == 0) {
        return vec3(0.0);
    }

    vec3 sum = vec3(0.0);
    for (int i = 0; i < count; ++i) {
        sum += neighbors[i];
    }
    return sum / float(count);
}

vec3 computeWeightedCentroid(vec3 neighbors[MAX_NEIGHBORS], int count, vec3 rayOrigin, vec3 rayDirection, int ringSize) {
    if (count == 0) {
        return vec3(0.0);
    }

    vec3 weightedSum = vec3(0.0);
    float totalWeight = 0.0;
    
    for (int i = 0; i < count; ++i) {
        float weight = computeDistanceWeight(neighbors[i], rayOrigin, rayDirection, ringSize);
        weightedSum += neighbors[i] * weight;
        totalWeight += weight;
    }

    return weightedSum / totalWeight;
}

vec3 computeVoxelCentricWeightedCentroid(vec3 neighbors[MAX_NEIGHBORS], int count, vec3 currentRayPos, int ringSize) {
    if (count == 0) {
        return vec3(0.0);
    }

    vec3 weightedSum = vec3(0.0);
    float totalWeight = 0.0;
    
    for (int i = 0; i < count; ++i) {
        float weight = computeVoxelCentricWeight(neighbors[i], currentRayPos, ringSize);
        weightedSum += neighbors[i] * weight;
        totalWeight += weight;
    }
    
    return weightedSum / totalWeight;
}

mat3 computeCovarianceMatrix(vec3 neighbors[MAX_NEIGHBORS], int count, vec3 mean) {
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

mat3 computeWeightedCovarianceMatrix(vec3 neighbors[MAX_NEIGHBORS], int count, vec3 mean, vec3 rayOrigin, vec3 rayDirection, int ringSize) {
    if (count <= 1) {
        return mat3(0.0);
    }

    mat3 covariance = mat3(0.0);
    float totalWeight = 0.0;

    for (int i = 0; i < count; ++i) {
        float weight = computeDistanceWeight(neighbors[i], rayOrigin, rayDirection, ringSize);
        vec3 diff = neighbors[i] - mean;
        covariance += weight * outerProduct(diff, diff);
        totalWeight += weight;
    }

    covariance = covariance / totalWeight;

    return covariance;
}

mat3 computeVoxelCentricWeightedCovarianceMatrix(vec3 neighbors[MAX_NEIGHBORS], int count, vec3 mean, vec3 currentRayPos, int ringSize) {
    if (count <= 1) return mat3(0.0);
    mat3 covariance = mat3(0.0);
    float totalWeight = 0.0;
    for (int i = 0; i < count; ++i) {
        float weight = computeVoxelCentricWeight(neighbors[i], currentRayPos, ringSize);
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

// https://lucidar.me/en/mathematics/least-squares-fitting-of-sphere/
// Returns vec4(center.xyz, radius)
// Weighted version - each point contributes according to its weight
vec4 fitSphere(vec3 neighbors[MAX_NEIGHBORS], float weights[MAX_NEIGHBORS], int N) {
    // STEP 1: Check we have enough points
    if (N < 4) {
        return vec4(0.0, 0.0, 0.0, -1.0);
    }
    
    // STEP 2: Initialize matrices
    mat4 ATA = mat4(0.0);
    vec4 ATB = vec4(0.0);
    
    // STEP 3: Accumulate A^T·A and A^T·B with weights
    for (int i = 0; i < N; ++i) {
        vec3 p = neighbors[i];
        float weight = weights[i];
        
        // Row of A: [x, y, z, 1]
        float x = p.x;
        float y = p.y;
        float z = p.z;
        float w = 1.0;
        
        // Element of B: x² + y² + z²
        float b_i = x*x + y*y + z*z;
        
        // Build A^T·A (symmetric 4x4) with weights
        ATA[0][0] += weight * x * x;  // Σwi·xi²
        ATA[0][1] += weight * x * y;  // Σwi·xi·yi
        ATA[0][2] += weight * x * z;  // Σwi·xi·zi
        ATA[0][3] += weight * x * w;  // Σwi·xi
        
        ATA[1][0] += weight * y * x;  // Σwi·yi·xi (symmetric)
        ATA[1][1] += weight * y * y;  // Σwi·yi²
        ATA[1][2] += weight * y * z;  // Σwi·yi·zi
        ATA[1][3] += weight * y * w;  // Σwi·yi
        
        ATA[2][0] += weight * z * x;  // Σwi·zi·xi
        ATA[2][1] += weight * z * y;  // Σwi·zi·yi
        ATA[2][2] += weight * z * z;  // Σwi·zi²
        ATA[2][3] += weight * z * w;  // Σwi·zi
        
        ATA[3][0] += weight * w * x;  // Σwi·xi
        ATA[3][1] += weight * w * y;  // Σwi·yi
        ATA[3][2] += weight * w * z;  // Σwi·zi
        ATA[3][3] += weight * w * w;  // Σwi
        
        // Build A^T·B with weights
        ATB.x += weight * x * b_i;  // Σwi·xi·(xi²+yi²+zi²)
        ATB.y += weight * y * b_i;  // Σwi·yi·(xi²+yi²+zi²)
        ATB.z += weight * z * b_i;  // Σwi·zi·(xi²+yi²+zi²)
        ATB.w += weight * w * b_i;  // Σwi·(xi²+yi²+zi²)
    }
    
    // STEP 4: Solve X = (A^T·A)^-1·(A^T·B)
    vec4 X = inverse(ATA) * ATB;
    
    // X = [a, b, c, d]
    float a = X.x;  // = 2xc
    float b = X.y;  // = 2yc
    float c = X.z;  // = 2zc
    float d = X.w;  // = r² - xc² - yc² - zc²
    
    // STEP 5: Extract center
    vec3 center;
    center.x = a * 0.5;  // xc = a/2
    center.y = b * 0.5;  // yc = b/2
    center.z = c * 0.5;  // zc = c/2
    
    // STEP 6: Calculate radius
    // r² = (4d + a² + b² + c²) / 4
    float r_squared = (4.0 * d + a*a + b*b + c*c) * 0.25;

    
    if (r_squared <= 0.0) {
        return vec4(center, -2.0);  // Invalid
    }
    
    float radius = sqrt(r_squared);
    
    return vec4(center, radius);
}

// https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-sphere-intersection.html
bool intersectSphere(vec3 ro, vec3 rd, vec3 center, float radius, out float t) {
    vec3 L = ro - center;
    float a = dot(rd, rd);
    float b = 2.0 * dot(rd, L);
    float c = dot(L, L) - radius * radius;
    
    float discriminant = b * b - 4.0 * a * c;
    
    if (discriminant < 0.0) {
        return false;
    }
    
    float sqrtDisc = sqrt(discriminant);
    float q = (b > 0.0) ? 
        -0.5 * (b + sqrtDisc) : 
        -0.5 * (b - sqrtDisc);
    
    float t0 = q / a;
    float t1 = c / q;
    
    if (t0 > t1) {
        float temp = t0;
        t0 = t1;
        t1 = temp;
    }
    
    if (t0 > 0.0) {
        t = t0;
        return true;
    } else if (t1 > 0.0) {
        t = t1;
        return true;
    }
    
    return false;
}

// https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-plane-and-ray-disk-intersection.html
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

// Extract smallest eigenvalue index from eigenvalues vector
int findSmallestEigenvalueIndex(vec3 eigenvalues) {
    int smallest = 0;
    if (eigenvalues[1] < eigenvalues[smallest]) {
        smallest = 1;
    }
    if (eigenvalues[2] < eigenvalues[smallest]) {
        smallest = 2;
    }
    return smallest;
}

// Render final surface with optional lighting
void renderSurface(vec3 normal, vec3 intersection_point) {
    setFragmentDepth(intersection_point);
    if (usePhongLighting) {
        vec3 result = calculatePhongLighting(normal, intersection_point);
        FragColor = vec4(result, 1.0);
    } else {
        FragColor = vec4(abs(normal) * 0.7 + 0.3, 1.0);
    }
}

// Fit plane to neighbors using PCA (weighted or unweighted)
vec3 computePlaneNormal(vec3 neighbors[MAX_NEIGHBORS], int neighborCount, vec3 rayOrigin, vec3 rayDir, vec3 referencePoint) {
    vec3 planeCenter;
    mat3 covariance;
    
    if (useDistanceWeighting) {
        // Weighted: neighbors closer to ray have more influence
        float effectiveRingSize = (useSphericalNeighborhood == 1) ? sphericalRadius : float(neighborhoodRingSize);
        int ringSize = int(effectiveRingSize);
        planeCenter = computeWeightedCentroid(neighbors, neighborCount, rayOrigin, rayDir, ringSize);
        covariance = computeWeightedCovarianceMatrix(neighbors, neighborCount, planeCenter, rayOrigin, rayDir, ringSize);
    } else {
        // Unweighted: all neighbors contribute equally (standard PCA plane fitting)
        planeCenter = computeCentroid(neighbors, neighborCount);
        covariance = computeCovarianceMatrix(neighbors, neighborCount, planeCenter);
    }
    
    vec3 eigenvalues;
    mat3 eigenvectors;
    solveEigenSystem(covariance, eigenvalues, eigenvectors);
    
    int smallest = findSmallestEigenvalueIndex(eigenvalues);
    vec3 planeNormal = eigenvectors[smallest];
    
    // Orient normal toward camera
    vec3 viewDirection = normalize(rayOrigin - referencePoint);
    if (dot(planeNormal, viewDirection) < 0.0) {
        planeNormal = -planeNormal;
    }
    
    return normalize(planeNormal);
}

// Fit plane with voxel-centric weighting
void computePlaneVoxelCentric(vec3 neighbors[MAX_NEIGHBORS], int neighborCount, vec3 currentPos, vec3 rayOrigin, vec3 referencePoint, out vec3 planeCenter, out vec3 planeNormal) {
    float effectiveRingSize = (useSphericalNeighborhood == 1) ? sphericalRadius : float(neighborhoodRingSize);
    int ringSize = int(effectiveRingSize);
    
    planeCenter = computeVoxelCentricWeightedCentroid(neighbors, neighborCount, currentPos, ringSize);
    mat3 covariance = computeVoxelCentricWeightedCovarianceMatrix(neighbors, neighborCount, planeCenter, currentPos, ringSize);
    
    vec3 eigenvalues;
    mat3 eigenvectors;
    solveEigenSystem(covariance, eigenvalues, eigenvectors);
    
    int smallest = findSmallestEigenvalueIndex(eigenvalues);
    planeNormal = eigenvectors[smallest];
    
    // Orient normal toward camera
    vec3 viewDirection = normalize(rayOrigin - referencePoint);
    if (dot(planeNormal, viewDirection) < 0.0) {
        planeNormal = -planeNormal;
    }
}

// Initialize DDA traversal parameters
void initDDA(vec3 rayOrigin, vec3 rayDir, float t, vec3 startPos, out ivec3 voxel, out vec3 deltaT, out ivec3 step, out vec3 tMax) {
    voxel = ivec3(floor(startPos));
    deltaT = abs(1.0 / rayDir);
    step = ivec3(sign(rayDir));
    
    // If starting outside grid, clamp to grid boundaries
    voxel = clamp(voxel, ivec3(0), ivec3(gridSize - 1));
    
    for (int i = 0; i < 3; ++i) {
        if (step[i] > 0) {
            tMax[i] = t + (float(voxel[i] + 1) - startPos[i]) * deltaT[i];
        } else {
            tMax[i] = t + (startPos[i] - float(voxel[i])) * deltaT[i];
        }
        
        // Handle case where we're outside the grid
        if (startPos[i] < 0.0 && step[i] > 0) {
            tMax[i] = t + (1.0 - startPos[i]) * deltaT[i];
        } else if (startPos[i] >= float(gridSize) && step[i] < 0) {
            tMax[i] = t + (startPos[i] - float(gridSize - 1)) * deltaT[i];
        }
    }
}

// Check if voxel is within grid bounds
bool isInBounds(ivec3 voxel) {
    return voxel.x >= 0 && voxel.x < gridSize &&
           voxel.y >= 0 && voxel.y < gridSize &&
           voxel.z >= 0 && voxel.z < gridSize;
}

// Get sample point for spherical neighborhood
vec3 findSamplePoint(ivec3 voxel, vec3 rayOrigin, vec3 rayDir, vec3 tMax, vec3 deltaT) {
    if (useVoxelCenterForSphere) {
        return vec3(voxel) + 0.5;
    } else {
        float hitT = min(min(tMax.x, tMax.y), tMax.z) - min(min(deltaT.x, deltaT.y), deltaT.z);
        return rayOrigin + rayDir * hitT;
    }
}

// Update picking data if this pixel is at the mouse cursor
void updatePickingAtMouse(ivec3 voxel, vec3 samplePoint) {
    // Check if we're rendering the pixel under the mouse (use 1.5 pixel threshold for safety)
    if (distance(gl_FragCoord.xy, mousePixel) < 1.5) {
        // This is the hovered voxel - update SSBO
        pickingData.hoveredVoxel = ivec4(voxel, 1);
        
        // Gather neighbors based on current mode
        int neighborCount = 0;
        
        if (useSphericalNeighborhood > 0) {
            // Spherical neighborhood
            int maxOffset = int(ceil(sphericalRadius));
            for (int z = -maxOffset; z <= maxOffset && neighborCount < MAX_NEIGHBORS; z++) {
                for (int y = -maxOffset; y <= maxOffset && neighborCount < MAX_NEIGHBORS; y++) {
                    for (int x = -maxOffset; x <= maxOffset && neighborCount < MAX_NEIGHBORS; x++) {
                        ivec3 neighbor = voxel + ivec3(x, y, z);
                        if (isInBounds(neighbor) && texelFetch(voxelData, neighbor, 0).r > 0.0) {
                            vec3 neighborPos = useVoxelCenterForSphere ? 
                                (vec3(neighbor) + 0.5) : vec3(neighbor);
                            if (distance(samplePoint, neighborPos) <= sphericalRadius) {
                                pickingData.hoveredNeighbors[neighborCount++] = ivec4(neighbor, 1);
                            }
                        }
                    }
                }
            }
        } else {
            // Cubic neighborhood
            for (int z = -neighborhoodRingSize; z <= neighborhoodRingSize && neighborCount < MAX_NEIGHBORS; z++) {
                for (int y = -neighborhoodRingSize; y <= neighborhoodRingSize && neighborCount < MAX_NEIGHBORS; y++) {
                    for (int x = -neighborhoodRingSize; x <= neighborhoodRingSize && neighborCount < MAX_NEIGHBORS; x++) {
                        ivec3 neighbor = voxel + ivec3(x, y, z);
                        if (isInBounds(neighbor) && texelFetch(voxelData, neighbor, 0).r > 0.0) {
                            pickingData.hoveredNeighbors[neighborCount++] = ivec4(neighbor, 1);
                        }
                    }
                }
            }
        }
        
        pickingData.hoveredNeighborCount = neighborCount;
    }
    
    // Handle click - check anywhere in the frame, not just at mouse pixel
    // This allows clicking to work even if we process a different pixel first
    if (shouldUpdateClicked && pickingData.hoveredVoxel.w > 0) {
        pickingData.clickedVoxel = pickingData.hoveredVoxel;
        pickingData.clickedNeighborCount = pickingData.hoveredNeighborCount;
        for (int i = 0; i < pickingData.hoveredNeighborCount; i++) {
            pickingData.clickedNeighbors[i] = pickingData.hoveredNeighbors[i];
        }
    }
}

// Overload that calculates sample point from DDA parameters
void updatePickingAtMouse(ivec3 voxel, vec3 rayOrigin, vec3 rayDir, vec3 tMax, vec3 deltaT) {
    vec3 samplePoint = findSamplePoint(voxel, rayOrigin, rayDir, tMax, deltaT);
    updatePickingAtMouse(voxel, samplePoint);
}

// Check if a voxel should be highlighted
vec4 getHighlightColor(ivec3 voxel) {
    // Check if this is the hovered voxel (white)
    if (pickingData.hoveredVoxel.w > 0 && 
        all(equal(voxel, pickingData.hoveredVoxel.xyz))) {
        return vec4(1.0, 1.0, 1.0, 1.0);
    }
    
    // Check if this is one of the clicked neighbors (yellow)
    if (pickingData.clickedVoxel.w > 0) {
        for (int i = 0; i < pickingData.clickedNeighborCount; i++) {
            if (all(equal(voxel, pickingData.clickedNeighbors[i].xyz))) {
                return vec4(1.0, 1.0, 0.0, 1.0);
            }
        }
    }
    
    return vec4(0.0); // No highlight
}

// Calculate basic voxel normal from DDA traversal direction
vec3 getVoxelNormal(vec3 tMax, vec3 deltaT, ivec3 step) {
    if (tMax.x - deltaT.x > tMax.y - deltaT.y && tMax.x - deltaT.x > tMax.z - deltaT.z) {
        return vec3(-step.x, 0.0, 0.0);
    } else if (tMax.y - deltaT.y > tMax.z - deltaT.z) {
        return vec3(0.0, -step.y, 0.0);
    } else {
        return vec3(0.0, 0.0, -step.z);
    }
}

// Handle voxel-centric plane fitting with iterative refinement
bool tracePlaneVoxelCentric(vec3 rayOrigin, vec3 rayDir, vec3 tMax, vec3 deltaT) {
    float hitT = min(min(tMax.x, tMax.y), tMax.z) - min(min(deltaT.x, deltaT.y), deltaT.z);
    vec3 currentPos = rayOrigin + rayDir * max(0.0, hitT);
    
    for (int iter = 0; iter < voxelCentricMaxIterations; iter++) {
        ivec3 currentVoxel = ivec3(floor(currentPos));
        
        // Check if still in grid
        if (!isInBounds(currentVoxel)) {
            break;
        }
        
        // Check if current voxel is solid
        if (texelFetch(voxelData, currentVoxel, 0).r <= 0.0) {
            currentPos += rayDir * voxelCentricStepSize;
            continue;
        }
        
        // Get neighborhood around current position
        int neighborCount;
        
        if (useSphericalNeighborhood == 1) {
            neighborCount = getNeighborsSpherical(currentPos, sphericalRadius, g_neighbors);
        } else {
            neighborCount = getNeighborsVoxel(currentVoxel, neighborhoodRingSize, g_neighbors);
        }
        
        if (neighborCount < 3) {
            currentPos += rayDir * voxelCentricStepSize;
            continue;
        }
        
        // Compute plane with voxel-centric weighting
        vec3 centerVoxelPos = vec3(currentVoxel) + 0.5;
        vec3 planeCenter, planeNormal;
        computePlaneVoxelCentric(g_neighbors, neighborCount, currentPos, rayOrigin, centerVoxelPos, planeCenter, planeNormal);
        
        // Intersect with plane
        float planeT;
        bool planeHit = intersectPlane(rayOrigin, rayDir, planeCenter, planeNormal, planeT);
        
        if (planeHit && planeT > 0.0) {
            vec3 intersectionPoint = rayOrigin + rayDir * planeT;
            
            // Check if intersection is within epsilon
            float dist = distance(intersectionPoint, currentPos);
            
            if (dist < voxelCentricEpsilon) {
                updatePickingAtMouse(currentVoxel, rayOrigin, rayDir, tMax, deltaT);
                renderSurface(normalize(planeNormal), intersectionPoint);
                return true;
            }
        }
        
        currentPos += rayDir * voxelCentricStepSize;
    }
    
    return false;
}

// Handle ray-centric plane fitting mode
bool tracePlaneRayCentric(ivec3 voxel, vec3 rayOrigin, vec3 rayDir, vec3 tMax, vec3 deltaT) {
    int neighborCount;
    
    // Get neighbors
    if (useSphericalNeighborhood == 1) {
        vec3 samplePoint = findSamplePoint(voxel, rayOrigin, rayDir, tMax, deltaT);
        neighborCount = getNeighborsSpherical(samplePoint, sphericalRadius, g_neighbors);
    } else {
        neighborCount = getNeighborsVoxel(voxel, neighborhoodRingSize, g_neighbors);
    }
    
    // Fit plane using PCA
    vec3 planeNormal = computePlaneNormal(g_neighbors, neighborCount, rayOrigin, rayDir, vec3(voxel) + 0.5);
    
    // Calculate plane center for intersection
    vec3 planeCenter;
    if (useDistanceWeighting) {
        float effectiveRingSize = (useSphericalNeighborhood == 1) ? sphericalRadius : float(neighborhoodRingSize);
        int ringSize = int(effectiveRingSize);
        planeCenter = computeWeightedCentroid(g_neighbors, neighborCount, rayOrigin, rayDir, ringSize);
    } else {
        planeCenter = computeCentroid(g_neighbors, neighborCount);
    }
    
    // Intersect with plane
    float planeT;
    bool hit = intersectPlane(rayOrigin, rayDir, planeCenter, planeNormal, planeT);
    
    if (hit) {
        vec3 intersectionPoint = rayOrigin + rayDir * planeT;
        ivec3 voxelCoords = ivec3(floor(intersectionPoint));

        // Check if intersection is in the current voxel (if bounds checking is enabled)
        if (checkBounds) {
            if (all(equal(voxelCoords, voxel))) {
                updatePickingAtMouse(voxel, rayOrigin, rayDir, tMax, deltaT);
                renderSurface(planeNormal, intersectionPoint);
                return true;
            }
        } else {
            updatePickingAtMouse(voxel, rayOrigin, rayDir, tMax, deltaT);
            renderSurface(planeNormal, intersectionPoint);
            return true;
        }
    }
    
    return false;
}

bool areCoplanar(int count) {
    if (count < 4) return true;

    const float epsilon = 1e-6f;

    vec3 A = g_neighbors[0];
    int idxB = -1, idxC = -1;
    for (int i = 1; i < count; ++i) {
        if (distance(g_neighbors[i], A) > epsilon) {
            idxB = i;
            break;
        }
    }
    if (idxB == -1) return true; // all points equal

    vec3 B = g_neighbors[idxB];
    for (int i = idxB + 1; i < count; ++i) {
        vec3 v1 = B - A;
        vec3 v2 = g_neighbors[i] - A;
        if (length(cross(v1,v2)) > epsilon) {
            idxC = i;
            break;
        }
    }
    if (idxC == -1) return true; // all points are collinear

    vec3 C = g_neighbors[idxC];
    vec3 normal = normalize(cross(B- A, C - A));

    for (int i = 0; i < count; ++i) {
        float d = dot(normal, g_neighbors[i] - A);
        if (abs(d) > epsilon) return false; // not coplanar
    }

    return true; // coplanar, no points were outside plane
}

bool traceSphere(ivec3 voxel, vec3 rayOrigin, vec3 rayDir) {
    int neighborCount;
    neighborCount = getNeighborsVoxel(voxel, neighborhoodRingSize, g_neighbors);

    if (areCoplanar(neighborCount)) {
        updatePickingAtMouse(voxel, vec3(voxel) + 0.5);
        FragColor = vec4(1.0, 0.0, 0.0, 1.0); // coplanar
        return true;
    }

    for (int i = 0; i < neighborCount; i++) {
        g_weights[i] = 1.0;
    }

    vec4 sphere = fitSphere(g_neighbors, g_weights, neighborCount);
    vec3 center = sphere.xyz;
    float radius = sphere.w;

    if (visualizeRadius && !checkBounds) {
        updatePickingAtMouse(voxel, vec3(voxel) + 0.5);
        float mapped = clamp(radius / 1000.0, 0.0, 1.0);
        FragColor = vec4(vec3(mapped), 1.0);
        return true;
    }

    if (radius == -1.0) {
        updatePickingAtMouse(voxel, vec3(voxel) + 0.5);
        FragColor = vec4(1.0, 1.0, 0.0, 0.0); // invalid radius
        return true;
    } else if (radius == -2.0) {
        updatePickingAtMouse(voxel, vec3(voxel) + 0.5);
        FragColor = vec4(0.0, 1.0, 1.0, 0.0); // failed to fit
        return true;
    }

    float t_sphere;
    if (intersectSphere(rayOrigin, rayDir, center, radius, t_sphere)) {
        vec3 intersection = rayOrigin + rayDir * t_sphere;

        vec3 normal = normalize(intersection - sphere.xyz);
        setFragmentDepth(intersection);

        ivec3 hit_voxel = ivec3(floor(intersection));
                
        if (checkBounds) {
            if (all(equal(hit_voxel, voxel))) {
            if (visualizeRadius) {
                updatePickingAtMouse(voxel, vec3(voxel) + 0.5);
                float mapped = clamp(radius / 10.0, 0.0, 1.0);
                FragColor = vec4(vec3(mapped), 1.0);
                return true;
            }
                updatePickingAtMouse(voxel, vec3(voxel) + 0.5);
                renderSurface(normal, intersection);
                return true;
            }
        } else {
            updatePickingAtMouse(voxel, vec3(voxel) + 0.5);
            renderSurface(normal, intersection);
            return true;
        }
    }
    return false;
}

void main()
{
    // Ray generation
    float x = (gl_FragCoord.x / 640.0) * 2.0 - 1.0;
    float y = (gl_FragCoord.y / 480.0) * 2.0 - 1.0;
    vec4 target = invProjection * vec4(x, y, 1.0, 1.0);
    vec3 rayDir = vec3(invView * vec4(normalize(vec3(target) / target.w), 0.0));
    vec3 rayOrigin = cameraPos;

    // Bounding box intersection
    vec3 gridMin = vec3(0.0);
    vec3 gridMax = vec3(gridSize);
    float t = ray_aabb(rayOrigin, rayDir, gridMin, gridMax);
    if (t == 1e30) { // didnt hit bounding box
        // Clear hovered voxel if this is the mouse pixel
        if (distance(gl_FragCoord.xy, mousePixel) < 1.5) {
            pickingData.hoveredVoxel = ivec4(-1, -1, -1, 0);
            pickingData.hoveredNeighborCount = 0;
        }
        FragColor = vec4(0.1, 0.1, 0.1, 1.0);
        return;
    }

    const int MAX_STEPS = 256;
    const float T_MIN = 0.001;

    t = max(t, T_MIN);
    vec3 startPos = rayOrigin + rayDir * t;
    
    // DDA initialization
    ivec3 voxel;
    vec3 deltaT;
    ivec3 step;
    vec3 tMax;
    initDDA(rayOrigin, rayDir, t, startPos, voxel, deltaT, step, tMax);
    
    // DDA traversal
    for (int i = 0; i < MAX_STEPS; ++i) {
        if (!isInBounds(voxel)) {
            break;
        }
        
        if (texelFetch(voxelData, voxel, 0).r > 0.0) { // if voxel is solid
            
            // Check for highlighting
            vec4 highlightColor = getHighlightColor(voxel);
            if (highlightColor.w > 0.0) {
                vec3 intersectionPoint = rayOrigin + rayDir * (min(min(tMax.x, tMax.y), tMax.z) - min(min(deltaT.x, deltaT.y), deltaT.z));
                setFragmentDepth(intersectionPoint);
                FragColor = highlightColor;
                return;
            }
             
            if (useFitting) {
                // === SPHERE FITTING MODE ===
                if (surfaceType == 1) {
                    if (traceSphere(voxel, rayOrigin, rayDir)) {
                        return;
                    }
                    
                } else { // === PLANE FITTING MODES ===
                    // === VOXEL-CENTRIC WEIGHTING MODE ===
                    if (useVoxelCentricWeighting) {
                        if (tracePlaneVoxelCentric(rayOrigin, rayDir, tMax, deltaT)) {
                            return;
                        }
                        
                    } else { // === RAY-CENTRIC WEIGHTING MODE ===
                        if (tracePlaneRayCentric(voxel, rayOrigin, rayDir, tMax, deltaT)) {
                            return;
                        }
                    }
                }

            } else { // Basic voxel rendering without fitting
                vec3 hitNormal = getVoxelNormal(tMax, deltaT, step);
                vec3 intersectionPoint = rayOrigin + rayDir * (min(min(tMax.x, tMax.y), tMax.z) - min(min(deltaT.x, deltaT.y), deltaT.z));
                updatePickingAtMouse(voxel, rayOrigin, rayDir, tMax, deltaT);
                renderSurface(hitNormal, intersectionPoint);
                return;
            }
        }
        // Step to next voxel
        if (tMax.x < tMax.y) {
            if (tMax.x < tMax.z) {
                voxel.x += step.x;
                tMax.x += deltaT.x;
            } else {
                voxel.z += step.z;
                tMax.z += deltaT.z;
            }
        } else {
            if (tMax.y < tMax.z) {
                voxel.y += step.y;
                tMax.y += deltaT.y;
            } else {
                voxel.z += step.z;
                tMax.z += deltaT.z;
            }
        }
    }
    
    // Clear hovered voxel if this is the mouse pixel and we didn't hit anything
    if (distance(gl_FragCoord.xy, mousePixel) < 1.5) {
        pickingData.hoveredVoxel = ivec4(-1, -1, -1, 0);
        pickingData.hoveredNeighborCount = 0;
    }

    FragColor = vec4(0.1, 0.1, 0.1, 1.0); // Didn't hit a voxel
}
