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
uniform float boundsTolerance; // Tolerance for bounds checking (voxel units)
uniform bool usePlaneFallback; // Fallback to plane fitting when quadric is nearly planar
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
uniform int surfaceType; // 0 = plane, 1 = sphere, 2 = quadric

// Quadric parameters
uniform float quadricA, quadricB, quadricC;
uniform float quadricD, quadricE, quadricF;
uniform float quadricG, quadricH, quadricI, quadricJ;
uniform vec3 quadricCenter;

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
    
    // Hovered quadric coefficients (updated every frame when hovering)
    float hoveredQuadricA, hoveredQuadricB, hoveredQuadricC;
    float hoveredQuadricD, hoveredQuadricE, hoveredQuadricF;
    float hoveredQuadricG, hoveredQuadricH, hoveredQuadricI, hoveredQuadricJ;
    int hoveredQuadricValid;
    
    // Clicked quadric coefficients (only updated on click)
    float clickedQuadricA, clickedQuadricB, clickedQuadricC;
    float clickedQuadricD, clickedQuadricE, clickedQuadricF;
    float clickedQuadricG, clickedQuadricH, clickedQuadricI, clickedQuadricJ;
    int clickedQuadricValid;
} pickingData;

// Global arrays to avoid register pressure from multiple large local arrays
vec3 g_neighbors[MAX_NEIGHBORS];
float g_weights[MAX_NEIGHBORS];

// Check if intersection point is within voxel bounds with tolerance
bool isWithinBounds(vec3 intersection, ivec3 voxel) {
    vec3 localPos = intersection - vec3(voxel);
    // Accept if within voxel or slightly outside by boundsTolerance
    return all(greaterThanEqual(localPos, vec3(-boundsTolerance))) && 
           all(lessThan(localPos, vec3(1.0 + boundsTolerance)));
}

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
    
    // STEP 2: Compute weighted center for numerical stability
    vec3 dataCenter = vec3(0.0);
    float totalWeight = 0.0;
    for (int i = 0; i < N; ++i) {
        dataCenter += weights[i] * neighbors[i];
        totalWeight += weights[i];
    }
    dataCenter /= totalWeight;
    
    // STEP 3: Initialize matrices
    mat4 ATA = mat4(0.0);
    vec4 ATB = vec4(0.0);
    
    // STEP 4: Accumulate A^T·A and A^T·B with centered coordinates
    for (int i = 0; i < N; ++i) {
        // Translate to origin
        vec3 p = neighbors[i] - dataCenter;
        float weight = weights[i];
        
        // Row of A: [x, y, z, 1]
        float x = p.x;
        float y = p.y;
        float z = p.z;
        float w = 1.0;
        
        // Element of B: x² + y² + z²
        float b_i = x*x + y*y + z*z;
        
        // Build A^T·A (symmetric 4x4) with weights
        ATA[0][0] += weight * x * x;
        ATA[0][1] += weight * x * y;
        ATA[0][2] += weight * x * z;
        ATA[0][3] += weight * x * w;
        
        ATA[1][0] += weight * y * x;
        ATA[1][1] += weight * y * y;
        ATA[1][2] += weight * y * z;
        ATA[1][3] += weight * y * w;
        
        ATA[2][0] += weight * z * x;
        ATA[2][1] += weight * z * y;
        ATA[2][2] += weight * z * z;
        ATA[2][3] += weight * z * w;
        
        ATA[3][0] += weight * w * x;
        ATA[3][1] += weight * w * y;
        ATA[3][2] += weight * w * z;
        ATA[3][3] += weight * w * w;
        
        // Build A^T·B with weights
        ATB.x += weight * x * b_i;
        ATB.y += weight * y * b_i;
        ATB.z += weight * z * b_i;
        ATB.w += weight * w * b_i;
    }
    
    // STEP 5: Solve X = (A^T·A)^-1·(A^T·B)
    vec4 X = inverse(ATA) * ATB;
    
    // X = [a, b, c, d]
    float a = X.x;  // = 2xc (relative to centroid)
    float b = X.y;  // = 2yc
    float c = X.z;  // = 2zc
    float d = X.w;  // = r² - xc² - yc² - zc²
    
    // STEP 6: Extract center (relative to dataCenter)
    vec3 centerRelative;
    centerRelative.x = a * 0.5;
    centerRelative.y = b * 0.5;
    centerRelative.z = c * 0.5;
    
    // STEP 7: Translate back to world space
    vec3 center = centerRelative + dataCenter;
    
    // STEP 8: Calculate radius
    float r_squared = (4.0 * d + a*a + b*b + c*c) * 0.25;
    
    if (r_squared <= 0.0) {
        return vec4(center, -2.0);  // Invalid
    }
    
    float radius = sqrt(r_squared);
    
    return vec4(center, radius);
}

// Fit a general quadric surface: Ax² + By² + Cz² + Dxy + Exz + Fyz + Gx + Hy + Iz + J = 0
// We normalize by setting J = -1, giving us 9 unknowns: A, B, C, D, E, F, G, H, I
// Returns true if successful, outputs coefficients and the data center used
bool fitQuadric(vec3 neighbors[MAX_NEIGHBORS], float weights[MAX_NEIGHBORS], int N,
                out float A, out float B, out float C,
                out float D, out float E, out float F,
                out float G, out float H, out float I,
                out vec3 dataCenter) {
    // // Need at least 9 points for 9 unknowns
    // if (N < 9) {
    //     return false;
    // }
    
    // Compute weighted center for numerical stability
    dataCenter = vec3(0.0);
    float totalWeight = 0.0;
    for (int i = 0; i < N; ++i) {
        dataCenter += weights[i] * neighbors[i];
        totalWeight += weights[i];
    }
    dataCenter /= totalWeight;
    
    // Build the 9x9 system: M^T*M * [A,B,C,D,E,F,G,H,I]^T = M^T * b
    // where b = [1,1,...,1]^T (since J = -1)
    // Row for each point: [x², y², z², xy, xz, yz, x, y, z]
    
    // We'll store the 9x9 matrix as 3x3 blocks of mat3
    mat3 M00 = mat3(0.0); // rows [0-2], cols [0-2]
    mat3 M01 = mat3(0.0); // rows [0-2], cols [3-5]
    mat3 M02 = mat3(0.0); // rows [0-2], cols [6-8]
    mat3 M10 = mat3(0.0); // rows [3-5], cols [0-2]
    mat3 M11 = mat3(0.0); // rows [3-5], cols [3-5]
    mat3 M12 = mat3(0.0); // rows [3-5], cols [6-8]
    mat3 M20 = mat3(0.0); // rows [6-8], cols [0-2]
    mat3 M21 = mat3(0.0); // rows [6-8], cols [3-5]
    mat3 M22 = mat3(0.0); // rows [6-8], cols [6-8]
    
    vec3 b0 = vec3(0.0); // b elements [0-2]
    vec3 b1 = vec3(0.0); // b elements [3-5]
    vec3 b2 = vec3(0.0); // b elements [6-8]
    
    // Build M^T*M and M^T*b
    for (int i = 0; i < N; ++i) {
        vec3 p = neighbors[i] - dataCenter; // Center the data
        float w = weights[i];
        
        // Row vector: [x², y², z², xy, xz, yz, x, y, z]
        float x = p.x, y = p.y, z = p.z;
        vec3 r0 = vec3(x*x, y*y, z*z);
        vec3 r1 = vec3(x*y, x*z, y*z);
        vec3 r2 = vec3(x, y, z);
        
        // Accumulate M^T*M (symmetric matrix, so M[i][j] = M[j][i])
        // Block [0-2, 0-2]
        M00[0] += w * r0 * r0.x;
        M00[1] += w * r0 * r0.y;
        M00[2] += w * r0 * r0.z;
        
        // Block [0-2, 3-5]
        M01[0] += w * r0 * r1.x;
        M01[1] += w * r0 * r1.y;
        M01[2] += w * r0 * r1.z;
        
        // Block [0-2, 6-8]
        M02[0] += w * r0 * r2.x;
        M02[1] += w * r0 * r2.y;
        M02[2] += w * r0 * r2.z;
        
        // Block [3-5, 0-2] (symmetric)
        M10[0] += w * r1 * r0.x;
        M10[1] += w * r1 * r0.y;
        M10[2] += w * r1 * r0.z;
        
        // Block [3-5, 3-5]
        M11[0] += w * r1 * r1.x;
        M11[1] += w * r1 * r1.y;
        M11[2] += w * r1 * r1.z;
        
        // Block [3-5, 6-8]
        M12[0] += w * r1 * r2.x;
        M12[1] += w * r1 * r2.y;
        M12[2] += w * r1 * r2.z;
        
        // Block [6-8, 0-2] (symmetric)
        M20[0] += w * r2 * r0.x;
        M20[1] += w * r2 * r0.y;
        M20[2] += w * r2 * r0.z;
        
        // Block [6-8, 3-5] (symmetric)
        M21[0] += w * r2 * r1.x;
        M21[1] += w * r2 * r1.y;
        M21[2] += w * r2 * r1.z;
        
        // Block [6-8, 6-8]
        M22[0] += w * r2 * r2.x;
        M22[1] += w * r2 * r2.y;
        M22[2] += w * r2 * r2.z;
        
        // Accumulate M^T*b (b_i = 1 since J = -1)
        b0 += w * r0;
        b1 += w * r1;
        b2 += w * r2;
    }
    
    // Solve the 9x9 system using block Gaussian elimination
    // We'll reduce it step by step
    
    // Step 1: Eliminate first block column
    if (abs(determinant(M00)) < 1e-10) {
        return false;
    }
    mat3 M00_inv = inverse(M00);
    
    // Update blocks: M11 -= M10 * M00^-1 * M01
    mat3 M11_new = M11 - M10 * M00_inv * M01;
    mat3 M12_new = M12 - M10 * M00_inv * M02;
    vec3 b1_new = b1 - M10 * M00_inv * b0;
    
    // Update blocks: M21 -= M20 * M00^-1 * M01
    mat3 M21_new = M21 - M20 * M00_inv * M01;
    mat3 M22_new = M22 - M20 * M00_inv * M02;
    vec3 b2_new = b2 - M20 * M00_inv * b0;
    
    // Step 2: Eliminate second block column from the reduced system
    if (abs(determinant(M11_new)) < 1e-10) {
        return false;
    }
    mat3 M11_inv = inverse(M11_new);
    
    // Final 3x3 system: M22_final * [G,H,I]^T = b2_final
    mat3 M22_final = M22_new - M21_new * M11_inv * M12_new;
    vec3 b2_final = b2_new - M21_new * M11_inv * b1_new;
    
    if (abs(determinant(M22_final)) < 1e-10) {
        return false;
    }
    
    // Solve for [G,H,I]
    vec3 ghi = inverse(M22_final) * b2_final;
    G = ghi.x;
    H = ghi.y;
    I = ghi.z;
    
    // Back-substitute for [D,E,F]
    vec3 def = M11_inv * (b1_new - M12_new * ghi);
    D = def.x;
    E = def.y;
    F = def.z;
    
    // Back-substitute for [A,B,C]
    vec3 abc = M00_inv * (b0 - M01 * def - M02 * ghi);
    A = abc.x;
    B = abc.y;
    C = abc.z;
    
    return true;
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

// http://www.bmsc.washington.edu/people/merritt/graphics/quadrics.html
// General quadric surface: Ax² + By² + Cz² + Dxy + Exz + Fyz + Gx + Hy + Iz + J = 0
// Returns true if intersection found, outputs t and normal at intersection
bool intersectQuadric(vec3 ro, vec3 rd, 
                      float A, float B, float C, float D, float E, float F,
                      float G, float H, float I, float J,
                      out float t, out vec3 normal) {
    // Ray: R(t) = ro + t*rd
    // Substitute into quadric equation to get: Aq*t² + Bq*t + Cq = 0
    
    float xd = rd.x, yd = rd.y, zd = rd.z;
    float xo = ro.x, yo = ro.y, zo = ro.z;
    
    // Compute coefficients of quadratic equation in t
    float Aq = A*xd*xd + B*yd*yd + C*zd*zd + D*xd*yd + E*xd*zd + F*yd*zd;
    
    float Bq = 2.0*A*xo*xd + 2.0*B*yo*yd + 2.0*C*zo*zd 
             + D*(xo*yd + yo*xd) + E*(xo*zd + zo*xd) + F*(yo*zd + zo*yd)
             + G*xd + H*yd + I*zd;
    
    float Cq = A*xo*xo + B*yo*yo + C*zo*zo 
             + D*xo*yo + E*xo*zo + F*yo*zo 
             + G*xo + H*yo + I*zo + J;
    
    // Check for degenerate case (ray parallel to quadric)
    if (abs(Aq) < 1e-6) {
        // Linear equation: Bq*t + Cq = 0
        if (abs(Bq) < 1e-6) {
            return false; // No intersection
        }
        t = -Cq / Bq;
        if (t <= 0.0) {
            return false;
        }
    } else {
        // Quadratic equation: solve for t
        float discriminant = Bq*Bq - 4.0*Aq*Cq;
        
        if (discriminant < 0.0) {
            return false; // No real intersection
        }
        
        float sqrtDisc = sqrt(discriminant);
        float t0 = (-Bq - sqrtDisc) / (2.0 * Aq);
        float t1 = (-Bq + sqrtDisc) / (2.0 * Aq);
        
        // Choose closest positive t
        if (t0 > 0.0) {
            t = t0;
        } else if (t1 > 0.0) {
            t = t1;
        } else {
            return false; // Both intersections behind ray origin
        }
    }
    
    // Compute intersection point
    vec3 intersection = ro + t * rd;
    float xi = intersection.x;
    float yi = intersection.y;
    float zi = intersection.z;
    
    // Compute normal at intersection using partial derivatives
    // Normal = ∇F = [∂F/∂x, ∂F/∂y, ∂F/∂z]
    normal.x = 2.0*A*xi + D*yi + E*zi + G;
    normal.y = 2.0*B*yi + D*xi + F*zi + H;
    normal.z = 2.0*C*zi + E*xi + F*yi + I;
    
    normal = normalize(normal);
    
    // Make sure normal points toward ray origin
    if (dot(normal, rd) > 0.0) {
        normal = -normal;
    }
    
    return true;
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

        // Check if intersection is in the current voxel (if bounds checking is enabled)
        if (checkBounds) {
            if (isWithinBounds(intersectionPoint, voxel)) {
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

bool traceSphere(ivec3 voxel, vec3 rayOrigin, vec3 rayDir, vec3 tMax, vec3 deltaT) {
    int neighborCount;
    
    // Get neighbors
    if (useSphericalNeighborhood == 1) {
        vec3 samplePoint = findSamplePoint(voxel, rayOrigin, rayDir, tMax, deltaT);
        neighborCount = getNeighborsSpherical(samplePoint, sphericalRadius, g_neighbors);
    } else {
        neighborCount = getNeighborsVoxel(voxel, neighborhoodRingSize, g_neighbors);
    }

    if (areCoplanar(neighborCount)) {
        updatePickingAtMouse(voxel, vec3(voxel) + 0.5);
        FragColor = vec4(1.0, 0.0, 0.0, 1.0); // coplanar
        return true;
    }

    // Initialize weights (ray-centric distance weighting)
    if (useDistanceWeighting) {
        float effectiveRingSize = (useSphericalNeighborhood == 1) ? sphericalRadius : float(neighborhoodRingSize);
        int ringSize = int(effectiveRingSize);
        for (int i = 0; i < neighborCount; i++) {
            g_weights[i] = computeDistanceWeight(g_neighbors[i], rayOrigin, rayDir, ringSize);
        }
    } else {
        for (int i = 0; i < neighborCount; i++) {
            g_weights[i] = 1.0;
        }
    }

    vec4 sphere = fitSphere(g_neighbors, g_weights, neighborCount);
    vec3 center = sphere.xyz;
    float radius = sphere.w;

    if (visualizeRadius && !checkBounds) {
        updatePickingAtMouse(voxel, vec3(voxel) + 0.5);
        float mapped = clamp(radius / 10.0, 0.0, 1.0);
        FragColor = vec4(vec3(mapped), 1.0);
        return true;
    }

    if (radius == -1.0) {
        updatePickingAtMouse(voxel, vec3(voxel) + 0.5);
        FragColor = vec4(1.0, 1.0, 0.0, 1.0); // invalid radius
        return true;
    } else if (radius == -2.0) {
        updatePickingAtMouse(voxel, vec3(voxel) + 0.5);
        FragColor = vec4(0.0, 1.0, 1.0, 1.0); // failed to fit
        return true;
    }

    float t_sphere;
    if (intersectSphere(rayOrigin, rayDir, center, radius, t_sphere)) {
        vec3 intersection = rayOrigin + rayDir * t_sphere;

        vec3 normal = normalize(intersection - sphere.xyz);
        setFragmentDepth(intersection);
                
        if (checkBounds) {
            if (isWithinBounds(intersection, voxel)) {
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

bool traceQuadric(ivec3 voxel, vec3 rayOrigin, vec3 rayDir, vec3 tMax, vec3 deltaT) {
    int neighborCount;
    
    // Get neighbors
    if (useSphericalNeighborhood == 1) {
        vec3 samplePoint = findSamplePoint(voxel, rayOrigin, rayDir, tMax, deltaT);
        neighborCount = getNeighborsSpherical(samplePoint, sphericalRadius, g_neighbors);
    } else {
        neighborCount = getNeighborsVoxel(voxel, neighborhoodRingSize, g_neighbors);
    }

    // Check if we have enough points (need 9 for general quadric)
    if (neighborCount < 9) {
        updatePickingAtMouse(voxel, vec3(voxel) + 0.5);
        FragColor = vec4(1.0, 0.0, 0.0, 1.0); // not enough points
        return true;
    }

    // plane is degenerate quadric, dont check for coplanar

    // Initialize weights (ray-centric distance weighting)
    if (useDistanceWeighting) {
        float effectiveRingSize = (useSphericalNeighborhood == 1) ? sphericalRadius : float(neighborhoodRingSize);
        int ringSize = int(effectiveRingSize);
        for (int i = 0; i < neighborCount; i++) {
            g_weights[i] = computeDistanceWeight(g_neighbors[i], rayOrigin, rayDir, ringSize);
        }
    } else {
        for (int i = 0; i < neighborCount; i++) {
            g_weights[i] = 1.0;
        }
    }

    // Fit general quadric (includes rotation via D, E, F)
    float A, B, C, D, E, F, G, H, I;
    vec3 dataCenter;
    bool success = fitQuadric(g_neighbors, g_weights, neighborCount, A, B, C, D, E, F, G, H, I, dataCenter);

    if (!success) {
        if (usePlaneFallback) {
            // Fitting failed - fall back to plane fitting
            return tracePlaneRayCentric(voxel, rayOrigin, rayDir, tMax, deltaT);
        } else {
            updatePickingAtMouse(voxel, vec3(voxel) + 0.5);
            FragColor = vec4(0.0, 1.0, 1.0, 1.0); // failed to fit
            return true;
        }
    }
    
    // Check if fitted quadric is nearly planar (degenerate case)
    if (usePlaneFallback) {
        // If all curvature terms are very small, it's essentially a plane
        float curvatureSum = abs(A) + abs(B) + abs(C) + abs(D) + abs(E) + abs(F);
        float linearSum = abs(G) + abs(H) + abs(I);
        
        // If curvature is negligible compared to linear terms, use plane fitting instead
        if (curvatureSum < 0.01 * linearSum || curvatureSum < 1e-6) {
            return tracePlaneRayCentric(voxel, rayOrigin, rayDir, tMax, deltaT);
        }
    }

    // Transform the quadric coefficients back to world space
    // The fitted quadric is: A*x'² + B*y'² + C*z'² + D*x'y' + E*x'z' + F*y'z' + G*x' + H*y' + I*z' - 1 = 0
    // where (x',y',z') = (x,y,z) - dataCenter
    // We need to express it in world coordinates
    
    // Let cx = dataCenter.x, cy = dataCenter.y, cz = dataCenter.z
    // x' = x - cx, y' = y - cy, z' = z - cz
    // 
    // Expanding the cross terms:
    // D*(x-cx)*(y-cy) = D*xy - D*cx*y - D*cy*x + D*cx*cy
    // E*(x-cx)*(z-cz) = E*xz - E*cx*z - E*cz*x + E*cx*cz
    // F*(y-cy)*(z-cz) = F*yz - F*cy*z - F*cz*y + F*cy*cz
    //
    // Collecting terms:
    float cx = dataCenter.x, cy = dataCenter.y, cz = dataCenter.z;
    
    float A_world = A;
    float B_world = B;
    float C_world = C;
    float D_world = D;
    float E_world = E;
    float F_world = F;
    float G_world = G - 2.0*A*cx - D*cy - E*cz;
    float H_world = H - 2.0*B*cy - D*cx - F*cz;
    float I_world = I - 2.0*C*cz - E*cx - F*cy;
    float J_world = A*cx*cx + B*cy*cy + C*cz*cz + D*cx*cy + E*cx*cz + F*cy*cz
                    - G*cx - H*cy - I*cz - 1.0;

    // Store fitted quadric coefficients in SSBO for GUI display
    // Always update hovered quadric when hovering
    if (distance(gl_FragCoord.xy, mousePixel) < 1.5) {
        pickingData.hoveredQuadricA = A_world;
        pickingData.hoveredQuadricB = B_world;
        pickingData.hoveredQuadricC = C_world;
        pickingData.hoveredQuadricD = D_world;
        pickingData.hoveredQuadricE = E_world;
        pickingData.hoveredQuadricF = F_world;
        pickingData.hoveredQuadricG = G_world;
        pickingData.hoveredQuadricH = H_world;
        pickingData.hoveredQuadricI = I_world;
        pickingData.hoveredQuadricJ = J_world;
        pickingData.hoveredQuadricValid = 1;
    }
    
    // Copy to clicked quadric when user clicks
    if (shouldUpdateClicked && pickingData.hoveredQuadricValid > 0) {
        pickingData.clickedQuadricA = pickingData.hoveredQuadricA;
        pickingData.clickedQuadricB = pickingData.hoveredQuadricB;
        pickingData.clickedQuadricC = pickingData.hoveredQuadricC;
        pickingData.clickedQuadricD = pickingData.hoveredQuadricD;
        pickingData.clickedQuadricE = pickingData.hoveredQuadricE;
        pickingData.clickedQuadricF = pickingData.hoveredQuadricF;
        pickingData.clickedQuadricG = pickingData.hoveredQuadricG;
        pickingData.clickedQuadricH = pickingData.hoveredQuadricH;
        pickingData.clickedQuadricI = pickingData.hoveredQuadricI;
        pickingData.clickedQuadricJ = pickingData.hoveredQuadricJ;
        pickingData.clickedQuadricValid = 1;
    }

    // Intersect ray with the fitted quadric
    float t_quadric;
    vec3 normal;
    if (intersectQuadric(rayOrigin, rayDir,
                        A_world, B_world, C_world, D_world, E_world, F_world,
                        G_world, H_world, I_world, J_world,
                        t_quadric, normal)) {
        vec3 intersection = rayOrigin + rayDir * t_quadric;
        setFragmentDepth(intersection);
        
        if (checkBounds) {
            if (isWithinBounds(intersection, voxel)) {
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
    
    // === QUADRIC MODE: Render hardcoded quadric (not from voxel data) ===
    if (surfaceType == 2) {
        // Translate ray to quadric's local space
        vec3 localRayOrigin = rayOrigin - quadricCenter;
        
        float t_quadric;
        vec3 quadricNormal;
        
        if (intersectQuadric(localRayOrigin, rayDir,
                            quadricA, quadricB, quadricC, quadricD, quadricE, quadricF,
                            quadricG, quadricH, quadricI, quadricJ,
                            t_quadric, quadricNormal)) {
            vec3 intersection = rayOrigin + rayDir * t_quadric;
            
            // Check if intersection is within grid bounds
            if (intersection.x >= 0.0 && intersection.x <= float(gridSize) &&
                intersection.y >= 0.0 && intersection.y <= float(gridSize) &&
                intersection.z >= 0.0 && intersection.z <= float(gridSize)) {
                renderSurface(quadricNormal, intersection);
                return;
            }
        }
        
        // No intersection or outside bounds - render background
        FragColor = vec4(0.1, 0.1, 0.1, 1.0);
        return;
    }

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
                    if (traceSphere(voxel, rayOrigin, rayDir, tMax, deltaT)) {
                        return;
                    }
                
                } else if (surfaceType == 3) { // === QUADRIC FITTING MODE ===
                    if (traceQuadric(voxel, rayOrigin, rayDir, tMax, deltaT)) {
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
