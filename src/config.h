#pragma once
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "Camera.h"

#include "SurfaceReconstruction.h"

// --- Hoppe et al. Surface Reconstruction ---
// CPU pre-computation pass to generate a Signed Distance Function (SDF) for GPU ray-marching.
// Pipeline: Voxel Grid -> Point Cloud -> Tangent Planes -> Oriented Planes -> SDF Textures.

// == STAGE 1: Tangent Plane Estimation ==
// For each point p_i in the point cloud:
// 1. Find k-nearest neighbors (Nbhd(p_i)).
// 2. Compute centroid of Nbhd(p_i). This is the plane's center o_i.
// 3. Compute 3x3 covariance matrix: CV = sum((p - o_i) * transpose(p - o_i)) for p in Nbhd(p_i).
// 4. Perform PCA on CV. The plane normal n_i is the eigenvector of the smallest eigenvalue.
//    Result: unoriented tangent plane (o_i, n_i).

// == STAGE 2: Consistent Tangent Plane Orientation ==
// Unify normal directions across all planes.
// 1. Construct a Riemannian Graph:
//    - Nodes: One per tangent plane.
//    - Edges: Connect planes i and j if o_i is in k-neighborhood of o_j or vice-versa.
// 2. Assign edge weights: weight(i, j) = 1.0 - abs(dot(n_i, n_j)). (Measures curvature).
// 3. Compute the Minimum Spanning Tree (MST) of the weighted graph.
// 4. Initialize orientation:
//    - Find plane with max Z center (o_i.z). This is the MST root.
//    - Orient its normal n_i to point along +Z.
// 5. Propagate orientation:
//    - Traverse the MST from the root (e.g., depth-first).
//    - For each edge (parent, child), if dot(n_parent, n_child) < 0, flip the child normal: n_child *= -1.0.

// == STAGE 3: Signed Distance Function (SDF) Generation ==
// Create lookup data for a GPU-side SDF.
// 1. For each solid voxel in the original grid, store its final oriented plane (o_i, n_i).
// 2. Store this data in two 3D textures (GL_RGB32F): tangentCentersTexture and tangentNormalsTexture.
// 3. The GPU shader calculates the SDF value at a point p by:
//    a. Finding the corresponding voxel for p.
//    b. Fetching the plane data (o_i, n_i) from the textures.
//    c. Calculating distance: dist = dot(p - o_i, n_i).