#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <glm/glm.hpp>
#include <algorithm>
#include <glm/mat3x3.hpp>
#include <glm/gtc/matrix_access.hpp>
#include <iostream>
#include <set>
#include <queue>

struct TangentPlane {
    glm::vec3 center;
    glm::vec3 normal;
};

// connects node u and v with given weight
struct GraphEdge {
    int u;
    int v;
    float weight;

    // sorting
    bool operator<(const GraphEdge& other) const {
        return weight < other.weight;
    }
};

struct RiemannianGraph {
    std::vector<GraphEdge> edges;
    int numNodes;
};

/**
 * @param voxelGrid The flat 1D vector representing the 3D grid data.
 * @param gridSize The dimension of the cubic grid (e.g., 64).
 * @return A vector of glm::vec3 points, where each point is a coordinate.
 */
std::vector<glm::vec3> createPointCloudFromVoxelGrid(
    const std::vector<unsigned char>& voxelGrid,
    int gridSize
);

/**
 * @param pointCloud The entire set of points to search within.
 * @param queryPointIndex The index of the point for which to find neighbors.
 * @param k The number of neighbors to find.
 * @return A vector of integers, where each integer is an index into the pointCloud.
 */
std::vector<int> findKNearestNeighbors(
    const std::vector<glm::vec3>& pointCloud,
    int queryPointIndex,
    int k
);

/**
 * @param pointCloud The entire set of points.
 * @param neighborIndices A list of indices into the pointCloud that specifies the neighborhood.
 * @return The centroid of the specified points as a glm::vec3.
 */
glm::vec3 calculateCentroid(
    const std::vector<glm::vec3>& pointCloud,
    const std::vector<int>& neighborIndices
);

/**
 * @param pointCloud The entire set of points.
 * @param neighborIndices The indices of the points in the neighborhood.
 * @param centroid The pre-calculated centroid of the neighborhood.
 * @return The 3x3 covariance matrix as a glm::mat3.
 */
glm::mat3 calculateCovarianceMatrix(
    const std::vector<glm::vec3>& pointCloud,
    const std::vector<int>& neighborIndices,
    const glm::vec3& centroid
);

TangentPlane calculateTangentPlaneForPoint(
    const std::vector<glm::vec3>& pointCloud,
    int queryPointIndex,
    int k
);

/**
 * @param pointCloud The entire set of points.
 * @param k The number of neighbors to use for each calculation.
 * @return A vector of TangentPlane structs, one for each point in the cloud.
 */
std::vector<TangentPlane> calculateTangentPlanes(
    const std::vector<glm::vec3>& pointCloud,
    int k
);

/**
 * @param planes A vector containing all the pre-calculated tangent planes.
 * @param k The neighborhood size to determine connectivity.
 * @return A RiemannianGraph structure.
 */
RiemannianGraph buildRiemannianGraph(
    const std::vector<TangentPlane>& planes,
    int k
);

/**
 * @param graph The RiemannianGraph containing all nodes and potential edges.
 * @return A vector of GraphEdges that form the MST.
 */
std::vector<GraphEdge> calculateMinimumSpanningTree(const RiemannianGraph& graph);

/**
 * Modifies the 'normal' vectors of the planes in-place.
 * @param planes A vector of all tangent planes. This will be modified.
 * @param mst The pre-calculated Minimum Spanning Tree connecting the planes.
 */
void orientTangentPlanes(
    std::vector<TangentPlane>& planes,
    const std::vector<GraphEdge>& mst
);

void findEigenvectors(const glm::mat3& A, glm::mat3& eigenvectors, glm::vec3& eigenvalues);
