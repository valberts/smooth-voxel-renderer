#include "SurfaceReconstruction.h"

void findEigenvectors(const glm::mat3& A, glm::mat3& eigenvectors, glm::vec3& eigenvalues);

std::vector<glm::vec3> createPointCloudFromVoxelGrid(
	const std::vector<unsigned char>& voxelGrid,
	int gridSize
) {
	std::vector<glm::vec3> pointCloud;

	for (int z = 0; z < gridSize; z++) {
		for (int y = 0; y < gridSize; y++) {
			for (int x = 0; x < gridSize; x++) {
				int index = x + y * gridSize + z * gridSize * gridSize;
				if (voxelGrid[index] > 0) {
					pointCloud.push_back(glm::vec3(x, y, z) + 0.5f); // fixed +0.5f
				}
			}
		}
	}

	return pointCloud;
}

std::vector<int> findKNearestNeighbors(
	const std::vector<glm::vec3>& pointCloud,
	int queryPointIndex,
	int k
) {
	// holds pairs of {distance, index}
	// distance from query point to every other point
	std::vector<std::pair<float, int>> distances;
	const glm::vec3 queryPoint = pointCloud[queryPointIndex];
	
	for (int i = 0; i < pointCloud.size(); i++) {
		glm::vec3 diff = queryPoint - pointCloud[i];
		float dist2 = glm::dot(diff, diff); // squared distance
		distances.push_back({ dist2, i });
	}
	
	std::sort(distances.begin(), distances.end(), [](auto& left, auto& right) {
		return left.first < right.first;
	});

	std::vector<int> result;
	
	for (int i = 0; i < k; i++) {
		result.push_back(distances[i].second);
	}

	return result;
}

glm::vec3 calculateCentroid(
	const std::vector<glm::vec3>& pointCloud,
	const std::vector<int>& neighborIndices
) {
	if (neighborIndices.size() == 0) return glm::vec3(0.0f);

	glm::vec3 sum = glm::vec3(0.0f);
	for (int i : neighborIndices) {
		sum += pointCloud[i];
	}
	glm::vec3 centroid = sum / (float)neighborIndices.size();
	return centroid;
}

glm::mat3 calculateCovarianceMatrix(
	const std::vector<glm::vec3>& pointCloud,
	const std::vector<int>& neighborIndices,
	const glm::vec3& centroid
) {
	glm::mat3 covariance = glm::mat3(0.0f);

	for (int i : neighborIndices) {
		const glm::vec3& y = pointCloud[i];
		glm::vec3 diff = y - centroid;
		covariance += glm::outerProduct(diff, diff);
	}

	return covariance;
}

TangentPlane calculateTangentPlaneForPoint(
	const std::vector<glm::vec3>& pointCloud,
	int queryPointIndex,
	int k
) {
	std::vector<int> neighbors = findKNearestNeighbors(pointCloud, queryPointIndex, k);

	glm::vec3 centroid = calculateCentroid(pointCloud, neighbors);

	glm::mat3 covariance = calculateCovarianceMatrix(pointCloud, neighbors, centroid);

	glm::mat3 eigenvectors;
	glm::vec3 eigenvalues;
	findEigenvectors(covariance, eigenvectors, eigenvalues);

	int smallestEigenvalueIndex = 0;
	if (eigenvalues[1] < eigenvalues[smallestEigenvalueIndex]) {
		smallestEigenvalueIndex = 1;
	} 
	if (eigenvalues[2] < eigenvalues[smallestEigenvalueIndex]) {
		smallestEigenvalueIndex = 2;
	}

	glm::vec3 normal = eigenvectors[smallestEigenvalueIndex];

	TangentPlane plane;
	plane.center = centroid;
	plane.normal = normal;
	return plane;
}

std::vector<TangentPlane> calculateTangentPlanes(
	const std::vector<glm::vec3>& pointCloud,
	int k
) {
	std::vector<TangentPlane> allPlanes;
	allPlanes.reserve(pointCloud.size());

	for (int i = 0; i < pointCloud.size(); ++i) {
		// debug
		if (i % 1000 == 0) {
			std::cout << "Calculating plane " << i << " / " << pointCloud.size() << std::endl;
		}
		allPlanes.push_back(calculateTangentPlaneForPoint(pointCloud, i, k));
	}

	// debug
	std::cout << "Finished calculating all tangent planes." << std::endl;
	return allPlanes;
}


RiemannianGraph buildRiemannianGraph(
	const std::vector<TangentPlane>& planes,
	int k
) {
	std::vector<glm::vec3> centers;
	for (const auto& plane : planes) {
		centers.push_back(plane.center);
	}

	RiemannianGraph rg;
	rg.numNodes = planes.size();

	std::set<std::pair<int, int>> existingEdges;

	for (int i = 0; i < centers.size(); i++) {
		// debug
		if (i % 1000 == 0) {
			std::cout << "Building graph: processing node " << i << " / " << centers.size() << std::endl;
		}

		std::vector<int> neighbors = findKNearestNeighbors(centers, i, k);

		for (int j : neighbors) {
			if (i == j) {
				continue;
			}

			int u = std::min(i, j);
			int v = std::max(i, j);

			if (existingEdges.count({ u, v }) == 0) {
				float weight = 1.0f - glm::abs(glm::dot(planes[i].normal, planes[j].normal));

				rg.edges.push_back(GraphEdge { u, v, weight });
				existingEdges.insert({ u, v });
			}
		}
	}

	// debug
	std::cout << "Finished building graph with " << rg.edges.size() << " edges." << std::endl;
	return rg;
}

bool pathExists(int startNode, int endNode, int numNodes, const std::vector<GraphEdge>& edges) {
	if (startNode == endNode) return true;

	std::vector<std::vector<int>> adj(numNodes);
	for (const auto& edge : edges) {
		adj[edge.u].push_back(edge.v);
		adj[edge.v].push_back(edge.u);
	}

	std::queue<int> q;
	q.push(startNode);
	std::vector<bool> visited(numNodes, false);
	visited[startNode] = true;

	while (!q.empty()) {
		int u = q.front();
		q.pop();

		if (u == endNode) return true;

		for (int v : adj[u]) {
			if (!visited[v]) {
				visited[v] = true;
				q.push(v);
			}
		}
	}

	return false;
}

std::vector<GraphEdge> calculateMinimumSpanningTree(const RiemannianGraph& graph) {
	std::cout << "Calculating MST..." << std::endl; // debug
	std::vector<GraphEdge> sortedEdges = graph.edges;
	std::sort(sortedEdges.begin(), sortedEdges.end());

	std::vector<GraphEdge> mst;
	int edgesChecked = 0;

	for (const auto& edge : sortedEdges) {
		// debug
		if (++edgesChecked % 1000 == 0) {
			std::cout << "  - MST progress: checking edge " << edgesChecked << " / " << sortedEdges.size()
				<< "  (MST size: " << mst.size() << " / " << graph.numNodes - 1 << ")" << std::endl;
		}

		if (!pathExists(edge.u, edge.v, graph.numNodes, mst)) {
			mst.push_back(edge);
		}

		if (mst.size() == graph.numNodes - 1) {
			break;
		}
	}

	// debug
	std::cout << "  - MST progress: checked edge " << edgesChecked << " / " << sortedEdges.size() 
              << "  (MST size: " << mst.size() << " / " << graph.numNodes - 1 << ")" << std::endl;

	std::cout << "MST calculation complete." << std::endl;
	return mst;
}

void dfs_orient(
	int currentNode,
	int parentNode,
	std::vector<TangentPlane>& planes,
	const std::vector<std::vector<int>>& adj,
	std::vector<bool>& visited
) {
	visited[currentNode] = true;

	if (parentNode != -1) {
		const glm::vec3& parentNormal = planes[parentNode].normal;

		glm::vec3& currentNormal = planes[currentNode].normal;

		if (glm::dot(parentNormal, currentNormal) < 0) {
			currentNormal *= -1.0f;
		}
	}

	for (int neighbor : adj[currentNode]) {
		if (!visited[neighbor]) {
			dfs_orient(neighbor, currentNode, planes, adj, visited);
		}
	}
}

void orientTangentPlanes(
	std::vector<TangentPlane>& planes,
	const std::vector<GraphEdge>& mst
) {
	if (planes.empty()) {
		std::cout << "Error: No tangent planes given." << std::endl; // debug
		return;
	}

	int rootNode = -1;
	float maxZ = -std::numeric_limits<float>::min();

	for (int i = 0; i < planes.size(); ++i) {
		if (planes[i].center.z > maxZ) {
			maxZ = planes[i].center.z;
			rootNode = i;
		}
	}

	if (rootNode == -1) {
		std::cout << "Error: Could not find a root node for orientation." << std::endl; // debug
		return;
	}

	// plane with highest Z, point toward +Z
	if (planes[rootNode].normal.z < 0) {
		planes[rootNode].normal *= -1.0f;
	}

	std::vector<std::vector<int>> adj(planes.size());
	for (const auto& edge : mst) {
		adj[edge.u].push_back(edge.v);
		adj[edge.v].push_back(edge.u);
	}
	std::vector<bool> visited(planes.size(), false);
	dfs_orient(rootNode, -1, planes, adj, visited);

	std::cout << "Orientation propagation complete." << std::endl; // debug
}

void createSdf(
	const std::vector<TangentPlane>& planes,
	const std::vector<glm::vec3>& pointCloud,
	int gridSize,
	std::vector<float>& centerData,
	std::vector<float>& normalData
) {
	int totalVoxels = gridSize * gridSize * gridSize;
	centerData.resize(totalVoxels * 3, 0.0f); 
	normalData.resize(totalVoxels * 3, 0.0f); 

	for (int i = 0; i < planes.size(); ++i) {
		const glm::vec3& p = pointCloud[i];
		int x = static_cast<int>(p.x);
		int y = static_cast<int>(p.y);
		int z = static_cast<int>(p.z);

		int baseIndex = (x + y * gridSize + z * gridSize * gridSize) * 3;

		const TangentPlane& plane = planes[i];

		centerData[baseIndex + 0] = plane.center.x;
		centerData[baseIndex + 1] = plane.center.y;
		centerData[baseIndex + 2] = plane.center.z;

		normalData[baseIndex + 0] = plane.normal.x;
		normalData[baseIndex + 1] = plane.normal.y;
		normalData[baseIndex + 2] = plane.normal.z;
	}

	std::cout << "SDF data arrays created." << std::endl; // debug
}

// --- Eigensolver for 3x3 symmetric matrices ---
// A self-contained function to find the eigenvalues and eigenvectors of a 3x3 symmetric matrix.
// Inputs:
//   A: The 3x3 symmetric matrix.
// Outputs:
//   eigenvectors: A 3x3 matrix where columns are the eigenvectors.
//   eigenvalues: A 3-component vector of the corresponding eigenvalues.
//
// Note: This is a simplified implementation (e.g., of the Jacobi method).
void findEigenvectors(const glm::mat3& A, glm::mat3& eigenvectors, glm::vec3& eigenvalues) {
	// This is a complex numerical algorithm. We will treat it as a given utility.
	// The implementation details are less important than what it does:
	// It diagonalizes the matrix A, finding the vectors that define its principal axes.

	// For a 3x3 symmetric matrix, we can use an analytical solution or an iterative method.
	// The following is a common approach based on Cardano's formula for cubic equations,
	// which is what the characteristic polynomial of a 3x3 matrix expands to.

	const float p = A[0][1] * A[0][1] + A[0][2] * A[0][2] + A[1][2] * A[1][2];
	if (p == 0) {
		// A is already diagonal
		eigenvectors = glm::mat3(1.0f); // Identity matrix
		eigenvalues = glm::vec3(A[0][0], A[1][1], A[2][2]);
		return;
	}

	const float q = (A[0][0] + A[1][1] + A[2][2]) / 3.0f; // trace / 3
	const float b00 = A[0][0] - q, b11 = A[1][1] - q, b22 = A[2][2] - q;
	const float p2 = b00 * b00 + b11 * b11 + b22 * b22 + 2.0f * p;
	const float p_sqrt = sqrt(p2 / 6.0f);

	glm::mat3 B = A;
	B[0][0] -= q; B[1][1] -= q; B[2][2] -= q; // B = A - q*I

	const float det_B = glm::determinant(B);
	const float r = det_B / (2.0f * pow(p_sqrt, 3.0f));

	float phi = 0;
	if (r <= -1.0f) phi = M_PI / 3.0f;
	else if (r >= 1.0f) phi = 0;
	else phi = acos(r) / 3.0f;

	eigenvalues.x = q + 2.0f * p_sqrt * cos(phi);
	eigenvalues.z = q + 2.0f * p_sqrt * cos(phi + (2.0f * M_PI / 3.0f));
	eigenvalues.y = 3.0f * q - eigenvalues.x - eigenvalues.z; // Since trace(A) = sum of eigenvalues

	// Eigenvectors
	auto compute_eigenvector = [&](float eigenvalue) {
		glm::vec3 row0 = A[0]; row0.x -= eigenvalue;
		glm::vec3 row1 = A[1]; row1.y -= eigenvalue;
		glm::vec3 row2 = A[2]; row2.z -= eigenvalue;

		glm::vec3 r0xr1 = glm::cross(row0, row1);
		glm::vec3 r0xr2 = glm::cross(row0, row2);
		glm::vec3 r1xr2 = glm::cross(row1, row2);

		float d0 = glm::dot(r0xr1, r0xr1);
		float d1 = glm::dot(r0xr2, r0xr2);
		float d2 = glm::dot(r1xr2, r1xr2);

		float dmax = d0; int imax = 0;
		if (d1 > dmax) { dmax = d1; imax = 1; }
		if (d2 > dmax) { imax = 2; }

		if (imax == 0) return glm::normalize(r0xr1);
		if (imax == 1) return glm::normalize(r0xr2);
		return glm::normalize(r1xr2);
	};

	eigenvectors[0] = compute_eigenvector(eigenvalues.x);
	eigenvectors[1] = compute_eigenvector(eigenvalues.y);
	eigenvectors[2] = glm::cross(eigenvectors[0], eigenvectors[1]); // Ensure orthogonality for the third vector
}