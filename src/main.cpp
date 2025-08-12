#include "config.h"

// --- globals ---
unsigned int VAO, VBO;
const int GRID_SIZE = 64;
std::vector<unsigned char> voxelGrid(GRID_SIZE* GRID_SIZE* GRID_SIZE, 0);
unsigned int voxelTexture;

// --- camera ---
Camera camera(glm::vec3(GRID_SIZE * 1.5f, GRID_SIZE * 1.5f, GRID_SIZE * 1.5f));

// --- timing ---
float deltaTime = 0.0f;
float lastFrame = 0.0f;

// --- functions ---
unsigned int make_shader(const std::string& vertex_filepath, const std::string& fragment_filepath);
unsigned int make_module(const std::string& filepath, unsigned int module_type);
void setupQuad();
void setupVoxelGrid();
void setupVoxelTexture();
void processInput(GLFWwindow* window);
void precomputeSDF();

int main()
{
    // --- setup ---
    GLFWwindow* window;

    if (!glfwInit()) {
        std::cout << "GLFW couldn't start" << std::endl;
        return -1;
    }

    window = glfwCreateWindow(640, 480, "", NULL, NULL); // empty window name
    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        glfwTerminate();
        return -1;
    }

    glClearColor(0.25f, 0.5f, 0.75f, 1.0f);

    unsigned int shader = make_shader("src/shaders/shader.vert", "src/shaders/shader.frag");

    setupVoxelGrid();
    precomputeSDF();
    setupVoxelTexture();
    setupQuad();

    // --- rendering ---
    while (!glfwWindowShouldClose(window)) {
        float currentFrame = (float)glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        processInput(window);

        glfwPollEvents();
        glClear(GL_COLOR_BUFFER_BIT);
        glUseProgram(shader);

        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        glViewport(0, 0, width, height);

        // camera logic
        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)width / (float)height, 0.1f, 1000.0f);
        glm::mat4 view = camera.GetViewMatrix(); // Get the view matrix directly from the camera object

        glm::mat4 invProjection = glm::inverse(projection);
        glm::mat4 invView = glm::inverse(view);

        // Send uniforms to the shader
        glUniformMatrix4fv(glGetUniformLocation(shader, "invProjection"), 1, GL_FALSE, glm::value_ptr(invProjection));
        glUniformMatrix4fv(glGetUniformLocation(shader, "invView"), 1, GL_FALSE, glm::value_ptr(invView));
        glUniform3fv(glGetUniformLocation(shader, "cameraPos"), 1, glm::value_ptr(camera.Position));
        glUniform1i(glGetUniformLocation(shader, "usePlaneFitting"), 1); // 1 on, 0 off
        glUniform1i(glGetUniformLocation(shader, "neighborhoodSize"), 5); // for a 5x5x5 cube

        // bind voxel data texture and draw
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_3D, voxelTexture);
        glUniform1i(glGetUniformLocation(shader, "voxelData"), 0);
        glBindVertexArray(VAO);
        glDrawArrays(GL_TRIANGLES, 0, 6);
        glfwSwapBuffers(window);
    }

    // --- cleanup ---
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteProgram(shader);
    glfwTerminate();
    return 0;
}

void processInput(GLFWwindow* window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

   glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ? camera.MovementSpeed = SPEED * 2.0f : camera.MovementSpeed = SPEED;

    // Movement
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
        camera.ProcessKeyboard(UP, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
        camera.ProcessKeyboard(DOWN, deltaTime);

    // Rotation
    float rotationSpeed = 2.5f; // constant speed for arrow keys
    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
        camera.ProcessMouseMovement(-rotationSpeed, 0);
    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
        camera.ProcessMouseMovement(rotationSpeed, 0);
    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
        camera.ProcessMouseMovement(0, rotationSpeed);
    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
        camera.ProcessMouseMovement(0, -rotationSpeed);
}

unsigned int make_shader(const std::string& vertex_filepath, const std::string& fragment_filepath) {
    std::vector<unsigned int> modules;
    modules.push_back(make_module(vertex_filepath, GL_VERTEX_SHADER));
    modules.push_back(make_module(fragment_filepath, GL_FRAGMENT_SHADER));

    unsigned int shader = glCreateProgram();
    for (unsigned int shaderModule : modules) {
        glAttachShader(shader, shaderModule);
    }
    glLinkProgram(shader);

    int success;
    glGetProgramiv(shader, GL_LINK_STATUS, &success);
    if (!success) {
        char errorLog[1024];
        glGetProgramInfoLog(shader, 1024, NULL, errorLog);
        std::cout << "Shader Linking error:\n" << errorLog << std::endl;
    }

    for (unsigned int shaderModule : modules) {
        glDeleteShader(shaderModule);
    }

    return shader;
}

unsigned int make_module(const std::string& filepath, unsigned int module_type) {
    std::ifstream file;
    std::stringstream bufferedLines;
    std::string line;
    // ../../../src/shaders/shader.frag
    file.open("../../../" + filepath);
    while (std::getline(file, line)) {
        bufferedLines << line << "\n";
    }
    std::string shaderSource = bufferedLines.str();
    const char* shaderSrc = shaderSource.c_str();
    bufferedLines.str("");
    file.close();

    unsigned int shaderModule = glCreateShader(module_type);
    glShaderSource(shaderModule, 1, &shaderSrc, NULL);
    glCompileShader(shaderModule);

    int success;
    glGetShaderiv(shaderModule, GL_COMPILE_STATUS, &success);
    if (!success) {
        char errorLog[1024];
        glGetShaderInfoLog(shaderModule, 1024, NULL, errorLog);
        std::cout << "Shader Module compilation error:\n" << errorLog << std::endl;
    }

    return shaderModule;
}

void setupQuad() {
    // two tri that cover entire screen in ndc
    float vertices[] = {
        // positions
        -1.0f,  1.0f,
        -1.0f, -1.0f,
         1.0f, -1.0f,

        -1.0f,  1.0f,
         1.0f, -1.0f,
         1.0f,  1.0f
    };

    // create vao and vbo
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    // Bind the VAO first, then bind and set VBO data
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    // Set up the vertex attribute pointer
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0); // Enable layout (location = 0)

    // Unbind the VBO and VAO
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

// create sphere in voxel grid
void setupVoxelGrid() {
    glm::vec3 center(GRID_SIZE / 2.0f);
    float radius = GRID_SIZE / 4.0f;

    for (int z = 0; z < GRID_SIZE; ++z) {
        for (int y = 0; y < GRID_SIZE; ++y) {
            for (int x = 0; x < GRID_SIZE; ++x) {
                float dist = glm::distance(glm::vec3(x, y, z), center);
                if (dist < radius) {
                    // 3d to id array index
                    voxelGrid[x + y * GRID_SIZE + z * GRID_SIZE * GRID_SIZE] = 255; // Use 255 for 'true'
                }
            }
        }
    }
}

void setupVoxelTexture() {
    glGenTextures(1, &voxelTexture);
    glBindTexture(GL_TEXTURE_3D, voxelTexture);

    // Set texture parameters. GL_NEAREST is crucial for blocky look.
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    // Upload the voxel data. We use GL_RED because we only have one channel (on/off).
    glTexImage3D(GL_TEXTURE_3D, 0, GL_RED, GRID_SIZE, GRID_SIZE, GRID_SIZE, 0,
        GL_RED, GL_UNSIGNED_BYTE, voxelGrid.data());
}

void precomputeSDF() {
    std::cout << "Starting surface reconstruction pre-computation..." << std::endl;
    std::vector<glm::vec3> pointCloud = createPointCloudFromVoxelGrid(voxelGrid, GRID_SIZE);
    std::cout << "Step 1: Point cloud created with " << pointCloud.size() << " points." << std::endl;

    if (!pointCloud.empty()) {
        int testPointIndex = 0; // Start by assuming the first point is the one
        // Loop through the entire point cloud to find the point with the largest X-coordinate
        for (int i = 1; i < pointCloud.size(); ++i) {
            if (pointCloud[i].x > pointCloud[testPointIndex].x) {
                testPointIndex = i;
            }
        }
        std::cout << "Found a test point on the far +X surface." << std::endl;
        int k = 15;
        std::vector<int> neighbors = findKNearestNeighbors(pointCloud, testPointIndex, k);

        std::cout << "Step 2: Found " << k << " nearest neighbors for point "
            << testPointIndex << " ("
            << pointCloud[testPointIndex].x << ", "
            << pointCloud[testPointIndex].y << ", "
            << pointCloud[testPointIndex].z << "):" << std::endl;

        for (int neighborIndex : neighbors) {
            const auto& point = pointCloud[neighborIndex];
            std::cout << "  - Point " << neighborIndex << " at ("
                << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
        }

        glm::vec3 centroid = calculateCentroid(pointCloud, neighbors);
        std::cout << "  - Calculated Centroid: ("
            << centroid.x << ", " << centroid.y << ", " << centroid.z << ")" << std::endl;

        glm::mat3 covariance = calculateCovarianceMatrix(pointCloud, neighbors, centroid);
        std::cout << "Step 3: Calculated Covariance Matrix:" << std::endl;
        std::cout << "  [" << covariance[0][0] << ", " << covariance[1][0] << ", " << covariance[2][0] << "]" << std::endl;
        std::cout << "  [" << covariance[0][1] << ", " << covariance[1][1] << ", " << covariance[2][1] << "]" << std::endl;
        std::cout << "  [" << covariance[0][2] << ", " << covariance[1][2] << ", " << covariance[2][2] << "]" << std::endl;

        glm::mat3 eigenvectors;
        glm::vec3 eigenvalues;
        findEigenvectors(covariance, eigenvectors, eigenvalues);

        std::cout << "Step 4: Eigendecomposition Results:" << std::endl;
        std::cout << "  - Eigenvalue 0: " << eigenvalues[0] << ", Eigenvector: (" << eigenvectors[0].x << ", " << eigenvectors[0].y << ", " << eigenvectors[0].z << ")" << std::endl;
        std::cout << "  - Eigenvalue 1: " << eigenvalues[1] << ", Eigenvector: (" << eigenvectors[1].x << ", " << eigenvectors[1].y << ", " << eigenvectors[1].z << ")" << std::endl;
        std::cout << "  - Eigenvalue 2: " << eigenvalues[2] << ", Eigenvector: (" << eigenvectors[2].x << ", " << eigenvectors[2].y << ", " << eigenvectors[2].z << ")" << std::endl;

        // Find the smallest one to predict the normal
        int smallestIdx = 0;
        if (eigenvalues[1] < eigenvalues[smallestIdx]) smallestIdx = 1;
        if (eigenvalues[2] < eigenvalues[smallestIdx]) smallestIdx = 2;
        std::cout << "  -> Smallest Eigenvalue is at index " << smallestIdx << ". The normal will be Eigenvector " << smallestIdx << "." << std::endl;

        // ------------------------------------

        std::cout << "\n--- Calculating Tangent Plane for Point " << testPointIndex << " ---" << std::endl;
        TangentPlane plane = calculateTangentPlaneForPoint(pointCloud, testPointIndex, k);

        std::cout << "Result -> Centroid: (" << plane.center.x << ", " << plane.center.y << ", " << plane.center.z << ")" << std::endl;
        std::cout << "Result -> Normal:   (" << plane.normal.x << ", " << plane.normal.y << ", " << plane.normal.z << ")" << std::endl;
        std::vector<TangentPlane> unorientedPlanes = calculateTangentPlanes(pointCloud, k);
        std::cout << "Stage 1 Complete: Calculated " << unorientedPlanes.size() << " unoriented tangent planes." << std::endl;
    }
}