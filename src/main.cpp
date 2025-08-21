#include "config.h"

// --- globals ---
unsigned int VAO, VBO;
const int GRID_SIZE = 64;
std::vector<unsigned char> voxelGrid(GRID_SIZE *GRID_SIZE *GRID_SIZE, 0);
unsigned int voxelTexture;
unsigned int sdfCentersTexture;
unsigned int sdfNormalsTexture;
bool usePlaneFitting = 1;
int k_neighbors = 64;

// --- camera ---
Camera camera(glm::vec3(GRID_SIZE * 1.5f, GRID_SIZE * 1.5f, GRID_SIZE * 1.5f));

// --- timing ---
float deltaTime = 0.0f;
float lastFrame = 0.0f;

// --- functions ---
unsigned int make_shader(const std::string &vertex_filepath, const std::string &fragment_filepath);
unsigned int make_module(const std::string &filepath, unsigned int module_type);
void setupQuad();
void setupVoxelGrid();
void setupVoxelTexture();
void processInput(GLFWwindow *window);
void precomputeSdf();
void setupSdfTextures(const std::vector<float> &centerData, const std::vector<float> &normalData);
void drawGui(float deltaTime);

// --- shape ---
enum ShapeType {
    SHAPE_SPHERE,
    SHAPE_STAIRCASE_1_1,
    SHAPE_STAIRCASE_2_1,
    SHAPE_CUBE
};
ShapeType currentShape = SHAPE_SPHERE; 

int main()
{
    // --- setup ---
    GLFWwindow *window;

    if (!glfwInit())
    {
        std::cout << "GLFW couldn't start" << std::endl;
        return -1;
    }

    window = glfwCreateWindow(640, 480, "", NULL, NULL); // empty window name
    glfwMakeContextCurrent(window);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        glfwTerminate();
        return -1;
    }

    glClearColor(0.25f, 0.5f, 0.75f, 1.0f);

    // --- imgui ---
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    unsigned int shader = make_shader("src/shaders/shader.vert", "src/shaders/shader.frag");

    setupVoxelGrid();
    precomputeSdf();
    setupVoxelTexture();
    setupQuad();

    // --- rendering ---
    while (!glfwWindowShouldClose(window))
    {
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
        glm::mat4 view = camera.GetViewMatrix(); // get view matrix directly from camera

        glm::mat4 invProjection = glm::inverse(projection);
        glm::mat4 invView = glm::inverse(view);

        // Send uniforms to the shader
        glUniformMatrix4fv(glGetUniformLocation(shader, "invProjection"), 1, GL_FALSE, glm::value_ptr(invProjection));
        glUniformMatrix4fv(glGetUniformLocation(shader, "invView"), 1, GL_FALSE, glm::value_ptr(invView));
        glUniform3fv(glGetUniformLocation(shader, "cameraPos"), 1, glm::value_ptr(camera.Position));
        glUniform1i(glGetUniformLocation(shader, "usePlaneFitting"), usePlaneFitting); // 1 on, 0 off

        // bind voxel data texture and draw
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_3D, voxelTexture);
        glUniform1i(glGetUniformLocation(shader, "voxelData"), 0);

        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_3D, sdfCentersTexture);
        glUniform1i(glGetUniformLocation(shader, "sdfCenters"), 1);

        glActiveTexture(GL_TEXTURE2);
        glBindTexture(GL_TEXTURE_3D, sdfNormalsTexture);
        glUniform1i(glGetUniformLocation(shader, "sdfNormals"), 2);

        glBindVertexArray(VAO);
        glDrawArrays(GL_TRIANGLES, 0, 6);

        drawGui(deltaTime);

        glfwSwapBuffers(window);
    }

    // --- cleanup ---
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteProgram(shader);
    glfwTerminate();
    return 0;
}

void processInput(GLFWwindow *window)
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
    float rotationSpeed = 2.5f;
    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
        camera.ProcessMouseMovement(-rotationSpeed, 0);
    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
        camera.ProcessMouseMovement(rotationSpeed, 0);
    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
        camera.ProcessMouseMovement(0, rotationSpeed);
    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
        camera.ProcessMouseMovement(0, -rotationSpeed);
}

void drawGui(float deltaTime)
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    {
        ImGui::Begin("Debug");

        ImGui::Text("FPS: %.1f", 1.0f / deltaTime);
        ImGui::Text("Frame Time: %.3f ms", deltaTime * 1000.0f);

        ImGui::Text("Camera Position: (%.1f, %.1f, %.1f)",
                    camera.Position.x, camera.Position.y, camera.Position.z);
        ImGui::Separator();
        ImGui::Checkbox("Use Plane Fitting", &usePlaneFitting);

        ImGui::Separator();

        // --- Shape Selector ---
        const char* items[] = {
            "Sphere",
            "Staircase (1:1)",
            "Staircase (2:1)",
            "Cube"
        };
        int current_item_index = static_cast<int>(currentShape); 

        if (ImGui::Combo("Shape", &current_item_index, items, IM_ARRAYSIZE(items))) {
            currentShape = static_cast<ShapeType>(current_item_index); 
            std::cout << "Shape changed, regenerating voxel grid..." << std::endl;
            setupVoxelGrid();
            precomputeSdf();
        }

        ImGui::Separator();

        // --- k-Neighborhood Selector ---
        const char* k_items[] = { "1", "2", "4", "8", "16", "32", "64" };
        const int k_values[] = { 1,   2,   4,   8,   16,   32,   64 };

        int current_k_index = -1;
        for (int n = 0; n < IM_ARRAYSIZE(k_values); n++) {
            if (k_values[n] == k_neighbors) {
                current_k_index = n;
                break;
            }
        }

        if (current_k_index == -1) {
            current_k_index = 6;
            k_neighbors = k_values[current_k_index];
        }


        if (ImGui::Combo("k-Neighbors", &current_k_index, k_items, IM_ARRAYSIZE(k_items))) {
            k_neighbors = k_values[current_k_index];

            std::cout << "k value changed to " << k_neighbors << ", re-computing SDF..." << std::endl;
            precomputeSdf();
        }
        ImGui::End();
    }

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

unsigned int make_shader(const std::string &vertex_filepath, const std::string &fragment_filepath)
{
    std::vector<unsigned int> modules;
    modules.push_back(make_module(vertex_filepath, GL_VERTEX_SHADER));
    modules.push_back(make_module(fragment_filepath, GL_FRAGMENT_SHADER));

    unsigned int shader = glCreateProgram();
    for (unsigned int shaderModule : modules)
    {
        glAttachShader(shader, shaderModule);
    }
    glLinkProgram(shader);

    int success;
    glGetProgramiv(shader, GL_LINK_STATUS, &success);
    if (!success)
    {
        char errorLog[1024];
        glGetProgramInfoLog(shader, 1024, NULL, errorLog);
        std::cout << "Shader Linking error:\n"
                  << errorLog << std::endl;
    }

    for (unsigned int shaderModule : modules)
    {
        glDeleteShader(shaderModule);
    }

    return shader;
}

unsigned int make_module(const std::string &filepath, unsigned int module_type)
{
    std::ifstream file;
    std::stringstream bufferedLines;
    std::string line;
    // ../../../src/shaders/shader.frag
    file.open("../../../" + filepath);
    while (std::getline(file, line))
    {
        bufferedLines << line << "\n";
    }
    std::string shaderSource = bufferedLines.str();
    const char *shaderSrc = shaderSource.c_str();
    bufferedLines.str("");
    file.close();

    unsigned int shaderModule = glCreateShader(module_type);
    glShaderSource(shaderModule, 1, &shaderSrc, NULL);
    glCompileShader(shaderModule);

    int success;
    glGetShaderiv(shaderModule, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        char errorLog[1024];
        glGetShaderInfoLog(shaderModule, 1024, NULL, errorLog);
        std::cout << "Shader Module compilation error:\n"
                  << errorLog << std::endl;
    }

    return shaderModule;
}

void setupQuad()
{
    // two tri that cover entire screen in ndc
    float vertices[] = {
        // positions
        -1.0f, 1.0f,
        -1.0f, -1.0f,
        1.0f, -1.0f,

        -1.0f, 1.0f,
        1.0f, -1.0f,
        1.0f, 1.0f};

    // create vao and vbo
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    // Set up the vertex attribute pointer
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);

    // Unbind the VBO and VAO
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void generateSphere() {
    glm::vec3 center(GRID_SIZE / 2.0f);
    float radius = GRID_SIZE / 12.0f;
    float thickness = 1.5f;
    for (int z = 0; z < GRID_SIZE; ++z) for (int y = 0; y < GRID_SIZE; ++y) for (int x = 0; x < GRID_SIZE; ++x) {
        float dist = glm::distance(glm::vec3(x, y, z), center);
        if (abs(dist - radius) < thickness) {
            voxelGrid[x + y * GRID_SIZE + z * GRID_SIZE * GRID_SIZE] = 255;
        }
    }
}

void generateStaircase(int treadWidth, int riserHeight) {
    int start_x = GRID_SIZE / 12;
    int start_y = GRID_SIZE / 12;
    int num_steps = 10;
    int depth = 4; 

    auto placeVoxelSlab = [&](int x, int y) {
        for (int z = start_y; z < start_y + depth; ++z) {
            if (x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE && z >= 0 && z < GRID_SIZE) {
                voxelGrid[x + y * GRID_SIZE + z * GRID_SIZE * GRID_SIZE] = 255;
            }
        }
    };

    for (int i = 0; i < num_steps; ++i) {

        int x_base = start_x + i * treadWidth;
        int y_base = start_y + i * riserHeight;

        for (int t = 0; t < treadWidth; ++t) {
            placeVoxelSlab(x_base + t, y_base);
        }

        for (int r = 0; r < riserHeight; ++r) {
            placeVoxelSlab(x_base + treadWidth, y_base + r);
        }
    }
}

void generateCube() {
    int min_coord = GRID_SIZE / 12;
    int max_coord = GRID_SIZE * 3 / 12;

    for (int x = min_coord; x <= max_coord; ++x) {
        for (int y = min_coord; y <= max_coord; ++y) {
            for (int z = min_coord; z <= max_coord; ++z) {
                if (x == min_coord || x == max_coord ||
                    y == min_coord || y == max_coord ||
                    z == min_coord || z == max_coord) {

                    voxelGrid[x + y * GRID_SIZE + z * GRID_SIZE * GRID_SIZE] = 255;
                }
            }
        }
    }
}

void setupVoxelGrid() {
    std::fill(voxelGrid.begin(), voxelGrid.end(), 0);

    switch (currentShape) {
    case SHAPE_SPHERE:
        generateSphere();
        break;
    case SHAPE_STAIRCASE_1_1:
        generateStaircase(1, 1);
        break;
    case SHAPE_STAIRCASE_2_1:
        generateStaircase(2, 1);
        break;
    case SHAPE_CUBE:
        generateCube();
        break;
    }

    setupVoxelTexture();
}

void setupVoxelTexture()
{
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

void precomputeSdf()
{
    std::cout << "Starting surface reconstruction pre-computation..." << std::endl;
    std::vector<glm::vec3> pointCloud = createPointCloudFromVoxelGrid(voxelGrid, GRID_SIZE);
    std::cout << "Step 1: Point cloud created with " << pointCloud.size() << " points." << std::endl;

    if (!pointCloud.empty())
    {
        int testPointIndex = 0; // Start by assuming the first point is the one
        // find point with largest x coordinate
        for (int i = 1; i < pointCloud.size(); ++i)
        {
            if (pointCloud[i].x > pointCloud[testPointIndex].x)
            {
                testPointIndex = i;
            }
        }
        std::cout << "Found a test point on the far +X surface." << std::endl;
        std::vector<int> neighbors = findKNearestNeighbors(pointCloud, testPointIndex, k_neighbors);

        std::cout << "Step 2: Found " << k_neighbors << " nearest neighbors for point "
                  << testPointIndex << " ("
                  << pointCloud[testPointIndex].x << ", "
                  << pointCloud[testPointIndex].y << ", "
                  << pointCloud[testPointIndex].z << "):" << std::endl;

        for (int neighborIndex : neighbors)
        {
            const auto &point = pointCloud[neighborIndex];
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

        int smallestIdx = 0;
        if (eigenvalues[1] < eigenvalues[smallestIdx])
            smallestIdx = 1;
        if (eigenvalues[2] < eigenvalues[smallestIdx])
            smallestIdx = 2;
        std::cout << "  -> Smallest Eigenvalue is at index " << smallestIdx << ". The normal will be Eigenvector " << smallestIdx << "." << std::endl;

        std::cout << "\n--- Calculating Tangent Plane for Point " << testPointIndex << " ---" << std::endl;
        TangentPlane plane = calculateTangentPlaneForPoint(pointCloud, testPointIndex, k_neighbors);

        std::cout << "Result -> Centroid: (" << plane.center.x << ", " << plane.center.y << ", " << plane.center.z << ")" << std::endl;
        std::cout << "Result -> Normal:   (" << plane.normal.x << ", " << plane.normal.y << ", " << plane.normal.z << ")" << std::endl;
        std::vector<TangentPlane> unorientedPlanes = calculateTangentPlanes(pointCloud, k_neighbors);
        std::cout << "Stage 1 Complete: Calculated " << unorientedPlanes.size() << " unoriented tangent planes." << std::endl;

        std::cout << "\n--- Starting Stage 2: Consistent Tangent Plane Orientation ---" << std::endl;
        RiemannianGraph graph = buildRiemannianGraph(unorientedPlanes, k_neighbors);
        std::cout << "Stage 2 Part 1: Riemannian Graph created." << std::endl;
        std::cout << "  - Nodes: " << graph.numNodes << std::endl;
        std::cout << "  - Edges: " << graph.edges.size() << std::endl;

        std::vector<GraphEdge> mst = calculateMinimumSpanningTree(graph);
        std::cout << "Stage 2 Part 2: Minimum Spanning Tree (MST) calculated." << std::endl;
        std::cout << "  - MST contains " << mst.size() << " edges." << std::endl;

        orientTangentPlanes(unorientedPlanes, mst);
        std::vector<TangentPlane> &orientedPlanes = unorientedPlanes;
        std::cout << "Stage 2 Part 3: Orientation propagation complete." << std::endl;
        testPointIndex = 0;
        for (int i = 1; i < pointCloud.size(); ++i)
        {
            if (pointCloud[i].x > pointCloud[testPointIndex].x)
            {
                testPointIndex = i;
            }
        }
        std::cout << "\n--- Verifying Orientation ---" << std::endl;
        std::cout << "Test point on +X surface (index " << testPointIndex << ")." << std::endl;
        std::cout << "Normal AFTER orientation: ("
                  << orientedPlanes[testPointIndex].normal.x << ", "
                  << orientedPlanes[testPointIndex].normal.y << ", "
                  << orientedPlanes[testPointIndex].normal.z << ")" << std::endl;

        std::cout << "\n--- Starting Stage 3: SDF Data Generation ---" << std::endl;
        std::vector<float> centerData, normalData;
        createSdf(orientedPlanes, pointCloud, GRID_SIZE, centerData, normalData);

        setupSdfTextures(centerData, normalData);

        std::cout << "\n--- All CPU pre-computation is complete! ---" << std::endl;
    }
}

void setupSdfTextures(const std::vector<float> &centerData, const std::vector<float> &normalData)
{
    glGenTextures(1, &sdfCentersTexture);
    glBindTexture(GL_TEXTURE_3D, sdfCentersTexture);

    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    glTexImage3D(GL_TEXTURE_3D, 0, GL_RGB32F, GRID_SIZE, GRID_SIZE, GRID_SIZE, 0,
                 GL_RGB, GL_FLOAT, centerData.data());

    std::cout << "SDF centers texture created and uploaded." << std::endl;

    glGenTextures(1, &sdfNormalsTexture);
    glBindTexture(GL_TEXTURE_3D, sdfNormalsTexture);

    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    glTexImage3D(GL_TEXTURE_3D, 0, GL_RGB32F, GRID_SIZE, GRID_SIZE, GRID_SIZE, 0,
                 GL_RGB, GL_FLOAT, normalData.data());

    std::cout << "SDF normals texture created and uploaded." << std::endl;
}