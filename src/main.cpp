#include "config.h"
#include "picking.h"

// --- globals ---
unsigned int VAO, VBO;
unsigned int wireframeVAO, wireframeVBO;
int wireframeVertexCount = 0;
int GRID_SIZE = 64;
std::vector<unsigned char> voxelGrid(GRID_SIZE * GRID_SIZE * GRID_SIZE, 0);
unsigned int voxelTexture;
unsigned int sdfCentersTexture;
unsigned int sdfNormalsTexture;

bool useFitting = 1;
bool checkBounds = 1;
int k_neighbors = 64;
int neighborhood_ring_size = 1;
float sphereRadius = 1.0f;
float sphereThickness = 1.0f;

// debug wireframe
bool showDebugWireframe = false;
bool wireframeUseFading = true;
float wireframeFadeStart = 5.0f;
float wireframeFadeEnd = 20.0f;
glm::vec3 wireframeColor(1.0f, 1.0f, 1.0f);

// Phong lighting variables
bool usePhongLighting = false;

// Distance weighting variables
bool useDistanceWeighting = false;
float distanceWeightMultiplier = 3.0f;
int falloffMode = 0; // 0 = linear, 1 = gaussian

// Surface type
int surfaceType = 0; // 0 = plane, 1 = sphere
bool visualizeRadius = false;

// Voxel-centric weighting variables
bool useVoxelCentricWeighting = false;
float voxelCentricStepSize = 0.05f;
float voxelCentricEpsilon = 0.05f;
int voxelCentricMaxIterations = 16;

// spherical neighborhood
bool useSphericalNeighborhood = false;
float sphericalRadius = 1.5f;
bool useVoxelCenterForSphere = true;

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
void setupTestCase();
int coordsToIndex(int x, int y, int z);
void setupWireframeGeometry();
void updateWireframeGeometry();

// --- shape ---
enum ShapeType
{
    SHAPE_SPHERE,
    SHAPE_SMALLSPHERE,
    SHAPE_STAIRCASE_1_1,
    SHAPE_STAIRCASE_2_1,
    SHAPE_CUBE,
    SHAPE_CONCAVE
};
ShapeType currentShape = SHAPE_SPHERE;

// --- test mode ---
bool testMode = false;
enum TestCase
{
    TEST_SINGLE_CENTER, // Single voxel at center (1,1,1)
    TEST_LINE_X,        // Line along X axis (3 voxels)
    TEST_PLANE_XZ,
    TEST_STAIRS,
    TEST_STAIRS2,
    TEST_STAIRS3,
    TEST_STAIRS4,
    TEST_CORNER,
    TEST_CORNER2,
    TEST_CUBE
};
int currentTestCase = TestCase::TEST_SINGLE_CENTER;

// Forward declarations
void setupVoxelGrid();
void setupVoxelTexture();

// Keyboard callback for voxel deletion
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_X && action == GLFW_PRESS) {
        glm::ivec3 hoveredVoxel = getHoveredVoxel();
        if (hoveredVoxel.x >= 0 && hoveredVoxel.x < GRID_SIZE &&
            hoveredVoxel.y >= 0 && hoveredVoxel.y < GRID_SIZE &&
            hoveredVoxel.z >= 0 && hoveredVoxel.z < GRID_SIZE) {
            
            // Delete the voxel
            int idx = hoveredVoxel.x + hoveredVoxel.y * GRID_SIZE + hoveredVoxel.z * GRID_SIZE * GRID_SIZE;
            voxelGrid[idx] = 0;
            
            // Update the GPU texture (don't regenerate the whole grid)
            setupVoxelTexture();
            
            std::cout << "Deleted voxel at (" << hoveredVoxel.x << ", " 
                      << hoveredVoxel.y << ", " << hoveredVoxel.z << ")" << std::endl;
        }
    }
}

int main()
{
    // --- setup ---
    GLFWwindow *window;

    if (!glfwInit())
    {
        std::cout << "GLFW couldn't start" << std::endl;
        return -1;
    }

#ifdef __APPLE__
    // Set OpenGL version and profile for macOS compatibility
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // Required on macOS
#endif

    window = glfwCreateWindow(640, 480, "", NULL, NULL); // empty window name
    glfwMakeContextCurrent(window);

    // Register input callbacks
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetKeyCallback(window, key_callback);

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
    unsigned int wireframeShader = make_shader("src/shaders/wireframe.vert", "src/shaders/wireframe.frag");

    setupVoxelGrid();
    // precomputeSdf();
    setupVoxelTexture();
    setupQuad();
    setupWireframeGeometry();

    // Setup picking SSBO
    setupPickingSSBO();

    // --- rendering ---
    while (!glfwWindowShouldClose(window))
    {
        float currentFrame = (float)glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        processInput(window);

        glfwPollEvents();
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
        glUseProgram(shader);

        int width, height;
        glfwGetFramebufferSize(window, &width, &height);

        // camera logic
        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)width / (float)height, 0.1f, 1000.0f);
        glm::mat4 view = camera.GetViewMatrix(); // get view matrix directly from camera

        glm::mat4 invProjection = glm::inverse(projection);
        glm::mat4 invView = glm::inverse(view);

        // Render to screen
        glViewport(0, 0, width, height);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
        glUseProgram(shader);
        glUniformMatrix4fv(glGetUniformLocation(shader, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
        glUniformMatrix4fv(glGetUniformLocation(shader, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(shader, "invProjection"), 1, GL_FALSE, glm::value_ptr(invProjection));
        glUniformMatrix4fv(glGetUniformLocation(shader, "invView"), 1, GL_FALSE, glm::value_ptr(invView));
        glUniform3fv(glGetUniformLocation(shader, "cameraPos"), 1, glm::value_ptr(camera.Position));
        glUniform1i(glGetUniformLocation(shader, "useFitting"), useFitting);
        glUniform1i(glGetUniformLocation(shader, "checkBounds"), checkBounds);
        glUniform1i(glGetUniformLocation(shader, "neighborhoodRingSize"), neighborhood_ring_size);
        glUniform1i(glGetUniformLocation(shader, "gridSize"), GRID_SIZE);
        glUniform1i(glGetUniformLocation(shader, "usePhongLighting"), usePhongLighting);
        glUniform1i(glGetUniformLocation(shader, "useDistanceWeighting"), useDistanceWeighting);
        glUniform1f(glGetUniformLocation(shader, "distanceWeightMultiplier"), distanceWeightMultiplier);
        glUniform1i(glGetUniformLocation(shader, "falloffMode"), falloffMode);
        glUniform1i(glGetUniformLocation(shader, "surfaceType"), surfaceType);
        glUniform1i(glGetUniformLocation(shader, "useVoxelCentricWeighting"), useVoxelCentricWeighting);
        glUniform1f(glGetUniformLocation(shader, "voxelCentricStepSize"), voxelCentricStepSize);
        glUniform1f(glGetUniformLocation(shader, "voxelCentricEpsilon"), voxelCentricEpsilon);
        glUniform1i(glGetUniformLocation(shader, "voxelCentricMaxIterations"), voxelCentricMaxIterations);
        glUniform1i(glGetUniformLocation(shader, "useSphericalNeighborhood"), useSphericalNeighborhood);
        glUniform1f(glGetUniformLocation(shader, "sphericalRadius"), sphericalRadius);
        glUniform1i(glGetUniformLocation(shader, "useVoxelCenterForSphere"), useVoxelCenterForSphere);
        glUniform1i(glGetUniformLocation(shader, "visualizeRadius"), visualizeRadius);

        // Mouse picking uniforms
        glUniform2f(glGetUniformLocation(shader, "mousePixel"), (float)lastMouseX, (float)height - (float)lastMouseY);
        glUniform1i(glGetUniformLocation(shader, "shouldUpdateClicked"), hasClickedVoxel);

        // Bind picking SSBO
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, pickingSSBO);

        // bind voxel data texture and draw
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_3D, voxelTexture);
        glUniform1i(glGetUniformLocation(shader, "voxelData"), 0);

        glBindVertexArray(VAO);
        glDrawArrays(GL_TRIANGLES, 0, 6);

        // Ensure SSBO writes are complete before reading back
        glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

        // Reset clicked flag after frame
        if (hasClickedVoxel)
        {
            hasClickedVoxel = false;
        }

        // render wireframe overlay
        if (showDebugWireframe && wireframeVertexCount > 0)
        {
            glUseProgram(wireframeShader);

            // disable depth testing
            glDisable(GL_DEPTH_TEST);

            // alpha blending for transparency
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

            glUniformMatrix4fv(glGetUniformLocation(wireframeShader, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
            glUniformMatrix4fv(glGetUniformLocation(wireframeShader, "view"), 1, GL_FALSE, glm::value_ptr(view));
            glUniform3fv(glGetUniformLocation(wireframeShader, "cameraPos"), 1, glm::value_ptr(camera.Position));

            glUniform3fv(glGetUniformLocation(wireframeShader, "wireframeColor"), 1, glm::value_ptr(wireframeColor));
            glUniform1i(glGetUniformLocation(wireframeShader, "useFading"), wireframeUseFading);
            glUniform1f(glGetUniformLocation(wireframeShader, "fadeStart"), wireframeFadeStart);
            glUniform1f(glGetUniformLocation(wireframeShader, "fadeEnd"), wireframeFadeEnd);

            glBindVertexArray(wireframeVAO);
            glDrawArrays(GL_LINES, 0, wireframeVertexCount);

            glDisable(GL_BLEND);
        }

        glDisable(GL_DEPTH_TEST);

        drawGui(deltaTime);

        glfwSwapBuffers(window);
    }

    // --- cleanup ---
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteVertexArrays(1, &wireframeVAO);
    glDeleteBuffers(1, &wireframeVBO);
    glDeleteProgram(shader);
    glDeleteProgram(wireframeShader);
    cleanupPickingSSBO();
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
        ImGui::Checkbox("Use Fitting", &useFitting);
        if (useFitting)
        {
            const char *surfaceItems[] = {"Plane", "Sphere"};
            ImGui::Combo("Surface Type", &surfaceType, surfaceItems, IM_ARRAYSIZE(surfaceItems));

            if (surfaceType == 1)
            {
                ImGui::Checkbox("Show Sphere Radius", &visualizeRadius);
            }

            ImGui::Checkbox("Use Ray-Centric Weighting", &useDistanceWeighting);
            if (useDistanceWeighting)
            {
                ImGui::InputFloat("Distance Weight Multiplier", &distanceWeightMultiplier, 0.1f, 1.0f, "%.2f");
            }

            ImGui::Checkbox("Use Voxel-Centric Weighting", &useVoxelCentricWeighting);
            if (useVoxelCentricWeighting)
            {
                ImGui::InputFloat("Step Size", &voxelCentricStepSize, 0.01f, 0.1f, "%.3f");
                ImGui::InputFloat("Epsilon", &voxelCentricEpsilon, 0.05f, 0.5f, "%.2f");
                ImGui::InputInt("Max Iterations", &voxelCentricMaxIterations, 1, 10);
            }

            if (useDistanceWeighting || useVoxelCentricWeighting)
            {
                const char *falloffItems[] = {"Linear", "Gaussian"};
                ImGui::Combo("Falloff Mode", &falloffMode, falloffItems, IM_ARRAYSIZE(falloffItems));
            }
        }
        ImGui::Checkbox("Check Bounds", &checkBounds);
        ImGui::Checkbox("Use Phong Lighting", &usePhongLighting);

        ImGui::Separator();

        // --- Shape Selector (only show in normal mode) ---
        if (!testMode)
        {
            const char *items[] = {
                "Sphere",
                "Small Sphere",
                "Staircase (1:1)",
                "Staircase (2:1)",
                "Cube",
                "Concave"};
            int current_item_index = static_cast<int>(currentShape);

            if (ImGui::Combo("Shape", &current_item_index, items, IM_ARRAYSIZE(items)))
            {
                currentShape = static_cast<ShapeType>(current_item_index);
                std::cout << "Shape changed, regenerating voxel grid..." << std::endl;
                setupVoxelGrid();
            }
            if (currentShape == ShapeType::SHAPE_SMALLSPHERE)
            {
                bool radiusChanged = ImGui::SliderFloat("Sphere Radius", &sphereRadius, 1.0f, 10.0f);
                bool thicknessChanged = ImGui::SliderFloat("Sphere Thickness", &sphereThickness, 1.0f, 10.0f);
                if (radiusChanged || thicknessChanged)
                {
                    setupVoxelGrid();
                }
            }
        }

        // --- Test Case Selector (only show in test mode) ---
        if (testMode)
        {
            const char *testCaseItems[] = {
                "Single Center",
                "Line",
                "Plane",
                "Stairs",
                "Stairs2",
                "Stairs3",
                "Stairs4",
                "Corner",
                "Corner2",
                "Cube"};
            int current_test_index = static_cast<int>(currentTestCase);

            if (ImGui::Combo("Test Case", &current_test_index, testCaseItems, IM_ARRAYSIZE(testCaseItems)))
            {
                currentTestCase = static_cast<TestCase>(current_test_index);
                std::cout << "Test case changed, regenerating voxel grid..." << std::endl;
                setupVoxelGrid();
            }

            // Show current grid size for reference
            ImGui::Text("Grid Size: %dx%dx%d", GRID_SIZE, GRID_SIZE, GRID_SIZE);
        }
        else
        {
            // Show current grid size for normal mode too
            ImGui::Text("Grid Size: %dx%dx%d", GRID_SIZE, GRID_SIZE, GRID_SIZE);
        }

        // --- Test Mode Toggle ---
        if (ImGui::Checkbox("Test Mode (3x3x3)", &testMode))
        {
            std::cout << "Test mode " << (testMode ? "enabled" : "disabled") << ", regenerating voxel grid..." << std::endl;
            setupVoxelGrid();
        }

        ImGui::Separator();

        ImGui::Checkbox("Use Spherical Neighborhood", &useSphericalNeighborhood);
        if (useSphericalNeighborhood)
        {
            ImGui::SliderFloat("Radius", &sphericalRadius, 1.5f, 3.5f);
            ImGui::Checkbox("Use Voxel Center Distance", &useVoxelCenterForSphere);
        }
        else
        {
            ImGui::SliderInt("Ring Size", &neighborhood_ring_size, 1, 3);
        }

        ImGui::Separator();
        ImGui::Checkbox("Show Wireframe", &showDebugWireframe);

        ImGui::Separator();
        // Read picking data from SSBO for display
        PickingData pickingInfo = readPickingData();
        if (pickingInfo.hoveredVoxel.w > 0)
        {
            ImGui::Text("Hovered: (%d, %d, %d) [count: %d]",
                        pickingInfo.hoveredVoxel.x, pickingInfo.hoveredVoxel.y, pickingInfo.hoveredVoxel.z,
                        pickingInfo.hoveredNeighborCount);
        }
        else
        {
            ImGui::Text("Hovered: None (w=%d)", pickingInfo.hoveredVoxel.w);
        }

        if (pickingInfo.clickedVoxel.w > 0)
        {
            ImGui::Text("Clicked neighbors: %d", pickingInfo.clickedNeighborCount);
            if (useSphericalNeighborhood)
            {
                ImGui::Text("Mode: Spherical (r=%.1f)", sphericalRadius);
            }
            else
            {
                ImGui::Text("Mode: Cubic (ring=%d)", neighborhood_ring_size);
            }
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
    std::string fullPath;

    // ../../../src/shaders/shader.frag
#ifdef __APPLE__
    // On macOS, use relative path from build directory
    fullPath = "../" + filepath;
#else
    // On other platforms (Windows, Linux), use the original path
    fullPath = "../../../" + filepath;
#endif

    file.open(fullPath);
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

void generateSphere()
{
    glm::vec3 center(GRID_SIZE / 2.0f);
    float radius = GRID_SIZE / 12.0f;
    float thickness = 1.5f;
    for (int z = 0; z < GRID_SIZE; ++z)
        for (int y = 0; y < GRID_SIZE; ++y)
            for (int x = 0; x < GRID_SIZE; ++x)
            {
                float dist = glm::distance(glm::vec3(x, y, z), center);
                if (abs(dist - radius) < thickness)
                {
                    voxelGrid[x + y * GRID_SIZE + z * GRID_SIZE * GRID_SIZE] = 255;
                }
            }
}

void generateSmallSphere()
{
    glm::vec3 center(GRID_SIZE / 2.0);
    // grid size is 64x64x64
    float outerRadius = sphereRadius;
    float shellThickness = sphereThickness;
    float innerRadius = outerRadius - shellThickness;

    for (int z = 0; z < GRID_SIZE; ++z)
        for (int y = 0; y < GRID_SIZE; ++y)
            for (int x = 0; x < GRID_SIZE; ++x)
            {
                float dist = glm::distance(glm::vec3(x, y, z), center);
                int idx = x + y * GRID_SIZE + z * GRID_SIZE * GRID_SIZE;
                if (dist <= outerRadius && dist > innerRadius)
                {
                    voxelGrid[idx] = 255; // shell
                }
                else
                {
                    voxelGrid[idx] = 0; // empty (inside or outside)
                }
            }
}

void generateStaircase(int treadWidth, int riserHeight)
{
    int start_x = GRID_SIZE / 12;
    int start_y = GRID_SIZE / 12;
    int num_steps = 10;
    int depth = 4;

    auto placeVoxelSlab = [&](int x, int y)
    {
        for (int z = start_y; z < start_y + depth; ++z)
        {
            if (x >= 0 && x < GRID_SIZE && y >= 0 && y < GRID_SIZE && z >= 0 && z < GRID_SIZE)
            {
                voxelGrid[x + y * GRID_SIZE + z * GRID_SIZE * GRID_SIZE] = 255;
            }
        }
    };

    for (int i = 0; i < num_steps; ++i)
    {

        int x_base = start_x + i * treadWidth;
        int y_base = start_y + i * riserHeight;

        for (int t = 0; t < treadWidth; ++t)
        {
            placeVoxelSlab(x_base + t, y_base);
        }

        for (int r = 0; r < riserHeight; ++r)
        {
            placeVoxelSlab(x_base + treadWidth, y_base + r);
        }
    }
}

void generateCube(int thickness = 1)
{
    int min_coord = GRID_SIZE / 12;
    int max_coord = GRID_SIZE * 3 / 12;

    for (int x = min_coord; x <= max_coord; ++x)
    {
        for (int y = min_coord; y <= max_coord; ++y)
        {
            for (int z = min_coord; z <= max_coord; ++z)
            {
                // Calculate distance to nearest face
                int dist_to_face = std::min({
                    x - min_coord, // distance to left face
                    max_coord - x, // distance to right face
                    y - min_coord, // distance to bottom face
                    max_coord - y, // distance to top face
                    z - min_coord, // distance to front face
                    max_coord - z  // distance to back face
                });

                if (dist_to_face < thickness)
                {
                    voxelGrid[x + y * GRID_SIZE + z * GRID_SIZE * GRID_SIZE] = 255;
                }
            }
        }
    }
}

void generateConcave()
{
    int min_coord = GRID_SIZE / 12;
    int max_coord = GRID_SIZE * 3 / 12;

    for (int x = min_coord; x <= max_coord; ++x)
        for (int y = min_coord; y <= max_coord; ++y)
            for (int z = min_coord; z <= max_coord; ++z)
            {
                // 1 voxel thick shell
                bool onShell = (x == min_coord || x == max_coord ||
                                y == min_coord || y == max_coord ||
                                z == min_coord);

                // Skip one face, e.g. the +Z face
                bool skipFace = (z == max_coord);

                if (onShell && !skipFace)
                    voxelGrid[x + y * GRID_SIZE + z * GRID_SIZE * GRID_SIZE] = 255;
            }
}

void setupVoxelGrid()
{
    // Set grid size based on test mode
    if (testMode)
    {
        GRID_SIZE = 3;
        voxelGrid.resize(3 * 3 * 3, 0);
    }
    else
    {
        GRID_SIZE = 64;
        voxelGrid.resize(64 * 64 * 64, 0);
    }

    // should we update camera position for new grid size?

    std::fill(voxelGrid.begin(), voxelGrid.end(), 0);

    if (testMode)
    {
        // generate a test case
        setupTestCase();
    }
    else
    {
        switch (currentShape)
        {
        case SHAPE_SPHERE:
            generateSphere();
            break;
        case SHAPE_SMALLSPHERE:
            generateSmallSphere();
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
        case SHAPE_CONCAVE:
            generateConcave();
            break;
        }
    }

    setupVoxelTexture();

    if (wireframeVAO != 0)
    {
        updateWireframeGeometry();
    }
}

void setupVoxelTexture()
{
    if (voxelTexture != 0)
    {
        glDeleteTextures(1, &voxelTexture);
        voxelTexture = 0;
    }
    glGenTextures(1, &voxelTexture);
    glBindTexture(GL_TEXTURE_3D, voxelTexture);

    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    // Upload the voxel data. We use GL_RED because we only have one channel (on/off).
    glTexImage3D(GL_TEXTURE_3D, 0, GL_RED, GRID_SIZE, GRID_SIZE, GRID_SIZE, 0,
                 GL_RED, GL_UNSIGNED_BYTE, voxelGrid.data());
}

// Convert 3D coordinates to 1D index in the voxel grid
int coordsToIndex(int x, int y, int z)
{
    if (x < 0 || x >= GRID_SIZE || y < 0 || y >= GRID_SIZE || z < 0 || z >= GRID_SIZE)
    {
        return -1;
    }
    return x + y * GRID_SIZE + z * GRID_SIZE * GRID_SIZE;
}

void placeVoxel(int x, int y, int z)
{
    // assumes GRID_SIZE = 3
    if (x < 0 || x >= 3 || y < 0 || y >= 3 || z < 0 || z >= 3)
    {
        return;
    }
    voxelGrid[x + y * 3 + z * 3 * 3] = 255;
}

void setupTestCase()
{
    std::fill(voxelGrid.begin(), voxelGrid.end(), 0);

    switch (currentTestCase)
    {
    case TestCase::TEST_SINGLE_CENTER:
    {
        placeVoxel(1, 1, 1);
        break;
    }
    case TestCase::TEST_LINE_X:
    {
        placeVoxel(0, 1, 1);
        placeVoxel(1, 1, 1);
        placeVoxel(2, 1, 1);
        break;
    }
    case TestCase::TEST_PLANE_XZ:
    {
        for (int x = 0; x < 3; ++x)
        {
            for (int z = 0; z < 3; ++z)
            {
                placeVoxel(x, 1, z);
            }
        }
        break;
    }
    case TestCase::TEST_STAIRS:
    {
        placeVoxel(0, 0, 0);
        placeVoxel(1, 0, 0);
        placeVoxel(1, 1, 0);
        placeVoxel(2, 1, 0);
        placeVoxel(2, 2, 0);
        break;
    }
    case TestCase::TEST_STAIRS2:
    {
        placeVoxel(0, 0, 0);
        placeVoxel(1, 0, 0);
        placeVoxel(1, 1, 0);
        placeVoxel(2, 1, 0);
        placeVoxel(2, 2, 0);

        placeVoxel(0, 0, 1);
        placeVoxel(1, 0, 1);
        placeVoxel(1, 1, 1);
        placeVoxel(2, 1, 1);
        placeVoxel(2, 2, 1);
        break;
    }
    case TestCase::TEST_STAIRS3:
    {
        placeVoxel(0, 0, 0);
        placeVoxel(1, 0, 0);
        placeVoxel(1, 1, 0);
        placeVoxel(2, 1, 0);
        placeVoxel(2, 2, 0);

        placeVoxel(0, 0, 1);
        placeVoxel(1, 0, 1);
        placeVoxel(1, 1, 1);
        placeVoxel(2, 1, 1);
        placeVoxel(2, 2, 1);

        placeVoxel(0, 0, 2);
        placeVoxel(1, 0, 2);
        placeVoxel(1, 1, 2);
        placeVoxel(2, 1, 2);
        placeVoxel(2, 2, 2);
        break;
    }

    case TestCase::TEST_STAIRS4:
    {
        placeVoxel(0, 0, 0);
        placeVoxel(1, 1, 0);
        placeVoxel(2, 2, 0);

        placeVoxel(0, 0, 1);
        placeVoxel(1, 1, 1);
        placeVoxel(2, 2, 1);

        placeVoxel(0, 0, 2);
        placeVoxel(1, 1, 2);
        placeVoxel(2, 2, 2);
        break;
    }
    case TestCase::TEST_CORNER:
    {
        placeVoxel(0, 0, 0);
        placeVoxel(0, 0, 1);
        placeVoxel(0, 0, 2);
        placeVoxel(0, 1, 2);
        placeVoxel(0, 2, 2);
        break;
    }
    case TestCase::TEST_CORNER2:
    {
        placeVoxel(0, 0, 0);
        placeVoxel(0, 0, 1);
        placeVoxel(0, 0, 2);

        placeVoxel(0, 1, 2);
        placeVoxel(0, 2, 2);

        placeVoxel(1, 0, 2);
        placeVoxel(2, 0, 2);

        break;
    }
    case TestCase::TEST_CUBE:
    {
        for (int z = 0; z < GRID_SIZE; z++)
        {
            for (int y = 0; y < GRID_SIZE; y++)
            {
                for (int x = 0; x < GRID_SIZE; x++)
                {
                    placeVoxel(x, y, z);
                }
            }
        }
        break;
    }
    }
}

void setupWireframeGeometry()
{
    // Initialize the VAO and VBO for wireframes
    glGenVertexArrays(1, &wireframeVAO);
    glGenBuffers(1, &wireframeVBO);

    glBindVertexArray(wireframeVAO);
    glBindBuffer(GL_ARRAY_BUFFER, wireframeVBO);

    // Set up the vertex attribute pointer (position only)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    // Generate initial wireframe geometry
    updateWireframeGeometry();
}

void updateWireframeGeometry()
{
    std::vector<float> wireframeVertices;

    // For each voxel that is solid, add the edges of its cube
    for (int z = 0; z < GRID_SIZE; ++z)
    {
        for (int y = 0; y < GRID_SIZE; ++y)
        {
            for (int x = 0; x < GRID_SIZE; ++x)
            {
                int idx = coordsToIndex(x, y, z);
                if (voxelGrid[idx] > 0)
                {
                    // Define the 8 corners of the voxel cube
                    float x0 = (float)x, x1 = (float)(x + 1);
                    float y0 = (float)y, y1 = (float)(y + 1);
                    float z0 = (float)z, z1 = (float)(z + 1);

                    // 12 edges of a cube (each edge needs 2 vertices)
                    // Bottom face (z0)
                    wireframeVertices.insert(wireframeVertices.end(), {x0, y0, z0, x1, y0, z0}); // edge 0
                    wireframeVertices.insert(wireframeVertices.end(), {x1, y0, z0, x1, y1, z0}); // edge 1
                    wireframeVertices.insert(wireframeVertices.end(), {x1, y1, z0, x0, y1, z0}); // edge 2
                    wireframeVertices.insert(wireframeVertices.end(), {x0, y1, z0, x0, y0, z0}); // edge 3

                    // Top face (z1)
                    wireframeVertices.insert(wireframeVertices.end(), {x0, y0, z1, x1, y0, z1}); // edge 4
                    wireframeVertices.insert(wireframeVertices.end(), {x1, y0, z1, x1, y1, z1}); // edge 5
                    wireframeVertices.insert(wireframeVertices.end(), {x1, y1, z1, x0, y1, z1}); // edge 6
                    wireframeVertices.insert(wireframeVertices.end(), {x0, y1, z1, x0, y0, z1}); // edge 7

                    // Vertical edges connecting bottom and top faces
                    wireframeVertices.insert(wireframeVertices.end(), {x0, y0, z0, x0, y0, z1}); // edge 8
                    wireframeVertices.insert(wireframeVertices.end(), {x1, y0, z0, x1, y0, z1}); // edge 9
                    wireframeVertices.insert(wireframeVertices.end(), {x1, y1, z0, x1, y1, z1}); // edge 10
                    wireframeVertices.insert(wireframeVertices.end(), {x0, y1, z0, x0, y1, z1}); // edge 11
                }
            }
        }
    }

    wireframeVertexCount = wireframeVertices.size() / 3;

    glBindBuffer(GL_ARRAY_BUFFER, wireframeVBO);
    glBufferData(GL_ARRAY_BUFFER, wireframeVertices.size() * sizeof(float), wireframeVertices.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}
