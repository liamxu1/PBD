//
// Created by Jack Purvis
//

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw_gl3.h>
#include <main.hpp>
#include <simulation.hpp>

// Frame timing
double lastTime = 0.0f;
int frameCount = 0;

// Mouse
bool mousePressed = false;
float mouseX = 0.0f;
float mouseY = 0.0f;

// Instances
Simulation* simulation;

bool resetStatus = false;

std::ofstream outStream("../debug.log");

void mouseMovedCallback(GLFWwindow* win, double xPos, double yPos) {
    float xChange =  (float) xPos - mouseX;
    float yChange =  (float) yPos - mouseY;

    mouseX = (float) xPos;
    mouseY = (float) yPos;

    if (mousePressed && !ImGui::IsMouseHoveringAnyWindow()) {
        simulation->scene->pitch += yChange / 360.0f;
        simulation->scene->yaw += xChange / 360.0f;
    }
}

void mouseButtonCallback(GLFWwindow *win, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) mousePressed = true;
        else if (action == GLFW_RELEASE) mousePressed = false;
    }
}

void keyCallback(GLFWwindow *win, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS) {
        if (key == GLFW_KEY_SPACE) {
            resetStatus = true;
            cout << "resetting...\n";
        }
    }

    if (action == GLFW_PRESS || action == GLFW_REPEAT) {

        switch (key)
        {
        case GLFW_KEY_LEFT:
            simulation->scene->translateInteraction(Vector3f(-0.3f, 0.0f, 0.0f), 0);
            break;
        case GLFW_KEY_RIGHT:
            simulation->scene->translateInteraction(Vector3f(0.3f, 0.0f, 0.0f), 0);
            break;
        case GLFW_KEY_COMMA:
            simulation->scene->translateInteraction(Vector3f(0.0f, 0.0f, 0.3f), 2);
            break;
        case GLFW_KEY_PERIOD:
            simulation->scene->translateInteraction(Vector3f(0.0f, 0.0f, -0.3f), 2);
            break;
        case GLFW_KEY_UP:
            simulation->scene->translateInteraction(Vector3f(0.0f, 0.3f, 0.0f), 1);
            break;
        case GLFW_KEY_DOWN:
            simulation->scene->translateInteraction(Vector3f(0.0f, -0.3f, 0.0f), 1);
            break;
        case GLFW_KEY_W:
            simulation->scene->translation(2) += 0.3f;
            break;
        case GLFW_KEY_S:
            simulation->scene->translation(2) -= 0.3f;
            break;
        case GLFW_KEY_A:
            simulation->scene->translation(0) -= 0.3f;
            break;
        case GLFW_KEY_D:
            simulation->scene->translation(0) += 0.3f;
            break;
        case GLFW_KEY_Q:
            simulation->scene->translation(1) += 0.3f;
            break;
        case GLFW_KEY_E:
            simulation->scene->translation(1) -= 0.3f;
            break;
        default:
            break;
        }
    }
}

int main() {

    // Initialise GLFW
    if(!glfwInit()) {
        fprintf(stderr, "Failed to initialize GLFW\n");
        return -1;
    }

    glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); // We want OpenGL 3.3
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // For MacOS happy
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // We don't want the old OpenGL

    // Open a window and create its OpenGL context
    GLFWwindow* window;
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    glfwWindowHint(GLFW_AUTO_ICONIFY, GL_FALSE);
    glfwWindowHint(GLFW_DECORATED, BORDERLESS ? GL_FALSE : GL_TRUE);
    window = glfwCreateWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Window", (FULL_SCREEN ? glfwGetPrimaryMonitor() : NULL), NULL);
    if (window == NULL) {
        fprintf(stderr, "Failed to open GLFW window\n");
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    // Initialize GLEW
    glewExperimental = 1; // Needed in core profile
    if (glewInit() != GLEW_OK) {
        fprintf(stderr, "Failed to initialize GLEW\n");
        return -1;
    }

    // Ensure we can capture the escape key being pressed below
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

    // Input callbacks
    glfwSetCursorPosCallback(window, mouseMovedCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetKeyCallback(window, keyCallback);

    // Disable VSync
    // glfwSwapInterval(0);

    // Setup the vertex array object
    GLuint vertexArray;
    glGenVertexArrays(1, &vertexArray);
    glBindVertexArray(vertexArray);

    // Setup ImGui binding
    ImGui_ImplGlfwGL3_Init(window, false);

    // Setup simulation
    simulation = new Simulation();

    // Setup frame timing
    lastTime = glfwGetTime();
    frameCount = 0;

    // Enable depth test
    glEnable(GL_DEPTH_TEST);

    // Check if the escape key was pressed or the window was closed
    while(glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS && glfwWindowShouldClose(window) == 0) {

        // Measure speed
        double currentTime = glfwGetTime();
        frameCount++;

        // Print the frame time every second
        if (currentTime - lastTime >= 1.0) {
            printf("%f ms/frame\n", 1000.0 / (double) frameCount);
            frameCount = 0;
            lastTime += 1.0;
        }

        // Poll events and setup GUI for the current frame
        glfwPollEvents();
        ImGui_ImplGlfwGL3_NewFrame();

        // Clear the screen
        glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Update simulation
        simulation->update();

        if (resetStatus)
        {
            resetStatus = false;
            simulation->reset();
        }

        // Render gui
        simulation->renderGUI();
        ImGui::Render();

        glfwSwapBuffers(window);
    }

    ImGui_ImplGlfwGL3_Shutdown();
    glfwTerminate();

    delete simulation;
    outStream.close();

    return 0;
}