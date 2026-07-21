#include <glad/glad.h>
#include <GLFW/glfw3.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include "graphics.hpp"
#include "camera.hpp"

const double RADIAN_CONVERSION = M_PI / 180.0;

void glfw_error_callback(int error, const char* descriprion) {
    std::cerr << "GLFW ERROR: " << error << " : " << descriprion << std::endl;
}

void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos) {
    Camera* input = static_cast<Camera*>(glfwGetWindowUserPointer(window));

    if (input) {
        (*input).camera_pan_handler(xpos, ypos);
        (*input).mouse_pos(xpos, ypos);
    }
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    Camera* input = static_cast<Camera*>(glfwGetWindowUserPointer(window));

    if (input && (button == GLFW_MOUSE_BUTTON_LEFT)) {

        if (action == GLFW_PRESS) {
            (*input).activate_pan();
        }
        else if (action == GLFW_RELEASE) {
            (*input).deactivate_pan();
        }
    }
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    Camera* input = static_cast<Camera*>(glfwGetWindowUserPointer(window));

    if (input) {
        (*input).camera_scroll_handler(yoffset);
    }
}

void checkCompileErrors(unsigned int shader, std::string type) {
    int success;
    char infoLog[1024];
    if (type != "PROGRAM") {
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (!success) {
            glGetShaderInfoLog(shader, 1024, NULL, infoLog);
            std::cerr << "ERROR::SHADER_COMPILATION_ERROR of type: " << type << "\n" << infoLog << std::endl;
        }
    } else {
        glGetProgramiv(shader, GL_LINK_STATUS, &success);
        if (!success) {
            glGetProgramInfoLog(shader, 1024, NULL, infoLog);
            std::cerr << "ERROR::PROGRAM_LINKING_ERROR of type: " << type << "\n" << infoLog << std::endl;
        }
    }
}

void graphicsEngine::loadAndCompileShaders(const char* vertexPath, const char* fragmentPath) {
    std::string vertexCode;
    std::string fragmentCode;
    std::ifstream vShaderFile;
    std::ifstream fShaderFile;
    
    vShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    fShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    
    try {
        vShaderFile.open(vertexPath);
        fShaderFile.open(fragmentPath);
        std::stringstream vShaderStream, fShaderStream;
        
        vShaderStream << vShaderFile.rdbuf();
        fShaderStream << fShaderFile.rdbuf();
        
        vShaderFile.close();
        fShaderFile.close();
        
        vertexCode = vShaderStream.str();
        fragmentCode = fShaderStream.str();
    }
    catch (std::ifstream::failure& e) {
        std::cerr << "ERROR::SHADER::FILE_NOT_SUCCESSFULLY_READ: " << e.what() << std::endl;
    }
    
    const char* vShaderSource = vertexCode.c_str();
    const char* fShaderSource = fragmentCode.c_str();

    unsigned int vertex, fragment;
    
    vertex = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex, 1, &vShaderSource, NULL);
    glCompileShader(vertex);
    checkCompileErrors(vertex, "VERTEX");
    
    fragment = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment, 1, &fShaderSource, NULL);
    glCompileShader(fragment);
    checkCompileErrors(fragment, "FRAGMENT");
    
    unsigned int ID = glCreateProgram();
    glAttachShader(ID, vertex);
    glAttachShader(ID, fragment);
    glLinkProgram(ID);
    checkCompileErrors(ID, "PROGRAM");
    
    glDeleteShader(vertex);
    glDeleteShader(fragment);

    this->shaderProgram = ID;
}

std::array<unsigned int, 3> graphicsEngine::initialize_gl(const std::vector<float>& vertices, const std::vector<unsigned int>& indices) {

    unsigned int VAO, VBO, EBO;

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    std::array<unsigned int, 3> out = {VAO, EBO, VBO};
    return out;
}

std::array<unsigned int, 2> graphicsEngine::initialize_gl_grid(const std::vector<float>& vertices) {

    unsigned int VAO, VBO;

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    std::array<unsigned int, 2> out = {VAO, VBO};
    return out;
}

void clean_gl(std::array<unsigned int, 3> rocket, std::array<unsigned int, 3> pad, std::array<unsigned int, 2> grid) {

    glDeleteVertexArrays(1, &rocket[0]);
    glDeleteBuffers(1, &rocket[1]);
    glDeleteBuffers(1, &rocket[2]);

    glDeleteVertexArrays(1, &pad[0]);
    glDeleteBuffers(1, &pad[1]);
    glDeleteBuffers(1, &pad[2]);

    glDeleteVertexArrays(1, &grid[0]);
    glDeleteBuffers(1, &grid[1]);
}

Eigen::Matrix<double, 4, 4> compute_model_matrix(const Eigen::Matrix<double, 13, 1>& state, double height) {

    Eigen::Quaternion<double> orientation;
    orientation.w() = state(6);
    orientation.x() = state(7);
    orientation.y() = state(8);
    orientation.z() = state(9);

    Eigen::Quaternion<double> offset;
    offset.w() = std::cos(45 * RADIAN_CONVERSION);
    offset.x() = 0.0;
    offset.y() = std::cos(45 * RADIAN_CONVERSION);
    offset.z() = 0.0;

    orientation = orientation * offset;

    Eigen::Vector3d translation;
    translation << state(0), state(1), state(2) + (height / 2);

    Eigen::Matrix<double, 3, 3> rot_matrix = orientation.toRotationMatrix();

    Eigen::Matrix<double, 4, 4> out = Eigen::Matrix4d::Identity();
    out.block<3, 3>(0, 0) = rot_matrix;
    out.block<3, 1>(0, 3) = translation;

    return out;
}

Eigen::Matrix<double, 4, 4> compute_view_matrix(const Eigen::Vector3d& eye, const Eigen::Matrix<double, 13, 1>& state, double height) {

    Eigen::Vector3d up(0, 0, 1);

    Eigen::Vector3d translation;
    translation << state(0), state(1), state(2) + (height / 2);

    Eigen::Vector3d y = eye - translation;
    y.normalize();

    Eigen::Vector3d x = up.cross(y);
    x.normalize();

    Eigen::Vector3d z = y.cross(x);
    z.normalize();

    Eigen::Matrix<double, 4, 4> out = Eigen::Matrix4d::Identity();
    out.block<1, 3>(0, 0) = x;
    out.block<1, 3>(1, 0) = z;
    out.block<1, 3>(2, 0) = y;

    Eigen::Vector3d dot_products;
    dot_products << -1.0 * x.dot(eye), -1.0 * z.dot(eye), -1.0 * y.dot(eye);

    out.block<3, 1>(0, 3) = dot_products;

    return out;
}

Eigen::Matrix<double, 4, 4> compute_projection_matrix(double width, double height, double fov, double near_plane, double far_plane) {

    double aspect_ratio = width/height;

    Eigen::Matrix4d out = Eigen::Matrix4d::Zero();
    out(0, 0) = 1.0 / (aspect_ratio * std::tan(fov/2));
    out(1, 1) = 1.0 / (std::tan(fov/2));
    out(2, 2) = -1.0 * (far_plane + near_plane) / (far_plane - near_plane);
    out(2, 3) = -2.0 * far_plane * near_plane / (far_plane - near_plane);
    out(3, 2) = -1.0;

    return out;
}

void drag_pan(Camera& cam, const Eigen::Matrix<double, 13, 1>& state, Eigen::Vector3d& eye, double height) {

    double radius = cam.get_r();
    double cam_theta = cam.get_theta();
    double cam_phi = cam.get_phi();

    eye(0) = state(0) + (radius * std::cos(cam_phi) * std::sin(cam_theta));
    eye(1) = state(1) + (radius * std::cos(cam_phi) * std::cos(cam_theta));
    eye(2) = (state(2) + height / 2) + (radius * std::sin(cam_phi));
}

void initialize_rocket(std::vector<float>& vertices, std::vector<unsigned int>& indices, const double radius, const double height) {

    double twopi = 2 * M_PI;

    // lower disc
    for (int i = 0; i < 24; i++) {
        double theta = i * (twopi / 24);

        vertices[i * 3] = radius * std::cos(theta);
        vertices[(i * 3) + 1] = radius * std::sin(theta);
        vertices[(i * 3) + 2] = -1.0 * height / 2;
    }

    // higher disc
    for (int i = 0; i < 24; i++) {
        double theta = i * (twopi / 24);

        vertices[(i * 3) + 72] = radius * std::cos(theta);
        vertices[(i * 3) + 73] = radius * std::sin(theta);
        vertices[(i * 3) + 74] = height / 2;
    }

    vertices[144] = 0.0;
    vertices[145] = 0.0;
    vertices[146] = -1.0 * height / 2;

    vertices[147] = 0.0;
    vertices[148] = 0.0;
    vertices[149] = (11 * height / 20); //(nosecone protrusion)

    for (int i = 0; i < 24; i++) {

        if (i != 23){
            indices[i * 12] = i;
            indices[(i * 12) + 1] = i + 24;
            indices[(i * 12) + 2] = i + 25;

            indices[(i * 12) + 3] = i;
            indices[(i * 12) + 4] = i + 25;
            indices[(i * 12) + 5] = i + 1;

            indices[(i * 12) + 6] = i;
            indices[(i * 12) + 7] = i + 1;
            indices[(i * 12) + 8] = 48;

            indices[(i * 12) + 9] = i + 24;
            indices[(i * 12) + 10] = i + 25;
            indices[(i * 12) + 11] = 49;
        }
        else {
            indices[i * 12] = i;
            indices[(i * 12) + 1] = i + 24;
            indices[(i * 12) + 2] = 24;

            indices[(i * 12) + 3] = i;
            indices[(i * 12) + 4] = 24;
            indices[(i * 12) + 5] = 0;

            indices[(i * 12) + 6] = i;
            indices[(i * 12) + 7] = 0;
            indices[(i * 12) + 8] = 48;

            indices[(i * 12) + 9] = i + 24;
            indices[(i * 12) + 10] = 24;
            indices[(i * 12) + 11] = 49;
        }
    }
}

void initialize_landing_pad(std::vector<float>& meshvertices, std::vector<float>& gridvertices, std::vector<unsigned int>& indices, double size) {

    meshvertices[0] = -1.0 * size / 2;
    meshvertices[1] = -1.0 * size / 2;
    meshvertices[2] = -0.02;

    meshvertices[3] = -1.0 * size / 2;
    meshvertices[4] = size / 2;
    meshvertices[5] = -0.02;

    meshvertices[6] = size / 2;
    meshvertices[7] = size / 2;
    meshvertices[8] = -0.02;

    meshvertices[9] = size / 2;
    meshvertices[10] = -1.0 * size / 2;
    meshvertices[11] = -0.02;

    indices = {
        0, 1, 2,
        0, 2, 3        
    };

    for (int i = 0; i <= (int)(size / 20); i++) {

        double halfsize = size / 2;

        gridvertices[(i * 12)] = (i * 20) - halfsize;
        gridvertices[(i * 12) + 1] = 0 - halfsize;
        gridvertices[(i * 12) + 2] = 0;

        gridvertices[(i * 12) + 3] = (i * 20) - halfsize;
        gridvertices[(i * 12) + 4] = size - halfsize;
        gridvertices[(i * 12) + 5] = 0;

        gridvertices[(i * 12) + 6] = 0 - halfsize;
        gridvertices[(i * 12) + 7] = (i * 20) - halfsize;
        gridvertices[(i * 12) + 8] = 0;

        gridvertices[(i * 12) + 9] = size - halfsize;
        gridvertices[(i * 12) + 10] = (i * 20) - halfsize;
        gridvertices[(i * 12) + 11] = 0;
    }
}

void graphicsEngine::locate_uniforms() {

    shaderUniformLocations.push_back(glGetUniformLocation(shaderProgram, "backgroundColor"));

    shaderUniformLocations.push_back(glGetUniformLocation(shaderProgram, "fogBegin"));
    shaderUniformLocations.push_back(glGetUniformLocation(shaderProgram, "fogClamp"));

    shaderUniformLocations.push_back(glGetUniformLocation(shaderProgram, "Projection"));
    shaderUniformLocations.push_back(glGetUniformLocation(shaderProgram, "View"));
    shaderUniformLocations.push_back(glGetUniformLocation(shaderProgram, "Model"));

    shaderUniformLocations.push_back(glGetUniformLocation(shaderProgram, "ObjectColor"));
    shaderUniformLocations.push_back(glGetUniformLocation(shaderProgram, "eyePosition"));

    shaderUniformLocations.push_back(glGetUniformLocation(shaderProgram, "shininess"));
    shaderUniformLocations.push_back(glGetUniformLocation(shaderProgram, "ambience"));
}

int graphicsEngine::graphics_init() {

    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit()) {
        std::cerr << "GLFW failed to initialize" << std::endl;
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

    this->window = glfwCreateWindow(this->window_width, this->window_height, "FUJITORA", nullptr, nullptr);
    if (!window) {
        std::cerr << "failed to create glfw window" << std::endl;
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);

    glfwSetWindowUserPointer(window, &cam);

    glfwSetCursorPosCallback(window, cursor_pos_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetScrollCallback(window, scroll_callback);

    glfwSwapInterval(1);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "dailed to initialize GLAD/OpenGL context pointers" << std::endl;
        glfwDestroyWindow(window);
        glfwTerminate();
        return -1;
    }

    glEnable(GL_DEPTH_TEST);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    std::vector<float> rocket_vertices(150);
    std::vector<unsigned int> rocket_indices(288);
    initialize_rocket(rocket_vertices, rocket_indices, 1.85, 40);

    std::vector<float> grid_vertices(6012);
    std::vector<float> mesh_vertices(12);
    std::vector<unsigned int> mesh_indices(6);

    initialize_landing_pad(mesh_vertices, grid_vertices, mesh_indices, 10000);

    loadAndCompileShaders("src/shaders/shader.vert", "src/shaders/shader.frag");
    this->rocket_buffers_arrays = initialize_gl(rocket_vertices, rocket_indices);
    this->grid_buffers_arrays = initialize_gl_grid(grid_vertices);
    this->pad_buffers_arrays = initialize_gl(mesh_vertices, mesh_indices);

    Eigen::Vector3d newEye(0.0, 65.0, 20.0);
    this->eye = newEye;

    sun_coords[0] = 30.0;
    sun_coords[1] = 100.0;
    sun_coords[2] = 20.0;

    rocket_color[0] = 0.92;
    rocket_color[1] = 0.9;
    rocket_color[2] = 0.95;

    pad_color[0] = 0.05;
    pad_color[1] = 0.02;
    pad_color[2] = 0.14;

    grid_color[0] = 0.95;
    grid_color[1] = 0.95;
    grid_color[2] = 0.95;

    bg_color[0] = 0.16;
    bg_color[1] = 0.14;
    bg_color[2] = 0.24;

    locate_uniforms();

    return 0;
}

bool graphicsEngine::graphics_should_close() const{
    return glfwWindowShouldClose(this->window);
}

void graphicsEngine::graphics_end() {
    
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    clean_gl(rocket_buffers_arrays, pad_buffers_arrays, grid_buffers_arrays);
    glDeleteProgram(shaderProgram);

    glfwDestroyWindow(window);
    glfwTerminate();
}

void graphicsEngine::graphics_step(const Eigen::Matrix<double, 13, 1>& state) {

    Eigen::Matrix<float, 4, 4> i = Eigen::Matrix4f::Identity();

    glfwPollEvents();

    glClearColor(0.16, 0.14, 0.24, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::Begin("EYE");
    ImGui::Text("x: %f, y: %f. z: %f", eye(0), eye(1), eye(2));
    ImGui::End();

    ImGui::Begin("ROCKET");
    ImGui::Text("x: %f, y: %f. z: %f", state(0), state(1), state(2));
    ImGui::End();

    drag_pan(cam, state, eye, 40);
    Eigen::Matrix<float, 4, 4> model_matrix = compute_model_matrix(state, 40).cast<float>();
    Eigen::Matrix<float, 4, 4> view_matrix = compute_view_matrix(eye, state, 40).cast<float>();
    Eigen::Matrix<float, 4, 4> projection_matrix = compute_projection_matrix(window_width, window_height, 50.0 * RADIAN_CONVERSION, 0.1, 1000.0).cast<float>();

    glUseProgram(shaderProgram);

    glUniform3fv(shaderUniformLocations[0], 1, bg_color);
    glUniform1f(shaderUniformLocations[1], 400.0);
    glUniform1f(shaderUniformLocations[2], 600.0);

    glUniformMatrix4fv(shaderUniformLocations[3], 1, GL_FALSE, projection_matrix.data());
    glUniformMatrix4fv(shaderUniformLocations[4], 1, GL_FALSE, view_matrix.data());
    
    //rocket
    glBindVertexArray(rocket_buffers_arrays[0]);

    glUniformMatrix4fv(shaderUniformLocations[5], 1, GL_FALSE, model_matrix.data());

    glUniform3fv(shaderUniformLocations[6], 1, rocket_color);
    glUniform1f(shaderUniformLocations[8], 0.3);
    glUniform1f(shaderUniformLocations[9], 0.2);

    Eigen::Vector3f eye_pass = eye.cast<float>();
    glUniform3fv(shaderUniformLocations[7], 1, eye_pass.data());

    glDrawElements(GL_TRIANGLES, 288, GL_UNSIGNED_INT, 0);

    //ground
    glBindVertexArray(pad_buffers_arrays[0]);

    glUniformMatrix4fv(shaderUniformLocations[5], 1, GL_FALSE, i.data());
    glUniform3fv(shaderUniformLocations[6], 1, pad_color);
    glUniform1f(shaderUniformLocations[8], 0.01);
    glUniform1f(shaderUniformLocations[9], 1.0);

    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0, 1.0);

    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

    glDisable(GL_POLYGON_OFFSET_FILL);

    //grid
    glBindVertexArray(grid_buffers_arrays[0]);

    glUniform3fv(shaderUniformLocations[6], 1, grid_color);
    glUniform1f(shaderUniformLocations[9], 1.0);

    glLineWidth(5.0);

    glDrawArrays(GL_LINES, 0, 2004);

    glBindVertexArray(0);

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
}
