#pragma once
#include <Eigen/Dense>
#include "camera.hpp"
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui/imgui.h>
#include <imgui/imgui_impl_glfw.h>
#include <imgui/imgui_impl_opengl3.h>
#include "states.hpp"

class graphicsEngine {
private:
    const int window_width = 1920;
    const int window_height = 1080;
    Camera cam = Camera(2.0, 1.25, window_width, window_height);
    Eigen::Vector3d eye;
    GLFWwindow* window;

    unsigned int generalShaderProgram;
    unsigned int particleShaderProgram;
    unsigned int loadAndCompileShaders(const char* vertexPath, const char* fragmentPath);

    std::vector<int> shaderUniformLocations;
    void locate_uniforms();

    std::array<unsigned int, 3> rocket_buffers_arrays;
    std::array<unsigned int, 2> grid_buffers_arrays;
    std::array<unsigned int, 3> pad_buffers_arrays;
    std::array<unsigned int, 2> engine_exhaust_buffers_arrays;

    std::array<unsigned int, 3> initialize_gl(const std::vector<float>& vertices, const std::vector<unsigned int>& indices);
    std::array<unsigned int, 2> initialize_gl_grid(const std::vector<float>& vertices);
    std::array<unsigned int, 2> initialize_exhaust_particles();

    void update_exhaust_particles(unsigned int& VBO, std::vector<GPUparticle> active_particles);

    float sun_coords[3];
    float rocket_color[3];
    float pad_color[3];
    float grid_color[3];
    float bg_color[3];
public:
    int graphics_init();
    bool graphics_should_close() const;
    void graphics_step(const Eigen::Matrix<double, 13, 1>& state, particle_ring_buffer& engine_exhaust);
    void graphics_end();
};

