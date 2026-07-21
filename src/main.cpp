#define _USE_MATH_DEFINES
#include "graphics.hpp"
#include <iostream>
#include <fstream>
#include <cmath>
#include "rocket_physics.hpp"
#include "engine_physics.hpp"
#include "integrator_rk4.hpp"
#include "PID.hpp"

const double RADIAN_CONVERSION = M_PI / 180.0;

int main() {

    Eigen::Matrix<double, 13, 1> state;
    state << 
    0.0, 0.0, 0.0,
    80.0, 0.0, 80.0,
    std::cos(45 * RADIAN_CONVERSION), 0.0, -1.0 * std::cos(45 * RADIAN_CONVERSION), 0.0, 
    0.0, 0.0, 0.0;

    graphicsEngine ge;
    ge.graphics_init();

    double deltatime = 0.01;

    Rocket rocket(30000.0, 40.0);

    Eigen::Matrix<double, 3, 3> inertia_fuselage;
    inertia_fuselage << 
    51337.5, 0.0, 0.0,
    0.0, 4025668.75, 0.0,
    0.0, 0.0, 4025668.75;
    Eigen::Vector3d fuselage_offset;
    fuselage_offset << 20.0, 0.0, 0.0;

    Eigen::Matrix<double, 3, 3> inertia_bell;
    inertia_bell << 
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0;
    Eigen::Vector3d bell_offset;
    bell_offset << 0.0, 0.0, 0.0;

    RocketComponent fuselage(30000.0, inertia_fuselage, fuselage_offset);
    RocketComponent engine_bell(0.0, inertia_bell, bell_offset);

    Engine engine(0.0, 0.0, 0.0, engine_bell);
    engine.limits(5.0 * RADIAN_CONVERSION);

    IntegratorRK4 integrator(deltatime);

    PIDController pid_y;
    PIDController pid_z;
    double target_angle_y = 0;
    double target_angle_z = 0;

    std::vector<RocketComponent*> components = {&fuselage, &engine_bell};
    rocket.set_components(components);
    
    Eigen::Vector3d control_authority = rocket.control_authority(engine);

    //pid_y.initializePID(state, target_angle_y);
    //pid_y.compute_coefficients(control_authority(0), 3.0);
    //pid_z.initializePID(state, target_angle_z);
    //pid_z.compute_coefficients(control_authority(1), 3.0);

    std::ofstream log_file("logs/trajectory.csv");
    if (!log_file.is_open()) {
        std::cerr << "Error logging data" << std::endl;
        return 1;
    }
    log_file << "time,x,y,z,vx,vy,vz,q1,p2,p3,p4,omegax,omegay,omegaz,delta_y,delta_z\n";

    double current_time = 0.0;

    double accumulator_bucket = 0.0;
    double previous_time = glfwGetTime();

    while (!ge.graphics_should_close()) {

        Eigen::Vector2d deltas = engine.get_delta();

        log_file << current_time << "," << state(0) << "," << state(1) << "," << state(2) << "," << state(3) << "," << state(4) << "," << state(5) << "," << state(6) << "," << state(7) << "," << state(8) << "," << state(9) << "," << state(10) << "," << state(11) << "," << state(12) << "," << deltas(0) << "," << deltas(1) << "\n";

        //double new_delta_y = pid_y.step(deltatime, state, engine, target_angle_y);
        //double new_delta_z = pid_z.step(deltatime, state, engine, target_angle_z);

        double current_time = glfwGetTime();
        double frame_length = current_time - previous_time;
        previous_time = current_time;

        if (frame_length >= 0.1) {
            frame_length = 0.1;
        }

        accumulator_bucket += frame_length;

        while (accumulator_bucket >= deltatime) {

            //state = integrator.step(state, rocket, engine);
            accumulator_bucket -= deltatime;
        }

        ge.graphics_step(state);

        rocket.normalize_quaternion(state);
        //engine.set_delta(new_delta_z, new_delta_y);

        current_time += deltatime;
    }
    
    ge.graphics_end();

    log_file.close();
    return 0;
}
