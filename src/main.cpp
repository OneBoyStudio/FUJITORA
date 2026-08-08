#define _USE_MATH_DEFINES
#include "graphics.hpp"
#include <iostream>
#include <fstream>
#include <cmath>
#include "rocket_physics.hpp"
#include "engine_physics.hpp"
#include "integrator_rk4.hpp"
#include "PID.hpp"
#include "states.hpp"

const double RADIAN_CONVERSION = M_PI / 180.0;

int main() {

    Eigen::Matrix<double, 13, 1> state;
    state << 
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    std::cos(55 * RADIAN_CONVERSION), 0.0, -1.0 * std::sin(55 * RADIAN_CONVERSION), 0.0,
    0.0, 0.0, 0.0;

    graphicsEngine ge;
    ge.graphics_init();

    double deltatime = 0.01;

    Eigen::Vector3d bell_offset;
    bell_offset << 0.0, 0.0, 0.0;

    Eigen::Vector3d thruster1_offset;
    thruster1_offset << 0.0, 1.85, 0.0;
    Eigen::Vector3d thruster2_offset;
    thruster2_offset << 0.0, -1.85, 0.0;
    Eigen::Vector3d thruster3_offset;
    thruster3_offset << 40.0, 1.85, 0.0;
    Eigen::Vector3d thruster4_offset;
    thruster4_offset << 40.0, -1.85, 0.0;

    double rocket_radius = 1.85;

    RocketComponent engine_bell(4230.0, bell_offset, false, rocket_radius);

    RocketComponent thruster_pod1(0.0, thruster1_offset, false, rocket_radius);
    RocketComponent thruster_pod2(0.0, thruster2_offset, false, rocket_radius);
    RocketComponent thruster_pod3(0.0, thruster3_offset, false, rocket_radius);
    RocketComponent thruster_pod4(0.0, thruster4_offset, false, rocket_radius);

    Engine engine(0.0, 0.0, 7605000.0, 282, engine_bell);
    engine.limits(5.0 * RADIAN_CONVERSION);

    Thruster thruster1(1000, thruster_pod1);
    Thruster thruster2(1000, thruster_pod2);
    Thruster thruster3(1000, thruster_pod3);
    Thruster thruster4(1000, thruster_pod4);

    thruster1.set_zy_couple(3);
    thruster2.set_zy_couple(4);
    thruster3.set_zy_couple(1);
    thruster4.set_zy_couple(2);

    thruster1.set_x_couple(2);
    thruster2.set_x_couple(1);
    thruster3.set_x_couple(4);
    thruster4.set_x_couple(3);

    std::vector<Thruster*> thrusters = {&thruster1, &thruster2, &thruster3, &thruster4};
    Thrusters thrustpod(thrusters);

    RocketStructure structure(522600, 2.6, 1230.0, 832.5, rocket_radius, 70.0);
    structure.initialize_payload(15000.0);
    structure.initialize_dry_fuselage(23120.0);
    structure.initialize_engine_cluster(470, 2.9, 9);
    structure.initialize_rocket_structure();

    Rocket rocket(structure, thrustpod);
    rocket.compute_CoM(engine);

    Eigen::Quaterniond init_orientation;
    init_orientation.w() = state(6);
    init_orientation.x() = state(7);
    init_orientation.y() = state(8);
    init_orientation.z() = state(9);

    Eigen::Vector3d init_pos = init_orientation * rocket.get_CoM();
    state(0) = init_pos(0);
    state(1) = init_pos(1);
    state(2) = init_pos(2) + 10;

    IntegratorRK4 integrator(deltatime);
    
    PIDController pid;
    Eigen::Quaterniond target_orientation;
    target_orientation.w() = std::cos(50 * RADIAN_CONVERSION);
    target_orientation.x() = 0.0;
    target_orientation.y() = -1.0 * std::sin(50 * RADIAN_CONVERSION);
    target_orientation.z() = 0.0;

    Eigen::Vector3d control_authority = rocket.control_authority(engine, thrusters);
    Eigen::Vector3d natural_frequency(3.0, 3.0, 3.0);

    pid.initializePID(state, target_orientation);
    pid.compute_coefficients(control_authority, natural_frequency);

    particle_ring_buffer engine_buffer;
    engine_buffer.initialize_particle_ring_buffer();

    std::ofstream log_file("logs/trajectory.csv");
    if (!log_file.is_open()) {
        std::cerr << "Error logging data" << std::endl;
        return 1;
    }
    log_file << "time,x,y,z,vx,vy,vz,q1,q2,q3,q4,omegax,omegay,omegaz,delta_y,delta_z\n";

    double time = 0.0;
    //double length = 50.0;

    double accumulator_bucket = 0.0;
    double previous_time = glfwGetTime();

    while (!ge.graphics_should_close()) {
    //while (current_time <= length) {

        Eigen::Vector2d deltas = engine.get_delta();

        log_file << time << "," << state(0) << "," << state(1) << "," << state(2) << "," << state(3) << "," << state(4) << "," << state(5) << "," << state(6) << "," << state(7) << "," << state(8) << "," << state(9) << "," << state(10) << "," << state(11) << "," << state(12) << "," << deltas(0) << "," << deltas(1) << "\n";

        Eigen::Vector3d new_controls = pid.step(deltatime, state, engine, target_orientation);

        double current_time = glfwGetTime();
        double frame_length = current_time - previous_time;
        previous_time = current_time;

        if (frame_length >= 0.1) {
            frame_length = 0.1;
        }

        accumulator_bucket += frame_length;

        while (accumulator_bucket >= deltatime) {

            state = integrator.step(state, rocket, engine);
            accumulator_bucket -= deltatime;

            if (engine.get_thrust_magnitude() > 0) {
                Eigen::Quaterniond rocket_orientation(state(6), state(7), state(8), state(9));

                Eigen::Vector3d engine_pos(state(0), state(1), state(2));

                Eigen::Vector3d engine_displacement_offset = rocket_orientation * Eigen::Vector3d(-structure.get_height() / 2.0, 0, 0);
                engine_pos += engine_displacement_offset;

                Eigen::Vector3d particle_vel = deltatime * 10.0 * (rocket_orientation * engine.get_thrust_vectors()) / structure.get_mass();

                engine_buffer.instantiate(engine_pos, particle_vel, 0.5);
            }

            engine_buffer.step(deltatime);
        }

        ge.graphics_step(state, engine_buffer);

        rocket.normalize_quaternion(state);
        engine.set_delta(new_controls(1), -new_controls(2));
        
        Eigen::Vector3d rcs_in(new_controls(0), 0, 0);
        thrustpod.actuate_thruster(rcs_in);

        control_authority = rocket.control_authority(engine, thrusters);
        pid.compute_coefficients(control_authority, natural_frequency);

        time += deltatime;
    }
    
    ge.graphics_end();

    log_file.close();
    return 0;
}
