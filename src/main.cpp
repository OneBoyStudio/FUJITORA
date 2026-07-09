#define _USE_MATH_DEFINES
#include <iostream>
#include <fstream>
#include <cmath>
#include "rocket_physics.hpp"
#include "integrator_rk4.hpp"
#include "PID.hpp"

const double RADIAN_CONVERSION = M_PI / 180.0;

int main() {

    double deltatime = 0.01;

    Rocket rocket(2000.0, 12.0, 24000.0, 6.0);

    Engine engine(0, 26000.0);
    engine.limits(5.0 * RADIAN_CONVERSION);

    IntegratorRK4 integrator(deltatime);

    Eigen::Matrix<double, 6, 1> state;
    state << 0.0, 10000.0, 20.0 * RADIAN_CONVERSION,  0.0, 0.0, 0.0;

    PIDController pid;
    double target_angle = 0;
    pid.initializePID(state, target_angle);
    pid.compute_coefficients(rocket.control_authority(engine), 3.0);

    std::ofstream log_file("logs/trajectory.csv");
    if (!log_file.is_open()) {
        std::cerr << "Error logging data" << std::endl;
        return 1;
    }
    log_file << "time,x,z,theta,vx,vz,omega,delta,thrust\n";

    double current_time = 0.0;
    double sim_duration = 50.0;

    while (current_time <= sim_duration) {
        log_file << current_time << "," << state(0) << "," << state(1) << "," << state(2) << "," << state(3) << "," << state(4) << "," << state(5) << "," << engine.get_delta() << "," << engine.get_thrust() << "\n";
        double new_delta = pid.step(deltatime, state, engine, target_angle);
        state = integrator.step(state, rocket, engine);
        engine.set_delta(new_delta);
        current_time += deltatime;
    }

    log_file.close();
    return 0;
}