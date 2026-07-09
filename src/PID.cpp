#define _USE_MATH_DEFINES
#include <iostream>
#include "PID.hpp"
#include <cmath>

//PIDController::PIDController(double k_p, double k_i, double k_d) : k_p(k_p), k_i(k_i), k_d(k_d) {}

void PIDController::initializePID(const Eigen::Matrix<double, 6, 1> state, double target_angle) {
    I_prev = 0;
    theta_prev = state(2);
    error_prev = target_angle - state(2);
}

Eigen::Vector3d PIDController::compute_coefficients(double control_authority, double natural_frequency) {

    k_p = (natural_frequency * natural_frequency) / control_authority;
    k_d = 2 * natural_frequency / control_authority;
    k_i = k_p * natural_frequency / (10 * M_PI);

    Eigen::Vector3d coefficients;
    coefficients << k_p, k_i, k_d;
    return coefficients;
}

double PIDController::calculate_error(double pitch_angle, double target_angle) const {
    return target_angle - pitch_angle;
}

double PIDController::calculate_integral(double dt, Eigen::Vector2d error) const {
    return I_prev + ((error(0) + error(1)) * 0.5 * dt);
}

double PIDController::calculate_derivative(Eigen::Matrix<double, 6, 1> state) const {
    return -1.0 * state(5);
}

void PIDController::update_integral(double error, double correction, const Engine& engine, double integral) {

    const double gimbal_limit = engine.get_limit();
    if ((std::abs(correction) >= gimbal_limit) && (error * correction > 0)) {
        return;
    }
    
    I_prev = integral;
}

double PIDController::calculate_correction(const Engine& engine, double error, double integral, double derivative) const {

    double output = (k_p * error) + (k_i * integral) + (k_d * derivative);
    double clamp = engine.get_limit();

    if (std::abs(output) >= clamp) {
        return clamp;
    }
    return output;
}

double PIDController::step(double dt, const Eigen::Matrix<double, 6, 1> state, const Engine& engine, double target_angle) {

    Eigen::Vector2d error;
    error << calculate_error(state(2), target_angle) , error_prev;  

    double integral = calculate_integral(dt, error);
    double derivative = calculate_derivative(state);

    double correction = calculate_correction(engine, error(0), integral, derivative);

    update_integral(error(0), correction, engine, integral);
    theta_prev = state(2);
    error_prev = error(0);

    return correction;
}