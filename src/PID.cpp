#define _USE_MATH_DEFINES
#include <iostream>
#include "PID.hpp"
#include <cmath>
#include <algorithm>

//PIDController::PIDController(double k_p, double k_i, double k_d) : k_p(k_p), k_i(k_i), k_d(k_d) {}

void PIDController::initializePID(const Eigen::Matrix<double, 13, 1> state, Eigen::Quaterniond target_orientation) {
    I_prev = Eigen::Vector3d::Zero();

    Eigen::Quaterniond orientation(state(6), state(7), state(8), state(9));

    orientation_prev = orientation;

    Eigen::Quaterniond error_q = orientation.conjugate() * target_orientation;
    if (error_q.w() < 0) {
        error_q.coeffs() = -error_q.coeffs();
    }

    Eigen::Vector3d error(error_q.x(), error_q.y(), error_q.z());
    error = 2.0 * error;

    error_prev = error;
}

Eigen::Matrix<double, 3, 3> PIDController::compute_coefficients(Eigen::Vector3d control_authority, Eigen::Vector3d natural_frequency) {

    Eigen::Matrix<double, 3, 3> out;

    out(0, 0) = (natural_frequency(0) * natural_frequency(0)) / control_authority(0); //P
    out(0, 1) = (natural_frequency(1) * natural_frequency(1)) / control_authority(1);
    out(0, 2) = (natural_frequency(2) * natural_frequency(2)) / control_authority(2);

    out(1, 0) = 2 * natural_frequency(0) / control_authority(0); //D
    out(1, 1) = 2 * natural_frequency(1) / control_authority(1);
    out(1, 2) = 2 * natural_frequency(2) / control_authority(2);

    out(2, 0) = out(1, 0) * natural_frequency(0) / (10 * M_PI); //I
    out(2, 1) = out(1, 1) * natural_frequency(1) / (10 * M_PI);
    out(2, 2) = out(1, 2) * natural_frequency(2) / (10 * M_PI);

    gains = out;
    return out;
}

Eigen::Vector3d PIDController::calculate_error(Eigen::Quaterniond orientation, Eigen::Quaterniond target_orientation) const {

    Eigen::Quaterniond error = orientation.conjugate() * target_orientation;
    if (error.w() < 0) {
        error.coeffs() = -error.coeffs();
    }

    Eigen::Vector3d out(error.x(), error.y(), error.z());
    out = 2.0 * out;
    return out;
}

Eigen::Vector3d PIDController::calculate_integral(double dt, Eigen::Vector3d error, Eigen::Vector3d error_prev) const {
    return I_prev + ((error + error_prev) * 0.5 * dt);
}

Eigen::Vector3d PIDController::calculate_derivative(Eigen::Matrix<double, 13, 1> state) const {

    Eigen::Vector3d out(state(10), state(11), state(12));
    return -1.0 * out;
}

void PIDController::update_integral(Eigen::Vector3d error, Eigen::Vector3d correction, const Engine& engine, Eigen::Vector3d integral) {
    
    Eigen::Vector3d I_hold = I_prev;
    I_prev = integral;

    const double gimbal_limit = engine.get_limit();
    if ((std::abs(correction(1)) >= gimbal_limit) && (error(1) * correction(1) > 0)) {
        I_prev(1) = I_hold(1);
    }
    if ((std::abs(correction(2)) >= gimbal_limit) && (error(2) * correction(2) > 0)) {
        I_prev(2) = I_hold(2);
    }
    if ((std::abs(correction(0)) >= gimbal_limit) && (error(0) * correction(0) > 0)) {
        I_prev(0) = I_hold(0);
    }
}

Eigen::Vector3d PIDController::calculate_correction(const Engine& engine, Eigen::Vector3d error, Eigen::Vector3d integral, Eigen::Vector3d derivative) const {

    Eigen::Matrix<double, 3, 3> k_p = Eigen::Matrix3d::Zero();
    k_p(0, 0) = gains(0, 0);
    k_p(1, 1) = gains(0, 1);
    k_p(2, 2) = gains(0, 2);

    Eigen::Matrix<double, 3, 3> k_i = Eigen::Matrix3d::Zero();
    k_i(0, 0) = gains(2, 0);
    k_i(1, 1) = gains(2, 1);
    k_i(2, 2) = gains(2, 2);

    Eigen::Matrix<double, 3, 3> k_d = Eigen::Matrix3d::Zero();
    k_d(0, 0) = gains(1, 0);
    k_d(1, 1) = gains(1, 1);
    k_d(2, 2) = gains(1, 2);

    Eigen::Vector3d output = (k_p * error) + (k_i * integral) + (k_d * derivative);
    double engine_clamp = engine.get_limit();

    output(1) = std::max(-engine_clamp, std::min(output(1), engine_clamp));
    output(2) = std::max(-engine_clamp, std::min(output(2), engine_clamp));
    output(0) = std::max(-1.0, std::min(output(0), 1.0));

    return output;
}

Eigen::Vector3d PIDController::step(double dt, const Eigen::Matrix<double, 13, 1> state, const Engine& engine, Eigen::Quaterniond target_orientation) {

    Eigen::Quaterniond orientation;
    orientation.w() = state(6);
    orientation.x() = state(7);
    orientation.y() = state(8);
    orientation.z() = state(9);

    Eigen::Vector3d error = calculate_error(orientation, target_orientation);

    Eigen::Vector3d integral = calculate_integral(dt, error, error_prev);
    Eigen::Vector3d derivative = calculate_derivative(state);

    Eigen::Vector3d correction = calculate_correction(engine, error, integral, derivative);

    update_integral(error, correction, engine, integral);
    orientation_prev = orientation;
    error_prev = error;

    return correction;
}