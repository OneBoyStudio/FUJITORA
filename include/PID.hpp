#pragma once
#include "rocket_physics.hpp"
#include <Eigen/Dense>

class PIDController {
private:
    double k_p;
    double k_i;
    double k_d;

    double I_prev;
    double theta_prev;
    double error_prev;
public:
    //explicit PIDController(double k_p, double k_i, double k_d);
    void initializePID(const Eigen::Matrix<double, 13, 1> state, double target_angle);

    Eigen::Vector3d compute_coefficients(double control_authority, double natural_frequency);

    double calculate_error(double pitch_angle, double target_angle) const;
    double calculate_integral(double dt, Eigen::Vector2d error) const;
    double calculate_derivative(const Eigen::Matrix<double, 13, 1> state) const;

    void update_integral(double error, double correction, const Engine& engine, double integral);

    double calculate_correction(const Engine& engine, double error, double integral, double derivative) const;

    double step(double dt, const Eigen::Matrix<double, 13, 1> state, const Engine& engine, double target_angle);
};