#pragma once
#include "engine_physics.hpp"
#include <Eigen/Dense>

class PIDController {
private:
    Eigen::Matrix<double, 3, 3> gains;

    Eigen::Vector3d I_prev;
    Eigen::Quaterniond orientation_prev;
    Eigen::Vector3d error_prev;
public:
    //explicit PIDController(double k_p, double k_i, double k_d);
    void initializePID(const Eigen::Matrix<double, 13, 1> state, Eigen::Quaterniond target_orientation);

    Eigen::Matrix<double, 3, 3> compute_coefficients(Eigen::Vector3d control_authority, Eigen::Vector3d natural_frequency);

    Eigen::Vector3d calculate_error(Eigen::Quaterniond orientation, Eigen::Quaterniond target_orientation) const;
    Eigen::Vector3d calculate_integral(double dt, Eigen::Vector3d error, Eigen::Vector3d error_prev) const;
    Eigen::Vector3d calculate_derivative(const Eigen::Matrix<double, 13, 1> state) const;

    void update_integral(Eigen::Vector3d error, Eigen::Vector3d correction, const Engine& engine, Eigen::Vector3d integral);

    Eigen::Vector3d calculate_correction(const Engine& engine, Eigen::Vector3d error, Eigen::Vector3d integral, Eigen::Vector3d derivative) const;

    Eigen::Vector3d step(double dt, const Eigen::Matrix<double, 13, 1> state, const Engine& engine, Eigen::Quaterniond target_orientation);
};