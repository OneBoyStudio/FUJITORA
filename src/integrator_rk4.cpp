#include "integrator_rk4.hpp"

IntegratorRK4::IntegratorRK4(double timestep) : dt(timestep) {};

Eigen::Matrix<double, 6, 1> IntegratorRK4::step(const Eigen::Matrix<double, 6, 1>& state, const Rocket& rocket, const Engine& engine) const{

    Eigen::Matrix<double, 6, 1> k1 = rocket.compute_derivatives(state, engine, 9.81);
    Eigen::Matrix<double, 6, 1> k2 = rocket.compute_derivatives(state + (k1 * dt * 0.5), engine, 9.81);
    Eigen::Matrix<double, 6, 1> k3 = rocket.compute_derivatives(state + (k2 * dt * 0.5), engine, 9.81);
    Eigen::Matrix<double, 6, 1> k4 = rocket.compute_derivatives(state + (k3 * dt), engine, 9.81);

    return state + ((k1 + k4 + (2 * (k2 + k3))) * (dt / 6));
}