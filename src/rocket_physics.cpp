#include "rocket_physics.hpp"
#include <cmath>

Rocket::Rocket(double m, double l, double i, double CoM) : mass(m), length(l), inertial_moment(i), CoM(CoM) {};

Eigen::Matrix<double, 6, 1> Rocket::compute_derivatives(const Eigen::Matrix<double, 6, 1>& state, const Engine& engine, double gravity) const {

    Eigen::Vector3d thrust = engine.thrust_vectors(state, length);

    double a_x = thrust (0)/ mass;
    double a_z = (thrust(1) / mass) - gravity;
    double omega_prime = thrust(2) / inertial_moment;

    Eigen::Matrix<double, 6, 1> state_prime;
    state_prime << state(3), state(4), state(5), a_x, a_z, omega_prime;
    return state_prime;
}

double Rocket::control_authority(const Engine& engine) const {
    return (length * engine.get_thrust()) / (2 * inertial_moment);
}

Engine::Engine(double d, double t) : delta(d), thrust(t) {};

void Engine::limits(double limit) {
    gimbal_limit = limit;
}

double Engine::get_limit() const {
    return gimbal_limit;
}

double Engine::get_thrust() const {
    return thrust;
}

double Engine::get_delta() const {
    return delta;
}

void Engine::set_delta(double new_delta) {
    delta = new_delta;
}

Eigen::Vector3d Engine::thrust_vectors(const Eigen::Matrix<double, 6, 1>& state, double length) const {

    double thrust_x = thrust * std::sin(delta + state(2));
    double thrust_z = thrust * std::cos(delta + state(2));
    double torque = -0.5 * length * thrust * std::sin(delta);

    Eigen::Vector3d out;
    out << thrust_x, thrust_z, torque;
    return out;
}