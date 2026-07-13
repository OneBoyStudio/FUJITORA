#include "engine_physics.hpp"
#include "rocket_physics.hpp"

Engine::Engine(double d_z, double d_y, double t, RocketComponent& e) : delta_z(d_z), delta_y(d_y), thrust_magnitude(t), engine_component(e) {};

void Engine::limits(double limit) {
    gimbal_limit = limit;
}

double Engine::get_limit() const {
    return gimbal_limit;
}

Eigen::Vector3d Engine::get_thrust_vectors() const {
    return thrust;
}

double Engine::get_thrust_magnitude() const {
    return thrust_magnitude;
}

Eigen::Vector2d Engine::get_delta() const {
    Eigen::Vector2d out;
    out << delta_z, delta_y;
    return out;
}

void Engine::set_delta(double new_delta_z, double new_delta_y) {
    delta_z = new_delta_z;
    delta_y = new_delta_y;
}

void Engine::set_thrust_vector(Eigen::Vector3d tv) {
    thrust = tv;
}

Eigen::Vector3d Engine::thrust_vector() {

    double thrust_y = thrust_magnitude * std::sin(delta_y);
    double thrust_z = thrust_magnitude * std::sin(delta_z);

    double thrust_x = thrust_magnitude * std::cos(delta_z) * std::cos(delta_y);

    Eigen::Vector3d out;
    out << thrust_x, thrust_y, thrust_z;
    set_thrust_vector(out);
    return out;
}

Eigen::Vector3d Engine::torque_vector() const {

    Eigen::Vector3d gimbal_displacement = engine_component.get_displacement();
    Eigen::Vector3d torque = gimbal_displacement.cross(thrust);

    return torque; 
}