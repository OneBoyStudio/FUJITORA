#include "rocket_physics.hpp"
#include "engine_physics.hpp"
#include <iostream>
#include <cmath>

Rocket::Rocket(double m, double l) : mass(m), length(l) {};

Eigen::Matrix<double, 13, 1> Rocket::compute_derivatives(const Eigen::Matrix<double, 13, 1>& state, Engine& engine, double gravity) const {

    Eigen::Vector3d angular_velocity_in;
    angular_velocity_in << state(10), state(11), state(12);

    Eigen::Quaternion<double> quaternion_raw;
    quaternion_raw.w() = state(6);
    quaternion_raw.x() = state(7);
    quaternion_raw.y() = state(8);
    quaternion_raw.z() = state(9);

    Eigen::Vector3d raw_thrust = engine.thrust_vector();
    Eigen::Vector3d world_thrust = quaternion_raw * engine.thrust_vector();
    
    Eigen::Vector3d torque = engine.torque_vector();

    double a_x = world_thrust(0) / mass;
    double a_y = world_thrust(1) / mass;
    double a_z = (world_thrust(2) / mass) - gravity;

    Eigen::Vector3d angular_acceleration = compute_angular_acceleration(angular_velocity_in, torque);
    Eigen::Vector4d quaternion_prime = quaternion_derivative(state);

    Eigen::Matrix<double, 13, 1> state_prime;
    state_prime << state(3), state(4), state(5), a_x, a_y, a_z, quaternion_prime(0), quaternion_prime(1), quaternion_prime(2), quaternion_prime(3), angular_acceleration(0), angular_acceleration(1), angular_acceleration(2);
    return state_prime;
}

void Rocket::set_components(std::vector<RocketComponent*> new_components) {
    components = new_components;
    compute_CoM();
    compute_inertia();
}

Eigen::Vector3d Rocket::control_authority(const Engine& engine) const {

    Eigen::Matrix<double, 3, 3> control_effectiveness;

    double thrust = engine.get_thrust_magnitude();

    double torque_lever = std::hypot(std::hypot(engine.get_displacement()(0), engine.get_displacement()(1)), engine.get_displacement()(2));

    control_effectiveness <<
    thrust * torque_lever, 0, 0,
    0, thrust * torque_lever, 0,
    0, 0, thrust * torque_lever;

    Eigen::Matrix<double, 3, 3> B;
    B = inertia_tensor.inverse() * control_effectiveness;

    Eigen::Vector3d control_authority_vector;
    control_authority_vector << B(0, 0), B(1, 1), B(2, 2); // (must do rcs)
    return control_authority_vector;
}

void Rocket::compute_CoM() {

    double mpos[3] = {0,0,0};
    double msum = 0;

    for (const RocketComponent* component : components) {
        
        double mass = (*component).get_mass();
        Eigen::Vector3d offset = (*component).get_offset();

        mpos[0] += mass * offset(0);
        mpos[1] += mass * offset(1);
        mpos[2] += mass * offset(2);

        msum += mass;
    }

    CoM(0) = mpos[0] / msum;
    CoM(1) = mpos[1] / msum;
    CoM(2) = mpos[2] / msum;

    for (RocketComponent* component : components) {
        
        Eigen::Vector3d displacement = (*component).get_offset() - CoM;
        (*component).set_displacement(displacement);
    }
}

void Rocket::compute_inertia() {

    double inertia_diagonal[3] = {0,0,0};
    double inertia_off_diagonal[3] = {0,0,0}; //xy xz yz

    for (RocketComponent* component : components) {

        double mass = (*component).get_mass();
        Eigen::Vector3d displacement = (*component).get_displacement();
        Eigen::Matrix<double, 3, 3> comp_inertia = (*component).get_inertia();

        inertia_diagonal[0] += comp_inertia(0, 0) + (mass * ((displacement(1) * displacement(1)) + (displacement(2) * displacement(2)))); 
        inertia_diagonal[1] += comp_inertia(1, 1) + (mass * ((displacement(0) * displacement(0)) + (displacement(2) * displacement(2))));
        inertia_diagonal[2] += comp_inertia(2, 2) + (mass * ((displacement(0) * displacement(0)) + (displacement(1) * displacement(1))));

        inertia_off_diagonal[0] += -1.0 * mass * displacement(0) * displacement(1);
        inertia_off_diagonal[1] += -1.0 * mass * displacement(0) * displacement(2);
        inertia_off_diagonal[2] += -1.0 * mass * displacement(1) * displacement(2);
    }

    inertia_tensor << inertia_diagonal[0], inertia_off_diagonal[0], inertia_off_diagonal[1],
    inertia_off_diagonal[0], inertia_diagonal[1], inertia_off_diagonal[2],
    inertia_off_diagonal[1], inertia_off_diagonal[2], inertia_diagonal[2];
}

Eigen::Vector3d Rocket::compute_angular_acceleration(const Eigen::Vector3d& angular_velocities, const Eigen::Vector3d& torque) const{

    Eigen::Vector3d angular_momentum = inertia_tensor * angular_velocities;
    Eigen::Vector3d gyroscopic_torque = angular_velocities.cross(angular_momentum);
    Eigen::Vector3d angular_acceleration = inertia_tensor.inverse() * (torque - gyroscopic_torque);

    return angular_acceleration;
}

void Rocket::normalize_quaternion(Eigen::Matrix<double, 13, 1>& state) const {
    Eigen::Quaternion<double> raw_quaternion(state(6), state(7), state(8), state(9));
    raw_quaternion.normalize();

    state(6) = raw_quaternion.w();
    state(7) = raw_quaternion.x();
    state(8) = raw_quaternion.y();
    state(9) = raw_quaternion.z();
}

Eigen::Vector4d Rocket::quaternion_derivative(const Eigen::Matrix<double, 13, 1>& state) const {

    Eigen::Vector4d quaternion_in;
    quaternion_in << state(6), state(7), state(8), state(9);

    Eigen::Matrix<double, 4, 4> skew_sym;
    skew_sym << 
    0.0, -1.0 * state(10), -1.0 * state(11), -1.0 * state(12),
    state(10), 0.0, state(12), -1.0 * state(11),
    state(11), -1.0 * state(12), 0.0, state(10),
    state(12), -1.0 * state(11), state(10), 0.0;

    return 0.5 * skew_sym * quaternion_in;
}

// RocketComponent

RocketComponent::RocketComponent(double m, Eigen::Matrix<double, 3, 3> i, Eigen::Vector3d o) : mass(m), inertia(i), offset(o) {};

double RocketComponent::get_mass() const {
    return mass;
}

Eigen::Matrix<double, 3, 3> RocketComponent::get_inertia() const {
    return inertia;
}

Eigen::Vector3d RocketComponent::get_offset() const {
    return offset;
}

Eigen::Vector3d RocketComponent::get_displacement() const {
    return displacement;
}

void RocketComponent::set_displacement(Eigen::Vector3d d) {
    displacement = d;
}