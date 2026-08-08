#define _USE_MATH_DEFINES
#include "rocket_physics.hpp"
#include "engine_physics.hpp"
#include <iostream>
#include <cmath>

const double RADIAN_CONVERSION = M_PI / 180.0;

Rocket::Rocket(RocketStructure& s, Thrusters& rcs) : structure(s), rcs_thrusters(rcs) {};

Eigen::Matrix<double, 13, 1> Rocket::compute_derivatives(const Eigen::Matrix<double, 13, 1>& state, Engine& engine, double gravity) const {

    Eigen::Vector3d angular_velocity_in;
    angular_velocity_in << state(10), state(11), state(12);

    Eigen::Quaternion<double> quaternion_raw;
    quaternion_raw.w() = state(6);
    quaternion_raw.x() = state(7);
    quaternion_raw.y() = state(8);
    quaternion_raw.z() = state(9);

    Eigen::Vector3d world_thrust = quaternion_raw * engine.thrust_vector(structure.get_fuel_mass());

    Eigen::Vector3d torque = engine.torque_vector();

    for (int i = 0; i < rcs_thrusters.thruster_count(); i++) {
        torque = torque + rcs_thrusters.get_thruster(i)->torque_vector();
    }

    double mass = structure.get_mass();

    double a_x = world_thrust(0) / mass;
    double a_y = world_thrust(1) / mass;
    double a_z = (world_thrust(2) / mass) - gravity;

    Eigen::Vector3d angular_acceleration = compute_angular_acceleration(angular_velocity_in, torque);
    Eigen::Vector4d quaternion_prime = quaternion_derivative(state);

    Eigen::Matrix<double, 13, 1> state_prime;
    state_prime << state(3), state(4), state(5), a_x, a_y, a_z, quaternion_prime(0), quaternion_prime(1), quaternion_prime(2), quaternion_prime(3), angular_acceleration(0), angular_acceleration(1), angular_acceleration(2);
    return state_prime;
}

Eigen::Vector3d Rocket::control_authority(const Engine& engine, std::vector<Thruster*> thrusters) const {

    Eigen::Matrix<double, 3, 3> control_effectiveness;

    double engine_thrust = engine.get_thrust_magnitude();
    double thruster_thrust = thrusters[0]->get_thrust_magnitude();

    double engine_torque_lever = std::hypot(std::hypot(engine.get_displacement()(0), engine.get_displacement()(1)), engine.get_displacement()(2));
    double thruster_roll_torque_lever = std::hypot(std::hypot(thrusters[0]->get_displacement()(0), thrusters[0]->get_displacement()(1)), thrusters[0]->get_displacement()(2));

    control_effectiveness <<
    thruster_thrust * thruster_roll_torque_lever, 0, 0,
    0, engine_thrust * engine_torque_lever, 0,
    0, 0, engine_thrust * engine_torque_lever;

    Eigen::Matrix<double, 3, 3> B;
    B = inertia_tensor.inverse() * control_effectiveness;

    Eigen::Vector3d control_authority_vector;
    control_authority_vector << B(0, 0), B(1, 1), B(2, 2); // (must do rcs)
    return control_authority_vector;
}

void Rocket::compute_CoM(Engine& engine) {

    Eigen::Vector3d structure_CoM = structure.structure_CoM();

    CoM(0) = structure_CoM(0);
    CoM(1) = structure_CoM(1);
    CoM(2) = structure_CoM(2);

    structure.set_component_displacements(CoM);
    rcs_thrusters.set_thruster_displacements(CoM);
    engine.set_displacement(CoM);
}

void Rocket::compute_total_inertia() {

    inertia_tensor = structure.structure_inertia();
}

Eigen::Vector3d Rocket::compute_angular_acceleration(const Eigen::Vector3d& angular_velocities, const Eigen::Vector3d& torque) const{

    Eigen::Vector3d angular_momentum = inertia_tensor * angular_velocities;
    Eigen::Vector3d gyroscopic_torque = angular_velocities.cross(angular_momentum);
    Eigen::Vector3d angular_acceleration = inertia_tensor.inverse() * (torque - gyroscopic_torque);

    return angular_acceleration;
}

Eigen::Vector3d Rocket::get_CoM() const {
    return CoM;
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
    0.0, -state(10), -state(11), -state(12),
    state(10), 0.0, state(12), -state(11),
    state(11), -state(12), 0.0, state(10),
    state(12), state(11), -state(10), 0.0;

    return 0.5 * skew_sym * quaternion_in;
}

void Rocket::update_mass() {
    structure.compute_total_mass();
}

void Rocket::adjust_tank_masses(double of_ratio, double timestep, Engine engine) {
    double mass_flowrate = engine.get_thrust_magnitude() / (engine.get_specific_impulse() * 9.80665);
    double fuel_delta = (mass_flowrate * timestep) / (1.0 + of_ratio);
    double oxidizer_delta = (mass_flowrate * timestep) - fuel_delta;

    structure.update_fuel_ox_masses(fuel_delta, oxidizer_delta);
}

double Rocket::get_of_ratio() const {
    return structure.get_of_ratio();
}

// RocketComponent
RocketComponent::RocketComponent() : mass(0.0), offset(Eigen::Vector3d::Zero()) {};

RocketComponent::RocketComponent(double m, Eigen::Vector3d o, bool h, double r) : mass(m), offset(o), hollow(h) {
    if (hollow) {
        compute_inertia_hollow_cylinder(r);
    }
    else {    
        compute_inertia_solid_cylinder(r);
    }
}

void RocketComponent::set_hollow(bool h) {
    hollow = h;
}

void RocketComponent::set_mass(double m) {
    mass = m;
}

void RocketComponent::set_height(double h) {
    height = h;
}

double RocketComponent::get_height() const{ 
    return height;
}

double RocketComponent::get_mass() const {
    return mass;
}

Eigen::Matrix<double, 3, 3> RocketComponent::get_inertia() const {
    return inertia;
}

void RocketComponent::compute_inertia_solid_cylinder(double radius) {
    inertia = Eigen::Matrix3d::Zero();
    inertia(0, 0) = 0.5 * mass * (radius * radius);
    double I_transverse = (1.0 / 12.0) * mass * ((3 * radius * radius) + (height * height));
    inertia(1, 1) = I_transverse;
    inertia(2, 2) = I_transverse;
}

void RocketComponent::compute_inertia_hollow_cylinder(double radius) {
    inertia = Eigen::Matrix3d::Zero();
    inertia(0, 0) = mass * (radius * radius);
    double I_transverse = (1.0 / 12.0) * mass * ((6 * radius * radius) + (height * height));
    inertia(1, 1) = I_transverse;
    inertia(2, 2) = I_transverse;
}

Eigen::Matrix<double, 3, 3> RocketComponent::compute_inertia_contribution() const {

    Eigen::Matrix<double, 3, 3> out = Eigen::Matrix<double, 3, 3>::Zero();

    out(0, 0) = inertia(0, 0) + (mass * ((displacement(1) * displacement(1)) + (displacement(2) * displacement(2)))); 
    out(1, 1) = inertia(1, 1) + (mass * ((displacement(0) * displacement(0)) + (displacement(2) * displacement(2))));
    out(2, 2) = inertia(2, 2) + (mass * ((displacement(0) * displacement(0)) + (displacement(1) * displacement(1))));

    out(0, 1) = -1.0 * mass * displacement(0) * displacement(1);
    out(1, 0) = -1.0 * mass * displacement(0) * displacement(1);

    out(0, 2) = -1.0 * mass * displacement(0) * displacement(2);
    out(2, 0) = -1.0 * mass * displacement(0) * displacement(2);

    out(1, 2) = -1.0 * mass * displacement(1) * displacement(2);
    out(2, 1) = -1.0 * mass * displacement(1) * displacement(2);

    return out;
}

void RocketComponent::CoM_component(double& mass_sum, std::vector<double>& mpos) {
    mpos[0] += mass * offset(0);
    mpos[1] += mass * offset(1);
    mpos[2] += mass * offset(2);

    mass_sum += mass;
}

Eigen::Vector3d RocketComponent::get_offset() const {
    return offset;
}

Eigen::Vector3d RocketComponent::get_displacement() const {
    return displacement;
}

void RocketComponent::set_offset(Eigen::Vector3d o) {
    offset = o;
}

void RocketComponent::set_displacement(Eigen::Vector3d d) {
    displacement = d;
}

// RocketStructure

RocketStructure::RocketStructure()
    : initial_propellant_mass(0.0),
      oxidizer_fuel_ratio(1.0),
      oxidizer_density(0.0),
      fuel_density(0.0),
      radius(0.0),
      height(0.0)
{};

RocketStructure::RocketStructure(double initial_propellant_m, double o_f, double rho_o, double rho_f, double r, double h)
    : initial_propellant_mass(initial_propellant_m),
      oxidizer_fuel_ratio(o_f),
      oxidizer_density(rho_o),
      fuel_density(rho_f),
      radius(r),
      height(h)
{};

void RocketStructure::initialize_payload(double mass) {
    payload.set_mass(mass);
}

void RocketStructure::initialize_dry_fuselage(double mass) {
    dry_fuselage.set_height(height);
    Eigen::Vector3d offset(height / 2, 0, 0);
    dry_fuselage.set_offset(offset);
    dry_fuselage.set_mass(mass);
    dry_fuselage.compute_inertia_hollow_cylinder(radius);
    dry_fuselage.set_hollow(true);
}

void RocketStructure::initialize_engine_cluster(double engine_mass, double engine_height, int engine_count) {
    engine_cluster.set_height(engine_height);
    Eigen::Vector3d offset(engine_height / 2, 0, 0);
    engine_cluster.set_offset(offset);
    engine_cluster.set_mass(engine_mass * engine_count);
    engine_cluster.compute_inertia_solid_cylinder(radius);
}

void RocketStructure::initialize_fuel_tank() {
    fuel_tank.set_mass(initial_propellant_mass / (1.0 + oxidizer_fuel_ratio));
    fuel_tank.set_height(fuel_tank.get_mass() / (radius * radius * fuel_density * M_PI));
    Eigen::Vector3d offset((fuel_tank.get_height() / 2) + engine_cluster.get_height(), 0, 0);
    fuel_tank.set_offset(offset);
    fuel_tank.compute_inertia_solid_cylinder(radius);
}

void RocketStructure::initialize_oxidizer_tank() {
    oxidizer_tank.set_mass(initial_propellant_mass - fuel_tank.get_mass());
    oxidizer_tank.set_height(oxidizer_tank.get_mass() / (radius * radius * oxidizer_density * M_PI));
    Eigen::Vector3d offset((oxidizer_tank.get_height() / 2) + fuel_tank.get_height() + engine_cluster.get_height() + 0.5, 0, 0);
    oxidizer_tank.set_offset(offset);
    oxidizer_tank.compute_inertia_solid_cylinder(radius);
}

Eigen::Vector3d RocketStructure::structure_CoM() {
    std::vector<double> mpos = {0, 0, 0};
    double msum = 0;

    payload.CoM_component(msum, mpos);
    dry_fuselage.CoM_component(msum, mpos);
    oxidizer_tank.CoM_component(msum, mpos);
    fuel_tank.CoM_component(msum, mpos);
    engine_cluster.CoM_component(msum, mpos);

    Eigen::Vector3d out(mpos[0], mpos[1], mpos[2]);
    return out / msum;
}

Eigen::Matrix<double, 3, 3> RocketStructure::structure_inertia() {

    fuel_tank.set_height(fuel_tank.get_mass() / (radius * radius * fuel_density * M_PI));
    fuel_tank.compute_inertia_solid_cylinder(radius);

    oxidizer_tank.set_height(oxidizer_tank.get_mass() / (radius * radius * oxidizer_density * M_PI));
    oxidizer_tank.compute_inertia_solid_cylinder(radius);

    Eigen::Matrix<double, 3, 3> out = Eigen::Matrix<double, 3, 3>::Zero();

    out += payload.compute_inertia_contribution();
    out += dry_fuselage.compute_inertia_contribution();
    out += oxidizer_tank.compute_inertia_contribution();
    out += fuel_tank.compute_inertia_contribution();
    out += engine_cluster.compute_inertia_contribution();

    return out;
}

void RocketStructure::set_component_displacements(Eigen::Vector3d structure_CoM) {
    payload.set_displacement(payload.get_offset() - structure_CoM);
    dry_fuselage.set_displacement(dry_fuselage.get_offset() - structure_CoM);
    oxidizer_tank.set_displacement(oxidizer_tank.get_offset() - structure_CoM);
    fuel_tank.set_displacement(fuel_tank.get_offset() - structure_CoM);
    engine_cluster.set_displacement(engine_cluster.get_offset() - structure_CoM);
}

void RocketStructure::initialize_rocket_structure() {
    initialize_fuel_tank();
    initialize_oxidizer_tank();

    Eigen::Vector3d offset((engine_cluster.get_height() + fuel_tank.get_height() + 0.5 + oxidizer_tank.get_height() + 1.0 + height) / 2, 0, 0);
    payload.set_offset(offset);
    payload.set_height(height - (engine_cluster.get_height() + fuel_tank.get_height() + 0.5 + oxidizer_tank.get_height()) - 1.0);
    payload.compute_inertia_solid_cylinder(radius);

    compute_total_mass();
}

double RocketStructure::get_of_ratio() const {
    return oxidizer_fuel_ratio;
}

void RocketStructure::compute_total_mass() {
    total_mass = payload.get_mass() + dry_fuselage.get_mass() + oxidizer_tank.get_mass() + fuel_tank.get_mass() + engine_cluster.get_mass();
}

double RocketStructure::get_mass() const {
    return total_mass;
}

double RocketStructure::get_height() const {
    return height;
}

void RocketStructure::update_fuel_ox_masses(double fuel_delta, double ox_delta) {
    fuel_tank.set_mass(fuel_tank.get_mass() - fuel_delta);
    oxidizer_tank.set_mass(oxidizer_tank.get_mass() - ox_delta);
}

double RocketStructure::get_fuel_mass() const {
    return fuel_tank.get_mass();
}