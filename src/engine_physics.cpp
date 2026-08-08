#include "engine_physics.hpp"
#include "rocket_physics.hpp"
#include <set>
#include <iostream>

Engine::Engine(double d_z, double d_y, double t, double i, RocketComponent& ec)
    : delta_z(d_z),
      delta_y(d_y),
      thrust_magnitude(t),
      specific_impulse(i),
      engine_component(ec)
{};

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
    delta_y = new_delta_y;
    delta_z = new_delta_z;
}

Eigen::Vector3d Engine::get_displacement() const {
    return engine_component.get_displacement();
}

void Engine::set_displacement(Eigen::Vector3d CoM) {
    engine_component.set_displacement(engine_component.get_offset() - CoM);
}

double Engine::get_specific_impulse() const {
    return specific_impulse;
}

void Engine::set_thrust_vector(Eigen::Vector3d tv) {
    thrust = tv;
}

Eigen::Vector3d Engine::thrust_vector(double fuel_mass) {
    
    Eigen::Vector3d out;
    if (fuel_mass > 0.0) {
        double thrust_y = thrust_magnitude * std::sin(delta_y);
        double thrust_z = thrust_magnitude * std::sin(delta_z);

        double thrust_x = thrust_magnitude * std::cos(delta_z) * std::cos(delta_y);

        out << thrust_x, thrust_y, thrust_z;
        set_thrust_vector(out);
    }
    else {
        out << 0.0, 0.0, 0.0;
    }
    
    return out;
}

Eigen::Vector3d Engine::torque_vector() const {

    Eigen::Vector3d gimbal_displacement = engine_component.get_displacement();
    Eigen::Vector3d torque = gimbal_displacement.cross(thrust);

    return torque; 
}

// thruster

Thruster::Thruster(double t, RocketComponent& tc) : thrust_magnitude(t), thruster_component(tc) {};

Eigen::Vector3d Thruster::get_offset() const {
    return thruster_component.get_offset();
}

void Thruster::thrust_x(double input) {

    if (input > 0) {
        thrust(0) = thrust_magnitude;
    }
    else if (input == 0) {
        thrust(0) = 0;
    }
    else if (input < 0) {
        thrust(0) = -thrust_magnitude;
    }
}

void Thruster::thrust_y(double input) {

    if (input > 0) {
        thrust(1) = thrust_magnitude;
    }
    else if (input == 0)  {
        thrust(1) = 0;
    }
    else if (input < 0) {
        thrust(1) = -thrust_magnitude;
    }
}

void Thruster::thrust_z(double input) {

    if (input > 0) {
        thrust(2) = thrust_magnitude;
    }
    else if (input == 0)  {
        thrust(2) = 0;
    }
    else if (input < 0) {
        thrust(2) = -thrust_magnitude;
    }
}

double Thruster::get_thrust_magnitude() {
    return thrust_magnitude;
}

Eigen::Vector3d Thruster::torque_vector() const {

    Eigen::Vector3d thruster_displacement = thruster_component.get_displacement();
    Eigen::Vector3d torque = thruster_displacement.cross(thrust);

    return torque; 
}

Eigen::Vector3d Thruster::get_displacement() const {
    return thruster_component.get_displacement();
}

Eigen::Vector3d Thruster::get_thrust_vector() const {
    return thrust;
}

void Thruster::set_displacement(Eigen::Vector3d displacement) {
    thruster_component.set_displacement(displacement);
}

void Thruster::set_zy_couple(int thruster) {
    zy_couple = thruster;
}

void Thruster::set_x_couple(int thruster) {
    x_couple = thruster;
}


int Thruster::get_zy_couple() const {
    return zy_couple;
}

int Thruster::get_x_couple() const {
    return x_couple;
}

// Thrusters

Thrusters::Thrusters() : thrusters() {};
Thrusters::Thrusters(std::vector<Thruster*> t) : thrusters(t) {};

void Thrusters::set_thruster_displacements(Eigen::Vector3d structure_CoM) {
    for (int i = 0; i < thrusters.size(); i++) {
        thrusters[i]->set_displacement(thrusters[i]->get_offset() - structure_CoM);
    }
}

void Thrusters::actuate_thruster(Eigen::Vector3d input) {

    // completmentary thrusters that are accounted for by the first thruster in the pair
    std::set<int> zy_accounted;
    std::set<int> x_accounted;

    double thrust_magnitude = thrusters[0]->get_thrust_magnitude();

    for (int i = 0; i < thrusters.size(); i++) {

        double x;
        double y;
        double z;

        int x_couple = thrusters[i]->get_x_couple() - 1;
        int zy_couple = thrusters[i]->get_zy_couple() - 1;

        Eigen::Vector3d thrust_vector = thrusters[i]->get_thrust_vector();

        if (x_accounted.find(i) == x_accounted.end()){
 
            if (std::abs(thrust_vector(0)) > 0) {
                if (std::abs(input(0)) < 0.04) {
                    x = 0; // we want this to represent roll torque about the x axis but it maps to y and z thrust (choose z for convenience)

                    thrusters[i]->thrust_z(x);
                    thrusters[x_couple]->thrust_z(x);
                    x_accounted.insert(x_couple);
                }
            }
            else {
                if (std::abs(input(0)) > 0.1) {
                    x = thrust_magnitude; // again choose z for convenience

                    thrusters[i]->thrust_z(x);
                    thrusters[x_couple]->thrust_z(x);
                    x_accounted.insert(x_couple);
                }
            }            
        }

        if (zy_accounted.find(i) == zy_accounted.end()) {

            if (std::abs(thrust_vector(1)) > 0) {
                if (std::abs(input(1)) < 0.04) {
                    y = 0; // choose x

                    thrusters[i]->thrust_x(y);
                    thrusters[zy_couple]->thrust_x(y);
                    zy_accounted.insert(zy_couple);
                }
            }
            else {
                if (std::abs(input(1)) > 0.1) {
                    y = thrust_magnitude; // x

                    thrusters[i]->thrust_x(y);
                    thrusters[zy_couple]->thrust_x(y);
                    zy_accounted.insert(zy_couple);
                }
            } 

            if (std::abs(thrust_vector(2)) > 0) {
                if (std::abs(input(2)) < 0.04) {
                    z = 0; // y

                    thrusters[i]->thrust_y(z);
                    thrusters[zy_couple]->thrust_y(z);
                    zy_accounted.insert(zy_couple);
                }
            }
            else {
                if (std::abs(input(2)) > 0.1) {
                    z = thrust_magnitude; // y

                    thrusters[i]->thrust_y(z);
                    thrusters[zy_couple]->thrust_y(z);
                    zy_accounted.insert(zy_couple);
                }
            }
        }
    } // 0.04 is lower bound for closing while 0.1 is upper bound for opening
}

Thruster* Thrusters::get_thruster(int index) const {
    return thrusters[index];
}

int Thrusters::thruster_count() const {
    return thrusters.size();
}