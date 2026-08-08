#pragma once
#include <Eigen/Dense>
#include <vector>

class RocketComponent;

class Engine {
private:
    double delta_z;
    double delta_y;
    double gimbal_limit;

    double thrust_magnitude;
    Eigen::Vector3d thrust;

    double specific_impulse;

    RocketComponent& engine_component;
public:
    explicit Engine(double d_z, double d_y, double t, double i, RocketComponent& ec);

    void limits(double limit);
    double get_limit() const;
    double get_thrust_magnitude() const;
    Eigen::Vector3d get_thrust_vectors() const;
    Eigen::Vector2d get_delta() const;

    void set_delta(double new_delta_z, double new_delta_y);
    void set_thrust_vector(Eigen::Vector3d tv);

    Eigen::Vector3d get_displacement() const;
    void set_displacement(Eigen::Vector3d CoM);

    double get_specific_impulse() const;

    Eigen::Vector3d thrust_vector(double fuel_mass);
    Eigen::Vector3d torque_vector() const; //only computes z and y components of torque (pitch and yaw)
};

class Thruster {
private:
    double thrust_magnitude;
    Eigen::Vector3d thrust = Eigen::Vector3d::Zero();

    RocketComponent& thruster_component;

    int zy_couple;
    int x_couple;
public:
    explicit Thruster(double t, RocketComponent& tc);

    Eigen::Vector3d get_offset() const;

    void set_displacement(Eigen::Vector3d);
    Eigen::Vector3d get_displacement() const;

    Eigen::Vector3d get_thrust_vector() const;

    void set_zy_couple(int thruster);
    void set_x_couple(int thruster);

    int get_zy_couple() const;
    int get_x_couple() const;

    double get_thrust_magnitude();

    void thrust_x(double input);
    void thrust_y(double input);
    void thrust_z(double input);

    Eigen::Vector3d torque_vector() const; //only computes x torque (roll)
};

class Thrusters {
private:
    std::vector<Thruster*> thrusters;
public:
    Thrusters();
    explicit Thrusters(std::vector<Thruster*> t);

    Thruster* get_thruster(int index) const;
    int thruster_count() const;

    void set_thruster_displacements(Eigen::Vector3d structure_CoM);
    void actuate_thruster(Eigen::Vector3d input);
};