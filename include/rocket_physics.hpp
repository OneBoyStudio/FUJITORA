#pragma once
#include <Eigen/Dense>

class Engine;
class RocketComponent;

class Rocket {
private:
    double mass;
    double length;
    Eigen::Matrix<double, 3, 3> inertia_tensor;
    Eigen::Vector3d CoM;

    std::vector<RocketComponent*> components;

public:
    explicit Rocket(double m, double l);

    Eigen::Matrix<double, 13, 1> compute_derivatives(const Eigen::Matrix<double, 13, 1>& state, Engine& engine, double gravity) const;

    void set_components(std::vector<RocketComponent*> new_components);

    Eigen::Vector3d control_authority(const Engine& engine) const;

    void compute_CoM();
    void compute_inertia();
    Eigen::Vector3d compute_angular_acceleration(const Eigen::Vector3d& angular_velocities, const Eigen::Vector3d& torque) const;

    Eigen::Vector4d quaternion_derivative(const Eigen::Matrix<double, 13, 1>& state) const;
    void normalize_quaternion(Eigen::Matrix<double, 13, 1>& state) const;
};

class RocketComponent {
private:
    double mass;
    Eigen::Matrix<double, 3, 3> inertia;
    Eigen::Vector3d offset;
    Eigen::Vector3d displacement;
public:
    explicit RocketComponent(double m, Eigen::Matrix<double, 3, 3> i, Eigen::Vector3d o);

    double get_mass() const;
    Eigen::Matrix<double, 3, 3> get_inertia() const;
    Eigen::Vector3d get_offset() const;
    Eigen::Vector3d get_displacement() const;

    void set_displacement(Eigen::Vector3d d);
};