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

class Engine {
private:
    double delta_z;
    double delta_y;
    double gimbal_limit;

    double thrust_magnitude;
    Eigen::Vector3d thrust;

    RocketComponent& engine_component;
public:
    explicit Engine(double d_z, double d_y, double t, RocketComponent& e);

    void limits(double limit);
    double get_limit() const;
    double get_thrust_magnitude() const;
    Eigen::Vector3d get_thrust_vectors() const;
    Eigen::Vector2d get_delta() const;

    void set_delta(double new_delta_z, double new_delta_y);
    void set_thrust_vector(Eigen::Vector3d tv);

    Eigen::Vector3d thrust_vector();
    Eigen::Vector3d torque_vector() const; //only computes z and y components of torque
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