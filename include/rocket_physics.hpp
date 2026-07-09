#pragma once
#include <Eigen/Dense>

class Engine;

class Rocket {
private:
    double mass;
    double length;
    double inertial_moment;
    double CoM;
public:
    explicit Rocket(double m, double l, double i, double CoM);

    Eigen::Matrix<double, 6, 1> compute_derivatives(const Eigen::Matrix<double, 6, 1>& state, const Engine& engine, double gravity) const;
    double control_authority(const Engine& engine) const;
};

class Engine {
private:
    double delta;
    double thrust;
    double gimbal_limit;
public:
    explicit Engine(double d, double t);

    void limits(double limit);
    double get_limit() const;
    double get_thrust() const;
    double get_delta() const;

    void set_delta(double new_delta);

    Eigen::Vector3d thrust_vectors(const Eigen::Matrix<double, 6, 1>& state, double length) const; //thrust and torque vector
};