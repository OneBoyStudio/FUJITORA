#pragma once
#include <Eigen/Dense>

class RocketComponent;

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
