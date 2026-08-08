#pragma once
#include "rocket_physics.hpp"
#include "engine_physics.hpp"
#include <Eigen/Dense>

class Thruster;

class IntegratorRK4 {
private:
    double dt;
public:
    explicit IntegratorRK4(double timestep);

    Eigen::Matrix<double, 13, 1> step(const Eigen::Matrix<double, 13, 1>& state, Rocket& rocket, Engine& engine) const;
};