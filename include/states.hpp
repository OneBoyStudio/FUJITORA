#pragma once
#include <Eigen/Dense>

struct FlightState {

    Eigen::Matrix<double, 13, 1> state; 
};

struct InputBuffer {

    double k_p; //just here to be here honestly
};