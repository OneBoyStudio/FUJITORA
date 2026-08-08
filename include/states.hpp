#pragma once
#include <Eigen/Dense>
#include <random>

struct FlightState {
private:
    Eigen::Matrix<double, 13, 1> state; 
};

struct particle {
private:
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    double lifespan;
public:
    particle();
    explicit particle(Eigen::Vector3d p, Eigen::Vector3d v, double l);

    void set_position(Eigen::Vector3d pos);
    void set_velocity(Eigen::Vector3d vel);
    void set_lifespan(double l);

    Eigen::Vector3d get_position();
    double get_lifespan();

    void step(double timestep);
};

struct GPUparticle {
    float x, y, z;
    float lifespan_ratio;
};

struct particle_ring_buffer {
private:
    particle particles[100];
    int head;

    std::mt19937 rng;

    std::normal_distribution<double> pos_rand;
    std::normal_distribution<double> angle_rand;
public:
    particle_ring_buffer();
    void initialize_particle_ring_buffer();

    void instantiate(Eigen::Vector3d initial_pos, Eigen::Vector3d initial_vel, double max_life);

    void step(double timestep);

    std::vector<GPUparticle> get_active_particle_positions(int& particle_count);
};

struct InputBuffer {

    double k_p; //just here to be here honestly
};