#include "states.hpp"
#include <random>

particle::particle() {
    Eigen::Vector3d empty(0, 0, 0);
    position = empty;
    velocity = empty;
    lifespan = 0;
}

particle::particle(Eigen::Vector3d p, Eigen::Vector3d v, double l) : position(p), velocity(v), lifespan(l) {};

void particle::set_position(Eigen::Vector3d pos){
    position = pos;
}

void particle::set_velocity(Eigen::Vector3d vel){
    velocity = vel;
}

void particle::set_lifespan(double l){
    lifespan = l;
}

Eigen::Vector3d particle::get_position() {
    return position;
}

double particle::get_lifespan() {
    return lifespan;
}

void particle::step(double timestep) {
    position += timestep * velocity;
    lifespan -= timestep;
}

// particle ring buffer

particle_ring_buffer::particle_ring_buffer() {
    std::random_device rd;
    rng.seed(rd());
}

void particle_ring_buffer::initialize_particle_ring_buffer() {
    pos_rand.param(std::normal_distribution<double>::param_type(0.0, 0.5));
    angle_rand.param(std::normal_distribution<double>::param_type(0.0, 0.524));
}

void particle_ring_buffer::instantiate(Eigen::Vector3d initial_pos, Eigen::Vector3d initial_vel, double max_life) {

    Eigen::Vector3d pos(0, 0, 0);

    pos(0) = initial_pos(0) + std::max(-0.5, std::min(pos_rand(rng), 0.5));
    pos(1) = initial_pos(1) + std::max(-0.5, std::min(pos_rand(rng), 0.5));
    pos(2) = initial_pos(2); //+ std::max(-0.5, std::min(pos_rand(rng), 0.5));

    Eigen::Vector3d vel(0, 0, 0);

    double rot_angle = std::max(-0.524, std::min(angle_rand(rng), 0.524)) / 2.0;
    Eigen::Quaterniond rotation(1, std::sin(rot_angle) * initial_vel);
    rotation.normalize();

    vel = rotation * initial_vel;

    particles[head] = particle(pos, vel, max_life);
    head++;
    if (head == 100) head = 0;
}

void particle_ring_buffer::step(double timestep){
    for (int i = 0; i < 100; i++) {
        particles[i].step(timestep);
    }
}

std::vector<GPUparticle> particle_ring_buffer::get_active_particle_positions(int& particle_count) {
    std::vector<GPUparticle> active_particle_positions;
    active_particle_positions.reserve(100);

    particle_count = 0;

    for (; particle_count < 100; particle_count++) {
        if (particles[particle_count].get_lifespan() > 0.0) {
            Eigen::Vector3d pos = particles[particle_count].get_position();
            float lifespan = (float)particles[particle_count].get_lifespan();

            GPUparticle new_particle;
            new_particle.x = (float)pos(0);
            new_particle.y = (float)pos(1);
            new_particle.z = (float)pos(2);
            new_particle.lifespan_ratio = lifespan / 0.5;

            active_particle_positions.push_back(new_particle);
        }
    }

    return active_particle_positions;
}