#pragma once
#include <Eigen/Dense>
#include <vector>

class Engine;
class RocketComponent;
class RocketStructure;
class Thrusters;
class Thruster;

class Rocket {
private:
    Eigen::Matrix<double, 3, 3> inertia_tensor;
    Eigen::Vector3d CoM;

    RocketStructure& structure;
    Thrusters& rcs_thrusters; //these are approximated as massless structural components

public:
    explicit Rocket(RocketStructure& s, Thrusters& rcs);

    Eigen::Matrix<double, 13, 1> compute_derivatives(const Eigen::Matrix<double, 13, 1>& state, Engine& engine, double gravity) const;

    Eigen::Vector3d control_authority(const Engine& engine, std::vector<Thruster*> thrusters) const;

    void compute_CoM(Engine& engine);
    void compute_total_inertia();
    Eigen::Vector3d compute_angular_acceleration(const Eigen::Vector3d& angular_velocities, const Eigen::Vector3d& torque) const;

    Eigen::Vector3d get_CoM() const;

    Eigen::Vector4d quaternion_derivative(const Eigen::Matrix<double, 13, 1>& state) const;
    void normalize_quaternion(Eigen::Matrix<double, 13, 1>& state) const;

    void update_mass();
    void adjust_tank_masses(double of_ratio, double timestep, Engine engine);

    double get_of_ratio() const;
};

class RocketComponent {
private:
    double mass;
    double height;

    Eigen::Matrix<double, 3, 3> inertia;
    Eigen::Vector3d offset;
    Eigen::Vector3d displacement;

    bool hollow = false;
public:
    RocketComponent();
    RocketComponent(double m, Eigen::Vector3d o, bool h, double r);

    void set_hollow(bool h);

    void set_mass(double m);
    void set_height(double h);

    double get_height() const;
    double get_mass() const;
    Eigen::Matrix<double, 3, 3> get_inertia() const;

    void compute_inertia_solid_cylinder(double radius);
    void compute_inertia_hollow_cylinder(double radius);
    Eigen::Matrix<double, 3, 3> compute_inertia_contribution() const;

    void CoM_component(double& mass_sum, std::vector<double>& mpos);

    Eigen::Vector3d get_offset() const;
    Eigen::Vector3d get_displacement() const;

    void set_offset(Eigen::Vector3d o);
    void set_displacement(Eigen::Vector3d d);
};

class RocketStructure {
private:
    double total_mass;
    double initial_propellant_mass;

    double oxidizer_fuel_ratio;

    double oxidizer_density;
    double fuel_density;

    double radius;
    double height;

    RocketComponent payload;
    RocketComponent dry_fuselage;
    RocketComponent oxidizer_tank;
    RocketComponent fuel_tank;
    RocketComponent engine_cluster; //all engines represented as 1 large engine of cumulative mass

public:
    RocketStructure();
    RocketStructure(double initial_propellant_m, double o_f, double rho_o, double rho_f, double r, double h);

    void initialize_payload(double mass);
    void initialize_dry_fuselage(double mass);
    void initialize_engine_cluster(double engine_mass, double engine_height, int engine_count);
    
    void initialize_oxidizer_tank();
    void initialize_fuel_tank();

    Eigen::Vector3d structure_CoM();
    Eigen::Matrix<double, 3, 3> structure_inertia();

    void set_component_displacements(Eigen::Vector3d structure_CoM);

    void initialize_rocket_structure();

    double get_of_ratio() const;

    void compute_total_mass();

    double get_mass() const;
    double get_height() const;

    void update_fuel_ox_masses(double fuel_delta, double ox_delta);
    double get_fuel_mass() const;
};