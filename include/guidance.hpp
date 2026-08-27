#pragma once
#include <vector>
#include <string>
#include <atomic>
#include <mutex>
#include <future>
#include <Eigen/Dense>

struct TrajectoryStateNode {
    double t;
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d target_thrust;
};

class Guidance {
private:
    std::string solver_program;

    std::string input_path_csv = "guidance/input.csv";
    std::string output_path_csv = "guidance/trajectory_out.csv";

    std::atomic<bool> curr_solving{false};
    std::atomic<bool> update_available{false};

    std::future<void> worker_future;
    std::mutex data_mutex;
    std::vector<TrajectoryStateNode> current_trajectory;

    void write_csv(const Eigen::VectorXd& state);
    std::vector<TrajectoryStateNode> read_csv();
    void worker(Eigen::VectorXd snapshot);
public:
    Guidance(std::string solver_path);
    ~Guidance();

    void trigger_asynchronous_solve(const Eigen::VectorXd& state);
    bool is_solver_running() const;
    bool new_trajectory_obtained() const;
    std::vector<TrajectoryStateNode> get_latest_trajectory();
};