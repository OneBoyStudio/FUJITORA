#pragma once
#include <vector>
#include <string>
#include <atomic>
#include <mutex>
#include <future>
#include <Eigen/Dense>
#include <filesystem>

namespace fs = std::filesystem;

struct TrajectoryStateNode {
    double t;
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d target_thrust;
};

class Guidance {
private:
    fs::path solver_program;

    fs::path input_path_csv;
    fs::path output_path_csv;

    std::atomic<bool> curr_solving{false};
    std::atomic<bool> update_available{false};

    std::future<void> worker_future;
    std::mutex data_mutex;
    std::vector<TrajectoryStateNode> current_trajectory;

    fs::path resolve_path(const fs::path& rel_path);
    void write_csv(const Eigen::VectorXd& state);
    std::vector<TrajectoryStateNode> read_csv();
    void worker(Eigen::VectorXd snapshot);
public:
    Guidance(const fs::path& solver_rel_path, const fs::path& guidance_rel_dir);
    ~Guidance();

    void trigger_asynchronous_solve(const Eigen::VectorXd& state);
    bool is_solver_running() const;
    bool new_trajectory_obtained() const;
    std::vector<TrajectoryStateNode> get_latest_trajectory();
};