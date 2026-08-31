#include "guidance.hpp"
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <iostream>

Guidance::Guidance(const fs::path& solver_rel_path, const fs::path& guidance_rel_dir) : solver_program(std::move(solver_rel_path)) {

    solver_program = resolve_path(solver_rel_path);

    fs::path guidance_dir = resolve_path(guidance_rel_dir);
    input_path_csv = guidance_dir / "input.csv";
    output_path_csv = guidance_dir / "trajectory_out.csv";
}

Guidance::~Guidance() {
    if (worker_future.valid()) {
        worker_future.wait();
    }
}

fs::path Guidance::resolve_path(const fs::path& rel_path) {
    if (fs::exists(rel_path)) {
        return fs::canonical(rel_path);
    }

    fs::path stepped_path = fs::path("..") / rel_path;
    if (fs::exists(stepped_path)) {
        return fs::canonical(stepped_path);
    }

    return rel_path;
}

bool Guidance::is_solver_running() const {
    return curr_solving.load();
}

bool Guidance::new_trajectory_obtained() const {
    return update_available.load();
}

void Guidance::write_csv(const Eigen::VectorXd& state) {
    if (input_path_csv.has_parent_path()) {
        fs::create_directories(input_path_csv.parent_path());
    }

    std::ofstream file(input_path_csv, std::ios::trunc);
    for (int i = 0; i < state.size(); i++) {
        file << state(i);
        if(i == state.size() - 1) {
            file << "";
        }
        else {
            file << ",";
        }
    }
    file << "\n";
    file.close();
}

std::vector<TrajectoryStateNode> Guidance::read_csv() {
    std::vector<TrajectoryStateNode> nodes;
    std::ifstream file(output_path_csv);
    if (!file.is_open()) return nodes;

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string val;
        std::vector<double> row;

        while (std::getline(ss, val, ',')) {
            row.push_back(std::stod(val));
        }

        if (row.size() >= 10) {
            TrajectoryStateNode node;
            node.t = row[0];
            node.pos = Eigen::Vector3d(row[1], row[2], row[3]);
            node.vel = Eigen::Vector3d(row[4], row[5], row[6]);
            node.target_thrust = Eigen::Vector3d(row[7], row[8], row[9]);
            nodes.push_back(node);
        }
    }
    file.close();
    return nodes;
}

void Guidance::worker(Eigen::VectorXd snapshot) {
    write_csv(snapshot);

    fs::path target_program = solver_program;
    solver_program.make_preferred();

    std::string command = "\"" + target_program.string() + "\"";

    int exit_code = std::system(command.c_str());

    if (exit_code == 0) {
        auto parsed_nodes = read_csv();
        if (!parsed_nodes.empty()) {
            std::lock_guard<std::mutex> lock(data_mutex);
            current_trajectory = std::move(parsed_nodes);
            update_available.store(true);
        }
    }

    curr_solving.store(false);
}

void Guidance::trigger_asynchronous_solve(const Eigen::VectorXd& state) {
    if (curr_solving.load()) return;

    curr_solving.store(true);
    worker_future = std::async(std::launch::async, &Guidance::worker, this, state);
}

std::vector<TrajectoryStateNode> Guidance::get_latest_trajectory() {
    std::lock_guard<std::mutex> lock(data_mutex);
    update_available.store(false);
    return current_trajectory;
}