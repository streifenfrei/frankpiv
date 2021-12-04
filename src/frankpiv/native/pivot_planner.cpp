#include <thread>
#include <mutex>

#include <eigen_conversions/eigen_kdl.h>

#include "frankpiv/pivot_planner.hpp"
#include "frankpiv/utilities.hpp"

using namespace Eigen;
using namespace frankpiv::util;


namespace frankpiv::pivot_planner {
    PivotPlanner::PivotPlanner(std::shared_ptr <frankpiv::PivotFrame> pivot_frame, const std::string &base_link, const std::string &tip_link, const std::string &urdf_param, double ik_timeout, unsigned int threads) :
            pivot_frame_(pivot_frame),
            base_link(base_link),
            eef_link(tip_link),
            urdf_param(urdf_param),
            ik_timeout_(ik_timeout) {
        this->max_threads(threads);
        this->ik_solvers[0]->getKDLChain(this->chain);
        this->fk_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(this->chain);
        this->dof_ = this->chain.getNrOfJoints();
    }

    bool PivotPlanner::toPYRZ(const KDL::JntArray &joint_array, Vector4d &pyrz) {
        KDL::Frame pose_kdl;
        if (this->fk_solver->JntToCart(joint_array, pose_kdl) < 0) {
            return false;
        }
        Affine3d pose_eigen;
        tf::transformKDLToEigen(pose_kdl, pose_eigen);
        pyrz = this->pivot_frame_->getPYRZ(pose_eigen);
        return true;
    }

    bool PivotPlanner::getIKSolutions(const Vector4d &pyrz, const KDL::JntArray &seed, std::vector <KDL::JntArray> &solutions, TRAC_IK::TRAC_IK *ik_solver) {
        if (!ik_solver)
            return false;
        KDL::Frame pose;
        tf::transformEigenToKDL(this->pivot_frame_->getPose(pyrz), pose);
        KDL::JntArray dummy_out;
        if (ik_solver->CartToJnt(seed, pose, dummy_out) < 0) {
            return false;
        }
        return ik_solver->getSolutions(solutions);
    }

    void PivotPlanner::createSearchSpace(unsigned int thread_id) {
        for (size_t i = thread_id + 1; i <= this->state_count; i += this->max_threads_ik) {
            if (this->abort) {
                return;
            }
            Vector4d pyrz_state = this->start_pyrz + this->pyrz_step_size * i;
            KDL::JntArray seed;
            KDL::Multiply(joint_step_size, i, seed);
            KDL::Add(this->joint_states[0][0], seed, seed);
            if (!getIKSolutions(pyrz_state, seed, this->joint_states[i], this->ik_solvers[thread_id].get())) {
                this->abort = true;
            }
            if (thread_id == 0) {
                std::chrono::duration<double> duration = std::chrono::system_clock::now() - this->start_timestamp;
                if (duration.count() >= this->request->allowed_planning_time) {
                    this->abort = true;
                    this->response.status = Status::Timeout;
                }
            }
        }
    }

    void PivotPlanner::tsDoStepIfValid(unsigned int step, unsigned int state_index, unsigned int next_state_index, std::vector <Vector2i> &current_path, const unsigned int &thread_id) {
        if (next_state_index >= 0 && next_state_index < this->joint_states[step + 1].size()) {
            bool &connection_explored = this->connections[step](state_index, next_state_index);
            if (!connection_explored) {
                connection_explored = true;
                if ((this->joint_states[step][state_index].data - this->joint_states[step + 1][next_state_index].data).norm() < this->max_joint_state_distance_) {
                    current_path.emplace_back(step, state_index);
                    if (step + 1 == this->joint_states.size() - 1) {
                        this->solution_found = true;
                        std::lock_guard <std::mutex> lock(this->solution_mutex);
                        this->response.status = Status::Success;
                        for (auto &state : current_path) {
                            this->response.trajectory.push_back(this->joint_states[state[0]][state[1]].data);
                        }
                    } else {
                        this->tsChooseNextStep(step + 1, next_state_index, current_path, thread_id);
                    }
                }
            }
        }
    };

    void PivotPlanner::tsChooseNextStep(unsigned int step, unsigned int state_index, std::vector <Vector2i> &current_path, const unsigned int &thread_id) {
        for (size_t i = 0; i < this->joint_states[step + 1].size(); i++) {
            if (this->abort || this->solution_found)
                return;
            unsigned int next_index = (state_index + i) % this->joint_states[step + 1].size();
            this->tsDoStepIfValid(step, state_index, next_index, current_path, thread_id);
            if (thread_id == 0) {
                std::chrono::duration<double> duration = std::chrono::system_clock::now() - this->start_timestamp;
                if (duration.count() >= this->request->allowed_planning_time) {
                    this->abort = true;
                    this->response.status = Status::Timeout;
                }
            }
        }
        current_path.pop_back();
    };

    void PivotPlanner::tsStart(unsigned int start_index, const unsigned int &thread_id) {
        std::vector <Vector2i> path;
        for (size_t i = 0; i < this->joint_states[1].size(); i++) {
            if (this->abort || this->solution_found)
                return;
            unsigned int next_index = (start_index + i) % this->joint_states[1].size();
            this->tsDoStepIfValid(0, 0, next_index, path, thread_id);
        }
    }

    PivotPlanningResponse PivotPlanner::solve(const PivotPlanningRequest &request) {
        TIME_ANALYSIS(this->ik_duration = std::chrono::duration<double>();
                      this->ts_duration = std::chrono::duration<double>();)
        this->request = &request;
        this->response = PivotPlanningResponse();
        this->start_timestamp = std::chrono::system_clock::now();
        KDL::JntArray start_joint(this->dof_);
        for (size_t i = 0; i < this->dof_; i++) {
            start_joint(i) = this->request->start_state[i];
        }
        toPYRZ(start_joint, this->start_pyrz);
        const Vector4d &goal_pyrz = this->request->goal_pyrz;
        Vector4d pyrz_distances = goal_pyrz - start_pyrz;
        this->state_count = pyrz_distances.squaredNorm() / this->max_pyrz_step_size_;
        this->pyrz_step_size = pyrz_distances / (this->state_count + 1);
        this->joint_states = std::vector < std::vector < KDL::JntArray >> (this->state_count + 2);
        this->joint_states[0].push_back(std::move(start_joint)); // start_joint becomes invalid here
        TIME_ANALYSIS(std::chrono::time_point < std::chrono::system_clock > before = std::chrono::system_clock::now();)
        if (!this->getIKSolutions(goal_pyrz, this->joint_states[0][0], this->joint_states[this->state_count + 1], this->ik_solvers[0].get())) {
            TIME_ANALYSIS(this->ik_duration += std::chrono::system_clock::now() - before;)
            TIME_ANALYSIS_PRINT
            return response;
        }
        TIME_ANALYSIS(this->ik_duration += std::chrono::system_clock::now() - before;)
        KDL::Subtract(this->joint_states[this->state_count + 1][0], this->joint_states[0][0], this->joint_step_size);
        KDL::Divide(this->joint_step_size, this->state_count + 1, this->joint_step_size);
        // create search space
        this->abort = false;
        std::vector <std::thread> threads;
        TIME_ANALYSIS(before = std::chrono::system_clock::now();)
        for (size_t i = 0; i < this->max_threads_ik; i++) {
            threads.emplace_back(&PivotPlanner::createSearchSpace, this, i);
        }
        for (auto &t : threads) {
            t.join();
        }
        TIME_ANALYSIS(this->ik_duration += std::chrono::system_clock::now() - before;)
        if (this->abort) {
            TIME_ANALYSIS_PRINT
            return response;
        }
        // prepare tree search
        this->max_step = 0;
        this->solution_found = false;
        this->connections.clear();
        for (size_t i = 0; i < this->state_count + 1; i++) {
            connections.push_back(Matrix<bool, Dynamic, Dynamic>::Zero(this->joint_states[i].size(), this->joint_states[i + 1].size()));
        }
        this->response.status = Status::SearchFailed;
        unsigned int start_index_step = this->joint_states[1].size() / this->max_threads_;
        threads.clear();
        TIME_ANALYSIS(before = std::chrono::system_clock::now();)
        for (size_t i = 0; i < this->max_threads_; i++) {
            threads.emplace_back(&PivotPlanner::tsStart, this, i * start_index_step, i);
        }
        for (auto &t : threads) {
            t.join();
        }
        TIME_ANALYSIS(this->ts_duration = std::chrono::system_clock::now() - before;)
        TIME_ANALYSIS_PRINT
        return this->response;
    }

    void PivotPlanner::createIKSolvers() {
        this->ik_solvers.clear();
        for (size_t i = 0; i < this->max_threads_ik; i++) {
            this->ik_solvers.push_back(std::make_unique<TRAC_IK::TRAC_IK>(this->base_link, this->eef_link, this->urdf_param, this->ik_timeout_, PivotPlanner::ik_eps, PivotPlanner::ik_solve_type));
        }
    }
}
