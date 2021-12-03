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
        }
    }

    void PivotPlanner::tsDoStepIfValid(unsigned int step, unsigned int state_index, unsigned int next_state_index, std::vector <Vector2i> &current_path) {
        if (next_state_index >= 0 && next_state_index < this->joint_states[step + 1].size()) {
            int &connection_status = this->connections[step](state_index, next_state_index);
            if (connection_status == -1) {
                this->connections[step](state_index, next_state_index) = (this->joint_states[step][state_index].data - this->joint_states[step + 1][next_state_index].data).norm() < this->max_joint_state_distance_ ? 1 : 0;
            }
            if (connection_status == 1) {
                current_path.emplace_back(step, state_index);
                if (step + 1 == this->joint_states.size() - 1) {
                    this->solution_found = true;
                    std::lock_guard <std::mutex> lock(this->solution_mutex);
                    this->solution = current_path;
                } else {
                    this->tsChooseNextStep(step + 1, next_state_index, current_path);
                }
            }
        }
    };

    void PivotPlanner::tsChooseNextStep(unsigned int step, unsigned int state_index, std::vector <Vector2i> &current_path) {
        for (size_t i = 0; i < this->joint_states[step + 1].size(); i++) {
            if (this->solution_found)
                return;
            unsigned int next_index = (state_index + i) % this->joint_states[step + 1].size();
            this->tsDoStepIfValid(step, state_index, next_index, current_path);
        }
        current_path.pop_back();
    };

    void PivotPlanner::tsStart(unsigned int start_index) {
        std::vector <Vector2i> path;
        for (size_t i = 0; i < this->joint_states[1].size(); i++) {
            if (this->solution_found)
                return;
            unsigned int next_index = (start_index + i) % this->joint_states[1].size();
            this->tsDoStepIfValid(0, 0, next_index, path);
        }
    }

    PivotPlanningResponse PivotPlanner::solve(const PivotPlanningRequest &request) {
        PivotPlanningResponse response;
        KDL::JntArray start_joint(this->dof_);
        for (size_t i = 0; i < this->dof_; i++) {
            start_joint(i) = request.start_state[i];
        }
        toPYRZ(start_joint, this->start_pyrz);
        const Vector4d &goal_pyrz = request.goal_pyrz;
        Vector4d pyrz_distances = goal_pyrz - start_pyrz;
        this->state_count = pyrz_distances.squaredNorm() / this->max_pyrz_step_size_;
        this->pyrz_step_size = pyrz_distances / (this->state_count + 1);
        this->joint_states = std::vector < std::vector < KDL::JntArray >> (this->state_count + 2);
        this->joint_states[0].push_back(std::move(start_joint)); // start_joint becomes invalid here
        if (!this->getIKSolutions(goal_pyrz, this->joint_states[0][0], this->joint_states[this->state_count + 1], this->ik_solvers[0].get())) {
            return response;
        }
        KDL::Subtract(this->joint_states[this->state_count + 1][0], this->joint_states[0][0], this->joint_step_size);
        KDL::Divide(this->joint_step_size, this->state_count + 1, this->joint_step_size);
        // create search space
        this->abort = false;
        std::vector <std::thread> threads;
        for (size_t i = 0; i < this->max_threads_ik; i++) {
            threads.emplace_back(&PivotPlanner::createSearchSpace, this, i);
        }
        for (auto &t : threads) {
            t.join();
        }
        if (this->abort) {
            return response;
        }
        // prepare tree search
        this->max_step = 0;
        this->solution_found = false;
        this->connections.clear();
        for (size_t i = 0; i < this->state_count + 1; i++) {
            connections.push_back(MatrixXi::Constant(this->joint_states[i].size(), this->joint_states[i + 1].size(), -1));
        }
        unsigned int start_index_step = this->joint_states[1].size() / this->max_threads_;
        threads.clear();
        for (size_t i = 0; i < this->max_threads_; i++) {
            threads.emplace_back(&PivotPlanner::tsStart, this, i * start_index_step);
        }
        for (auto &t : threads) {
            t.join();
        }
        if (solution_found) {
            for (auto &state : solution) {
                response.trajectory.push_back(this->joint_states[state[0]][state[1]].data);
            }
            response.status = Status::Success;
        } else {
            response.status = Status::SearchFailed;
        }
        return response;
    }

    void PivotPlanner::createIKSolvers() {
        this->ik_solvers.clear();
        for (size_t i = 0; i < this->max_threads_ik; i++) {
            this->ik_solvers.push_back(std::make_unique<TRAC_IK::TRAC_IK>(this->base_link, this->eef_link, this->urdf_param, this->ik_timeout_, PivotPlanner::ik_eps, PivotPlanner::ik_solve_type));
        }
    }
}
