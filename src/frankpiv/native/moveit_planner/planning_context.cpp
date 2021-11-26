#include <thread>
#include <mutex>
#include <chrono>

#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include "frankpiv/moveit_planner/planning_context.hpp"
#include "frankpiv/moveit_planner/state_sampler.hpp"
#include "frankpiv/moveit_planner/validity_checker.hpp"
#include "frankpiv/utilities.hpp"

using namespace Eigen;
using namespace frankpiv::util;


namespace frankpiv::moveit_planner {
    PivotPlanningContext::PivotPlanningContext(const std::string &name, const robot_model::RobotModelConstPtr &robot_model, const std::string &group_name, std::shared_ptr <frankpiv::PivotFrame> pivot_frame) :
            PlanningContext(name, group_name),
            robot_model(robot_model),
            group_name(group_name),
            simple_setup(std::make_shared<og::SimpleSetup>(initStateSpace())),
            pivot_frame_(pivot_frame),
            planner_data(simple_setup->getSpaceInformation()) {
        this->simple_setup->setStateValidityChecker(std::make_shared<PivotStateValidityChecker>(this->simple_setup->getSpaceInformation(),
                                                                                                this->robot_model,
                                                                                                this->group_name,
                                                                                                planning_scene_,
                                                                                                this->pivot_frame_));
        this->simple_setup->getSpaceInformation()->setStateValidityCheckingResolution(0.03); // default is 0.01
        this->simple_setup->setPlanner(std::make_shared<og::PRM>(this->planner_data));
    }

    std::shared_ptr <ob::RealVectorStateSpace> PivotPlanningContext::initStateSpace() const {
        unsigned int dof = this->robot_model->getJointModelGroup(this->group_name)->getVariableCount();
        auto state_space = std::make_shared<ob::RealVectorStateSpace>(dof);
        ob::RealVectorBounds bounds(dof);
        bounds.setLow(-2 * M_PI);
        bounds.setHigh(2 * M_PI);
        state_space->setBounds(bounds);
        return state_space;
    }

    void PivotPlanningContext::clear() {}

    bool PivotPlanningContext::solve(planning_interface::MotionPlanResponse &res) {
        unsigned int dof = this->robot_model->getJointModelGroup(this->group_name)->getVariableCount();
        std::vector<double> goal_joint_positions(dof);
        for (size_t i = 0; i < dof; i++) {
            goal_joint_positions[i] = this->request_.goal_constraints[0].joint_constraints[i].position;
        }
        ob::ScopedState <ob::RealVectorStateSpace> start_state(this->simple_setup->getStateSpace());
        ob::ScopedState <ob::RealVectorStateSpace> goal_state(this->simple_setup->getStateSpace());
        start_state = this->request_.start_state.joint_state.position;
        goal_state = goal_joint_positions;
        this->simple_setup->setStartAndGoalStates(start_state, goal_state);
        ob::ValidStateSamplerAllocator pivot_state_sampler_allocator = [this](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
            return std::make_shared<PivotStateSampler>(si, this->robot_model, this->group_name, this->pivot_frame_);
        };
        this->simple_setup->getSpaceInformation()->setValidStateSamplerAllocator(pivot_state_sampler_allocator);
        bool success = this->simple_setup->solve(this->request_.allowed_planning_time) == ob::PlannerStatus::EXACT_SOLUTION;
        if (success) {
            this->simple_setup->simplifySolution();
            auto path = this->simple_setup->getSolutionPath();
            path.interpolate();
            res.trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(this->robot_model, this->group_name);
            for (std::size_t path_i = 0; path_i < path.getStateCount(); path_i++) {
                ob::RealVectorStateSpace::StateType &ompl_state = *path.getState(path_i)->as<ob::RealVectorStateSpace::StateType>();
                auto moveit_state = std::make_shared<moveit::core::RobotState>(toMoveitState(ompl_state, this->robot_model, this->group_name));
                res.trajectory_->addSuffixWayPoint(moveit_state, 0.0);
            }
            res.planning_time_ = this->simple_setup->getLastPlanComputationTime();
        }
        return success;
    }

    bool PivotPlanningContext::solve(planning_interface::MotionPlanDetailedResponse &res) { return false; }

    bool PivotPlanningContext::terminate() { return true; }

    void PivotPlanningContext::pivot_frame(std::shared_ptr <frankpiv::PivotFrame> pivot_frame) {
        this->pivot_frame_ = pivot_frame;
        this->simple_setup->setStateValidityChecker(std::make_shared<PivotStateValidityChecker>(this->simple_setup->getSpaceInformation(),
                                                                                                this->robot_model,
                                                                                                this->group_name,
                                                                                                planning_scene_,
                                                                                                this->pivot_frame_));
    }

    void PivotPlanningContext::preSample(double angle_resolution, double z_translation_resolution, int thread_count) {
        // TODO calculate optimal resolution parameters from validity_checking_resolution
        // pyrz boundaries and discretization parameters for the sampling loop
        size_t discrete_angle_steps = 2 * this->pivot_frame_->max_angle() / angle_resolution;
        auto roll_boundaries = this->pivot_frame_->roll_boundaries();
        double roll_range = roll_boundaries(1) - roll_boundaries(0);
        size_t discrete_roll_steps = roll_range / angle_resolution;
        auto z_translation_boundaries = this->pivot_frame_->z_translation_boundaries();
        double z_translation_range = z_translation_boundaries(1) - z_translation_boundaries(0);
        size_t discrete_translation_steps = z_translation_range / z_translation_resolution;
        // locks
        std::mutex m1;
        std::mutex m2;
        std::mutex m3;
        // logging related
        size_t total_iterations = discrete_angle_steps * discrete_angle_steps * discrete_angle_steps * discrete_translation_steps;
        size_t current_iteration = 0;
        // data structure to store vertices
        std::vector < std::vector < std::vector < std::vector < ob::PlannerDataVertex >> >> vertices(discrete_angle_steps,
                                                                                                     std::vector < std::vector < std::vector < ob::PlannerDataVertex >>> (discrete_angle_steps,
                std::vector < std::vector < ob::PlannerDataVertex >> (discrete_translation_steps,
                        std::vector<ob::PlannerDataVertex>())));
        // striding loop to sample valid states
        auto sample_vertices = [&, this](int thread_id) {
            size_t index = 0;
            for (size_t pitch_step = 0; pitch_step < discrete_angle_steps; pitch_step++) {
                double pitch = -this->pivot_frame_->max_angle() + angle_resolution * pitch_step;
                for (size_t yaw_step = 0; yaw_step < discrete_angle_steps; yaw_step++) {
                    double yaw = -this->pivot_frame_->max_angle() + angle_resolution * yaw_step;
                    for (size_t roll_step = 0; roll_step < discrete_angle_steps; roll_step++) {
                        double roll = roll_boundaries(0) + angle_resolution * roll_step;
                        for (size_t translation_step = 0; translation_step < discrete_translation_steps; translation_step++) {
                            if (index == thread_id) {
                                double z_translation = z_translation_boundaries(0) + (z_translation_resolution * translation_step);
                                Vector4d pyrz{pitch, yaw, roll, z_translation};
                                if (!this->pivot_frame_->clip(pyrz)) {
                                    auto eef_pose = this->pivot_frame_->getPose(pyrz);
                                    robot_state::RobotState state(this->robot_model);
                                    const moveit::core::JointModelGroup *joint_model_group = this->robot_model->getJointModelGroup(this->group_name);
                                    if (state.setFromIK(joint_model_group, toPoseMsg(eef_pose))) {
                                        auto state_ompl = this->simple_setup->getSpaceInformation()->allocState()->as<ob::RealVectorStateSpace::StateType>();
                                        size_t joint_i = 0;
                                        for (const moveit::core::JointModel *joint_model : joint_model_group->getActiveJointModels()) {
                                            (*state_ompl)[joint_i++] = state.getVariablePosition(joint_model->getFirstVariableIndex());
                                        }
                                        m1.lock();
                                        vertices[pitch_step][yaw_step][translation_step].emplace_back(state_ompl);
                                        this->planner_data.addVertex(vertices[pitch_step][yaw_step][translation_step].back());
                                        m1.unlock();
                                    }
                                }
                                // logging
                                m2.lock();
                                std::cout << "\r" << (int) (100 * (++current_iteration / (double) total_iterations)) << "%      ";
                                std::flush(std::cout);
                                m2.unlock();
                            }
                            index = ++index % thread_count;
                        }
                    }
                }
            }
        };
        ROS_INFO_STREAM("Sampling states...");
        std::vector <std::thread> threads;
        for (size_t i = 0; i < thread_count; i++) {
            threads.emplace_back(sample_vertices, i);
        }
        for (auto &t : threads) {
            t.join();
        }
        std::cout << "\r";

        std::vector < std::vector < std::vector < bool > >> processed_pyz(discrete_angle_steps, std::vector < std::vector < bool >> (discrete_angle_steps, std::vector<bool>(discrete_translation_steps, false)));
        auto space_information = this->simple_setup->getSpaceInformation();
        double max_joint_space_distance = space_information->getStateValidityCheckingResolution() * space_information->getMaximumExtent();
        total_iterations = discrete_angle_steps * discrete_angle_steps * discrete_translation_steps;
        current_iteration = 0;
        // striding loop to connect neighbouring states
        auto connect_vertices = [&, this](int thread_id) {
            size_t index = 0;
            auto last_timestamp = std::chrono::system_clock::now();
            size_t last_timestamp_iteration = 0;
            // iterate over all pyz
            for (size_t i = 0; i < discrete_angle_steps; i++) {
                for (size_t j = 0; j < discrete_angle_steps; j++) {
                    for (size_t k = 0; k < discrete_translation_steps; k++) {
                        if (index == thread_id) {
                            // connect current vertices with vertices of adjacent pyz
                            for (size_t i_ = std::max(i - 1, (size_t)0); i_ < std::min(i + 2, discrete_angle_steps); i_++) {
                                for (size_t j_ = std::max(j - 1, (size_t)0); j_ < std::min(j + 2, discrete_angle_steps); j_++) {
                                    for (size_t k_ = std::max(k - 1, (size_t)0); k_ < std::min(k + 2, discrete_translation_steps); k_++) {
                                        if ((i_ == i && j_ == j && k_ == k) || processed_pyz[i_][j_][k_]) {
                                            continue;
                                        }
                                        // iterate over all vertices of that pyz (one vertex for each roll value)
                                        for (ob::PlannerDataVertex &v1 : vertices[i][j][k]) {
                                            for (ob::PlannerDataVertex &v2 : vertices[i_][j_][k_]) {
                                                if (space_information->distance(v1.getState(), v2.getState()) <= max_joint_space_distance) {
                                                    m1.lock();
                                                    this->planner_data.addEdge(v1, v2);
                                                    this->planner_data.addEdge(v2, v1);
                                                    m1.unlock();
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                            m2.lock();
                            processed_pyz[i][j][k] = true;
                            m2.unlock();
                            // connect vertices of same pyz
                            for (ob::PlannerDataVertex &v1 : vertices[i][j][k]) {
                                for (ob::PlannerDataVertex &v2 : vertices[i][j][k]) {
                                    if (v1 != v2 && space_information->distance(v1.getState(), v2.getState()) <= max_joint_space_distance) {
                                        m1.lock();
                                        this->planner_data.addEdge(v1, v2);
                                        this->planner_data.addEdge(v2, v1);
                                        m1.unlock();
                                    }
                                }
                            }
                            m3.lock();
                            std::cout << "\r" << (int) (100 * (++current_iteration / (double) total_iterations)) << "%     ";
                            std::flush(std::cout);
                            m3.unlock();
                        }
                        index = ++index % thread_count;
                    }
                }
            }
        };
        ROS_INFO_STREAM("Connecting states...");
        threads.clear();
        for (size_t i = 0; i < thread_count; i++) {
            threads.emplace_back(connect_vertices, i);
        }
        for (auto &t : threads) {
            t.join();
        }
        std::cout << "\r";
        this->simple_setup->setPlanner(std::make_shared<og::PRM>(this->planner_data));
    }

    ob::PlannerPtr PivotPlanningContext::selectPlanner(const std::string &planner_id) const {
        auto space_information = this->simple_setup->getSpaceInformation();
        if (planner_id == "PRM") {
            return std::make_shared<og::PRM>(this->planner_data);
        } else if (planner_id == "PRMstar") {
            return std::make_shared<og::PRMstar>(this->planner_data);
        } else {
            ROS_ERROR_STREAM("Unknown planner id: " << planner_id);
            return nullptr;
        }
    }
}
