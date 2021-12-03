#ifndef PIVOT_PLANNER
#define PIVOT_PLANNER

#include <Eigen/Geometry>
#include <kdl/chainfksolver.hpp>
#include <trac_ik/trac_ik.hpp>

#include "frankpiv/pivot_frame.hpp"

namespace frankpiv::pivot_planner {
    class PivotPlanningRequest {
    public:
        PivotPlanningRequest() = default;
        ~PivotPlanningRequest() = default;

        Eigen::VectorXd start_state;
        Eigen::Vector4d goal_pyrz;
        double allowed_planning_time;
    };

    enum Status { Success, IKFailed, SearchFailed };

    class PivotPlanningResponse {
    public:
        PivotPlanningResponse() = default;
        ~PivotPlanningResponse() = default;

        Status status = Status::IKFailed;

        std::vector <Eigen::VectorXd> trajectory;
    };

    class PivotPlanner {
    public:
        PivotPlanner(std::shared_ptr <frankpiv::PivotFrame> pivot_frame, const std::string &base_link, const std::string& tip_link, const std::string &urdf_param, double ik_timeout = 5e-3, unsigned int threads = 1);

        ~PivotPlanner() = default;

        PivotPlanningResponse solve(const PivotPlanningRequest &request) ;

        [[nodiscard]] std::shared_ptr <frankpiv::PivotFrame> pivot_frame() const { return pivot_frame_; }

        void pivot_frame(std::shared_ptr <frankpiv::PivotFrame> pivot_frame) { this->pivot_frame_ = pivot_frame; };

        [[nodiscard]] double max_joint_state_distance() const { return max_joint_state_distance_; }

        void max_joint_state_distance(double max_joint_state_distance) { this->max_joint_state_distance_ = max_joint_state_distance; };

        [[nodiscard]] double max_pyrz_step_size() const { return max_pyrz_step_size_; }

        void max_pyrz_step_size(double max_pyrz_step_size) { this->max_pyrz_step_size_ = max_pyrz_step_size; };

        [[nodiscard]] unsigned int dof() const { return dof_; }

        [[nodiscard]] unsigned int max_threads() const { return max_threads_; }

        void max_threads(unsigned int max_threads) {
            this->max_threads_ = max_threads;
            this->max_threads_ik = std::max(1, (int)this->max_threads_ / 2);
            this->createIKSolvers();
        };

        [[nodiscard]] double ik_timeout() const { return ik_timeout_; }

        void ik_timeout(double ik_timeout) { this->ik_timeout_ = ik_timeout; this->createIKSolvers();};

    private:
        std::shared_ptr <frankpiv::PivotFrame> pivot_frame_;
        double max_joint_state_distance_ = 0.15;
        double max_pyrz_step_size_ = 0.01;
        unsigned int dof_;
        KDL::Chain chain;
        std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
        std::vector<std::unique_ptr<TRAC_IK::TRAC_IK>> ik_solvers;
        unsigned int max_threads_;
        unsigned int max_threads_ik;
        // ik solver constructor parameters
        const std::string& base_link;
        const std::string& eef_link;
        const std::string& urdf_param;
        double ik_timeout_;
        static constexpr double ik_eps = 1e-5;
        static constexpr TRAC_IK::SolveType ik_solve_type = TRAC_IK::Distance;
        // solving related datastructures
        bool abort;
        Eigen::Vector4d start_pyrz;
        Eigen::Vector4d pyrz_step_size;
        KDL::JntArray joint_step_size;
        unsigned int state_count;
        std::vector <std::vector <KDL::JntArray>> joint_states;
        std::vector <Eigen::MatrixXi> connections;
        std::vector <Eigen::Vector2i> solution;
        bool solution_found;
        std::mutex solution_mutex;

        int max_step;

        bool toPYRZ(const KDL::JntArray &joint_array, Eigen::Vector4d &pyrz);

        bool getIKSolutions(const Eigen::Vector4d& pyrz, const KDL::JntArray &seed, std::vector <KDL::JntArray>& solutions, TRAC_IK::TRAC_IK *ik_solver);

        // thread runnables
        void createSearchSpace(unsigned int thread_id);

        void tsDoStepIfValid(unsigned int step, unsigned int state_index, unsigned int next_state_index, std::vector <Eigen::Vector2i> &current_path);

        void tsChooseNextStep(unsigned int step, unsigned int state_index, std::vector <Eigen::Vector2i> &current_path);

        void tsStart(unsigned int start_index);

        void createIKSolvers();
    };
}

#endif  // PIVOT_PLANNER