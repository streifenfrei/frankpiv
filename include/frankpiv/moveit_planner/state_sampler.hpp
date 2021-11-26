#ifndef PIVOT_STATE_SAMPLER
#define PIVOT_STATE_SAMPLER

#include <Eigen/Geometry>
#include <ompl/base/ValidStateSampler.h>

#include "frankpiv/pivot_frame.hpp"
#include "frankpiv/moveit_planner/planning_context.hpp"

namespace ob = ompl::base;

namespace frankpiv::moveit_planner {
    class PivotStateSampler : public ob::ValidStateSampler {
    public:
        PivotStateSampler(const ob::SpaceInformation *si, moveit::core::RobotModelConstPtr robot_model,const std::string &group_name, std::shared_ptr <frankpiv::PivotFrame> pivot_frame);

        ~PivotStateSampler() override;

        bool sample(ompl::base::State *state) override;

        bool sampleNear(ompl::base::State *state, const ompl::base::State *near, double distance) override;

    private:
        moveit::core::RobotModelConstPtr robot_model;
        const std::string &group_name;
        std::shared_ptr <frankpiv::PivotFrame> pivot_frame;
        ompl::RNG rng_;
    };
}

#endif // PIVOT_STATE_SAMPLER
