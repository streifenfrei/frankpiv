#ifndef PIVOT_STATE_SAMPLER
#define PIVOT_STATE_SAMPLER

#include <Eigen/Geometry>
#include <ompl/base/ValidStateSampler.h>

#include "frankpiv/moveit_planner/planning_context.hpp"

namespace ob = ompl::base;

namespace frankpiv::moveit_planner {
    class PivotStateSampler : public ob::ValidStateSampler {
    public:
        PivotStateSampler(const ob::SpaceInformation *si);

        ~PivotStateSampler() override;

        bool sample(ompl::base::State *state) override;

        bool sampleNear(ompl::base::State *state, const ompl::base::State *near, double distance) override;

    private:
        //const robot_state::JointModelGroup joint_model_group;
        //const Eigen::Affine3d pivot_point;
    };

    ob::ValidStateSamplerPtr allocPivotStateSampler(const ob::SpaceInformation *si) {
        return std::make_shared<PivotStateSampler>(si);
    }
}

#endif // PIVOT_STATE_SAMPLER
