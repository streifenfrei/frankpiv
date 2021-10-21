#include "frankpiv/moveit_planner/state_sampler.hpp"


namespace frankpiv::moveit_planner {
    PivotStateSampler::PivotStateSampler(const ob::SpaceInformation *si) : ValidStateSampler(si) {

    }

    PivotStateSampler::~PivotStateSampler() = default;

    bool PivotStateSampler::sample(ob::State *state) { return false; }

    bool PivotStateSampler::sampleNear(ob::State *state, const ob::State *near, double distance) {
        return false;
    }

}
