#ifndef SRC_FREE_STATE_H
#define SRC_FREE_STATE_H

#include "basics/basics.h"
#include "state/state.h"

#include <cmath>
namespace common
{
    struct FreeState
    {
        double time_stamp{0.0};
        Eigen::Matrix<double, 2, 1> position{Eigen::Matrix<double, 2, 1>::Zero()};
        Eigen::Matrix<double, 2, 1> velocity{Eigen::Matrix<double, 2, 1>::Zero()};
        Eigen::Matrix<double, 2, 1> acceleration{Eigen::Matrix<double, 2, 1>::Zero()};
        double angle{0.0};

        void print() const
        {
            printf("position: (%lf, %lf).\n", position[0], position[1]);
            printf("velocity: (%lf, %lf).\n", velocity[0], velocity[1]);
            printf("acceleration: (%lf, %lf).\n", acceleration[0], acceleration[1]);
            printf("angle: %lf.\n", angle);
        }
    };

    void GetFreeStateFromState(const State& state, FreeState* free_state);
    void GetStateFromFreeState(const FreeState& free_state, State* state);
}  // namespace common

#endif //SRC_FREE_STATE_H
