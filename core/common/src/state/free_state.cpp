#include "state/free_state.h"

namespace common
{
    void GetFreeStateFromState(const State& state, FreeState* free_state)
    {
        free_state->position = state.vec_position;
        double cn = cos(state.angle);
        double sn = sin(state.angle);
        free_state->velocity[0] = state.velocity * cn;
        free_state->velocity[1] = state.velocity * sn;
        double normal_acc = state.velocity * state.velocity * state.curvature;
        free_state->acceleration[0] = state.acceleration * cn - normal_acc * sn;
        free_state->acceleration[1] = state.acceleration * sn + normal_acc * cn;
        free_state->angle = state.angle;
        free_state->time_stamp = state.time_stamp;
    }

    void GetStateFromFreeState(const FreeState& free_state, State* state) {
        state->angle = free_state.angle;
        state->vec_position = free_state.position;
        state->velocity = free_state.velocity.norm();
        double cn = cos(state->angle);
        double sn = sin(state->angle);
        Eigen::Matrix<double, 2, 1> tangent_vec{Eigen::Matrix<double, 2, 1>(cn, sn)};
        Eigen::Matrix<double, 2, 1> normal_vec{Eigen::Matrix<double, 2, 1>(-sn, cn)};
        auto a_tangent = free_state.acceleration.dot(tangent_vec);
        auto a_normal = free_state.acceleration.dot(normal_vec);
        state->acceleration = a_tangent;
        if (fabs(state->velocity) > constants::kBigEPS)
            state->curvature = a_normal / std::pow(state->velocity, 2);
        else
            state->curvature = 0.0;

        state->time_stamp = free_state.time_stamp;
    }

}  // namespace common