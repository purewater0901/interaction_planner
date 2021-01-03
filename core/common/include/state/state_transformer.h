#ifndef SRC_STATE_TRANSFORMER_H
#define SRC_STATE_TRANSFORMER_H

#include "basics/basics.h"
#include "lane/lane.h"
#include "state/frenet_state.h"
#include "state/state.h"

namespace common
{
    class StateTransformer
    {
    public:
        StateTransformer() {}
        StateTransformer(const Lane& lane) { lane_ = lane; }

        ErrorType GetStateFromFrenetState(const FrenetState& fs, State* s) const;

        /**
         *  ~ note that this function may introduce approx 1cm error in vec position
         *  ~ due to the finite sampling strategy. the time consumed is about 0.03ms.
         */
        ErrorType GetFrenetStateFromState(const State& s, FrenetState* fs) const;

        ErrorType GetFrenetStateVectorFromStates(const vec_E<State>& state_vec,
                                                 vec_E<FrenetState>* fs_vec) const;

        ErrorType GetStateVectorFromFrenetStates(const vec_E<FrenetState>& fs_vec,
                                                 vec_E<State>* state_vec) const;

        ErrorType GetFrenetPointFromPoint(const Eigen::Matrix<double,2,1>& s, Eigen::Matrix<double,2,1>* fs) const;

        ErrorType GetFrenetPointVectorFromPoints(const vec_E<Eigen::Matrix<double,2,1>>& s,
                                                 vec_E<Eigen::Matrix<double,2,1>>* fs) const;

        bool IsValid() const { return lane_.IsValid(); }

        void print() {}

    private:
        Lane lane_;
    };
}

#endif //SRC_STATE_TRANSFORMER_H
