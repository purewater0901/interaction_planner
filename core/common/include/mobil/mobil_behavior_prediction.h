#ifndef SRC_MOBIL_BEHAVIOR_PREDICTION_H
#define SRC_MOBIL_BEHAVIOR_PREDICTION_H

#include <set>

#include "basics/basics.h"
#include "basics/semantics.h"
#include "idm/intelligent_driver_model.h"
#include "mobil/mobil_model.h"
#include "rss/rss_checker.h"

namespace common
{

    class MobilBehaviorPrediction
    {
    public:
        static ErrorType LateralBehaviorPrediction(const Vehicle &vehicle,
                                                   const vec_E<Lane> &lanes,
                                                   const vec_E<common::Vehicle> &leading_vehicles,
                                                   const vec_E<common::FrenetState> &leading_frenet_states,
                                                   const vec_E<common::Vehicle> &following_vehicles,
                                                   const vec_E<common::FrenetState> &follow_frenet_states,
                                                   const common::VehicleSet &nearby_vehicles, ProbDistOfLatBehaviors *res);

        static ErrorType RemapGainsToProb(const bool is_lcl_safe,
                                          const double mobil_gain_left,
                                          const bool is_lcr_safe,
                                          const double mobil_gain_right,
                                          ProbDistOfLatBehaviors *res);
    };

}  // namespace common


#endif //SRC_MOBIL_BEHAVIOR_PREDICTION_H
