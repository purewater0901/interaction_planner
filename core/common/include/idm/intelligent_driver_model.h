#ifndef SRC_INTELLIGENT_DRIVER_MODEL_H
#define SRC_INTELLIGENT_DRIVER_MODEL_H

#include "basics/basics.h"

namespace common
{

    class IntelligentDriverModel
    {
    public:
        struct State
        {
            double s{0.0};        // longitudinal distance
            double v{0.0};        // longitudinal speed
            double s_front{0.0};  // leading vehicle
            double v_front{0.0};

            State() = default;
            State(const double &s_, const double &v_, const double &s_front_,
                  const double &v_front_)
                    : s(s_), v(v_), s_front(s_front_), v_front(v_front_) {}
        };

        struct Param
        {
            double kDesiredVelocity = 0.0;
            double kVehicleLength = 5.0;                   // l_alpha-1
            double kMinimumSpacing = 2.0;                  // s0
            double kDesiredHeadwayTime = 1.0;              // T
            double kAcceleration = 2.0;                    // a
            double kComfortableBrakingDeceleration = 3.0;  // b
            double kHardBrakingDeceleration = 5.0;
            int kExponent = 4;  // delta
        };

        static ErrorType GetIdmDesiredAcceleration(const Param &param,
                                                   const State &cur_state,
                                                   double *acc);
        static ErrorType GetIIdmDesiredAcceleration(const Param &param,
                                                    const State &cur_state,
                                                    double *acc);
        static ErrorType GetAccDesiredAcceleration(const Param &param,
                                                   const State &cur_state,
                                                   double *acc);
    };

}  // namespace common

#endif //SRC_INTELLIGENT_DRIVER_MODEL_H
