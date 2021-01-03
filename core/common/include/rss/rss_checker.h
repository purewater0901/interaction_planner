#ifndef SRC_RSS_CHECKER_H
#define SRC_RSS_CHECKER_H

#include "basics/basics.h"
#include "basics/semantics.h"
#include "state/frenet_state.h"
#include "state/state.h"
#include "state/state_transformer.h"

namespace common
{
    class RssChecker
    {
    public:
        enum LongitudinalDirection { Front = 0, Rear };
        enum LateralDirection { Left = 0, Right };
        enum class LongitudinalViolateType { Legal = 0, TooFast, TooSlow };

        struct RssConfig
        {
            double response_time = 0.1;
            double longitudinal_acc_max = 2.0;
            double longitudinal_brake_min = 4.0;
            double longitudinal_brake_max = 5.0;
            double lateral_acc_max = 1.0;
            double lateral_brake_min = 1.0;
            double lateral_brake_max = 1.0;
            double lateral_miu = 0.5;
            RssConfig() {}
            RssConfig(const double _response_time,
                      const double _longitudinal_acc_max,
                      const double _longitudinal_brake_min,
                      const double _longitudinal_brake_max,
                      const double _lateral_acc_max,
                      const double _lateral_brake_min,
                      const double _lateral_brake_max, const double _lateral_miu)
                    : response_time(_response_time),
                      longitudinal_acc_max(_longitudinal_acc_max),
                      longitudinal_brake_min(_longitudinal_brake_min),
                      longitudinal_brake_max(_longitudinal_brake_max),
                      lateral_acc_max(_lateral_acc_max),
                      lateral_brake_min(_lateral_brake_min),
                      lateral_brake_max(_lateral_brake_max),
                      lateral_miu(_lateral_miu) {}
        };

        static ErrorType CalculateSafeLongitudinalDistance(
                const double ego_vel, const double other_vel,
                const LongitudinalDirection& direction, const RssConfig& config,
                double* distance);

        static ErrorType CalculateSafeLateralDistance(
                const double ego_vel, const double other_vel,
                const LateralDirection& direction, const RssConfig& config,
                double* distance);

        static ErrorType CalculateRssSafeDistances(
                const std::vector<double>& ego_vels,
                const std::vector<double>& other_vels,
                const LongitudinalDirection& long_direct,
                const LateralDirection& lat_direct, const RssConfig& config,
                std::vector<double>* safe_distances);

        static ErrorType RssCheck(const FrenetState& ego_fs,
                                  const FrenetState& other_fs,
                                  const RssConfig& config, bool* is_safe);

        static ErrorType RssCheck(const Vehicle& ego_vehicle,
                                  const Vehicle& other_vehicle, const StateTransformer& stf,
                                  const RssConfig& config, bool* is_safe,
                                  LongitudinalViolateType* lon_type,
                                  double* rss_vel_low, double* rss_vel_up);

        static ErrorType CalculateSafeLongitudinalVelocity(
                const double other_vel, const LongitudinalDirection& direction,
                const double& lon_distance_abs, const RssConfig& config,
                double* ego_vel_low, double* ego_vel_upp);
    };
}

#endif //SRC_RSS_CHECKER_H
