#include "rss/rss_checker.h"

namespace common
{
    ErrorType RssChecker::CalculateSafeLongitudinalDistance(const double ego_vel,
                                                            const double other_vel,
                                                            const LongitudinalDirection& direction,
                                                            const RssConfig& config,
                                                            double* distance)
    {
        double ret = 0.0;
        double ego_vel_abs = fabs(ego_vel);
        double other_vel_abs = fabs(other_vel);
        double ego_vel_at_response_time   = ego_vel_abs   + config.longitudinal_acc_max * config.response_time;
        double other_vel_at_response_time = other_vel_abs + config.longitudinal_acc_max * config.response_time;

        double ego_distance_driven, other_distance_driven;
        if (direction == Front)
        {
            ego_distance_driven =
                    (ego_vel_abs + ego_vel_at_response_time) / 2.0 * config.response_time +
                    ego_vel_at_response_time * ego_vel_at_response_time /
                    (2 * config.longitudinal_brake_min);
            if (ego_vel >= 0.0 && other_vel >= 0.0) {
                // ego vehicle ==> other vehicle ->
                other_distance_driven =
                        (other_vel_abs * other_vel_abs) / (2 * config.longitudinal_brake_max);
                ret = ego_distance_driven - other_distance_driven;
            } else if (ego_vel >= 0.0 && other_vel <= 0.0) {
                // ego vehicle ==> <-- other vehicle
                other_distance_driven = (other_vel_abs + other_vel_at_response_time) /
                                        2.0 * config.response_time +
                                        other_vel_at_response_time *
                                        other_vel_at_response_time /
                                        (2 * config.longitudinal_brake_min);
                ret = ego_distance_driven + other_distance_driven;
            } else {
                // printf("[RssChecker]Currently do not support rear gear %lf.\n",
                // ego_vel);
                ret = 0.0;
            }
        }
        else if (direction == Rear)
        {
            ego_distance_driven = ego_vel_abs * ego_vel_abs / (2 * config.longitudinal_brake_max);
            if (ego_vel >= 0.0 && other_vel >= 0.0) {
                // other car --> ego car ==>
                other_distance_driven = (other_vel_abs + other_vel_at_response_time) /
                                        2.0 * config.response_time +
                                        other_vel_at_response_time *
                                        other_vel_at_response_time /
                                        (2 * config.longitudinal_brake_min);
                ret = other_distance_driven - ego_distance_driven;
            } else if (ego_vel >= 0.0 && other_vel <= 0.0) {
                ret = 0.0;
            } else {
                // printf("[RssChecker]Currently do not support rear gear %lf.\n",
                // ego_vel);
                ret = 0.0;
            }
        }
        *distance = ret > 0.0 ? ret : 0.0;
        return kSuccess;
    }

    ErrorType RssChecker::CalculateSafeLongitudinalVelocity(
            const double other_vel, const LongitudinalDirection& direction,
            const double& lon_distance_abs, const RssConfig& config,
            double* ego_vel_low, double* ego_vel_upp) {
        double other_vel_abs = fabs(other_vel);
        double other_vel_at_response_time =
                other_vel_abs + config.longitudinal_acc_max * config.response_time;
        double other_distance_driven;
        if (direction == Front) {
            if (other_vel >= 0.0) {
                // ego ---->(lon_distance) other --->
                // other hard brake
                other_distance_driven =
                        (other_vel_abs * other_vel_abs) / (2 * config.longitudinal_brake_max);
                // ego has vel upp
                double a = 1.0 / (2.0 * config.longitudinal_brake_min);
                double b = config.response_time +
                              (config.longitudinal_acc_max * config.response_time /
                               config.longitudinal_brake_min);
                double c = 0.5 *
                              (config.longitudinal_acc_max +
                               pow(config.longitudinal_acc_max, 2) /
                               config.longitudinal_brake_min) *
                              pow(config.response_time, 2) -
                              other_distance_driven - lon_distance_abs;
                *ego_vel_upp = (-b + sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
                *ego_vel_low = 0.0;
            } else {
                // ego ----> <---- other
                other_distance_driven = (other_vel_abs + other_vel_at_response_time) /
                                        2.0 * config.response_time +
                                        other_vel_at_response_time *
                                        other_vel_at_response_time /
                                        (2 * config.longitudinal_brake_min);
                if (other_distance_driven > lon_distance_abs) {
                    *ego_vel_upp = 0.0;
                    *ego_vel_low = 0.0;
                } else {
                    double a = 1.0 / (2.0 * config.longitudinal_brake_min);
                    double b = config.response_time +
                                  (config.longitudinal_acc_max * config.response_time /
                                   config.longitudinal_brake_min);
                    double c = 0.5 *
                                  (config.longitudinal_acc_max +
                                   pow(config.longitudinal_acc_max, 2) /
                                   config.longitudinal_brake_min) *
                                  pow(config.response_time, 2) -
                                  (lon_distance_abs - other_distance_driven);
                    *ego_vel_upp = (-b + sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
                    *ego_vel_low = 0.0;
                }
            }
        } else {
            if (other_vel >= 0.0) {
                // other ---> ego--->
                other_distance_driven = (other_vel_abs + other_vel_at_response_time) /
                                        2.0 * config.response_time +
                                        other_vel_at_response_time *
                                        other_vel_at_response_time /
                                        (2 * config.longitudinal_brake_min);
                if (other_distance_driven < lon_distance_abs) {
                    *ego_vel_upp = constants::kInf;
                    *ego_vel_low = 0.0;
                } else {
                    *ego_vel_upp = constants::kInf;
                    *ego_vel_low = sqrt(2 * config.longitudinal_brake_max *
                                        (other_distance_driven - lon_distance_abs));
                }
            } else {
                // <----other ego-->
                *ego_vel_upp = constants::kInf;
                *ego_vel_low = 0.0;
            }
        }
        return kSuccess;
    }

    ErrorType RssChecker::CalculateSafeLateralDistance(
            const double ego_vel, const double other_vel,
            const LateralDirection& direction, const RssConfig& config,
            double* distance) {
        double ret = 0.0;
        double ego_lat_vel_abs = fabs(ego_vel);
        double other_lat_vel_abs = fabs(other_vel);
        double distance_correction = config.lateral_miu;
        double ego_lat_vel_at_response_time =
                ego_lat_vel_abs + config.response_time * config.lateral_acc_max;
        double other_lat_vel_at_response_time =
                other_lat_vel_abs + config.response_time * config.lateral_acc_max;
        double ego_active_brake_distance =
                ego_lat_vel_abs * ego_lat_vel_abs / (2 * config.lateral_brake_max);
        double ego_passive_brake_distance =
                (ego_lat_vel_abs + ego_lat_vel_at_response_time) / 2.0 *
                config.response_time +
                ego_lat_vel_at_response_time * ego_lat_vel_at_response_time /
                (2 * config.lateral_brake_min);
        double other_active_brake_distance =
                other_lat_vel_abs * other_lat_vel_abs / (2 * config.lateral_brake_max);
        double other_passive_brake_distance =
                (other_lat_vel_abs + other_lat_vel_at_response_time) / 2.0 *
                config.response_time +
                other_lat_vel_at_response_time * other_lat_vel_at_response_time /
                (2 * config.lateral_brake_min);
        if (direction == Right) {
            if (ego_vel >= 0.0 && other_vel >= 0.0) {
                // -------------------------------
                // ego    ^^^^^^^^
                // -------------------------------
                // other  ^^^^^^^^
                // -------------------------------
                ret = other_passive_brake_distance - ego_active_brake_distance;
            } else if (ego_vel >= 0.0 && other_vel < 0.0) {
                // -------------------------------
                // ego    ^^^^^^^^
                // -------------------------------
                // other  vvvvvvvv
                // -------------------------------
                ret = 0.0;
            } else if (ego_vel < 0.0 && other_vel < 0.0) {
                // -------------------------------
                // ego    vvvvvvvv
                // -------------------------------
                // other  vvvvvvvv
                // -------------------------------
                ret = ego_passive_brake_distance - other_active_brake_distance;
            } else if (ego_vel < 0.0 && other_vel >= 0.0) {
                // -------------------------------
                // ego    vvvvvvvv
                // -------------------------------
                // other  ^^^^^^^^
                // -------------------------------
                ret = ego_passive_brake_distance + other_passive_brake_distance;
            } else {
                // printf("[RssChecker]Lat Error configuration.\n");
                // assert(false);
                ret = 0.0;
            }
        } else if (direction == Left) {
            if (ego_vel >= 0.0 && other_vel >= 0.0) {
                // -------------------------------
                // other    ^^^^^^^^
                // -------------------------------
                // ego      ^^^^^^^^
                // -------------------------------
                ret = ego_passive_brake_distance - other_active_brake_distance;
            } else if (ego_vel >= 0.0 && other_vel < 0.0) {
                // -------------------------------
                // other    vvvvvvvv
                // -------------------------------
                // ego      ^^^^^^^^
                // -------------------------------
                ret = ego_passive_brake_distance + other_passive_brake_distance;
            } else if (ego_vel < 0.0 && other_vel < 0.0) {
                // -------------------------------
                // other    vvvvvvvv
                // -------------------------------
                // ego      vvvvvvvv
                // -------------------------------
                ret = other_passive_brake_distance - ego_active_brake_distance;
            } else if (ego_vel < 0.0 && other_vel >= 0.0) {
                // -------------------------------
                // other    ^^^^^^^^
                // -------------------------------
                // ego      vvvvvvvv
                // -------------------------------
                ret = 0.0;
            } else {
                // printf("[RssChecker]Lat Error configuration.\n");
                // assert(false);
                ret = 0.0;
            }
        }
        ret = ret > 0.0 ? ret : 0.0;
        ret += distance_correction;
        *distance = ret;
        return kSuccess;
    }

    ErrorType RssChecker::CalculateRssSafeDistances(
            const std::vector<double>& ego_vels,
            const std::vector<double>& other_vels,
            const LongitudinalDirection& lon_direct, const LateralDirection& lat_direct,
            const RssConfig& config, std::vector<double>* safe_distances) {
        safe_distances->clear();
        double safe_long_distance, safe_lat_distance;
        CalculateSafeLongitudinalDistance(ego_vels[0], other_vels[0], lon_direct,
                                          config, &safe_long_distance);
        CalculateSafeLateralDistance(ego_vels[1], other_vels[1], lat_direct, config,
                                     &safe_lat_distance);
        safe_distances->push_back(safe_long_distance);
        safe_distances->push_back(safe_lat_distance);
        return kSuccess;
    }

    ErrorType RssChecker::RssCheck(const FrenetState& ego_fs,
                                   const FrenetState& other_fs,
                                   const RssConfig& config, bool* is_safe) {
        LongitudinalDirection lon_direct;
        LateralDirection lat_direct;
        if (ego_fs.vec_s[0] >= other_fs.vec_s[0]) {
            lon_direct = Rear;
        } else {
            lon_direct = Front;
        }

        if (ego_fs.vec_dt[0] >= other_fs.vec_dt[0]) {
            lat_direct = Right;
        } else {
            lat_direct = Left;
        }

        std::vector<double> ego_vels{ego_fs.vec_s[1], ego_fs.vec_dt[1]};
        std::vector<double> other_vels{other_fs.vec_s[1], other_fs.vec_dt[1]};
        std::vector<double> safe_distances;
        CalculateRssSafeDistances(ego_vels, other_vels, lon_direct, lat_direct,
                                  config, &safe_distances);

        if (fabs(ego_fs.vec_s[0] - other_fs.vec_s[0]) < safe_distances[0] &&
            fabs(ego_fs.vec_dt[0] - other_fs.vec_dt[0]) < safe_distances[1]) {
            *is_safe = false;
        } else {
            *is_safe = true;
        }
        return kSuccess;
    }

    ErrorType RssChecker::RssCheck(const Vehicle& ego_vehicle,
                                   const Vehicle& other_vehicle,
                                   const StateTransformer& stf, const RssConfig& config,
                                   bool* is_safe, LongitudinalViolateType* lon_type,
                                   double* rss_vel_low, double* rss_vel_up) {
        FrenetState ego_fs, other_fs;
        // TODO(lu.zhang): construct stf is a little bit heavy
        // StateTransformer stf(ref_lane);
        if (stf.GetFrenetStateFromState(ego_vehicle.state(), &ego_fs) != kSuccess) {
            printf("[RssChecker]ego not on ref lane.\n");
            return kWrongStatus;
        }
        if (stf.GetFrenetStateFromState(other_vehicle.state(), &other_fs) !=
            kSuccess) {
            printf("[RssChecker]other %d not on ref lane.\n", other_vehicle.id());
            return kWrongStatus;
        }

        LongitudinalDirection lon_direct;
        LateralDirection lat_direct;

        if (ego_fs.vec_s[0] >= other_fs.vec_s[0]) {
            lon_direct = Rear;
        } else {
            lon_direct = Front;
        }

        if (ego_fs.vec_dt[0] >= other_fs.vec_dt[0]) {
            lat_direct = Right;
        } else {
            lat_direct = Left;
        }

        if (ego_fs.vec_s[1] < 0.0) {
            *is_safe = true;
            *lon_type = LongitudinalViolateType::Legal;
            *rss_vel_up = 0.0;
            *rss_vel_low = 0.0;
            return kSuccess;
        }

        double safe_lat_distance;
        CalculateSafeLateralDistance(ego_fs.vec_dt[1], other_fs.vec_dt[1], lat_direct,
                                     config, &safe_lat_distance);
        safe_lat_distance +=
                0.5 * (ego_vehicle.param().width() + other_vehicle.param().width());

        if (fabs(ego_fs.vec_dt[0] - other_fs.vec_dt[0]) > safe_lat_distance) {
            *is_safe = true;
            *lon_type = LongitudinalViolateType::Legal;
            *rss_vel_up = 0.0;
            *rss_vel_low = 0.0;
            return kSuccess;
        }

        double lon_distance_abs, ego_vel_low, ego_vel_upp;
        if (lon_direct == Rear) {
            double other_rear_wheel_to_front_bump =
                    0.5 * other_vehicle.param().length() + other_vehicle.param().d_cr();
            double ego_rear_wheel_to_back_bump =
                    fabs(0.5 * ego_vehicle.param().length() - ego_vehicle.param().d_cr());
            lon_distance_abs = fabs(ego_fs.vec_s[0] - other_fs.vec_s[0]) -
                               other_rear_wheel_to_front_bump -
                               ego_rear_wheel_to_back_bump;
        } else if (lon_direct == Front) {
            double ego_rear_wheel_to_front_bump =
                    0.5 * ego_vehicle.param().length() + ego_vehicle.param().d_cr();
            double other_rear_wheel_to_back_bump = fabs(
                    0.5 * other_vehicle.param().length() - other_vehicle.param().d_cr());
            lon_distance_abs = fabs(ego_fs.vec_s[0] - other_fs.vec_s[0]) -
                               ego_rear_wheel_to_front_bump -
                               other_rear_wheel_to_back_bump;
        }

        if (lon_distance_abs < 0.0 && lon_direct == Front) {
            *is_safe = false;
            *lon_type = LongitudinalViolateType::TooFast;
            *rss_vel_up = 0.0;
            *rss_vel_low = 0.0;
            return kSuccess;
        }

        CalculateSafeLongitudinalVelocity(other_fs.vec_s[1], lon_direct,
                                          lon_distance_abs, config, &ego_vel_low,
                                          &ego_vel_upp);
        if (ego_fs.vec_s[1] > ego_vel_upp + constants::kEPS) {
            *is_safe = false;
            *lon_type = LongitudinalViolateType::TooFast;
            *rss_vel_up = ego_vel_upp;
            *rss_vel_low = ego_vel_low;
        } else if (ego_fs.vec_s[1] < ego_vel_low - constants::kEPS) {
            *is_safe = false;
            *lon_type = LongitudinalViolateType::TooSlow;
            *rss_vel_up = ego_vel_upp;
            *rss_vel_low = ego_vel_low;
        } else {
            *is_safe = true;
            *lon_type = LongitudinalViolateType::Legal;
            *rss_vel_up = 0.0;
            *rss_vel_low = 0.0;
        }
        return kSuccess;
    }
}