#ifndef SRC_LANE_GENERATOR_H
#define SRC_LANE_GENERATOR_H

#include "lane/lane.h"
#include "basics/basics.h"
#include "spline/spline_generator.h"

namespace common
{
    class LaneGenerator
    {
    public:
        static ErrorType GetLaneBySampleInterpolation(const vec_Vecf<LaneDim>& samples,
                                                      const std::vector<double>& para,
                                                      Lane* lane);

        static ErrorType GetLaneBySamplePoints(const vec_Vecf<LaneDim>& samples,
                                               Lane* lane);

        /**
         * @brief get lane by least square fitting with continuity constraints
         * @note  regulator recommendation: [1e6, 1e8)
         */
        static ErrorType GetLaneBySampleFitting(const vec_Vecf<LaneDim>& samples,
                                                const std::vector<double>& para,
                                                const Eigen::ArrayXf& breaks,
                                                const double regulator,
                                                Lane* lane);
    };
}

#endif //SRC_LANE_GENERATOR_H
