#ifndef SRC_LANE_H
#define SRC_LANE_H

#include "basics/basics.h"
#include "spline/spline.h"
#include "state/state.h"

namespace common
{
    class Lane {
    public:
        typedef Spline<LaneDegree, LaneDim> SplineType;

        Lane() = default;
        Lane(const SplineType& position_spline)
                : position_spline_(position_spline), is_valid_(true) {}
        bool IsValid() const { return is_valid_; }

        /**
         * @brief Set the parameterization of the lane
         * @param position spline, position spline with required degree
         */
        void set_position_spline(const SplineType& position_spline)
        {
            if (position_spline.vec_domain().empty()) return;
            position_spline_ = position_spline;
            is_valid_ = true;
        }

        /**
         * @brief Get curvature by arc length
         * @param arc_length, evaluation arc length
         * @param curvature, curvature returned
         * @param curvature, curvature derivative
         */
        ErrorType GetCurvatureByArcLength(const double& arc_length,
                                          double* curvature,
                                          double* curvature_derivative) const;

        ErrorType GetCurvatureByArcLength(const double& arc_length,
                                          double* curvature) const;

        /**
         * @brief Get derivative by arc length (d = 0 means position evaluation)
         * @param d, derivative to take
         * @param derivative, derivative returned
         */
        ErrorType GetDerivativeByArcLength(const double& arc_length,
                                           const int& d,
                                           Eigen::Matrix<double,LaneDim,1>* derivative) const;

        ErrorType GetPositionByArcLength(const double& arc_length,
                                         Eigen::Matrix<double,LaneDim,1>* derivative) const;

        ErrorType GetTangentVectorByArcLength(const double& arc_length,
                                              Eigen::Matrix<double,LaneDim,1>* tangent_vector) const;

        ErrorType GetNormalVectorByArcLength(const double& arc_length,
                                             Eigen::Matrix<double,LaneDim,1>* normal_vector) const;

        ErrorType GetOrientationByArcLength(const double& arc_length,
                                            double* angle) const;

        ErrorType GetArcLengthByVecPosition(const Eigen::Matrix<double,LaneDim,1>& vec_position,
                                            double* arc_length) const;

        ErrorType GetArcLengthByVecPositionWithInitialGuess(
                const Eigen::Matrix<double,LaneDim,1>& vec_position, const double& initial_guess,
                double* arc_length) const;

        ErrorType CheckInputArcLength(const double& arc_length) const;

        SplineType position_spline() const { return position_spline_; }

        double begin() const { return position_spline_.begin(); }

        double end() const { return position_spline_.end(); }

        void print() const { position_spline_.print(); }

    private:
        SplineType position_spline_;
        bool is_valid_ = false;
    };
}

#endif //SRC_LANE_H
