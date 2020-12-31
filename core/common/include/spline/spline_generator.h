#ifndef SRC_SPLINE_GENERATOR_H
#define SRC_SPLINE_GENERATOR_H

#include <vector>
#include "basics/basics.h"
#include "basics/semantics.h"
#include "basics/shapes.h"
#include "math/calculations.h"
#include "solver/qp_solver.h"
#include "spline/bezier.h"
#include "spline/lookup_table.h"
#include "spline/polynomial.h"
#include "spline/spline.h"
#include "solver/ooqp_interface.h"
#include "state/state.h"
#include "state/waypoint.h"
#include "tk_spline/spline.h"

namespace common
{
    template <int N_DEG, int N_DIM>
    class SplineGenerator {
    public:
        typedef Spline<N_DEG, N_DIM> SplineType;
        typedef BezierSpline<N_DEG, N_DIM> BezierSplineType;

        /**
         * @brief Return natural (cubic) spline fitting with respect to
         * parameterization
         * @param samples, the points you want to interterpolate
         * @param parameterization, the parameterization used to evaluate the spline
         * @param spline, the spline returned (wrap in the general N_DEG spline)
         */
        static ErrorType GetCubicSplineBySampleInterpolation(const vec_Vecf<N_DIM>& samples,
                                                             const std::vector<double>& para,
                                                             SplineType* spline);

        /**
         * @brief Return optimal spline fitting with continuity constraints
         * @param samples, the points you want to fit
         * @param para, the parameterization, typically arc length
         * @param breaks, break points (represent the segments of the spline)
         * @param spline, the spline returned (wrap in the general N_DEG spline)
         * @note this may be an expensive function depending on the number of breaks
         */
        static ErrorType GetQuinticSplineBySampleFitting(const vec_Vecf<N_DIM>& samples,
                                                         const std::vector<double>& para,
                                                         const Eigen::ArrayXf& breaks,
                                                         const double regulator,
                                                         SplineType* spline);

        /**
         * @brief Return waypoints from position samples
         * @param samples, the points you want to interterpolate
         * @param parameterization, the parameterization used to evaluate the spline
         * @param waypoints, output the waypoints
         */
        static ErrorType GetWaypointsFromPositionSamples(const vec_Vecf<N_DIM>& samples,
                                                         const std::vector<double>& para,
                                                         vec_E<Waypoint<N_DIM>>* waypoints);

        /**
         * @brief Return spline from state vec
         * @param state_vec, the vector of states
         * @param para, the parameterization set
         * @param spline, spline returned
         */
        static ErrorType GetSplineFromStateVec(const std::vector<double>& para,
                                               const vec_E<State>& state_vec,
                                               SplineType* spline);

        /**
         * @brief Return spline from state vec
         * @param state_vec, the vector of free states
         * @param para, the parameterization set
         * @param spline, spline returned
         */
        static ErrorType GetSplineFromFreeStateVec(const std::vector<double>& para,
                                                   const vec_E<FreeState>& free_state_vec,
                                                   SplineType* spline);

        /**
         * @brief Return the optimal bezier curve in corridor
         * @note typical usage parameterization is 2D-t, the corridor is 3D (2D+t)
         * @param start_constraints, pos, vel, acc, etc constraints at start point
         * @param end_constraints, pos, vel, acc, etc constrainst at end point
         */

        static ErrorType GetBezierSplineUsingCorridor(const vec_E<SpatioTemporalSemanticCubeNd<N_DIM>>& cubes,
                                                      const vec_E<Eigen::Matrix<double,N_DIM,1>>& start_constraints,
                                                      const vec_E<Eigen::Matrix<double,N_DIM,1>>& end_constraints,
                                                      const std::vector<double>& ref_stamps,
                                                      const vec_E<Eigen::Matrix<double,N_DIM,1>>& ref_points,
                                                      const double& weight_proximity,
                                                      BezierSplineType* bezier_spline);

        static ErrorType GetBezierSplineUsingCorridor(const vec_E<SpatioTemporalSemanticCubeNd<N_DIM>>& cubes,
                                                      const vec_E<Eigen::Matrix<double,N_DIM,1>>& start_constraints,
                                                      const vec_E<Eigen::Matrix<double,N_DIM,1>>& end_constraints,
                                                      BezierSplineType* bezier_spline);

    };  // class spline generator
}

#endif //SRC_SPLINE_GENERATOR_H
