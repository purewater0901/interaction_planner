#ifndef SRC_FRENET_PRIMITIVE_H
#define SRC_FRENET_PRIMITIVE_H

#include "spline/polynomial.h"
#include "state/frenet_state.h"

namespace common {
    class FrenetPrimitive
    {
    public:
        FrenetPrimitive() = default;

        /**
         * @brief Construct primitive in the frenet frame with two different modes
         * @param fs0, start frenet state
         * @param fs1, end frenet state
         * @param is_lateral_independent, whether high speed mode enabled
         */
        ErrorType Connect(const FrenetState &fs0, const FrenetState &fs1,
                          const double &stamp, const double &T,
                          bool is_lateral_independent);

        /**
         * @brief Construct primitive in the frenet frame with two different modes
         * @param fs0, start frenet state
         * @param u, longitudal and lateral control
         * @param stamp, start time stamp of the primitive
         * @param T, primitive duration
         */
        ErrorType Propagate(const FrenetState &fs0,
                            const Eigen::Matrix<double, 2, 1> &u,
                            const double &stamp, const double &T);

        /**
         * @brief Get the start point of the parameterization
         */
        double begin() const { return stamp_; }

        /**
         * @brief Get the end point of the parameterization
         */
        double end() const { return stamp_ + duration_; }

        /**
         * @brief Get the frenet state from the primitive
         * @note  when t is outside of the parameterization, extrapolation will be
         * applied
         */
        ErrorType GetFrenetState(const double &t_global, FrenetState *fs) const;

        ErrorType GetFrenetStateSamples(const double &step,
                                        const double &offset,
                                        vec_E<FrenetState> *fs_vec) const;

        ErrorType GetJ(double *c_s, double *c_d) const;

        double lateral_T() const;

        double longitudial_T() const;

        FrenetState fs1() const;

        FrenetState fs0() const;

        Polynomial<5> poly_s() const { return poly_s_; }

        Polynomial<5> poly_d() const { return poly_d_; }

        void set_poly_s(const Polynomial<5> &poly) { poly_s_ = poly; }

        void set_poly_d(const Polynomial<5> &poly) { poly_d_ = poly; }

        /**
         * @brief Debug print
         */
        void print() const
        {
            printf("frenet primitive in duration [%lf, %lf].\n", begin(), end());
            poly_s_.print();
            poly_d_.print();
            fs0_.print();
            fs1_.print();
        }

        bool is_lateral_independent_ = false;

    private:
        Polynomial<5> poly_s_;
        Polynomial<5> poly_d_;
        double stamp_{0.0};
        double duration_{0.0};
        FrenetState fs0_;
        FrenetState fs1_;
        double kSmallDistanceThreshold_ = 2.0;
    };
}

#endif //SRC_FRENET_PRIMITIVE_H
