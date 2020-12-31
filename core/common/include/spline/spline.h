#ifndef SRC_SPLINE_H
#define SRC_SPLINE_H

#include "spline/polynomial.h"
#include <cassert>
#include <vector>

namespace common
{
    template <int N_DEG, int N_DIM>
    class Spline
    {
    public:
        typedef PolynomialND<N_DEG, N_DIM> PolynomialType;

        Spline() = default;

        /**
         * @brief Set the vector domain
         * @param vec_domain: input vec domain
         */
        void set_vec_domain(const std::vector<double>& vec_domain) {
            assert(vec_domain.size() > 1);
            vec_domain_ = vec_domain;
            poly_.resize(vec_domain_.size() - 1);
        }

        /**
         * @brief Get the number of segments of the spline
         * @param n number of the segments
         */
        int num_segments() const { return static_cast<int>(poly_.size()); }

        /**
         * @brief Get the vec domain of the spline
         * @param n number of the segments
         */
        std::vector<double> vec_domain() const { return vec_domain_; }

        /**
         * @brief Get the begin of the parameterization
         */
        double begin() const
        {
            if (vec_domain_.empty()) return 0.0;
            return vec_domain_.front();
        }

        /**
         * @brief Get the end of the parameterization
         */
        double end() const
        {
            if (vec_domain_.empty()) return 0.0;
            return vec_domain_.back();
        }

        /**
         * @brief Return a reference to a certain 1D polynomial
         * @param n index of the segment
         * @param j index of the dimension
         */
        Polynomial<N_DEG>& operator()(int n, int j)
        {
            assert(n < this->num_segments());
            assert(j < N_DIM);
            return poly_[n][j];
        }

        /**
         * @brief Return evaluation of spline at given parameterization
         * @param s parameterization, s should be in the vector domain
         * @param d derivate to take
         * @note  the function will automatically do extrapolation if out of domain
         */
        ErrorType evaluate(const double s, int d, Eigen::Matrix<double,N_DIM,1>* ret) const
        {
            int num_pts = vec_domain_.size();
            if (num_pts < 1) return kIllegalInput;

            auto it = std::lower_bound(vec_domain_.begin(), vec_domain_.end(), s);
            int idx = std::max(static_cast<int>(it - vec_domain_.begin()) - 1, 0);
            double h = s - vec_domain_[idx];

            if (s < vec_domain_[0])
                poly_[0].evaluate(h, d, ret);
            else if (s > vec_domain_[num_pts - 1])
            {
                h = s - vec_domain_[idx - 1];
                poly_[num_pts - 2].evaluate(h, d, ret);
            }
            else
                poly_[idx].evaluate(h, d, ret);

            return kSuccess;
        }

        ErrorType evaluate(const double s, Eigen::Matrix<double,N_DIM,1>* ret) const
        {
            int num_pts = vec_domain_.size();
            if (num_pts < 1) return kIllegalInput;

            auto it = std::lower_bound(vec_domain_.begin(), vec_domain_.end(), s);
            int idx = std::max(static_cast<int>(it - vec_domain_.begin()) - 1, 0);
            double h = s - vec_domain_[idx];

            if (s < vec_domain_[0])
                poly_[0].evaluate(h, ret);
            else if (s > vec_domain_[num_pts - 1])
            {
                h = s - vec_domain_[idx - 1];
                poly_[num_pts - 2].evaluate(h, ret);
            }
            else
                poly_[idx].evaluate(h, ret);

            return kSuccess;
        }

        void print() const
        {
            int num_polys = static_cast<int>(poly_.size());
            for (int i = 0; i < num_polys; i++)
            {
                printf("vec domain (%lf, %lf).\n", vec_domain_[i], vec_domain_[i + 1]);
                poly_[i].print();
                Eigen::Matrix<double,2,1> v;
                poly_[i].evaluate(vec_domain_[i + 1] - vec_domain_[i], 0, &v);
                printf("end pos: (%lf, %lf).\n", v[0], v[1]);
                poly_[i].evaluate(vec_domain_[i + 1] - vec_domain_[i], 1, &v);
                printf("end vel: (%lf, %lf).\n", v[0], v[1]);
                poly_[i].evaluate(vec_domain_[i + 1] - vec_domain_[i], 2, &v);
                printf("end acc: (%lf, %lf).\n", v[0], v[1]);
                poly_[i].evaluate(vec_domain_[i + 1] - vec_domain_[i], 3, &v);
                printf("end jerk: (%lf, %lf).\n", v[0], v[1]);
            }
        }

    private:
        vec_E<PolynomialType> poly_;
        std::vector<double> vec_domain_;
    };
}

#endif //SRC_SPLINE_H
