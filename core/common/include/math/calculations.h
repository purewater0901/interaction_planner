#ifndef SRC_CALCULATION_H
#define SRC_CALCULATION_H

#include "basics/basics.h"

/**
 * @brief Sign function
 *
 * @tparam T
 * @param val
 * @return int
 */
template <typename T>
int sgn(const T val)
{
    return (T(0) < val) - (val < T(0));
}

/**
 * @brief Calculate factorial
 *
 * @param n Input value
 * @return long long Result
 */
long long fac(int n);

/**
 * @brief Calculate num of combinations
 *
 * @param n, k Input value
 * @return long long Result
 */
long long nchoosek(int n, int k);

/**
 * @brief Normalize angle to [-pi, pi)
 *
 * @param theta Input value
 * @return double Result
 */
double normalize_angle(const double& theta);

/**
 * @brief Shift the vector with anti-clock-wise degree
 *
 * @param v Input vector
 * @param angle Rotation angle
 * @return Vecf<2> Output vector
 */
Eigen::Matrix<double, 2, 1> rotate_vector_2d(const Eigen::Matrix<double, 2, 1>& v, const double& angle);

/**
 * @brief Return normalized angle by vec2d
 *
 * @param v Input vector
 * @return double Output angle
 */
double vec2d_to_angle(const Eigen::Matrix<double, 2, 1>& v);

/**
 * @brief Return truncated value
 *
 * @param val_in
 * @param upper
 * @param lower
 * @return double
 */
double truncate(const double& val_in,
                const double& lower,
                const double& upper);

/**
 * @brief Return normalize value
 *
 * @param val_in
 * @param lower
 * @param upper
 * @param new_lower
 * @param new_upper
 * @return double
 */
double normalize_with_bound(const double& val_in,
                            const double& lower,
                            const double& upper,
                            const double& new_lower,
                            const double& new_upper);

/**
 * @brief
 *
 * @param th
 * @param val_in
 * @param val_out
 * @return ErrorType
 */
ErrorType RemapUsingQuadraticFuncAroundSmallValue(const double& th,
                                                  const double& val_in,
                                                  double* val_out);

#endif //SRC_CALCULATION_H
