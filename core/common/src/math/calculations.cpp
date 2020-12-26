#include "math/calculations.h"

long long fac(int n)
{
    if (n == 0) return 1;
    if (n == 1) return 1;
    if (n == 2) return 2;
    if (n == 3) return 6;
    if (n == 4) return 24;
    if (n == 5) return 120;

    long long ans = 1;
    for (int i = 1; i <= n; i++) ans *= i;
    return ans;
}

long long nchoosek(int n, int k) { return fac(n) / fac(k) / fac(n - k); }

double normalize_angle(const double& theta)
{
    double theta_tmp = theta;
    theta_tmp -= (theta >= constants::kPi) * 2 * constants::kPi;
    theta_tmp += (theta < -constants::kPi) * 2 * constants::kPi;
    return theta_tmp;
}

Eigen::Matrix<double, 2, 1> rotate_vector_2d(const Eigen::Matrix<double, 2, 1>& v, const double& angle)
{
    return {v[0] * cos(angle) - v[1] * sin(angle), v[0] * sin(angle) + v[1] * cos(angle)};
}

double vec2d_to_angle(const Eigen::Matrix<double, 2, 1>& v) { return atan2(v[1], v[0]); }

double truncate(const double& val_in,
                const double& lower,
                const double& upper)
{
    if (lower > upper)
    {
        printf("[Calculations]Invalid input!\n");
        assert(false);
    }
    double res = val_in;
    res = std::max(res, lower);
    res = std::min(res, upper);
    return res;
}

double normalize_with_bound(const double& val_in, const double& lower,
                            const double& upper,
                            const double& new_lower,
                            const double& new_upper)
{
    if (new_lower > new_upper)
    {
        printf("[Calculations]Invalid input!\n");
        assert(false);
    }
    double val_bounded = truncate(val_in, lower, upper);
    double ratio = (val_bounded - lower) / (upper - lower);
    double res = new_lower + (new_upper - new_lower) * ratio;

    return res;
}

ErrorType RemapUsingQuadraticFuncAroundSmallValue(const double& th,
                                                  const double& val_in,
                                                  double* val_out)
{
    double c = 1.0 / th;
    if (fabs(val_in) <= fabs(th))
    {
        // quadratic, y = c * x ^ 2
        *val_out = sgn(val_in) * c * val_in * val_in;
    }
    else
    {
        // linear, y = x
        *val_out = val_in;
    }

    return kSuccess;
}