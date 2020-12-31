#include "lane/lane.h"

namespace common {

    ErrorType Lane::GetCurvatureByArcLength(const double& arc_length,
                                            double* curvature,
                                            double* curvature_derivative) const
    {
        if (CheckInputArcLength(arc_length) != kSuccess || LaneDim != 2)
            return kIllegalInput;

        Eigen::Matrix<double,LaneDim,1> vel, acc, jrk;
        GetDerivativeByArcLength(arc_length, 1, &vel);
        // printf("vel: (%lf, %lf) at arc length %lf.\n", vel[0], vel[1], arc_length);
        GetDerivativeByArcLength(arc_length, 2, &acc);
        // printf("acc: (%lf, %lf) at arc length %lf.\n", acc[0], acc[1], arc_length);
        GetDerivativeByArcLength(arc_length, 3, &jrk);
        // printf("jrk: (%lf, %lf) at arc length %lf.\n", jrk[0], jrk[1], arc_length);

        double c0 = vel[0] * acc[1] - vel[1] * acc[0];
        double c1 = vel.norm();
        *curvature = c0 / (c1 * c1 * c1);
        *curvature_derivative =
                ((acc[0] * acc[1] + vel[0] * jrk[1] - acc[1] * acc[0] - vel[1] * jrk[0]) /
                 c1 * c1 * c1 -
                 3 * c0 * (vel[0] * acc[0] + vel[1] * acc[1]) / (c1 * c1 * c1 * c1 * c1));
        return kSuccess;
    }

    ErrorType Lane::GetCurvatureByArcLength(const double& arc_length,
                                            double* curvature) const
    {
        if (CheckInputArcLength(arc_length) != kSuccess || LaneDim != 2)
            return kIllegalInput;

        Eigen::Matrix<double,LaneDim,1> vel, acc;
        GetDerivativeByArcLength(arc_length, 1, &vel);
        GetDerivativeByArcLength(arc_length, 2, &acc);

        double c0 = vel[0] * acc[1] - vel[1] * acc[0];
        double c1 = vel.norm();
        *curvature = c0 / (c1 * c1 * c1);
        return kSuccess;
    }

    ErrorType Lane::GetDerivativeByArcLength(const double& arc_length,
                                             const int& d,
                                             Eigen::Matrix<double,LaneDim,1>* derivative) const
    {
        return position_spline_.evaluate(arc_length, d, derivative);
    }

    ErrorType Lane::GetPositionByArcLength(const double& arc_length,
                                           Eigen::Matrix<double,LaneDim,1>* derivative) const
    {
        return position_spline_.evaluate(arc_length, derivative);
    }

    ErrorType Lane::GetTangentVectorByArcLength(const double& arc_length,
                                                Eigen::Matrix<double,LaneDim,1>* tangent_vector) const
    {
        if (CheckInputArcLength(arc_length) != kSuccess)
            return kIllegalInput;

        Eigen::Matrix<double,LaneDim,1> vel;
        GetDerivativeByArcLength(arc_length, 1, &vel);

        if (vel.norm() < constants::kEPS)
            return kWrongStatus;

        *tangent_vector = vel / vel.norm();
        return kSuccess;
    }

    ErrorType Lane::GetNormalVectorByArcLength(const double& arc_length,
                                               Eigen::Matrix<double,LaneDim,1>* normal_vector) const
    {
        if (CheckInputArcLength(arc_length) != kSuccess)
            return kIllegalInput;

        Eigen::Matrix<double,LaneDim,1> vel;
        GetDerivativeByArcLength(arc_length, 1, &vel);

        if (vel.norm() < constants::kEPS)
            return kWrongStatus;

        Eigen::Matrix<double,LaneDim,1> tangent_vector = vel / vel.norm();
        *normal_vector = rotate_vector_2d(tangent_vector, M_PI / 2.0);
        return kSuccess;
    }

    ErrorType Lane::GetOrientationByArcLength(const double& arc_length,
                                              double* angle) const
    {
        if (CheckInputArcLength(arc_length) != kSuccess)
            return kIllegalInput;

        Eigen::Matrix<double,LaneDim,1> vel;
        GetDerivativeByArcLength(arc_length, 1, &vel);

        if (vel.norm() < constants::kEPS)
            return kWrongStatus;

        Eigen::Matrix<double,LaneDim,1> tangent_vector = vel / vel.norm();
        *angle = vec2d_to_angle(tangent_vector);
        return kSuccess;
    }

    ErrorType Lane::GetArcLengthByVecPosition(const Eigen::Matrix<double,LaneDim,1>& vec_position,
                                              double* arc_length) const
    {
        if (!IsValid())
            return kWrongStatus;

        static constexpr int kMaxCnt = 4;
        static constexpr double kMaxDistSquare = 900.0;

        const double val_lb = position_spline_.begin();
        const double val_ub = position_spline_.end();
        double step = (val_ub - val_lb) * 0.5;

        double s1 = val_lb;
        double s2 = val_lb + step;
        double s3 = val_ub;
        double initial_guess = s2;

        Eigen::Matrix<double,LaneDim,1> start_pos, mid_pos, final_pos;
        position_spline_.evaluate(s1, &start_pos);
        position_spline_.evaluate(s2, &mid_pos);
        position_spline_.evaluate(s3, &final_pos);

        // ~ Step I: use binary search to find a initial guess
        double d1 = (start_pos - vec_position).squaredNorm();
        double d2 = (mid_pos - vec_position).squaredNorm();
        double d3 = (final_pos - vec_position).squaredNorm();

        for (int i = 0; i < kMaxCnt; ++i)
        {
            double min_dis = std::min(std::min(d1, d2), d3);
            if (min_dis < kMaxDistSquare) {
                if (min_dis == d1)
                    initial_guess = s1;
                else if (min_dis == d2)
                    initial_guess = s2;
                else if (min_dis == d3)
                    initial_guess = s3;
                else
                    assert(false);
                break;
            }
            step *= 0.5;
            if (min_dis == d1)
            {
                initial_guess = s1;
                s3 = s2;
                s2 = s1 + step;
                position_spline_.evaluate(s2, &mid_pos);
                position_spline_.evaluate(s3, &final_pos);
                d2 = (mid_pos - vec_position).squaredNorm();
                d3 = (final_pos - vec_position).squaredNorm();
            }
            else if (min_dis == d2)
            {
                initial_guess = s2;
                s1 = s2 - step;
                s3 = s2 + step;
                position_spline_.evaluate(s1, &start_pos);
                position_spline_.evaluate(s3, &final_pos);
                d1 = (start_pos - vec_position).squaredNorm();
                d3 = (final_pos - vec_position).squaredNorm();
            }
            else if (min_dis == d3)
            {
                initial_guess = s3;
                s1 = s2;
                s2 = s3 - step;
                position_spline_.evaluate(s1, &start_pos);
                position_spline_.evaluate(s2, &mid_pos);
                d1 = (start_pos - vec_position).squaredNorm();
                d2 = (mid_pos - vec_position).squaredNorm();
            }
            else
            {
                printf(
                        "[Lane]GetArcLengthByVecPosition - d1: %lf, d2: %lf, d3: %lf, "
                        "min_dis: %lf\n",
                        d1, d2, d3, min_dis);
                assert(false);
            }
        }

        // ~ Step II: use Newton's method to find the local minimum
        GetArcLengthByVecPositionWithInitialGuess(vec_position, initial_guess,
                                                  arc_length);

        return kSuccess;
    }

    ErrorType Lane::GetArcLengthByVecPositionWithInitialGuess(const Eigen::Matrix<double,LaneDim,1>& vec_position,
                                                              const double& initial_guess,
                                                              double* arc_length) const
    {
        if (!IsValid())
            return kWrongStatus;

        const double val_lb = position_spline_.begin();
        const double val_ub = position_spline_.end();

        // ~ use Newton's method to find the local minimum
        static constexpr double epsilon = 1e-3;
        static constexpr int kMaxIter = 8;
        double x = std::min(std::max(initial_guess, val_lb), val_ub);
        Eigen::Matrix<double,LaneDim,1> p, dp, ddp, tmp_vec;

        for (int i = 0; i < kMaxIter; ++i)
        {
            position_spline_.evaluate(x, 0, &p);
            position_spline_.evaluate(x, 1, &dp);
            position_spline_.evaluate(x, 2, &ddp);

            tmp_vec = p - vec_position;
            double f_1 = tmp_vec.dot(dp);
            double f_2 = dp.dot(dp) + tmp_vec.dot(ddp);
            double dx = -f_1 / f_2;

            if (std::fabs(dx) < epsilon)
                break;

            if (x + dx > val_ub)
            {
                x = val_ub;
                break;
            }
            else if (x + dx < val_lb)
            {
                x = val_lb;
                break;
            }

            x += dx;
        }

        *arc_length = x;

        return kSuccess;
    }

    ErrorType Lane::CheckInputArcLength(const double& arc_length) const
    {
        if (!IsValid())
        {
            printf("[CheckInputArcLength]Quering invalid lane.\n");
            return kWrongStatus;
        }
        if (arc_length < position_spline_.begin() - constants::kEPS ||
            arc_length > position_spline_.end() + constants::kEPS)
            return kIllegalInput;

        return kSuccess;
    }

}  // namespace common