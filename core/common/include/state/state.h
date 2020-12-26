#ifndef SRC_STATE_H
#define SRC_STATE_H

#include "basics/basics.h"

namespace common
{
    struct State
    {
        double time_stamp{0.0};
        Eigen::Matrix<double, 2, 1> vec_position{Eigen::Matrix<double, 2, 1>::Zero()};
        double angle{0.0};
        double curvature{0.0};
        double velocity{0.0};
        double acceleration{0.0};
        double steer{0.0};
        void print() const
        {
            printf("State:\n");
            printf(" -- time_stamp: %lf.\n", time_stamp);
            printf(" -- vec_position: (%lf, %lf).\n", vec_position[0], vec_position[1]);
            printf(" -- angle: %lf.\n", angle);
            printf(" -- curvature: %lf.\n", curvature);
            printf(" -- velocity: %lf.\n", velocity);
            printf(" -- acceleration: %lf.\n", acceleration);
            printf(" -- steer: %lf.\n", steer);
        }

        Eigen::Matrix<double, 3, 1> ToXYTheta() const
        {
            return Eigen::Matrix<double, 3, 1>(vec_position(0), vec_position(1), angle);
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

}  // namespace common

#endif //SRC_STATE_H
