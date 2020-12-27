#ifndef SRC_WAYPOINT_H
#define SRC_WAYPOINT_H

#include "basics/basics.h"
namespace common
{
    template <int N_DIM>
    struct Waypoint
    {
        Eigen::Matrix<double, N_DIM, 1> pos;
        Eigen::Matrix<double, N_DIM, 1> vel;
        Eigen::Matrix<double, N_DIM, 1> acc;
        Eigen::Matrix<double, N_DIM, 1> jrk;
        double t{0.0};
        bool fix_pos = false;
        bool fix_vel = false;
        bool fix_acc = false;
        bool fix_jrk = false;
        bool stamped = false;
    };

    typedef Waypoint<2> Waypoint2D;
    typedef Waypoint<3> Waypoint3D;
}  // namespace common

#endif //SRC_WAYPOINT_H
