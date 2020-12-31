#ifndef SRC_BASICS_H
#define SRC_BASICS_H

#include "basics/tic_toc.h"
#include <cmath>
#include <stdio.h>
#include <vector>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#define BACKWARD_HAS_UNWIND 1
#define BACKWARD_HAS_DW 1
#include "backward.hpp"

#define LaneDegree 5
#define LaneDim 2
#define TrajectoryDegree 5
#define TrajectoryDim 2

enum ErrorType { kSuccess = 0, kWrongStatus, kIllegalInput, kUnknown };

template <typename T>
using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;

template <int N>
using vec_Vecf = vec_E<Eigen::Matrix<double,N,1>>;

namespace constants
{
    const double kBigEPS = 1e-1;

    const double kEPS = 1e-6;

    const double kSmallEPS = 1e-10;

    const double kPi = acos(-1.0);

    const double kInf = 1e20;

    const int kInvalidAgentId = -1;
    const int kInvalidLaneId = -1;
}


#endif //SRC_BASICS_H
