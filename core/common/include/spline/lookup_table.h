#ifndef SRC_LOOKUP_TABLE_H
#define SRC_LOOKUP_TABLE_H

#include <map>
#include "basics/basics.h"
#include "math/calculations.h"

namespace common
{
    /**
     * @brief compute the mapping matrix (from coeff to derivative) inverse
     * @note  this is for monotonic basis
     */
    Eigen::Matrix<double, 6, 6> GetAInverse(const double& t);

    extern std::map<double, Eigen::Matrix<double,6,6>, std::less<double>,
    Eigen::aligned_allocator<std::pair<const double, Eigen::Matrix<double,6,6>>>> kTableAInverse;

}

#endif //SRC_LOOKUP_TABLE_H
