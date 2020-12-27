#ifndef SRC_PLANNER_H
#define SRC_PLANNER_H

#include <string>

#include "basics/basics.h"

namespace planning
{
/**
 * @brief A general base class for different planners including
 * path/motion/behavior planners
 */
    class Planner
    {
    public:
        Planner() = default;

        virtual ~Planner() = default;

        virtual std::string Name() = 0;

        virtual ErrorType Init(const std::string config) = 0;

        virtual ErrorType RunOnce() = 0;
    };

}  // namespace planning

#endif //SRC_PLANNER_H
