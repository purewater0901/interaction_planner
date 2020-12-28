#ifndef SRC_CIRCLE_ARC_H
#define SRC_CIRCLE_ARC_H

#include <cassert>
#include <iostream>
#include <vector>

#include "basics/basics.h"

namespace common
{
    class CircleArc {
    public:
        CircleArc()  = default;
        CircleArc(const Eigen::Matrix<double, 3, 1> &start_state, const double &curvature,
                  const double &arc_length);
        ~CircleArc() = default;

        inline double curvature() const { return curvature_; }
        inline double arc_length() const { return arc_length_; }
        inline double central_angle() const { return central_angle_; }

        inline Eigen::Matrix<double, 3, 1> start_state() const { return start_state_; }
        inline Eigen::Matrix<double, 3, 1> final_state() const { return final_state_; }
        inline Eigen::Matrix<double,2,1> center() const { return center_; }

        double x_d0(const double s) const;
        double x_d1(const double s) const;
        double x_d2(const double s) const;

        double y_d0(const double s) const;
        double y_d1(const double s) const;
        double y_d2(const double s) const;

        double theta_d0(const double s) const;
        double theta_d1(const double s) const;

        // * Tangent
        double tx_d0(const double s) const { return x_d1(s); }
        double ty_d0(const double s) const { return y_d1(s); }

        // * Normal Vector (Left)
        double nx_d0(const double s) const { return -this->ty_d0(s); }
        double ny_d0(const double s) const { return this->tx_d0(s); }

        double x_offs_d0(const double s, const double offs) const;
        double x_offs_d1(const double s, const double offs) const;

        double y_offs_d0(const double s, const double offs) const;
        double y_offs_d1(const double s, const double offs) const;

        double theta_offs_d0(const double s, const double offs) const;

        void GetSampledStates(const double s_step,
                              std::vector<Eigen::Matrix<double, 3, 1>> *p_sampled_states) const;

    private:
        Eigen::Matrix<double, 3, 1> start_state_;
        Eigen::Matrix<double, 3, 1> final_state_;

        double curvature_;
        double arc_length_;

        bool is_arc_ = true;
        double central_angle_;
        Eigen::Matrix<double,2,1> center_;
    };  // CircleArc

}

#endif //SRC_CIRCLE_ARC_H
