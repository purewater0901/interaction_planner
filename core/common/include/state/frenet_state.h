#ifndef SRC_FRENET_STATE_H
#define SRC_FRENET_STATE_H

namespace common
{
    /**
     * @brief Definition of frenet state
     * @param vec_s, s coordinate
     * @param vec_d, d coordinate (derivative may vary)
     * @param is_lateral_independent, if true, vec_d is w.r.t t, if false, vec_d is
     * w.r.t s, default to be true (high speed mode)
     */
    struct FrenetState
    {
        enum InitType { kInitWithDt, kInitWithDs };
        double time_stamp{0.0};
        Eigen::Matrix<double, 3, 1> vec_s{Eigen::Matrix<double, 3, 1>::Zero()};
        Eigen::Matrix<double, 3, 1> vec_dt{Eigen::Matrix<double, 3, 1>::Zero()};
        Eigen::Matrix<double, 3, 1> vec_ds{Eigen::Matrix<double, 3, 1>::Zero()};
        bool is_ds_usable = true;

        FrenetState() = default;
        void Load(const Eigen::Matrix<double, 3, 1>& s, const Eigen::Matrix<double, 3, 1>& d, const InitType& type)
        {
            vec_s = s;
            if (type == kInitWithDt)
            {
                vec_dt = d;
                vec_ds[0] = vec_dt[0];
                if (fabs(vec_s[1]) > constants::kEPS)
                {
                    vec_ds[1] = vec_dt[1] / vec_s[1];
                    vec_ds[2] = (vec_dt[2] - vec_ds[1] * vec_s[2]) / (vec_s[1] * vec_s[1]);
                    is_ds_usable = true;
                }
                else
                {
                    vec_ds[1] = 0.0;
                    vec_ds[2] = 0.0;
                    is_ds_usable = false;
                }
            }
            else if (type == kInitWithDs)
            {
                vec_ds = d;
                vec_dt[0] = vec_ds[0];
                vec_dt[1] = vec_s[1] * vec_ds[1];
                vec_dt[2] = vec_ds[2] * vec_s[1] * vec_s[1] + vec_ds[1] * vec_s[2];
                is_ds_usable = true;
            }
            else
                assert(false);
        }

        FrenetState(const Eigen::Matrix<double, 3, 1>& s,
                    const Eigen::Matrix<double, 3, 1>& dt,
                    const Eigen::Matrix<double, 3, 1>& ds)
        {
            vec_s = s;
            vec_dt = dt;
            vec_ds = ds;
        }

        void print() const
        {
            printf("frenet state stamp: %lf.\n", time_stamp);
            printf("-- vec_s: (%lf, %lf, %lf).\n", vec_s[0], vec_s[1], vec_s[2]);
            printf("-- vec_dt: (%lf, %lf, %lf).\n", vec_dt[0], vec_dt[1], vec_dt[2]);
            printf("-- vec_ds: (%lf, %lf, %lf).\n", vec_ds[0], vec_ds[1], vec_ds[2]);
            if (!is_ds_usable) printf("-- warning: ds not usable.\n");
        }
    };

}  // namespace common

#endif //SRC_FRENET_STATE_H
