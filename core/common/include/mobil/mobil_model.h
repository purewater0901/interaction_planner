#ifndef SRC_MOBIL_MODEL_H
#define SRC_MOBIL_MODEL_H

#include "basics/basics.h"
#include "basics/semantics.h"
#include "idm/intelligent_driver_model.h"
#include "rss/rss_checker.h"

namespace common
{

    class MobilLaneChangingModel
    {
    public:
        static ErrorType GetMobilAccChangesOnCurrentLane(const FrenetState &cur_fs,
                                                         const Vehicle &leading_vehicle,
                                                         const FrenetState &leading_fs,
                                                         const Vehicle &following_vehicle,
                                                         const FrenetState &following_fs,
                                                         double *acc_o,
                                                         double *acc_o_tilda,
                                                         double *acc_c);

        static ErrorType GetMobilAccChangesOnTargetLane(const FrenetState &projected_cur_fs,
                                                        const Vehicle &leading_vehicle,
                                                        const FrenetState &leading_fs,
                                                        const Vehicle &following_vehicle,
                                                        const FrenetState &following_fs,
                                                        bool *is_lc_safe,
                                                        double *acc_n,
                                                        double *acc_n_tilda,
                                                        double *acc_c_tilda);

    private:
        static ErrorType GetDesiredAccelerationUsingIdm(const IntelligentDriverModel::Param &param,
                                                        const FrenetState &rear_fs,
                                                        const FrenetState &front_fs,
                                                        const bool &use_virtual_front,
                                                        double *acc);
    };

}  // namespace common


#endif //SRC_MOBIL_MODEL_H
