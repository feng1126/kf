//
 // Created on 2021/02/22.
 //  EKF滤波算法
 //

#ifndef FILTER_EKF_H
#define FILTER_EKF_H

#include "filterBase.h"

namespace filter
{
    class filterEKF : public filterBase
    {
    public:

        filterEKF();
        ~filterEKF();
        void predict(double dtime) override;
        void correction(const std::shared_ptr<filterMessage>& message) override;
        void SetFilterStateInitial(const std::shared_ptr<filterMessage>& message) override;
        void update_gps(Eigen::Vector3d pos, Eigen::Vector3d ypr);
        void update_vehicle(Eigen::Vector3d vehicle);
    };
}

#endif