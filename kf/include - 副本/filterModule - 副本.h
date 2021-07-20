/*
 * @Author: your name
 * @Date: 2021-05-12 09:25:19
 * @LastEditTime: 2021-07-01 16:28:26
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \code\include\fusionalgorithm\filterModule.h
 */
 //
 // Created by niew on 2021/02/22.
 //
#ifndef filter_MODULE_H
#define filter_MODULE_H

#define MPU 0

#include "filterQueue.h"
#include "filterEKF.h"
#include "filterUKF.h"
#include "filterCommon.h"
#include "filterQueue.h"

#if MPU
#include "KTBaseModule.h"
#include "VHandleImp.h"
#include "KTRunable.h"
#include "sensorData.h"
#include "VHandle.h"
#include "systemDebug.h"
#endif


namespace filter
{
#if MPU
    class filterModule : public task_t::KTBaseModule, public  task_t::VHandleImp, public task_t::KTRunable
#else
    class filterModule
#endif
    {
    public:

        filterModule();
        ~filterModule();
#if MPU
        void ModuleParse() override;
        void PublisherTopic() override;
        void SubscriberTopic() override;
        void KTRun(void* arg) override;
#else
        void ModuleParse();
        void setObserve(observedMessagePtr observe);
#endif
        void GetOdom();
        void GetOdom(double& timeStamp, double& lat, double& lon, double& yaw);


    private:

#if MPU
        task_t::VHandle* pHandle;
        void GNSSDataParse(GNSSDataPtr gnssData);
        void vehicleDataParse(vehicleDataPtr vehicleData);

#endif
        filterQueue* pfilterQueue;
        filterBase* mpFilter;
        MeasurementPtr mInitialMessage;
        bool bvalidInitial;
        Eigen::Isometry3d mOdometryInv;
        Eigen::Isometry3d mOdometry;


        MeasurementPtr tranform(const double& timeStamp, const Eigen::Vector3d& UTM, const Eigen::Vector3d& YPR, const Eigen::Vector3d& Vehicle);
        MeasurementPtr tranform(const double& timeStamp, const Eigen::Vector3d& UTM, const Eigen::Vector3d& YPR);
        MeasurementPtr tranform(const double& timeStamp, const Eigen::Vector3d& Vehicle);

        int mStateMemberX;
        int mStateMemberY;
        int mStateMemberYaw;
        int mStateMemberVx;
        int mStateSIze;

    };

}

#endif
