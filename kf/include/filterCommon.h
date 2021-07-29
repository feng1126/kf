/*
 * @Author: your name
 * @Date: 2021-07-26 15:28:24
 * @LastEditTime: 2021-07-29 09:56:51
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \code\include\fusionalgorithm\filterCommon.h
 */
 //
 // Created by niew on 2021/02/22.
 // 卡尔曼滤波算法公共模块
 //
#ifndef Filter_COMMON_H
#define Filter_COMMON_H

#include <stddef.h>
#include <string.h>
#include <vector>
#include <map>
#include <iostream>
#include "Eigen/Dense"
#include <string>
#include "UTMTran.h"
#include <memory>
#include "enu.h"

namespace filter
{
    class observedMessage
    {

    public:
        int id = 0;
        double timestamp = 0.0f;
        float steerAngle;
        float cov[4];
        Eigen::Vector3d UTM;
        Eigen::Vector3d LLA;
        Eigen::Vector3d Vehicle;
        Eigen::Vector3d YPR;
        bool operator()(const std::shared_ptr<observedMessage> a, const std::shared_ptr<observedMessage> b)
        {
            return a->timestamp < b->timestamp;
        }
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    typedef std::shared_ptr<observedMessage> observedMessagePtr;

    class filterMessage
    {
    public:
        double timestamp;
        std::vector<bool> v_flag;
        Eigen::VectorXd v_observation;
        Eigen::MatrixXd m_convariance;

        bool operator()(const std::shared_ptr<filterMessage> a, const std::shared_ptr<filterMessage> b)
        {
            return a->timestamp > b->timestamp;
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    }; //融合滤波器输入结构体

    typedef std::shared_ptr<filterMessage> MeasurementPtr;
    const double TAU = 6.283185307179587;
    //将欧拉角转换为四元数表示
    Eigen::Quaterniond YPR2Quaterniond(float yaw, float pitch, float roll);
    //将四元数转换为欧拉角表示
    Eigen::Vector3d ToEulerAngles(const Eigen::Quaterniond& q);

}

enum StateMembers
{
    StateMemberX = 0,
    StateMemberY,
    StateMemberZ,
    StateMemberYaw,
    StateMemberPitch,
    StateMemberRoll,
    StateMemberVx,
    StateMemberVy,
    StateMemberVz,
    StateMemberVroll,
    StateMemberVpitch,
    StateMemberVyaw,
    StateMemberAx,
    StateMemberAy,
    StateMemberAz
};

#endif