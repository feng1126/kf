//
// Created by niew on 2021/02/22.
// 卡尔曼滤波算法基类
//

#ifndef ALGORIT_BASE_H
#define ALGORIT_BASE_H

#include "filterCommon.h"

//卡尔曼滤波算法基类
namespace filter
{
    class filterBase
    {
        public:
         filterBase();
        ~filterBase();
        //卡尔曼滤波算法预测
        virtual void predict(const double dtime);
        //卡尔曼滤波算法更新
        virtual void correction(const std::shared_ptr<filterMessage> &message);
        //卡尔曼滤波算法设置初始状态
        virtual void SetFilterStateInitial(const std::shared_ptr<filterMessage>& message);
        //卡尔曼滤波算法调用接口
        void filterProcess(const std::shared_ptr<filterMessage> &message);
        //卡尔曼滤波算法状态变量
        void SetState(const Eigen::VectorXd & state);
        //卡尔曼滤波算法Q噪声矩阵
        void SetProcessNoiseCovariance(const Eigen::MatrixXd & process_noise_covariance);
        //卡尔曼滤波算法设置测量时间
        void SetLastMeasurementTime(const double & last_measurement_time);
        //卡尔曼滤波算法设置P估计矩阵
        virtual void SetEstimateErrorCovariance(const Eigen::MatrixXd & estimate_error_covariance);
        //卡尔曼滤波算法获取系统初始化状态
        bool GetFilterStateInitial();
        //卡尔曼滤波算法获取上一时刻时间
        double GetLastMeasurementTime();
        //卡尔曼滤波算法对角度进行归一化
        void wrapStateAngles();
        double clampRotation(double rotation);
        void reset();
        void setSteerAngle(float steerAngle) { m_steer_angle  = steerAngle;}
        //卡尔曼滤波算法获取当前坐标系
        Eigen::Quaterniond GetOdometryCoordinate();
        //卡尔曼滤波算法获取初始化位置
        Eigen::Vector3d    GetOdometryFirstPosition();
        //卡尔曼滤波算法获取滤波后的状态变量
		const Eigen::VectorXd  GetCarPostion() { return mState; };
        //卡尔曼滤波算法获取滤波后的状态变量
        /*const int getStateX() { return mStateMemberX; };
        const int getStateY() { return mStateMemberY; };
        const int getStateYaw() { return mStateMemberYaw; };
        const int getStateVx() { return mStateMemberVx; };*/
        const int getStateSize() { return mStateSize; };
        //卡尔曼滤波算法获取系统的初始化坐标系
        const Eigen::Isometry3d GetOdometry() { return mOdometry; };
        //卡尔曼滤波算法获取系统的初始化坐标系（求逆）
        const Eigen::Isometry3d GetOdometryInv() { return mOdometryInv; };


        //卡尔曼滤波算法成员变量上一时刻时间
        double mLastMeasurementTime;
        //卡尔曼滤波算法成员系统初始化状态
        bool   mInitialMeasurementState;
        //卡尔曼滤波算法成员变量EKF雅克比矩阵
        Eigen::MatrixXd mTransferFunctionjacobian;
        //卡尔曼滤波算法成员变量状态转移矩阵F
        Eigen::MatrixXd mTransferFunction;
        //卡尔曼滤波算法成员变量系统估计矩阵P
        Eigen::MatrixXd mEstimateErrorCovariance;
        //卡尔曼滤波算法成员变量过程噪声矩阵
        Eigen::MatrixXd mProcessNoiseCovariance;
        //卡尔曼滤波算法成员变量系统状态
        Eigen::VectorXd mState;
        //卡尔曼滤波算法成员变量坐标系姿态
        Eigen::Quaterniond mOdometryCoordinate;
        Eigen::Quaterniond mCurrentCoordinate;
        //卡尔曼滤波算法成员变量初始化位置
        Eigen::Vector3d mFirstStatePosition;
        Eigen::MatrixXd identity_; 
        Eigen::Isometry3d  mOdometry;
        Eigen::Isometry3d  mOdometryInv;
        int  mStateSize;

        float m_steer_angle;
    };
}
#endif