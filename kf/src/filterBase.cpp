#include "filterBase.h"




namespace filter
{
    filterBase::filterBase()
    {  
        m_steer_angle = 0;
    }

    filterBase::~filterBase()
    {
        
    }
    //基函数卡尔曼预测函数定义
    void filterBase::predict(double dtime)
    {

    }
    //基函数卡尔曼更新函数定义
    void filterBase::correction(const std::shared_ptr<filterMessage> &message)
    {
        
    }
    //基函数卡尔曼设置状态
    void filterBase::SetState(const Eigen::VectorXd & state)
    {
     
    }
    //基函数卡尔曼设置过程噪声矩阵
    void filterBase::SetProcessNoiseCovariance(const Eigen::MatrixXd & process_noise_covariance)
    {
        mProcessNoiseCovariance = process_noise_covariance;
    }
    //基类卡尔曼设置测量时间定义
    void filterBase::SetLastMeasurementTime(const double & last_measurement_time)
    {
        mLastMeasurementTime = last_measurement_time;
    }
    //基类卡尔曼设置测量误差矩阵定义
    void filterBase::SetEstimateErrorCovariance(const Eigen::MatrixXd & estimate_error_covariance)
    {
        mEstimateErrorCovariance = estimate_error_covariance;
    }
    //基类卡尔曼获取系统初始化标志位
    bool filterBase::GetFilterStateInitial()
    {
        return mInitialMeasurementState;
    }
    //基类虚函数设置系统初始化
	void filterBase::SetFilterStateInitial(const std::shared_ptr<filterMessage>& message)
	{
	}
    //基类虚函数获取系统上一时刻时间
    double filterBase::GetLastMeasurementTime()
    {
        return mLastMeasurementTime;
    }

    Eigen::Quaterniond filterBase::GetOdometryCoordinate()
    {
        return mOdometryCoordinate;
    }

    Eigen::Vector3d  filterBase::GetOdometryFirstPosition()
    {
        return mFirstStatePosition;
    }

    double filterBase::clampRotation(double rotation)
    {
        while (rotation > 2 *  EIGEN_PI)
        {
            rotation = rotation -  2.0 * EIGEN_PI;
        }

        while (rotation < 0)
        {
            rotation = rotation +  2.0 * EIGEN_PI;
        }
        return rotation;
    }

    void filterBase::wrapStateAngles()
    {
        mState(mStateMemberYaw) = clampRotation(mState(mStateMemberYaw));
    }

    //卡尔曼滤波算法系统初始化
    void filterBase::reset()
    {
        mInitialMeasurementState = false;
        mState.setZero();
        identity_.setIdentity();
        mTransferFunction.setIdentity();
        mTransferFunctionjacobian.setZero();

        mEstimateErrorCovariance.setIdentity();
        mLastMeasurementTime = 0;
        mProcessNoiseCovariance.setZero();
        mFirstStatePosition.setZero();
    }

    //卡尔曼滤波算法进行预测和更新
    void filterBase::filterProcess(const std::shared_ptr<filterMessage> &message)
    {
  
            if( message->timestamp > mLastMeasurementTime )
            {
              predict(message->timestamp);
            }
            correction(message);           
            if( message->timestamp > mLastMeasurementTime )
            {
              mLastMeasurementTime = message->timestamp;
            }       
    }

}