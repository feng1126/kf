/*
 * @Author: your name
 * @Date: 2021-07-02 10:01:50
 * @LastEditTime: 2021-07-02 14:41:28
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \code\src\fusionalgorithm\filterEKF.cpp
 */


#include "filterEKF.h"
namespace filter
{
    filterEKF::filterEKF()
    {

        mStateSize = 15;
        mTransferFunction.resize(mStateSize, mStateSize);
        mProcessNoiseCovariance.resize(mStateSize, mStateSize);
        mEstimateErrorCovariance.resize(mStateSize, mStateSize);
        mTransferFunctionjacobian.resize(mStateSize, mStateSize);
        mState.resize(mStateSize);
        identity_.resize(mStateSize, mStateSize);
        reset();

        mStateMemberX = 0;
        mStateMemberY = 1;
        mStateMemberZ = 2;
        mStateMemberYaw = 3;
        mStateMemberVx = 4;
        mProcessNoiseCovariance(StateMemberX, StateMemberX) = 0.05;
        mProcessNoiseCovariance(StateMemberY, StateMemberY) = 0.05;
        mProcessNoiseCovariance(StateMemberZ, StateMemberZ) = 0.05;
        mProcessNoiseCovariance(StateMemberYaw, StateMemberYaw) = 0.05;
        mProcessNoiseCovariance(StateMemberPitch, StateMemberPitch) = 0.05;
        mProcessNoiseCovariance(StateMemberRoll, StateMemberRoll) = 0.05;
        mProcessNoiseCovariance(StateMemberVx, StateMemberVx) = 0.05;

    }

    filterEKF::~filterEKF()
    {

    }
    //EKF初始化EKF模块初始坐标
    void filterEKF::SetFilterStateInitial(const std::shared_ptr<filterMessage>& message)
    {
        mLastMeasurementTime = message->timestamp;
        mFirstStatePosition[StateMemberX] = message->v_observation[StateMemberX];
        mFirstStatePosition[StateMemberY] = message->v_observation[StateMemberY];
        mFirstStatePosition[StateMemberZ] = message->v_observation[StateMemberZ];
        mOdometryCoordinate = YPR2Quaterniond(message->v_observation[mStateMemberYaw], message->v_observation[mStateMemberPitch], message->v_observation[mStateMemberRoll]);
        mState[mStateMemberVx] = message->v_observation[mStateMemberVx];
        mState[mStateMemberYaw] = message->v_observation[mStateMemberYaw];
        mState[mStateMemberPitch] = message->v_observation[mStateMemberPitch];
        mState[mStateMemberRoll] = message->v_observation[mStateMemberRoll];
        mInitialMeasurementState = true;
        mOdometry = Eigen::Isometry3d::Identity();
        mOdometry.rotate(mOdometryCoordinate.toRotationMatrix());
        mOdometry.pretranslate(mFirstStatePosition);
        mOdometryInv = mOdometry.inverse();


    }

    void filterEKF::update_vehicle(Eigen::Vector3d vehicle)
    {


        size_t update_size = 3;
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(update_size, mState.rows());
        H.block(0, 6, 3, 3) = Eigen::Matrix3d::Identity();
        Eigen::MatrixXd R(update_size, update_size);
        R.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity() * 0.01;
        Eigen::MatrixXd K = mEstimateErrorCovariance * H.transpose() * (H * mEstimateErrorCovariance * H.transpose() + R).inverse();

        Eigen::VectorXd innovationSubset(update_size);  // z - Hx
        innovationSubset[0] = vehicle[0] - mState[StateMemberVx];
        innovationSubset[1] = vehicle[1] - mState[StateMemberVy];
        innovationSubset[2] = vehicle[2] - mState[StateMemberVz];

        // std::cout << "R" << std::endl << R << std::endl;
         //std::cout << "H" << std::endl << H << std::endl;
//
        // std::cout << "K" << std::endl << K  t << std::endl;
         
         //std::cout << "mEstimateErrorCovariance" << std::endl << mEstimateErrorCovariance << std::endl;
        mState = mState +  K * innovationSubset;
        mEstimateErrorCovariance = (identity_ - K * H) * mEstimateErrorCovariance;
        //std::cout << "mEstimateErrorCovariance" << std::endl << mEstimateErrorCovariance << std::endl;

    }

    
    void filterEKF::update_gps(Eigen::Vector3d pos, Eigen::Vector3d ypr)
    {

        size_t update_size = 6;
        Eigen::MatrixXd H(update_size, mState.rows());
        H.setIdentity();
        H.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
        H.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity();
        Eigen::MatrixXd R(update_size, update_size);
        R.setIdentity();
        R.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity() * 0.01;
        R.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity() * 0.001;

        Eigen::MatrixXd K = mEstimateErrorCovariance * H.transpose() * (H * mEstimateErrorCovariance * H.transpose() + R).inverse();

        Eigen::VectorXd innovationSubset(update_size);  // z - Hx
        innovationSubset[0] = pos[0] - mState[StateMemberX];
        innovationSubset[1] = pos[1] - mState[StateMemberY];
        innovationSubset[2] = pos[2] - mState[StateMemberZ];
        innovationSubset[3] = ypr[0] - mState[StateMemberYaw];
        innovationSubset[4] = ypr[1] - mState[StateMemberPitch];
        innovationSubset[5] = ypr[2] - mState[StateMemberRoll];
  
        
         while (innovationSubset(3) < -PI)
         {
              innovationSubset(3) += TAU;
         }

          while (innovationSubset(3) > PI)
          {
             innovationSubset(3) -= TAU;
          }

          //std::cout << "innovationSubset" << std::endl << innovationSubset << std::endl;
         // std::cout << "(identity_ - K * H)" << std::endl << (identity_ - K * H) * mEstimateErrorCovariance << std::endl;
        mState.noalias() += K * innovationSubset;
        mEstimateErrorCovariance = (identity_ - K * H) * mEstimateErrorCovariance; //;* (identity_ - K * H).inverse() + K * R * K.inverse() ;//(I -KH)P(I - KH)' + KRK'
        //std::cout << "K" << std::endl << K << std::endl;
        //std::cout << "mEstimateErrorCovariance" << std::endl << mEstimateErrorCovariance << std::endl;
    }

    void filterEKF::predict(double dtime)
    {
        const double delta_sec = dtime - GetLastMeasurementTime();
        double yaw = mState(StateMemberYaw);
        double x_vel = mState(StateMemberVx);

        mTransferFunction(StateMemberX, StateMemberVx) = std::cos(yaw) * delta_sec;
        mTransferFunction(StateMemberY, StateMemberVx) = std::sin(yaw) * delta_sec;
       // mState[StateMemberYaw] = mState[StateMemberYaw] + x_vel * delta_sec * std::tan(m_steer_angle) / 4.0;

        mTransferFunctionjacobian = mTransferFunction;
        mTransferFunctionjacobian(StateMemberX, StateMemberYaw) = -x_vel * std::sin(yaw)  * delta_sec;
        mTransferFunctionjacobian(StateMemberY, StateMemberYaw) = x_vel * std::cos(yaw) * delta_sec;

        // (2) Project the state forward: x = Ax + Bu (really, x = f(x, u))
        mState = mTransferFunction * mState;

        wrapStateAngles();
        // (3) Project the error forward: P = J * P * J' + Q
        mEstimateErrorCovariance = (mTransferFunctionjacobian * mEstimateErrorCovariance * mTransferFunctionjacobian.transpose());
        mEstimateErrorCovariance.noalias() += mProcessNoiseCovariance;

        if (dtime > mLastMeasurementTime)
        {
            mLastMeasurementTime = dtime;
        }
       // std::cout << "mEstimateErrorCovariance" << std::endl << mEstimateErrorCovariance << std::endl;
    }

    void filterEKF::correction(const std::shared_ptr<filterMessage>& message)
    {
        std::vector<size_t> update_indices;
        for (size_t i = 0; i < message->v_flag.size(); ++i)
        {
            if (message->v_flag[i])
            {
                update_indices.push_back(i);
            }
        }

        size_t update_size = update_indices.size();

        Eigen::VectorXd statesubset(update_size);       // x (in most literature)
        Eigen::VectorXd measurementSubset(update_size);  // z
        Eigen::MatrixXd measurementCovarianceSubset(update_size, update_size);  // R
        Eigen::MatrixXd statetoMeasurementSubset(update_size, mState.rows());  // H
        Eigen::MatrixXd kalmanGainSubset(mState.rows(), update_size);          // K
        Eigen::VectorXd innovationSubset(update_size);  // z - Hx

        statesubset.setZero();
        measurementSubset.setZero();
        measurementCovarianceSubset.setZero();
        statetoMeasurementSubset.setZero();
        kalmanGainSubset.setZero();
        innovationSubset.setZero();

        for (size_t i = 0; i < update_size; ++i)
        {
            measurementSubset(i) = message->v_observation[update_indices[i]];
            statesubset(i) = mState(update_indices[i]);

            for (size_t j = 0; j < update_size; ++j)
            {
                measurementCovarianceSubset(i, j) = message->m_convariance(update_indices[i], update_indices[j]);
            }

            if (measurementCovarianceSubset(i, i) < 0)
            {
                measurementCovarianceSubset(i, i) = std::fabs(measurementCovarianceSubset(i, i));
            }

            if (measurementCovarianceSubset(i, i) < 1e-9)
            {
                measurementCovarianceSubset(i, i) = 1e-9;
            }
        }

        for (size_t i = 0; i < update_size; ++i)
        {
            statetoMeasurementSubset(i, update_indices[i]) = 1;
        }


        //measurementCovarianceSubset.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity() * 0.01;
        //measurementCovarianceSubset.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity() * 0.001;
        // (1) Compute the Kalman gain: K = (PH') / (HPH' + R)
        Eigen::MatrixXd pht = mEstimateErrorCovariance * statetoMeasurementSubset.transpose();
        Eigen::MatrixXd hphr_inverse = (statetoMeasurementSubset * pht + measurementCovarianceSubset).inverse();
        //Eigen::MatrixXd hphr_inverse = (statetoMeasurementSubset * pht ).inverse();
        kalmanGainSubset.noalias() = pht * hphr_inverse;
        //std::cout << measurementCovarianceSubset << std::endl;
        innovationSubset = (measurementSubset - statesubset);

        // Wrap angles in the innovation
        for (size_t i = 0; i < update_size; ++i)
        {
            if (update_indices[i] == StateMemberYaw)
            {
                while (innovationSubset(i) < -PI)
                {
                    innovationSubset(i) += TAU;
                }

                while (innovationSubset(i) > PI)
                {
                    innovationSubset(i) -= TAU;
                }
            }
        }

        // (3) Apply the gain to the difference between the state and measurement: x = x + K(z - Hx)
        mState.noalias() += kalmanGainSubset * innovationSubset;

        // (4) Update the estimate error covariance using the Joseph form: (I -KH)P(I - KH)' + KRK'
           //(I -KH)*P
        mEstimateErrorCovariance.noalias() = (identity_ - kalmanGainSubset * statetoMeasurementSubset) * mEstimateErrorCovariance * (identity_ - kalmanGainSubset * statetoMeasurementSubset).transpose() + kalmanGainSubset * measurementCovarianceSubset * kalmanGainSubset.transpose();
        wrapStateAngles();
        //std::cout << "mEstimateErrorCovariance" << std::endl << mEstimateErrorCovariance << std::endl;

    }
}