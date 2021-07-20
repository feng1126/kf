#include "filterUKF.h"

namespace filter
{
    filterUKF::filterUKF()
    {

        mStateSize = 4;
        mTransferFunction.resize(mStateSize, mStateSize);
        mProcessNoiseCovariance.resize(mStateSize, mStateSize);
        mEstimateErrorCovariance.resize(mStateSize, mStateSize);
        mTransferFunctionjacobian.resize(mStateSize, mStateSize);
        mWeightedCovarSqrt.resize(mStateSize, mStateSize);
        mWeightedCovarSqrt.setZero();
        mState.resize(mStateSize);
        identity_.resize(mStateSize, mStateSize);
        reset();

        mStateMemberX = 0;
        mStateMemberY = 1;
        mStateMemberYaw = 2;
        mStateMemberVx = 3;



        mProcessNoiseCovariance(mStateMemberX, mStateMemberX) = 0.05;
        mProcessNoiseCovariance(mStateMemberY, mStateMemberY) = 0.05;
        mProcessNoiseCovariance(mStateMemberYaw, mStateMemberYaw) = 0.06;
        mProcessNoiseCovariance(mStateMemberVx, mStateMemberVx) = 0.025;

        mSigmaCount = 2 * mStateSize + 1;
        mSigmaPoints.resize(mStateSize, mSigmaCount);

        double alpha = 0.001;
        double kappa = 0.1;
        double beta = 2;
        // Prepare constants
        //mLambda = alpha * alpha * (mStateSize + kappa) - mStateSize;
        mLambda = 3 - mStateSize;

        mStateWeights.resize(mSigmaCount);
        mCovarWeights.resize(mSigmaCount);

        mStateWeights[0] = mLambda / (mStateSize + mLambda);
        mCovarWeights[0] = mStateWeights[0] + (1 - (alpha * alpha) + beta);
        mSigmaPoints.col(0).setZero();
        for (size_t i = 1; i < mSigmaCount; ++i)
        {
            mSigmaPoints.col(i).setZero();
            mStateWeights[i] = 1 / (2 * (mStateSize + mLambda));
            mCovarWeights[i] = mStateWeights[i];
        }
    }

    filterUKF::~filterUKF()
    {
    }

    void filterUKF::predict(double dtime)
    {

        const double delta = dtime - GetLastMeasurementTime();
        double yaw = mState(mStateMemberYaw);

        // 根据车辆运动学计算转移状态
        mTransferFunction(mStateMemberX, mStateMemberVx) = std::cos(yaw) * delta;
        mTransferFunction(mStateMemberY, mStateMemberVx) = std::sin(yaw) * delta;

        //通过LL分解计算均值;
        Eigen::MatrixXd temp_matrix = (mStateSize + mLambda) * mEstimateErrorCovariance;
        UKF_Cholesky(temp_matrix, mWeightedCovarSqrt, temp_matrix.cols());

        // 计算sigma点2n+1个
        mSigmaPoints.col(0) = mTransferFunction * mState;
        for (size_t sigmaInd = 0; sigmaInd < mStateSize; ++sigmaInd)
        {
            mSigmaPoints.col(sigmaInd + 1) = mTransferFunction * (mState + mWeightedCovarSqrt.col(sigmaInd));
            mSigmaPoints.col(sigmaInd + 1 + mStateSize) = mTransferFunction * (mState - mWeightedCovarSqrt.col(sigmaInd));
        }

        // 通过sigma点以及权重计算估计
        mState.setZero();
        for (size_t sigmaInd = 0; sigmaInd < mSigmaCount; ++sigmaInd)
        {
            mState.noalias() += mStateWeights[sigmaInd] * mSigmaPoints.col(sigmaInd);
        }

        //通过sigma点计算估计误差
        mEstimateErrorCovariance.setZero();
        Eigen::VectorXd sigmaDiff(mStateSize);
        for (size_t sigmaInd = 0; sigmaInd < mSigmaCount; ++sigmaInd)
        {
            sigmaDiff = (mSigmaPoints.col(sigmaInd) - mState);
            mEstimateErrorCovariance.noalias() += mCovarWeights[sigmaInd] * (sigmaDiff * sigmaDiff.transpose());
        }

        mEstimateErrorCovariance.noalias() += delta * mProcessNoiseCovariance;
        wrapStateAngles();

        if (dtime > mLastMeasurementTime)
        {
            mLastMeasurementTime = dtime;
        }
    }

    /*UKF correction */
    void filterUKF::correction(const std::shared_ptr<filterMessage> &message)
    {
        // 判断有多少个观测量
        std::vector<size_t> update_indices;
        for (size_t i = 0; i < message->v_flag.size(); ++i)
        {
            if (message->v_flag[i])
            {
                if (fabs(message->v_observation[i]) > 1e-9)
                {
                    update_indices.push_back(i);
                }
            }
        }

        size_t update_size = update_indices.size();
        Eigen::VectorXd stateSubset(update_size);                              // x (in most literature)
        Eigen::VectorXd measurement_subset(update_size);                       // z
        Eigen::MatrixXd measurementCovarianceSubset(update_size, update_size); // R
        Eigen::MatrixXd stateToMeasurementSubset(update_size, mStateSize);     // H
        Eigen::MatrixXd kalmanGainSubset(mStateSize, update_size);             // K
        Eigen::VectorXd innovationSubset(update_size);                         // z - Hx
        Eigen::VectorXd predictedMeasurement(update_size);
        Eigen::VectorXd sigmaDiff(update_size);
        Eigen::MatrixXd Pzk(update_size, update_size);
        Eigen::MatrixXd Pxz(mStateSize, update_size);

        std::vector<Eigen::VectorXd> sigmaPointMeasurements(mSigmaCount, Eigen::VectorXd(update_size));

        stateSubset.setZero();
        measurement_subset.setZero();
        measurementCovarianceSubset.setZero();
        stateToMeasurementSubset.setZero();
        kalmanGainSubset.setZero();
        innovationSubset.setZero();
        predictedMeasurement.setZero();
        Pzk.setZero();
        Pxz.setZero();
        for (size_t i = 0; i < update_size; ++i)
        {
            measurement_subset(i) = message->v_observation[update_indices[i]];
            stateSubset(i) = mState(update_indices[i]);

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
            stateToMeasurementSubset(i, update_indices[i]) = 1;
        }

        for (size_t sigmaInd = 0; sigmaInd < mSigmaCount; ++sigmaInd)
        {

            sigmaPointMeasurements[sigmaInd] = stateToMeasurementSubset * mSigmaPoints.col(sigmaInd);
            predictedMeasurement.noalias() += mStateWeights[sigmaInd] * sigmaPointMeasurements[sigmaInd];
        }

        for (size_t sigmaInd = 0; sigmaInd < mSigmaCount; ++sigmaInd)
        {
            sigmaDiff = sigmaPointMeasurements[sigmaInd] - predictedMeasurement;
            Pzk.noalias() += mCovarWeights[sigmaInd] * (sigmaDiff * sigmaDiff.transpose());
            Pxz.noalias() += mCovarWeights[sigmaInd] * ((mSigmaPoints.col(sigmaInd) - mState) * sigmaDiff.transpose());
        }

        //  K = P_xz * (P_zk + R)^-1
        Eigen::MatrixXd invInnovCov = (Pzk + measurementCovarianceSubset).inverse();
        kalmanGainSubset = Pxz * invInnovCov;

        // x = x + K(z - z_hat)
        innovationSubset = (measurement_subset - predictedMeasurement);

        // Wrap angles in the innovation
        for (size_t i = 0; i < update_size; ++i)
        {
            if (update_indices[i] == mStateMemberYaw)
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

        mState.noalias() += kalmanGainSubset * innovationSubset;

        // P = P - (K * P_zk * K')
        mEstimateErrorCovariance.noalias() -= (kalmanGainSubset * Pzk * kalmanGainSubset.transpose());
        wrapStateAngles();
    }

    void filterUKF::SetFilterStateInitial(const std::shared_ptr<filterMessage> &message)
    {

        mLastMeasurementTime = message->timestamp;
        mFirstStatePosition[mStateMemberX] = message->v_observation[mStateMemberX];
        mFirstStatePosition[mStateMemberY] = message->v_observation[mStateMemberY];
        mOdometryCoordinate = YPR2Quaterniond(message->v_observation[mStateMemberYaw], 0, 0);
        mState[mStateMemberVx] = message->v_observation[mStateMemberVx];
        mInitialMeasurementState = true;
        mOdometry = Eigen::Isometry3d::Identity();
        mOdometry.rotate(mOdometryCoordinate.toRotationMatrix());
        mOdometry.pretranslate(mFirstStatePosition);
        mOdometryInv = mOdometry.inverse();
    }

    void filterUKF::UKF_Cholesky(Eigen::MatrixXd A, Eigen::MatrixXd &L, int cols)
    {

#if 1
        for (int k = 0; k < cols; k++)
        {
            double sum = 0.0;

            for (int i = 0; i < k; i++)

                sum += L(k, i) * L(k, i);

            sum = A(k, k) - sum;

            L(k, k) = sqrt(sum > 0.0 ? sum : 0.0);

            for (int i = k + 1; i < cols; i++)
            {
                sum = 0.0;
                for (int j = 0; j < k; j++)
                    sum += L(i, j) * L(k, j);
                L(i, k) = (A(i, k) - sum) / L(k, k);
            }
            for (int j = 0; j < k; j++)
                L(j, k) = 0.0;
        }
#else
        for (int j = 0; j < cols; ++j)
        {
            double d = 0.0;
            for (int k = 0; k < j; ++k)
            {
                double s = 0;
                for (int i = 0; i < k; ++i)
                    s += L(k, i) * L(j, i);

                L(j, k) = s = (A(j, k) - s) / L(k, k);
                d = d + s * s;
                //spd = spd && (A[k][j] == A[j][k]);
            }

            d = A(j, j) - d;
            //spd = spd && ( d > 0 );

            L(j, j) = sqrt(d > 0.0 ? d : 0.0);
            for (int k = j + 1; k < cols; ++k)
                L(j, k) = 0.0;
        }
#endif
    }

}