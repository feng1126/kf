//
// Created by niew on 2021/02/22.
//  UKF滤波算法
//
#ifndef FILTER_UKF_H
#define FILTER_UKF_H

#include "filterBase.h"


namespace filter
{
	class filterUKF : public filterBase
	{
	public:
		filterUKF();
		~filterUKF();
		void predict(double dtime) override;
		void correction(const std::shared_ptr<filterMessage> &message) override;
		void SetFilterStateInitial(const std::shared_ptr<filterMessage>& message) override;

	private:
	    void UKF_Cholesky(Eigen::MatrixXd A, Eigen::MatrixXd & L, int cols);
		Eigen::MatrixXd mSigmaPoints;
		double mLambda;
		std::vector<double> mStateWeights;
		std::vector<double> mCovarWeights;
		Eigen::MatrixXd mWeightedCovarSqrt;
		int  mSigmaCount;
		int  mStateMemberX;
		int  mStateMemberY;
		int  mStateMemberZ;
		int  mStateMemberRoll;
		int  mStateMemberPitch;
		int  mStateMemberYaw;
		int  mStateMemberVx;
		int  mStateMemberVy;
		int  mStateMemberVz;
		int  mStateMemberVroll;
		int  mStateMemberVpitch;
		int  mStateMemberVyaw;
		int  mStateMemberAccX;
		int  mStateMemberAccY;
		int  mStateMemberAccZ;


	};
}

#endif