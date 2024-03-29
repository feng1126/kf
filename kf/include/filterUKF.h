/*
 * @Author: your name
 * @Date: 2021-07-27 16:04:34
 * @LastEditTime: 2021-07-27 16:04:35
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \code\include\fusionalgorithm\filterUKF.h
 */
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

	};
}

#endif