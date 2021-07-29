/*
 * @Author: your name
 * @Date: 2021-02-18 13:37:00
 * @LastEditTime: 2021-07-29 09:56:29
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \code\src\fusionalgorithm\filterModule.cpp
 */

#include "filterModule.h"

namespace filter
{
	filterModule::filterModule()
	{
		pfilterQueue = new filterQueue();
		pfilter = new filterEKF();
		mInitialMessage = std::make_shared<filterMessage>();
		mStateSize = pfilter->getStateSize();
		bvalidInitial = false;
	}

	filterModule::~filterModule()
	{
		delete pfilter;
		delete pfilterQueue;
	}

	// MeasurementPtr filterModule::tranform(const double &timeStamp, const Eigen::Vector3d &UTM, const Eigen::Vector3d &YPR, const Eigen::Vector3d &Vehicle, const Eigen::Vector4d &cov)
	// {
	// 	double a, b, c;
	// 	m_ENU.getENU(UTM[0], UTM[1], UTM[2], a, b, c);
	// 	MeasurementPtr message = std::make_shared<filterMessage>();
	// 	message->timestamp = timeStamp;
	// 	message->v_observation.resize(mStateSize);
	// 	message->v_observation.setZero();
	// 	message->v_observation[StateMemberX] = a;
	// 	message->v_observation[StateMemberY] = b;
	// 	message->v_observation[StateMemberZ] = c;
	// 	message->v_observation[StateMemberYaw] = YPR[0];
	// 	message->v_flag.resize(mStateSize, 0);

	// 	message->v_flag[StateMemberX] = 1;
	// 	message->v_flag[StateMemberY] = 1;
	// 	message->v_flag[StateMemberZ] = 1;
	// 	message->v_flag[StateMemberYaw] = 1;
	// 	message->m_convariance.resize(mStateSize, mStateSize);
	// 	message->m_convariance.setZero();
	// 	message->m_convariance(StateMemberX, StateMemberX) = cov[0];
	// 	message->m_convariance(StateMemberY, StateMemberY) = cov[1];
	// 	message->m_convariance(StateMemberZ, StateMemberZ) = 0.001;
	// 	message->m_convariance(StateMemberYaw, StateMemberYaw) = cov[2] * EIGEN_PI / 180.0;
	// 	return message;
	// }

	// MeasurementPtr filterModule::tranform(const double &timeStamp, const Eigen::Vector3d &UTM, const Eigen::Vector3d &YPR)
	// {

	// 	Eigen::Vector3d position = UTM - mOdometry.translation();
	// 	MeasurementPtr message = std::make_shared<filterMessage>();
	// 	message->timestamp = timeStamp;
	// 	message->v_observation.resize(mStateSize);
	// 	message->v_observation.setZero();
	// 	message->v_observation[StateMemberX] = position[0];
	// 	message->v_observation[StateMemberY] = position[1];
	// 	message->v_observation[StateMemberYaw] = YPR[0];
	// 	message->v_flag.resize(mStateSize, 0);
	// 	message->v_flag[StateMemberX] = 1;
	// 	message->v_flag[StateMemberY] = 1;
	// 	message->v_flag[StateMemberYaw] = 1;
	// 	message->m_convariance.resize(mStateSize, mStateSize);
	// 	message->m_convariance.setZero();
	// 	message->m_convariance(StateMemberX, StateMemberX) = 0.001;
	// 	message->m_convariance(StateMemberY, StateMemberY) = 0.001;
	// 	message->m_convariance(StateMemberYaw, StateMemberYaw) = 0.001;
	// 	return message;
	// }

	// MeasurementPtr filterModule::tranform(const double &timeStamp, const Eigen::Vector3d &Vehicle)
	// {
	// 	MeasurementPtr message = std::make_shared<filterMessage>();
	// 	message->timestamp = timeStamp;
	// 	message->v_observation.resize(mStateSize);
	// 	message->v_observation.setZero();
	// 	message->v_observation[StateMemberVx] = Vehicle[0];
	// 	message->v_flag.resize(mStateSize, 0);
	// 	message->v_flag[StateMemberVx] = 1;
	// 	message->m_convariance.resize(mStateSize, mStateSize);
	// 	message->m_convariance.setZero();
	// 	message->m_convariance(StateMemberVx, StateMemberVx) = 0.1;
	// 	return message;
	// }

	void filterModule::ModuleParse()
	{
		bool bState = pfilter->GetFilterStateInitial();
		if (!bState)
		{
			if (bvalidInitial)
			{
				pfilter->SetFilterStateInitial(mInitialMessage);
				mOdometry = pfilter->GetOdometry();
			}
		}
		else
		{
			while (!pfilterQueue->empty())
			{
				observedMessagePtr observe;
				pfilterQueue->pop_up(observe);
				if (observe->id == 0)
				{
					double a, b, c;
					m_ENU.getENU(observe->LLA[0], observe->LLA[1], observe->LLA[2], a, b, c);
					MeasurementPtr message = std::make_shared<filterMessage>();
					message->timestamp = observe->timestamp;
					message->v_observation.resize(mStateSize);
					message->v_observation.setZero();
					message->v_observation[StateMemberX] = a;
					message->v_observation[StateMemberY] = b;
					message->v_observation[StateMemberZ] = c;
					message->v_observation[StateMemberYaw] = observe->YPR[0];
					message->v_flag.resize(mStateSize, 0);

					message->v_flag[StateMemberX] = 1;
					message->v_flag[StateMemberY] = 1;
					message->v_flag[StateMemberZ] = 1;
					message->v_flag[StateMemberYaw] = 1;
					message->m_convariance.resize(mStateSize, mStateSize);
					message->m_convariance.setZero();
					message->m_convariance(StateMemberX, StateMemberX) = observe->cov[0];
					message->m_convariance(StateMemberY, StateMemberY) = observe->cov[1];
					message->m_convariance(StateMemberZ, StateMemberZ) = 0.001;
					message->m_convariance(StateMemberYaw, StateMemberYaw) = observe->cov[2] * EIGEN_PI / 180.0;
					pfilter->filterProcess(message);
				}
				if (observe->id == 1)
				{
					double a, b, c;
					m_ENU.getENU(observe->LLA[0], observe->LLA[1], observe->LLA[2], a, b, c);
					MeasurementPtr message = std::make_shared<filterMessage>();
					message->timestamp = observe->timestamp;
					message->v_observation.resize(mStateSize);
					message->v_observation.setZero();
					message->v_observation[StateMemberX] = a;
					message->v_observation[StateMemberY] = b;
					message->v_observation[StateMemberZ] = c;
					message->v_flag.resize(mStateSize, 0);
					message->v_flag[StateMemberX] = 1;
					message->v_flag[StateMemberY] = 1;
					message->v_flag[StateMemberZ] = 1;
					message->m_convariance.resize(mStateSize, mStateSize);
					message->m_convariance.setZero();
					message->m_convariance(StateMemberX, StateMemberX) = observe->cov[0];
					message->m_convariance(StateMemberY, StateMemberY) = observe->cov[1];
					message->m_convariance(StateMemberZ, StateMemberZ) = 0.001;
					pfilter->filterProcess(message);
				}
				if (observe->id == 2)
				{
					MeasurementPtr message = std::make_shared<filterMessage>();
					message->timestamp = observe->timestamp;
					message->v_observation.resize(mStateSize);
					message->v_observation.setZero();
					message->v_observation[StateMemberVx] = observe->Vehicle[0];
					message->v_flag.resize(mStateSize, 0);
					message->v_flag[StateMemberVx] = 1;
					message->m_convariance.resize(mStateSize, mStateSize);
					message->m_convariance.setZero();
					message->m_convariance(StateMemberVx, StateMemberVx) = 0.1;
					pfilter->filterProcess(message);
				}
			}
		}
	}

	void filterModule::GetOdom(double& timeStamp, double& lat, double& lon, double& yaw)
	{
		float hei;
		Eigen::VectorXd curOdom = pfilter->GetCarPostion();
		yaw = curOdom[StateMemberYaw] - 0.5 * EIGEN_PI;
		if (yaw < 0)
			yaw = yaw + 2 * EIGEN_PI;
		m_ENU.getLLA(curOdom[StateMemberX], curOdom[StateMemberY], curOdom[StateMemberZ], lat, lon, hei);

		timeStamp = pfilter->GetLastMeasurementTime();
		printf("ekf_out:%lf,%.7f,%.7f,%f,%f \r\n", pfilter->GetLastMeasurementTime(), lat, lon, yaw, curOdom[StateMemberVx]);
	}

#if MPU
	void filterModule::GetOdom()
	{

		double timeStamp, lat, lon, yaw, alt;
		float hei;

		Eigen::VectorXd curOdom = pfilter->GetCarPostion();
		yaw = curOdom[StateMemberYaw] - 0.5 * EIGEN_PI;
		if (yaw < 0)
			yaw = yaw + 2 * EIGEN_PI;
		m_ENU.getLLA(curOdom[StateMemberX], curOdom[StateMemberY], curOdom[StateMemberZ], lat, lon, hei);
		DFPrintf("ekf_out:%lf,%.7f,%.7f,%f \r\n", pfilter->GetLastMeasurementTime(), lat, lon, yaw);
		ekfDataPtr ekfPtr = std::make_shared<ekfData>();
		ekfPtr->timeStamp = pfilter->GetLastMeasurementTime();
		ekfPtr->lat = lat;
		ekfPtr->lon = lon;
		ekfPtr->heading = yaw;
		if (SAFE_CHECK(p_EKF_publisher))
		{
			p_EKF_publisher->Publisher(ekfPtr);
		}
	}

	void filterModule::PublisherTopic()
	{
		if (m_pVHandleImp != NULL)
		{
			p_EKF_publisher = m_pVHandleImp->Publisher<void, ekfDataPtr>("ekf");
		}
	}

	void filterModule::SubscriberTopic()
	{
		m_pVHandleImp->Subscriber("GNSS", this, &filterModule::GNSSDataParse);
		m_pVHandleImp->Subscriber("vehicle", this, &filterModule::vehicleDataParse);
		m_pVHandleImp->Subscriber("lane", this, &filterModule::laneDataParse);
		m_pVHandleImp->Subscriber("imu", this, &filterModule::imuDataParse);

	}

	void filterModule::KTRun(void* arg)
	{
		ModuleParse();
		if (pfilter->GetFilterStateInitial())
		{
			GetOdom();
		}
	}

	void filterModule::GNSSDataParse(GNSSDataPtr gnssData)
	{
		double heading = gnssData->heading;
		heading = heading + 0.5 * EIGEN_PI;
		if (heading > 2 * EIGEN_PI)
			heading = heading - 2.0 * EIGEN_PI;
		DFPrintf("ekf_in_gnss:%f,%f,%d,%d,%d,%.7f,%.7f,%f,%f,%f,%f,%f,%f,%f\n", gnssData->timeStamp, gnssData->iTOW, gnssData->fix_type, gnssData->RTKSatus, gnssData->numSV, gnssData->dlatitude, gnssData->dlongitude, gnssData->dheight, gnssData->heading, gnssData->fvelocity, gnssData->vAcc, gnssData->hAcc, gnssData->headAcc, gnssData->sAcc);
		if (pfilter->GetFilterStateInitial())
		{
			observedMessagePtr observe = std::make_shared<observedMessage>();
			observe->id = 0;
			observe->timestamp = gnssData->timeStamp;
			observe->LLA = Eigen::Vector3d{ gnssData->dlatitude, gnssData->dlongitude, gnssData->dheight };
			observe->Vehicle = Eigen::Vector3d{ gnssData->fvelocity, 0, 0 };
			observe->YPR = Eigen::Vector3d{ heading, 0, 0 };
			observe->cov[0] = gnssData->hAcc;
			observe->cov[1] = gnssData->vAcc;
			observe->cov[2] = gnssData->headAcc;
			observe->cov[3] = gnssData->sAcc;
			pfilterQueue->push_back(observe);
		}
		else
		{
			mInitialMessage->v_observation.resize(mStateSize);
			mInitialMessage->timestamp = gnssData->timeStamp;
			m_ENU.setStart(gnssData->dlatitude, gnssData->dlongitude, gnssData->dheight);
			double a, b, c;
			m_ENU.getENU(gnssData->dlatitude, gnssData->dlongitude, gnssData->dheight, a, b, c);
			mInitialMessage->v_observation[StateMemberX] = a;
			mInitialMessage->v_observation[StateMemberY] = b;
			mInitialMessage->v_observation[StateMemberZ] = c;
			mInitialMessage->v_observation[StateMemberYaw] = heading;
			mInitialMessage->v_observation[StateMemberVx] = gnssData->fvelocity;
			bvalidInitial = true;
		}
		return;
	}

	void filterModule::vehicleDataParse(vehicleDataPtr vehicleData)
	{
		DFPrintf("ekf_in_vel:%lf,%f, \r\n", vehicleData->timeStamp, vehicleData->vehicle_v, vehicleData->vehicle_steer);
		if (pfilter->GetFilterStateInitial())
		{
			observedMessagePtr observe = std::make_shared<observedMessage>();
			observe->id = 2;
			observe->timestamp = vehicleData->timeStamp;
			observe->Vehicle = Eigen::Vector3d{ vehicleData->vehicle_v, 0, 0 };
			observe->steerAngle = vehicleData->vehicle_steer;
			pfilterQueue->push_back(observe);
		}
		return;
	}

	void filterModule::laneDataParse(laneDataPtr laneData)
	{
		DFPrintf("ekf_in_lane:%lf,%lf,%lf \r\n", laneData->timeStamp, laneData->lat, laneData->lon);
		if (pfilter->GetFilterStateInitial())
		{
			observedMessagePtr observe = std::make_shared<observedMessage>();
			observe->id = 1;
			observe->timestamp = laneData->timeStamp;
			observe->LLA = Eigen::Vector3d{ laneData->lat, laneData->lon, laneData->height };
			observe->cov[0] = laneData->covX;
			observe->cov[1] = laneData->covY;
			pfilterQueue->push_back(observe);
		}
		return;
	}

	void filterModule::imuDataParse(IMUDataPtr imuData)
	{
		DFPrintf("ekf_in_imu:%lf,%lf,%lf,%lf \r\n", imuData->timeStamp, imuData->acc_x, imuData->acc_y, imuData->acc_z);
	}

#else
	void filterModule::setObserve(observedMessagePtr observe)
	{
		if (pfilter->GetFilterStateInitial() == true)
		{
			pfilterQueue->push_back(observe);
		}
		else
		{
			mInitialMessage->v_observation.resize(mStateSize);
			mInitialMessage->v_observation.setZero();
			mInitialMessage->timestamp = observe->timestamp;
			m_ENU.setStart(observe->LLA[0], observe->LLA[1], observe->LLA[2]);
			double a, b, c;
			m_ENU.getENU(observe->LLA[0], observe->LLA[1], observe->LLA[2], a, b, c);
			mInitialMessage->v_observation[StateMemberX] = a;
			mInitialMessage->v_observation[StateMemberY] = b;
			mInitialMessage->v_observation[StateMemberZ] = c;
			mInitialMessage->v_observation[StateMemberYaw] = observe->YPR[0];
			mInitialMessage->v_observation[StateMemberVx] = observe->Vehicle[0];
			bvalidInitial = true;
		}
		ModuleParse();

		return;
	}

#endif

}
