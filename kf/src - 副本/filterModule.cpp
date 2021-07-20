/*
 * @Author: your name
 * @Date: 2021-02-18 13:37:00
 * @LastEditTime: 2021-07-02 14:30:17
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
		mStateMemberX = pfilter->getStateX();
		mStateMemberY = pfilter->getStateY();
		mStateMemberYaw = pfilter->getStateYaw();
		mStateMemberVx = pfilter->getStateVx();
		mStateSIze = pfilter->getStateSize();
		bvalidInitial = false;
	}

	filterModule::~filterModule()
	{
		delete pfilter;
		delete pfilterQueue;
	}

	MeasurementPtr filterModule::tranform(const double& timeStamp, const Eigen::Vector3d& UTM, const Eigen::Vector3d& YPR, const Eigen::Vector3d& Vehicle)
	{
		Eigen::Vector3d position = UTM - mOdometry.translation();
		MeasurementPtr message = std::make_shared<filterMessage>();
		message->timestamp = timeStamp;
		message->v_observation.resize(mStateSIze);
		message->v_observation.setZero();
		message->v_observation[mStateMemberX] = position[0];
		message->v_observation[mStateMemberY] = position[1];
		message->v_observation[mStateMemberYaw] = YPR[0];
		message->v_observation[mStateMemberVx] = Vehicle[0];
		message->v_flag.resize(mStateSIze, 0);
		message->v_flag[mStateMemberX] = 1;
		message->v_flag[mStateMemberY] = 1;
		message->v_flag[mStateMemberYaw] = 1;
		message->v_flag[mStateMemberVx] = 1;
		message->m_convariance.resize(mStateSIze, mStateSIze);
		message->m_convariance.setZero();
		message->m_convariance(mStateMemberX, mStateMemberX) = 0.5;
		message->m_convariance(mStateMemberY, mStateMemberY) = 0.5;
		message->m_convariance(mStateMemberYaw, mStateMemberYaw) = 0.01;
		message->m_convariance(mStateMemberVx, mStateMemberVx) = 0.5;
		return message;
	}

	MeasurementPtr filterModule::tranform(const double& timeStamp, const Eigen::Vector3d& UTM, const Eigen::Vector3d& YPR)
	{

		Eigen::Vector3d position = UTM - mOdometry.translation();
		MeasurementPtr message = std::make_shared<filterMessage>();
		message->timestamp = timeStamp;
		message->v_observation.resize(mStateSIze);
		message->v_observation.setZero();
		message->v_observation[mStateMemberX] = position[0];
		message->v_observation[mStateMemberY] = position[1];
		message->v_observation[mStateMemberYaw] = YPR[0];
		message->v_flag.resize(mStateSIze, 0);
		message->v_flag[mStateMemberX] = 1;
		message->v_flag[mStateMemberY] = 1;
		message->v_flag[mStateMemberYaw] = 1;
		message->v_flag[mStateMemberVx] = 0;
		message->m_convariance.resize(mStateSIze, mStateSIze);
		message->m_convariance.setZero();
		message->m_convariance(mStateMemberX, mStateMemberX) = 0.05;
		message->m_convariance(mStateMemberY, mStateMemberY) = 0.05;
		message->m_convariance(mStateMemberYaw, mStateMemberYaw) = 0.01;
		return message;
	}

	MeasurementPtr filterModule::tranform(const double& timeStamp, const Eigen::Vector3d& Vehicle)
	{
		MeasurementPtr message = std::make_shared<filterMessage>();
		message->timestamp = timeStamp;
		message->v_observation.resize(mStateSIze);
		message->v_observation.setZero();
		message->v_observation[mStateMemberVx] = Vehicle[0];
		message->v_flag.resize(mStateSIze, 0);
		message->v_flag[mStateMemberX] = 0;
		message->v_flag[mStateMemberY] = 0;
		message->v_flag[mStateMemberYaw] = 0;
		message->v_flag[mStateMemberVx] = 1;
		message->m_convariance.resize(mStateSIze, mStateSIze);
		message->m_convariance.setZero();
		message->m_convariance(mStateMemberVx, mStateMemberVx) = 0.1;
		return message;
	}

	void filterModule::ModuleParse()
	{
		bool bState = pfilter->GetFilterStateInitial();
		if (!bState)
		{
			if (bvalidInitial)
			{
				pfilter->SetFilterStateInitial(mInitialMessage);
				mOdometryInv = pfilter->GetOdometryInv();
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
					MeasurementPtr measure = tranform(observe->timestamp, observe->UTM, observe->YPR, observe->Vehicle);
					pfilter->filterProcess(measure);
				}
				if (observe->id == 1)
				{
					MeasurementPtr measure = tranform(observe->timestamp, observe->UTM, observe->YPR);
					pfilter->filterProcess(measure);
					pfilter->predict(measure->timestamp);

				}
				if (observe->id == 2)
				{
					MeasurementPtr measure = tranform(observe->timestamp, observe->Vehicle);
					//pfilter->filterProcess(measure);
					pfilter->predict(measure->timestamp);
				}
			}
		}
	}

	void filterModule::GetOdom()
	{

		double timeStamp, lat, lon, yaw, alt;

		Eigen::VectorXd curOdom = pfilter->GetCarPostion();
		Eigen::Vector3d curState = curOdom.head(3);

		Eigen::Vector3d cur_position = curState + mOdometry.translation();
		yaw = curOdom[mStateMemberYaw];

		tool_t::UTMTransform::instance()->UTMtoLL(cur_position[1], cur_position[0], lat, lon);

		printf("ekf_out:%lf,%.7f,%.7f,%f \r\n", pfilter->GetLastMeasurementTime(), lat, lon, yaw);
		timeStamp = pfilter->GetLastMeasurementTime();

	}

	void filterModule::GetOdom(double& timeStamp, double& lat, double& lon, double& yaw)
	{


		Eigen::VectorXd curOdom = pfilter->GetCarPostion();
		Eigen::Vector3d curState = curOdom.head(3);

		Eigen::Vector3d cur_position = curState + mOdometry.translation();
		yaw = curOdom[mStateMemberYaw];

		tool_t::UTMTransform::instance()->UTMtoLL(cur_position[1], cur_position[0], lat, lon);
		timeStamp = pfilter->GetLastMeasurementTime();
		printf("ekf_out:%lf,%.7f,%.7f,%f \r\n", pfilter->GetLastMeasurementTime(), lat, lon, yaw);
	}

#if MPU

	void filterModule::PublisherTopic()
	{


	}

	void filterModule::SubscriberTopic()
	{
		m_pVHandleImp->Subscriber("GNSS", this, &filterModule::GNSSDataParse);
		m_pVHandleImp->Subscriber("vehicle", this, &filterModule::vehicleDataParse);
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
		double utmX, utmY, utmZ;
		tool_t::UTMTransform::instance()->LLToUTM(gnssData->dlatitude, gnssData->dlongitude, utmX, utmY);
		double heading = gnssData->heading;
		heading = heading + 0.5 * EIGEN_PI;
		if (heading > 2 * EIGEN_PI)  heading = heading - 2.0 * EIGEN_PI;
		printf("heading %f \n", heading);
		DFPrintf("ekf_in_gnss:%f,%f,%d,%d,%d,%.7f,%.7f,%f,%f,%f\n", gnssData->timeStamp, gnssData->iTOW, gnssData->fix_type, gnssData->RTKSatus, gnssData->numSV, gnssData->dlatitude, gnssData->dlongitude, gnssData->dheight, gnssData->heading, gnssData->fvelocity);
		if (pfilter->GetFilterStateInitial())
		{
			observeMessagePtr observe = std::make_shared<observeMessage>();
			observe->id = 0;
			observe->timestamp = gnssData->timeStamp;
			observe->UTM = Eigen::Vector3d{ utmX, utmY, utmZ };
			observe->Vehicle = Eigen::Vector3d{ gnssData->fvelocity, 0, 0 };
			observe->YPR = Eigen::Vector3d{ heading, 0, 0 };
			pfilterQueue->push_back(observe);
		}
		else
		{
			mInitialMessage->v_observation.resize(4);
			mInitialMessage->messageId = 0;
			mInitialMessage->timestamp = gnssData->timeStamp;
			mInitialMessage->v_observation[mStateMemberX] = utmX;
			mInitialMessage->v_observation[mStateMemberY] = utmY;
			mInitialMessage->v_observation[mStateMemberYaw] = heading;
			mInitialMessage->v_observation[mStateMemberVx] = gnssData->fvelocity;
			bvalidInitial = true;
			//std::cout<< mInitialMessage->v_observation <<std::endl;
		}
		return;
	}

	void filterModule::vehicleDataParse(vehicleDataPtr vehicleData)
	{
		DFPrintf("ekf_in_vel:%lf,%f \r\n", vehicleData->timeStamp, vehicleData->vehicle_v);
		if (pfilter->GetFilterStateInitial())
		{
			observeMessagePtr observe = std::make_shared<observeMessage>();
			observe->id = 2;
			observe->timestamp = vehicleData->timeStamp;
			observe->Vehicle = Eigen::Vector3d{ vehicleData->vehicle_v, 0, 0 };
			//pfilterQueue->push_back(observe);
		}
		return;
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
			mInitialMessage->v_observation.resize(4);
			mInitialMessage->v_observation.setZero();
			mInitialMessage->timestamp = observe->timestamp;
			mInitialMessage->v_observation[mStateMemberX] = observe->UTM[0];
			mInitialMessage->v_observation[mStateMemberY] = observe->UTM[1];
			mInitialMessage->v_observation[mStateMemberYaw] = observe->YPR[0];
			mInitialMessage->v_observation[mStateMemberVx] = observe->Vehicle[0];
			bvalidInitial = true;
		}


		ModuleParse();

		return;
	}

#endif

}
