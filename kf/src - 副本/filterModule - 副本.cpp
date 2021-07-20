#include "filterModule.h"

#if BUILD_MPU
std::string ANALYTICAL_VEHICLE_N = "analytical_vehicle";
std::string ANALYTICAL_GNSS_N = "analytical_gnss";
std::string Lane_Location_Info = "lane_location_info";
#endif

namespace filter
{

	filterModule::filterModule()
	{
		mpFilter = new filterEKF();
		mInitialMessage = std::make_shared<filterMessage>();
		pfilterQueue = new filterQueue();

		mStateMemberX = mpFilter->getStateX();
		mStateMemberY = mpFilter->getStateY();
		mStateMemberYaw = mpFilter->getStateYaw();
		mStateMemberVx = mpFilter->getStateVx();
		mStateSIze = mpFilter->getStateSize();
		bvalidInitial = false;
		//m_fileout.open("out.txt");
	}

	filterModule::~filterModule()
	{
	}

	void filterModule::ModuleParse()
	{

		bool bRet = false;
		bool bState = mpFilter->GetFilterStateInitial();

		if (!bState && bvalidInitial)
		{
			printf("EKF inital \r\n");
			mpFilter->SetFilterStateInitial(mInitialMessage);
		}
		else
		{
			while (!pfilterQueue->empty())
			{
				MeasurementPtr message;
				pfilterQueue->pop_up(message);
				if (bState == true)
				{
					//printf("ekf in message %lf %lf %lf \r\n",message->timestamp, message->v_observation[mStateMemberX],message->v_observation[mStateMemberY]);
					mpFilter->filterProcess(message);

				}
			}
		}

		return ;
	}


	MeasurementPtr filterModule::tranform(const double& timeStamp, const Eigen::Vector3d& UTM, const Eigen::Vector3d& YPR, const Eigen::Vector3d& Vehicle)
	{
		Eigen::Isometry3d mOdometry = mpFilter->GetOdometry();
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
		message->m_convariance(mStateMemberX, mStateMemberX) = 0.1;
		message->m_convariance(mStateMemberY, mStateMemberY) = 0.1;
		message->m_convariance(mStateMemberYaw,mStateMemberYaw) = 0.001;
		message->m_convariance(mStateMemberVx, mStateMemberVx) = 0.1;
		return message;
	}

	MeasurementPtr filterModule::tranform(const double& timeStamp,const Eigen::Vector3d& Vehicle)
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
		message->m_convariance(mStateMemberVx, mStateMemberVx) = 0.001;
		return message;
	}

	void filterModule::setObserve(observedMessagePtr data)
	{
		if (mpFilter->GetFilterStateInitial() == true)
		{
			if (data->id == 0)
			{
				MeasurementPtr message = tranform(data->timestamp, data->UTM, data->YPR,data->Vehicle);
				pfilterQueue->push_back(message);
			}
			else if (data->id == 1)
			{
				MeasurementPtr message = tranform(data->timestamp, data->Vehicle);
				pfilterQueue->push_back(message);
			}

		}
		else
		{
			if (data->id == 0)
			{
				double dStampTime;
				MeasurementPtr message = std::make_shared<filterMessage>();
				message->timestamp = data->timestamp;
				message->v_observation.resize(mStateSIze);
				message->v_observation.setZero();
				message->v_observation[mStateMemberX] = data->UTM[0];
				message->v_observation[mStateMemberY] = data->UTM[1];
				message->v_observation[mStateMemberYaw] = data->YPR[0];
				message->v_observation[mStateMemberVx] = data->Vehicle[0];
				mpFilter->SetFilterStateInitial(message);
			}
		}

		bool bState = mpFilter->GetFilterStateInitial();

		ModuleParse();

		return ;
	}


	void filterModule::GetOdom()
	{

		Eigen::Isometry3d odometry = mpFilter->GetOdometry();
		Eigen::VectorXd curOdom = mpFilter->GetCarPostion();
		Eigen::Vector3d curState = curOdom.head(3);
		curState[2] = 0;

		Eigen::Vector3d cur_position = odometry.rotation() * curState + odometry.translation();
		Eigen::Quaterniond odometryBase;
		odometryBase = odometry.rotation() * YPR2Quaterniond(curOdom[mStateMemberYaw], 0, 0);
		Eigen::Vector3d ypr = ToEulerAngles(odometryBase);
		double &Yaw = ypr[0];

		double Lat, Long;	
		filter::UTMTransform::instance()->UTMtoLL(cur_position[1], cur_position[0], Lat, Long);
		//m_fileout << std::to_string(Lat) << "," << std::to_string(Long) << std::endl;
	}

	void filterModule::GetOdom(double& timeStamp, double& lat, double& lon, double& yaw)
	{

		Eigen::Isometry3d odometry = mpFilter->GetOdometry();
		Eigen::VectorXd curOdom = mpFilter->GetCarPostion();
		Eigen::Vector3d curState = curOdom.head(3);
		curState[2] = 0;

		Eigen::Vector3d cur_position =  curState + odometry.translation();
		yaw = curOdom[mStateMemberYaw];

		filter::UTMTransform::instance()->UTMtoLL(cur_position[1], cur_position[0], lat, lon);
		//m_fileout << std::to_string(mpFilter->GetLastMeasurementTime()) << "," << std::to_string(Lat) << "," << std::to_string(Long)  << "," << std::to_string(Yaw) << std::endl;
		timeStamp = mpFilter->GetLastMeasurementTime();
	}

} // namespace algorithm_t
