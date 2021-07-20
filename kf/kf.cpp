// kf.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <vector>
#include <fstream>
#include "filterModule.h"
#include "UTMTran.h"
#include "matplotlibcpp.h"
#include <algorithm>
#include "enu.h"

std::vector<std::string> splitWithSymbol(const std::string& strs, const std::string& splitWithSymbol)
{
	std::vector<std::string> strData;
	size_t pos = strs.find(splitWithSymbol, 0);
	std::string head = strs.substr(0, pos);
	strData.push_back(head);
	std::string tmpStr = strs.substr(pos + 1, strs.size());
	strData.push_back(tmpStr);
	return strData;
}

std::vector<std::string> splitString(const std::string& strs)
{
	std::string temp;
	std::vector<std::string> splitOut;
	splitOut.clear();
	for (int i = 0; i < strs.size(); ++i)
	{
		if (strs[i] == ',')
		{
			splitOut.push_back(temp);
			temp = "";
		}
		else
		{
			temp = temp + strs[i];
		}

		if (i == strs.size() - 1)
		{
			splitOut.push_back(temp);
		}
	}
	return splitOut;
}
namespace plt = matplotlibcpp;

#if 0
int main()
{

	std::ifstream m_file_in;
	std::string LogFile = "C:\\Users\\niew\\Desktop\\MPUlog\\ukf1\\ReceivedTofile-COM5-2021_7_5_14-10-32_GNSS.txt";
	m_file_in.open(LogFile.c_str());
	std::string strs;

	std::vector<double> x1, y1, x2, y2;
	double shitx, shity;
	std::vector< filter::observedMessagePtr >  filterObserves;
	std::ofstream m_out, ekfOut;
	m_out.open("GNSS.txt");
	ekfOut.open("ekf.txt");
	bool init = false;
	int count = 0;
	while (std::getline(m_file_in, strs))
	{
		std::vector<std::string> logData = splitString(strs);

			double timestamp, lat, lon, alt, heading, utmX, utmY, utmZ;
			timestamp = stod(logData[0]);
			lat = stod(logData[2]);
			lon = stod(logData[3]);
			alt = stod(logData[4]);
			heading = stod(logData[5]);
			heading = heading * EIGEN_PI / 180.0;

			heading = heading + 0.5 * EIGEN_PI;

			//if (heading >  EIGEN_PI)  heading = heading -  2.0 * EIGEN_PI;
			if (heading > 2 * EIGEN_PI)  heading = heading - 2.0 * EIGEN_PI;

			//printf("%lf,%lf,%f,%f \n", lat, lon, alt, heading);
			filter::observedMessagePtr filterObserve_data = std::make_shared<filter::observedMessage>();
			filterObserve_data->timestamp = timestamp;
			double utm_x, utm_y;
			tool_t::UTMTransform::instance()->LLToUTM(lat, lon, utm_x, utm_y);
			filterObserve_data->UTM = Eigen::Vector3d(utm_x, utm_y, 0);
			filterObserve_data->YPR = Eigen::Vector3d(heading, 0, 0);
			//filterObserve_data->Vehicle = Eigen::Vector3d(stod(logData[9]), 0, 0);
			filterObserve_data->id = 1;
			count++;
			//if (count == 5)
			{
				filterObserves.push_back(filterObserve_data);
				count = 0;
			}


			tool_t::UTMTransform::instance()->LLToUTM(lat, lon, utmX, utmY);
			if (!init)
			{
				shitx = utmX;
				shity = utmY;
				init = true;
			}
			else
			{
				x1.push_back(utmX - shitx);
				y1.push_back(utmY - shity);

			}
			if (heading < 0) heading = heading + 2 * EIGEN_PI;
			m_out << std::to_string(timestamp) << "," << std::to_string(lat) << "," << std::to_string(lon) << "," << std::to_string(heading) << std::endl;

		
		
	}

	filter::filterModule EKF;
	init = false;

	for (auto i = 0; i < filterObserves.size(); i++)
	{
	
		double timestamp, lat, lon, yaw;
		EKF.setObserve(filterObserves[i]);
	
		
			EKF.GetOdom(timestamp, lat, lon, yaw);
			//std::cout << std::to_string(timestamp) << "," << std::to_string(lat) << "," << std::to_string(lon) << "," << std::to_string(yaw) << std::endl;
			double utmX, utmY;
			tool_t::UTMTransform::instance()->LLToUTM(lat, lon, utmX, utmY);
			x2.push_back(utmX - shitx);
			y2.push_back(utmY - shity);
			if (yaw < 0) yaw = yaw + 2 * EIGEN_PI;
			ekfOut << "ekf " << std::to_string(timestamp) << "," << std::to_string(lat) << "," << std::to_string(lon) << "," << std::to_string(yaw) << std::endl;
	}

	std::cout << "in size " << x1.size() << " out size " << x2.size() << std::endl;

	// Set the size of output image to 1200x780 pixels
	plt::figure_size(1200, 780);
	// Plot line from given x and y data. Color is selected automatically.
	//plt::plot(x, y);
	// Plot a red dashed line from given x and y data.
	plt::grid(true);
	plt::plot(x1, y1, "r*");
	plt::plot(x2, y2, "k*");
	plt::show();

	std::cout << "Hello World!\n";
}



#else

double* ecefToEnu(double x, double y, double z, double lat, double lng, double height)
{
	double a = 6378137;
	double b = 6356752.3142;
	double f = (a - b) / a;
	double e_sq = f * (2 - f);
	double lamb = lat;
	double phi =lng;
	double s = std::sin(lamb);
	double N = a / std::sqrt(1 - e_sq * s * s);
	double sin_lambda = std::sin(lamb);
	double cos_lambda = std::cos(lamb);
	double sin_phi = std::sin(phi);
	double cos_phi = std::cos(phi);

	double x0 = (height + N) * cos_lambda * cos_phi;
	double y0 = (height + N) * cos_lambda * sin_phi;
	double z0 = (height + (1 - e_sq) * N) * sin_lambda;

	double xd = x - x0;
	double yd = y - y0;
	double zd = z - z0;

	double t = -cos_phi * xd - sin_phi * yd;

	double xEast = -sin_phi * xd + cos_phi * yd;
	double yNorth = t * sin_lambda + cos_lambda * zd;
	double zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;
	printf("enu 2:%lf,%lf,%lf \n", xEast, yNorth, zUp);
	return new double[] { xEast, yNorth, zUp };
}




int main()
{

	std::ifstream m_file_in;
	std::string LogFile = "C:\\Users\\niew\\Desktop\\MPUlog\\e3.log";
	//std::string LogFile = "C:\\Users\\niew\\Desktop\\MPUlog\\rtk4.txt";
	m_file_in.open(LogFile.c_str());
	std::string strs;

	std::vector<double> x1, y1,z1, x2, y2, x3, y3,yaw1,yaw2;
	std::vector < std::string > w;
	double shitx, shity;
	std::vector< filter::observedMessagePtr >  filterObserves;
	std::ofstream m_out, ekfOut;
	LogFile = "C:\\Users\\niew\\Desktop\\MPUlog\\GNSS.txt";
	m_out.open(LogFile.c_str());
	LogFile = "C:\\Users\\niew\\Desktop\\MPUlog\\EKFOut.txt";
	ekfOut.open(LogFile.c_str());
	bool init = false;
	int count = 0;
	ENU mENU;
	

	double ecef_ref[3], lla_ref[3];
	while (std::getline(m_file_in, strs))
	{
		std::vector<std::string>  data = splitWithSymbol(strs, ":");
		if (data.size() < 2) continue;
		std::vector<std::string> logData = splitString(data[1]);

		if (data[0].compare("ekf_in_gnss") == 0)
		{
			double timestamp, lat, lon, alt, heading, utmX, utmY, utmZ;
			timestamp = stod(logData[0]);
			lat = stod(logData[5]);
			lon = stod(logData[6]);
			alt = stod(logData[7]);
			heading = stod(logData[8]); 
			yaw1.push_back(heading);
			heading = heading + 0.5 * EIGEN_PI;

			//if (heading >  EIGEN_PI)  heading = heading -  2.0 * EIGEN_PI;
			if (heading > 2 * EIGEN_PI)  heading = heading - 2.0 * EIGEN_PI;

			//printf("%lf,%lf,%f,%f \n", lat, lon, alt, heading);
			filter::observedMessagePtr filterObserve_data = std::make_shared<filter::observedMessage>();
			filterObserve_data->timestamp = timestamp;
			double utm_x, utm_y;
			tool_t::UTMTransform::instance()->LLToUTM(lat, lon, utm_x, utm_y);
			filterObserve_data->UTM = Eigen::Vector3d(utm_x, utm_y, alt);
			filterObserve_data->LLA = Eigen::Vector3d(lat, lon, alt);
			filterObserve_data->YPR = Eigen::Vector3d(heading, 0, 0);
			filterObserve_data->Vehicle = Eigen::Vector3d(stod(logData[9]), 0, 0);
			filterObserve_data->cov = Eigen::Vector4d(stod(logData[10]), stod(logData[11]), stod(logData[12]), stod(logData[13]));
			filterObserve_data->id = 0;
			count ++ ;
			//if (count == 5)
			{
				filterObserves.push_back(filterObserve_data);
				count = 0;
			}	

			tool_t::UTMTransform::instance()->LLToUTM(lat, lon, utmX, utmY);

			double enu[3];

			if (!init)
			{
				shitx = utmX;
				shity = utmY;
				init = true;
				mENU.setStart(lat, lon, alt);
			}
			else
			{
				x1.push_back(utmX - shitx);
				y1.push_back(utmY - shity);
				z1.push_back(stod(logData[4]));
				w.push_back(logData[11]);
				//x1.push_back(lat);
				//y1.push_back(lon);

			}
			//double *enuout = mENU.getENU(lat, lon, alt);
			//printf("enuout:%.7f,%.7f,%lf \n", enuout[0], enuout[1], enuout[2]);
			//double* llaout = mENU.getLLA(enuout[0], enuout[1], enuout[2]);
			//printf("llaout:%.7f,%.7f,%lf \n", llaout[0], llaout[1], llaout[2]);

			if (heading < 0) heading = heading + 2 * EIGEN_PI;
			m_out << std::to_string(timestamp) << "," << std::to_string(lat) << "," << std::to_string(lon) << "," << std::to_string(heading) << std::endl;

		}
		else if (data[0].compare("ekf_in_vel") == 0)
		{

			double timestamp = stod(logData[0]);
			filter::observedMessagePtr filterObserve_data = std::make_shared<filter::observedMessage>();
			filterObserve_data->timestamp = timestamp;
			filterObserve_data->Vehicle = Eigen::Vector3d(stod(logData[1]), 0, 0);
			filterObserve_data->steerAngle = 1;
			filterObserve_data->id = 2;
			filterObserves.push_back(filterObserve_data);
		}
		else if (data[0].compare("ekf_out") == 0)
		{
			double timestamp, lat, lon, alt, heading, utmX, utmY, utmZ;
			timestamp = stod(logData[0]);
			lat = stod(logData[1]);
			lon = stod(logData[2]);
			tool_t::UTMTransform::instance()->LLToUTM(lat, lon, utmX, utmY);
			x3.push_back(utmX - shitx);
			y3.push_back(utmY - shity);

		}
		else
		{
			continue;
		}	
	}

	filter::filterModule EKF;
	init = false;
	int start = 0;
	int end = filterObserves.size();
	count = 0;

	std::vector<double> dtVector;

	for (auto i = start; i < end; i++)
	{

		count++;
		if (!init)
		{
			if (filterObserves[i]->id != 0) continue;
			init = true;
		}
		//if (i > 1)
		//{
		//	double time = filterObserves[i]->timestamp - filterObserves[i - 1]->timestamp;
			//std::cout << "dt : " << time << std::endl;
		//	dtVector.push_back(time);
		//}

			
		double timestamp, lat, lon, yaw;
		EKF.setObserve(filterObserves[i]);
		//if (filterObserves[i]->id == 1)
		{
			EKF.GetOdom(timestamp, lat, lon, yaw);
			//std::cout << std::to_string(timestamp) << "," << std::to_string(lat) << "," << std::to_string(lon) << "," << std::to_string(yaw) << std::endl;
			double utmX, utmY;
	
			tool_t::UTMTransform::instance()->LLToUTM(lat, lon, utmX, utmY);
			x2.push_back(utmX - shitx);
			y2.push_back(utmY - shity);
			yaw2.push_back(yaw);
			ekfOut << "ekf " << std::to_string(timestamp) << "," << std::to_string(lat) << "," << std::to_string(lon) << "," << std::to_string(yaw) << std::endl;
		}

	}
	std::cout << filterObserves.size() << std::endl;
	std::cout << "EKF "<< count <<  "in size " << x1.size() << " out size " << x2.size() << std::endl;
	//std::sort(dtVector.begin(), dtVector.end());
	//std::cout << dtVector[dtVector.size() - 1] << std::endl;;
	//std::cout << dtVector[0] << std::endl;;
	// Set the size of output image to 1200x780 pixels
	plt::figure_size(1200, 780);
	// Plot line from given x and y data. Color is selected automatically.
	//plt::plot(x, y);
	// Plot a red dashed line from given x and y data.
	plt::grid(true);
	plt::plot(x1, y1, "r*");
	for (int i = 0; i < x1.size(); i= i+ 6)
	{
		//if (std::abs(z1[i] - 1533259107.829922) < 5)
		//plt::text(x1[i], y1[i], std::to_string(z1[i]).c_str());
		//plt::text(x1[i], y1[i], w[i].c_str());
	}
	plt::plot(x2, y2, "k*");
	//plt::plot(x3, y3, "b*");
	//plt::plot(yaw1, "k-");
	//plt::plot(yaw2, "r-");
	plt::show();

    std::cout << "Hello World!\n";
}

#endif

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
