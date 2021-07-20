#include "filterCommon.h"

namespace filter
{
    Eigen::Quaterniond YPR2Quaterniond(float yaw, float pitch, float roll)
    {
        Eigen::Vector3d eulerAngle(yaw, pitch, roll);
        Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond quaternion;
        quaternion = yawAngle * pitchAngle * rollAngle;
        return quaternion;
    }

    //ypr
    Eigen::Vector3d ToEulerAngles(const Eigen::Quaterniond& q)
    {
        Eigen::Vector3d angles;
        // roll (x-axis rotation)
        double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
        double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
        angles[2] = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
        if (std::abs(sinp) >= 1)
            angles[1] = sinp > 0 ? (EIGEN_PI / 2) : (-EIGEN_PI / 2);	//std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            angles[1] = std::asin(sinp);

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
        double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
        angles[0] = std::atan2(siny_cosp, cosy_cosp);

        return angles;
    }

    



}