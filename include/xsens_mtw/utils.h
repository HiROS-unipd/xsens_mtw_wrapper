#ifndef hiros_xsens_mtw_utils_h
#define hiros_xsens_mtw_utils_h

// Standard dependencies
#include <memory>
#include <set>

// ROS dependencies
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

// Xsens dependencies
#include "xsensdeviceapi.h"

namespace hiros {
  namespace xsens_mtw {
    namespace utils {

      typedef std::set<XsDevice*> XsDeviceSet;

      std::ostream& operator<<(std::ostream& t_out, const XsPortInfo& t_p);
      std::ostream& operator<<(std::ostream& t_out, const XsDevice& t_d);

      std::string toString(const XsPortInfo& t_p);
      std::string toString(const XsDevice& t_d);

      XsDeviceId toXsDeviceId(const std::string& t_string);

      tf2::Quaternion toTf2Quaternion(const XsQuaternion& t_q);
      tf2::Vector3 toTf2Vector3(const XsVector& t_v);
      tf2::Vector3 toTf2Vector3(const XsPressure& t_p);

      XsQuaternion toXsQuaternion(const tf2::Quaternion& t_q);
      XsVector toXsVector(const tf2::Vector3& t_v);
      XsPressure toXsPressure(const tf2::Vector3& t_p);

      template <class T>
      double interpolate(const T& t_e1, const T& t_e2, const double& t_w = 0.5)
      {
        return (1. - t_w) * t_e1 + t_w * t_e2;
      }

      XsQuaternion interpolate(const XsQuaternion& t_q1, const XsQuaternion& t_q2, const double& t_w = 0.5);
      XsVector interpolate(const XsVector& t_v1, const XsVector& t_v2, const double& t_w = 0.5);
      XsPressure interpolate(const XsPressure& t_p1, const XsPressure& t_p2, const double& t_w = 0.5);

      std::shared_ptr<XsDataPacket>
      interpolate(std::shared_ptr<XsDataPacket> t_p1, std::shared_ptr<XsDataPacket> t_p2, const double& t_w = 0.5);

    } // namespace utils
  } // namespace xsens_mtw
} // namespace hiros

#endif
