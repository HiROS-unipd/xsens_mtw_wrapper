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

      std::ostream& operator<<(std::ostream& out, const XsPortInfo& p);
      std::ostream& operator<<(std::ostream& out, const XsDevice& d);

      std::string toString(const XsPortInfo& p);
      std::string toString(const XsDevice& d);

      XsDeviceId toXsDeviceId(const std::string& string);

      tf2::Quaternion toTf2Quaternion(const XsQuaternion& q);
      tf2::Vector3 toTf2Vector3(const XsVector& v);
      tf2::Vector3 toTf2Vector3(const XsPressure& p);

      XsQuaternion toXsQuaternion(const tf2::Quaternion& q);
      XsVector toXsVector(const tf2::Vector3& v);
      XsPressure toXsPressure(const tf2::Vector3& p);

      template <class T>
      double interpolate(const T& e1, const T& e2, const double& w = 0.5)
      {
        return (1. - w) * e1 + w * e2;
      }

      XsQuaternion interpolate(const XsQuaternion& q1, const XsQuaternion& q2, const double& w = 0.5);
      XsVector interpolate(const XsVector& v1, const XsVector& v2, const double& w = 0.5);
      XsPressure interpolate(const XsPressure& p1, const XsPressure& p2, const double& w = 0.5);

      std::shared_ptr<XsDataPacket>
      interpolate(std::shared_ptr<XsDataPacket> p1, std::shared_ptr<XsDataPacket> p2, const double& w = 0.5);

    } // namespace utils
  } // namespace xsens_mtw
} // namespace hiros

#endif
