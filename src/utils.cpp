// Standard dependencies
#include <iomanip>

// Internal dependencies
#include "xsens_mtw/utils.h"

std::ostream& hiros::xsens_mtw::utils::operator<<(std::ostream& out, const XsPortInfo& p)
{
  out << "Port: " << std::setw(2) << std::right << p.portNumber() << " (" << p.portName().toStdString() << ") @ "
      << std::setw(7) << p.baudrate() << " Bd"
      << ", "
      << "ID: " << p.deviceId().toString().toStdString();
  return out;
}

std::ostream& hiros::xsens_mtw::utils::operator<<(std::ostream& out, const XsDevice& d)
{
  out << "ID: " << d.deviceId().toString().toStdString() << " (" << d.productCode().toStdString() << ")";
  return out;
}

std::string hiros::xsens_mtw::utils::toString(const XsPortInfo& p)
{
  return ("Port: " + std::to_string(p.portNumber()) + " (" + p.portName().toStdString() + ") @ "
          + std::to_string(p.baudrate()) + " Bd, ID: " + p.deviceId().toString().toStdString());
}

std::string hiros::xsens_mtw::utils::toString(const XsDevice& d)
{
  return ("ID: " + d.deviceId().toString().toStdString() + " (" + d.productCode().toStdString() + ")");
}

XsDeviceId hiros::xsens_mtw::utils::toXsDeviceId(const std::string& string)
{
  try {
    return XsDeviceId(static_cast<unsigned int>(std::stoi(string, nullptr, 16)));
  }
  catch (...) {
    return XsDeviceId(0);
  }
}

tf2::Quaternion hiros::xsens_mtw::utils::toTf2Quaternion(const XsQuaternion& q)
{
  return tf2::Quaternion(q.x(), q.y(), q.z(), q.w());
}

tf2::Vector3 hiros::xsens_mtw::utils::toTf2Vector3(const XsVector& v)
{
  return tf2::Vector3(v.at(0), v.at(1), v.at(2));
}

tf2::Vector3 hiros::xsens_mtw::utils::toTf2Vector3(const XsPressure& p)
{
  return tf2::Vector3(p.m_pressure, p.m_pressureAge, 0);
}

XsQuaternion hiros::xsens_mtw::utils::toXsQuaternion(const tf2::Quaternion& q)
{
  return XsQuaternion(q.w(), q.x(), q.y(), q.z());
}

XsVector hiros::xsens_mtw::utils::toXsVector(const tf2::Vector3& v)
{
  return XsVector3(v.x(), v.y(), v.z());
}

XsPressure hiros::xsens_mtw::utils::toXsPressure(const tf2::Vector3& p)
{
  return XsPressure(p.x(), static_cast<unsigned char>(p.y()));
}

XsQuaternion hiros::xsens_mtw::utils::interpolate(const XsQuaternion& q1, const XsQuaternion& q2, const double& w)
{
  return toXsQuaternion(toTf2Quaternion(q1).slerp(toTf2Quaternion(q2), w));
}

XsVector hiros::xsens_mtw::utils::interpolate(const XsVector& v1, const XsVector& v2, const double& w)
{
  return toXsVector(toTf2Vector3(v1).lerp(toTf2Vector3(v2), w));
}

XsPressure hiros::xsens_mtw::utils::interpolate(const XsPressure& p1, const XsPressure& p2, const double& w)
{
  return toXsPressure(toTf2Vector3(p1).lerp(toTf2Vector3(p2), w));
}

std::shared_ptr<XsDataPacket> hiros::xsens_mtw::utils::interpolate(std::shared_ptr<XsDataPacket> p1,
                                                                   std::shared_ptr<XsDataPacket> p2,
                                                                   const double& w)
{
  auto p = std::make_shared<XsDataPacket>(*p1);

  p->setPacketId(static_cast<int>(std::round(interpolate(p1->packetId(), p2->packetId(), w))));
  p->setOrientationQuaternion(interpolate(p1->orientationQuaternion(), p2->orientationQuaternion(), w),
                              p->coordinateSystemOrientation());
  p->setOrientationEuler(XsEuler(p->orientationQuaternion()), p->coordinateSystemOrientation());
  p->setCalibratedGyroscopeData(interpolate(p1->calibratedGyroscopeData(), p2->calibratedGyroscopeData(), w));
  p->setCalibratedAcceleration(interpolate(p1->calibratedAcceleration(), p2->calibratedAcceleration(), w));
  p->setCalibratedMagneticField(interpolate(p1->calibratedMagneticField(), p2->calibratedMagneticField(), w));
  p->setFreeAcceleration(interpolate(p1->freeAcceleration(), p2->freeAcceleration(), w));
  p->setPressure(interpolate(p1->pressure(), p2->pressure(), w));

  return p;
}
