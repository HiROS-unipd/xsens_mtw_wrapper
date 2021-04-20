// Standard dependencies
#include <iomanip>

// Internal dependencies
#include "xsens_mtw/utils.h"

std::ostream& hiros::xsens_mtw::utils::operator<<(std::ostream& t_out, const XsPortInfo& t_p)
{
  t_out << "Port: " << std::setw(2) << std::right << t_p.portNumber() << " (" << t_p.portName().toStdString() << ") @ "
        << std::setw(7) << t_p.baudrate() << " Bd"
        << ", "
        << "ID: " << t_p.deviceId().toString().toStdString();
  return t_out;
}

std::ostream& hiros::xsens_mtw::utils::operator<<(std::ostream& t_out, const XsDevice& t_d)
{
  t_out << "ID: " << t_d.deviceId().toString().toStdString() << " (" << t_d.productCode().toStdString() << ")";
  return t_out;
}

std::string hiros::xsens_mtw::utils::toString(const XsPortInfo& t_p)
{
  return ("Port: " + std::to_string(t_p.portNumber()) + " (" + t_p.portName().toStdString() + ") @ "
          + std::to_string(t_p.baudrate()) + " Bd, ID: " + t_p.deviceId().toString().toStdString());
}

std::string hiros::xsens_mtw::utils::toString(const XsDevice& t_d)
{
  return ("ID: " + t_d.deviceId().toString().toStdString() + " (" + t_d.productCode().toStdString() + ")");
}

XsDeviceId hiros::xsens_mtw::utils::toXsDeviceId(const std::string& t_string)
{
  try {
    return XsDeviceId(static_cast<unsigned int>(std::stoi(t_string, nullptr, 16)));
  }
  catch (...) {
    return XsDeviceId(0);
  }
}

tf2::Quaternion hiros::xsens_mtw::utils::toTf2Quaternion(const XsQuaternion& t_q)
{
  return tf2::Quaternion(t_q.x(), t_q.y(), t_q.z(), t_q.w());
}

tf2::Vector3 hiros::xsens_mtw::utils::toTf2Vector3(const XsVector& t_v)
{
  return tf2::Vector3(t_v.at(0), t_v.at(1), t_v.at(2));
}

tf2::Vector3 hiros::xsens_mtw::utils::toTf2Vector3(const XsPressure& t_p)
{
  return tf2::Vector3(t_p.m_pressure, t_p.m_pressureAge, 0);
}

XsQuaternion hiros::xsens_mtw::utils::toXsQuaternion(const tf2::Quaternion& t_q)
{
  return XsQuaternion(t_q.w(), t_q.x(), t_q.y(), t_q.z());
}

XsVector hiros::xsens_mtw::utils::toXsVector(const tf2::Vector3& t_v)
{
  return XsVector3(t_v.x(), t_v.y(), t_v.z());
}

XsPressure hiros::xsens_mtw::utils::toXsPressure(const tf2::Vector3& t_p)
{
  return XsPressure(t_p.x(), static_cast<unsigned char>(t_p.y()));
}

XsQuaternion hiros::xsens_mtw::utils::interpolate(const XsQuaternion& t_q1, const XsQuaternion& t_q2, const double& t_w)
{
  return toXsQuaternion(toTf2Quaternion(t_q1).slerp(toTf2Quaternion(t_q2), t_w));
}

XsVector hiros::xsens_mtw::utils::interpolate(const XsVector& t_v1, const XsVector& t_v2, const double& t_w)
{
  return toXsVector(toTf2Vector3(t_v1).lerp(toTf2Vector3(t_v2), t_w));
}

XsPressure hiros::xsens_mtw::utils::interpolate(const XsPressure& t_p1, const XsPressure& t_p2, const double& t_w)
{
  return toXsPressure(toTf2Vector3(t_p1).lerp(toTf2Vector3(t_p2), t_w));
}

std::shared_ptr<XsDataPacket> hiros::xsens_mtw::utils::interpolate(std::shared_ptr<XsDataPacket> t_p1,
                                                                   std::shared_ptr<XsDataPacket> t_p2,
                                                                   const double& t_w)
{
  auto p = std::make_shared<XsDataPacket>(*t_p1);

  p->setPacketId(static_cast<int>(std::round(interpolate(t_p1->packetId(), t_p2->packetId(), t_w))));
  p->setOrientationQuaternion(interpolate(t_p1->orientationQuaternion(), t_p2->orientationQuaternion(), t_w),
                              p->coordinateSystemOrientation());
  p->setOrientationEuler(XsEuler(p->orientationQuaternion()), p->coordinateSystemOrientation());
  p->setCalibratedGyroscopeData(interpolate(t_p1->calibratedGyroscopeData(), t_p2->calibratedGyroscopeData(), t_w));
  p->setCalibratedAcceleration(interpolate(t_p1->calibratedAcceleration(), t_p2->calibratedAcceleration(), t_w));
  p->setCalibratedMagneticField(interpolate(t_p1->calibratedMagneticField(), t_p2->calibratedMagneticField(), t_w));
  p->setFreeAcceleration(interpolate(t_p1->freeAcceleration(), t_p2->freeAcceleration(), t_w));
  p->setPressure(interpolate(t_p1->pressure(), t_p2->pressure(), t_w));

  return p;
}
