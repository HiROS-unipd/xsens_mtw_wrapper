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
