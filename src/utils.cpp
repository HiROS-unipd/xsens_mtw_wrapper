#include <iomanip>

#include "xsens_mtw/utils.h"

std::ostream& xsens::mtw::utils::operator<<(std::ostream& out, const XsPortInfo& p)
{
  out << "Port: " << std::setw(2) << std::right << p.portNumber() << " (" << p.portName().toStdString() << ") @ "
      << std::setw(7) << p.baudrate() << " Bd"
      << ", "
      << "ID: " << p.deviceId().toString().toStdString();
  return out;
}

std::ostream& xsens::mtw::utils::operator<<(std::ostream& out, const XsDevice& d)
{
  out << "ID: " << d.deviceId().toString().toStdString() << " (" << d.productCode().toStdString() << ")";
  return out;
}
