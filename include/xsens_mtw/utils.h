#ifndef xsens_mtw_driver_utils_h
#define xsens_mtw_driver_utils_h

#include <iostream>
#include <set>

#include "xsensdeviceapi.h"

namespace mtw {
  namespace utils {

    typedef std::set<XsDevice*> XsDeviceSet;

    std::ostream& operator<<(std::ostream& out, const XsPortInfo& p);

    std::ostream& operator<<(std::ostream& out, const XsDevice& d);

  } // namespace utils
} // namespace mtw

#endif
