#ifndef hiros_xsens_mtw_utils_h
#define hiros_xsens_mtw_utils_h

// Standard dependencies
#include <set>

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

      XsDeviceId toXsDeviceId(const std::string t_string);

    } // namespace utils
  } // namespace xsens_mtw
} // namespace hiros

#endif
