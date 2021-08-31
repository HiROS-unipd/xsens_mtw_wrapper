#ifndef hiros_xsens_mtw_wirelessMasterCallback_h
#define hiros_xsens_mtw_wirelessMasterCallback_h

// Xsens dependencies
#include "xsensdeviceapi.h"

// Internal dependencies
#include "xsens_mtw/utils.h"

namespace hiros {
  namespace xsens_mtw {

    //----------------------------------------------------------------------
    // Callback handler for wireless master
    //----------------------------------------------------------------------
    class WirelessMasterCallback : public XsCallback
    {
    public:
      utils::XsDeviceSet getWirelessMTWs() const;

    protected:
      virtual void onConnectivityChanged(XsDevice* device, XsConnectivityState new_state);

    private:
      mutable XsMutex mutex_;
      utils::XsDeviceSet connected_mtws_;
    };

  } // namespace xsens_mtw
} // namespace hiros

#endif
