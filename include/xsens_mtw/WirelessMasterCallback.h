#ifndef xsens_mtw_driver_WirelessMasterCallback_h
#define xsens_mtw_driver_WirelessMasterCallback_h

#include "xsensdeviceapi.h"

#include "xsens_mtw/utils.h"

namespace xsens {
  namespace mtw {

    //----------------------------------------------------------------------
    // Callback handler for wireless master
    //----------------------------------------------------------------------
    class WirelessMasterCallback : public XsCallback
    {
    public:
      utils::XsDeviceSet getWirelessMTWs() const;

    protected:
      virtual void onConnectivityChanged(XsDevice* dev, XsConnectivityState newState);

    private:
      mutable XsMutex m_mutex;
      utils::XsDeviceSet m_connectedMTWs;
    };

  } // namespace mtw
} // namespace xsens

#endif
