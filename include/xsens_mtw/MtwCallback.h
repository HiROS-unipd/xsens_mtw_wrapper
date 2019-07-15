#ifndef xsens_mtw_driver_MtwCallback_h
#define xsens_mtw_driver_MtwCallback_h

#include <list>

#include "xsensdeviceapi.h"

namespace xsens {
  namespace mtw {

    //----------------------------------------------------------------------
    // Callback handler for MTw
    // Handles onDataAvailable callbacks for MTW devices
    //----------------------------------------------------------------------
    class MtwCallback : public XsCallback
    {
    public:
      MtwCallback(int mtwIndex, XsDevice* device);

      bool dataAvailable() const;

      XsDataPacket const* getOldestPacket() const;

      void deleteOldestPacket();

      inline int getMtwIndex() const { return m_mtwIndex; }

      XsDevice const& device() const;

    protected:
      virtual void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet);

    private:
      mutable XsMutex m_mutex;
      std::list<XsDataPacket> m_packetBuffer;
      int m_mtwIndex;
      XsDevice* m_device;
    };

  } // namespace mtw
} // namespace xsens

#endif
