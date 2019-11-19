#ifndef hiros_xsens_mtw_mtwCallback_h
#define hiros_xsens_mtw_mtwCallback_h

// Standard dependencies
#include <list>

// Xsens dependencies
#include "xsensdeviceapi.h"

namespace hiros {
  namespace xsens_mtw {

    //----------------------------------------------------------------------
    // Callback handler for MTw
    // Handles onDataAvailable callbacks for MTW devices
    //----------------------------------------------------------------------
    class MtwCallback : public XsCallback
    {
    public:
      MtwCallback(int t_mtw_index, XsDevice* t_device);

      bool dataAvailable() const;
      bool newDataAvailable();

      XsDataPacket const* getOldestPacket() const;
      XsDataPacket const* getLatestPacket() const;

      void deleteOldestPacket();
      void deleteOldestPackets(const unsigned long& t_n_packets);

      inline int getMtwIndex() const { return m_mtw_index; }
      XsDevice const& device() const;

    protected:
      virtual void onLiveDataAvailable(XsDevice* t_device, const XsDataPacket* t_packet);

    private:
      mutable XsMutex m_mutex;
      std::list<XsDataPacket> m_packet_buffer;
      size_t m_buffer_size;
      int m_mtw_index;
      XsDevice* m_device;
    };

  } // namespace xsens_mtw
} // namespace hiros

#endif
