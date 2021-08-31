#ifndef hiros_xsens_mtw_mtwCallback_h
#define hiros_xsens_mtw_mtwCallback_h

// Standard dependencies
#include <deque>
#include <memory>

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
      MtwCallback(int mtw_index, XsDevice* device);

      bool dataAvailable() const;
      bool newDataAvailable();

      std::shared_ptr<XsDataPacket> getLatestPacket();

      void deleteOldestPacket();
      void deleteOldestPackets(const unsigned long& n_packets);

      inline int getMtwIndex() const { return mtw_index_; }
      XsDevice const& device() const;

    protected:
      virtual void onLiveDataAvailable(XsDevice* device, const XsDataPacket* packet);

    private:
      const unsigned long max_buffer_size_ = 300;

      mutable XsMutex mutex_;
      std::deque<XsDataPacket> packet_buffer_;
      int read_packets_;
      int mtw_index_;
      XsDevice* device_;
    };

  } // namespace xsens_mtw
} // namespace hiros

#endif
