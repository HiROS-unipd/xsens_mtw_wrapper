#ifndef hiros_xsens_mtw_synchronizer_h
#define hiros_xsens_mtw_synchronizer_h

// Standard dependencies
#include <deque>
#include <map>

// ROS dependencies
#include "ros/ros.h"

// Xsens dependencies
#include "xsensdeviceapi.h"

// Internal dependencies
#include "xsens_mtw/MtwCallback.h"

namespace hiros {
  namespace xsens_mtw {

    class Synchronizer
    {
    public:
      enum class SyncPolicy
      {
        fillPartialFrames,
        skipPartialFrames
      };

      Synchronizer(const std::map<XsDeviceId, MtwCallback*>& t_mtw_callbacks,
                   const SyncPolicy& t_sync_policy = SyncPolicy::skipPartialFrames);
      virtual ~Synchronizer();

      void add(const std::shared_ptr<XsDataPacket>& t_packet);
      bool newFrameAvailable();
      std::vector<std::shared_ptr<XsDataPacket>> getLatestFrame();
      void clearLatestFrame();

    private:
      void fillMissingPackets(const XsDeviceId& t_device_id);
      void sync(std::shared_ptr<XsDataPacket> t_packet);

      void checkNewFullFrame(std::shared_ptr<XsDataPacket> t_packet);
      void restructureBuffer();
      void fillFrame();

      bool isFullFrame(const long& t_packet_id);
      bool containsPacket(const XsDeviceId& t_device_id, const long& t_packet_id);
      bool isDoubleRow(const unsigned long& t_row);
      void eraseRow(const unsigned long& t_row);
      void clearInitialPackets();

      const std::map<XsDeviceId, MtwCallback*>& m_mtw_callbacks;
      SyncPolicy m_sync_policy;

      bool m_new_full_frame;
      unsigned long m_n_synched_frames;

      long m_last_full_packet_id;
      bool m_initialized;

      std::map<XsDeviceId, std::deque<std::shared_ptr<XsDataPacket>>> m_buffer;
      std::vector<std::shared_ptr<XsDataPacket>> m_frame;
    };

  } // namespace xsens_mtw
} // namespace hiros

#endif
