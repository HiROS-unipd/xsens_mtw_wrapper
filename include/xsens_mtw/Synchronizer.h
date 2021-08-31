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

      Synchronizer(const std::map<XsDeviceId, MtwCallback*>& mtw_callbacks,
                   const SyncPolicy& sync_policy = SyncPolicy::skipPartialFrames);
      virtual ~Synchronizer();

      void add(const std::shared_ptr<XsDataPacket>& packet);
      bool newFrameAvailable();
      std::vector<std::shared_ptr<XsDataPacket>> getLatestFrame();
      void clearLatestFrame();

    private:
      void fillMissingPackets(const XsDeviceId& device_id);
      void sync(std::shared_ptr<XsDataPacket> packet);

      void checkNewFullFrame(std::shared_ptr<XsDataPacket> packet);
      void restructureBuffer();
      void fillFrame();

      bool isFullFrame(const long& packet_id);
      bool containsPacket(const XsDeviceId& device_id, const long& packet_id);
      bool isDoubleRow(const unsigned long& row);
      void eraseRow(const unsigned long& row);
      void clearInitialPackets();

      const std::map<XsDeviceId, MtwCallback*>& mtw_callbacks_;
      SyncPolicy sync_policy_;

      bool new_full_frame_;
      unsigned long n_synched_frames_;

      long last_full_packet_id_;
      bool initialized_;

      std::map<XsDeviceId, std::deque<std::shared_ptr<XsDataPacket>>> buffer_;
      std::vector<std::shared_ptr<XsDataPacket>> frame_;
    };

  } // namespace xsens_mtw
} // namespace hiros

#endif
