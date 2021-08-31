// Internal dependencies
#include "xsens_mtw/Synchronizer.h"
#include "xsens_mtw/utils.h"

hiros::xsens_mtw::Synchronizer::Synchronizer(const std::map<XsDeviceId, MtwCallback*>& mtw_callbacks,
                                             const SyncPolicy& sync_policy)
  : mtw_callbacks_(mtw_callbacks)
  , sync_policy_(sync_policy)
  , new_full_frame_(false)
  , n_synched_frames_(0)
  , last_full_packet_id_(std::numeric_limits<long>::max())
  , initialized_(false)
{
  for (auto& cb : mtw_callbacks_) {
    buffer_.emplace(cb.first, std::deque<std::shared_ptr<XsDataPacket>>());
  }
  frame_.reserve(buffer_.size());
}

hiros::xsens_mtw::Synchronizer::~Synchronizer() {}

void hiros::xsens_mtw::Synchronizer::add(const std::shared_ptr<XsDataPacket>& packet)
{
  buffer_.at(packet->deviceId()).push_back(packet);

  switch (sync_policy_) {
    case SyncPolicy::fillPartialFrames:
      fillMissingPackets(packet->deviceId());
      break;

    case SyncPolicy::skipPartialFrames:
      break;
  }

  if (!newFrameAvailable()) {
    sync(packet);
  }
}

bool hiros::xsens_mtw::Synchronizer::newFrameAvailable()
{
  return (frame_.size() == buffer_.size());
}

std::vector<std::shared_ptr<XsDataPacket>> hiros::xsens_mtw::Synchronizer::getLatestFrame()
{
  return frame_;
}

void hiros::xsens_mtw::Synchronizer::clearLatestFrame()
{
  frame_.clear();

  switch (sync_policy_) {
    case SyncPolicy::fillPartialFrames:
      for (auto& pair : buffer_) {
        long previous_pid = pair.second.front()->packetId();

        pair.second.pop_front();
        if (pair.second.front()->packetId() > previous_pid) {
          mtw_callbacks_.at(pair.first)->deleteOldestPacket();
        }
      }

      --n_synched_frames_;
      break;

    case SyncPolicy::skipPartialFrames:
      for (auto& pair : buffer_) {
        unsigned long n_packets_to_delete = pair.second.size();

        pair.second.clear();
        mtw_callbacks_.at(pair.first)->deleteOldestPackets(n_packets_to_delete);
      }
      break;
  }
}

void hiros::xsens_mtw::Synchronizer::fillMissingPackets(const XsDeviceId& device_id)
{
  if (buffer_.at(device_id).size() > 1) {
    unsigned long n_packets_to_add =
      static_cast<unsigned long>(buffer_.at(device_id).rbegin()->get()->packetId()
                                 - std::next(buffer_.at(device_id).rbegin())->get()->packetId() - 1);

    if (n_packets_to_add > 0) {
      std::vector<std::shared_ptr<XsDataPacket>> interpolated_packets(n_packets_to_add);
      for (unsigned int i = 0; i < interpolated_packets.size(); ++i) {
        interpolated_packets.at(i) = utils::interpolate(*std::next(buffer_.at(device_id).rbegin()),
                                                        buffer_.at(device_id).back(),
                                                        (i + 1) / static_cast<double>(n_packets_to_add + 1));
      }

      buffer_.at(device_id).insert(
        std::prev(buffer_.at(device_id).end()), interpolated_packets.begin(), interpolated_packets.end());
    }
  }
}

void hiros::xsens_mtw::Synchronizer::sync(std::shared_ptr<XsDataPacket> packet)
{
  checkNewFullFrame(packet);

  switch (sync_policy_) {
    case SyncPolicy::fillPartialFrames:
      if (new_full_frame_) {
        restructureBuffer();
      }

      if (n_synched_frames_ > 0) {
        fillFrame();
      }
      break;

    case SyncPolicy::skipPartialFrames:
      if (new_full_frame_) {
        fillFrame();
      }
      break;
  }
}

void hiros::xsens_mtw::Synchronizer::checkNewFullFrame(std::shared_ptr<XsDataPacket> packet)
{
  if (!initialized_) {
    for (auto& pair : buffer_) {
      if (!pair.second.empty()) {
        last_full_packet_id_ = std::min(last_full_packet_id_, pair.second.front()->packetId() - 1);
      }
    }
  }

  new_full_frame_ = false;
  for (long pid = last_full_packet_id_ + 1; pid <= packet->packetId(); ++pid) {
    if (isFullFrame(pid)) {
      new_full_frame_ = true;
      last_full_packet_id_ = pid;
      break;
    }
  }

  if (!initialized_ && new_full_frame_) {
    clearInitialPackets();
    initialized_ = true;
  }
}

void hiros::xsens_mtw::Synchronizer::restructureBuffer()
{
  auto n_rows = std::min_element(buffer_.begin(), buffer_.end(), [](const auto& lhs, const auto& rhs) {
                  return lhs.second.size() < rhs.second.size();
                })->second.size();

  for (unsigned long row = 0; row < n_rows - 1; ++row) {
    if (isDoubleRow(row)) {
      eraseRow(row--);
    }
  }

  n_synched_frames_ = buffer_.begin()->second.size() - 1;
}

void hiros::xsens_mtw::Synchronizer::fillFrame()
{
  frame_.clear();

  for (auto& pair : buffer_) {
    switch (sync_policy_) {
      case SyncPolicy::fillPartialFrames:
        if (pair.second.size() > 1) {
          frame_.push_back(pair.second.at(1));
        }
        break;

      case SyncPolicy::skipPartialFrames:
        frame_.push_back(pair.second.back());
        break;
    }
  }
}

bool hiros::xsens_mtw::Synchronizer::isFullFrame(const long& packet_id)
{
  for (auto& pair : buffer_) {
    if (!containsPacket(pair.first, packet_id)) {
      return false;
    }
  }
  return true;
}

bool hiros::xsens_mtw::Synchronizer::containsPacket(const XsDeviceId& device_id, const long& packet_id)
{
  for (auto it = buffer_.at(device_id).rbegin(); it < buffer_.at(device_id).rend(); ++it) {
    if (it->get()->packetId() == packet_id) {
      return true;
    }
  }

  return false;
}

bool hiros::xsens_mtw::Synchronizer::isDoubleRow(const unsigned long& row)
{
  for (auto& pair : buffer_) {
    if (pair.second.size() <= row + 1 || pair.second.at(row + 1)->packetId() > pair.second.at(row)->packetId()) {
      return false;
    }
  }
  return true;
}

void hiros::xsens_mtw::Synchronizer::eraseRow(const unsigned long& row)
{
  for (auto& pair : buffer_) {
    pair.second.erase(pair.second.begin() + static_cast<long>(row));
  }
}

void hiros::xsens_mtw::Synchronizer::clearInitialPackets()
{
  for (auto& pair : buffer_) {
    for (auto& packet : pair.second) {
      if (packet->packetId() < last_full_packet_id_) {
        pair.second.pop_front();
        mtw_callbacks_.at(pair.first)->deleteOldestPacket();
      }
    }
  }
}
