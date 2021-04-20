// Internal dependencies
#include "xsens_mtw/Synchronizer.h"
#include "xsens_mtw/utils.h"

hiros::xsens_mtw::Synchronizer::Synchronizer(const std::map<XsDeviceId, MtwCallback*>& t_mtw_callbacks,
                                             const SyncPolicy& t_sync_policy)
  : m_mtw_callbacks(t_mtw_callbacks)
  , m_sync_policy(t_sync_policy)
  , m_new_full_frame(false)
  , m_n_synched_frames(0)
  , m_last_full_packet_id(std::numeric_limits<long>::max())
  , m_initialized(false)
{
  for (auto& cb : m_mtw_callbacks) {
    m_buffer.emplace(cb.first, std::deque<std::shared_ptr<XsDataPacket>>());
  }
  m_frame.reserve(m_buffer.size());
}

hiros::xsens_mtw::Synchronizer::~Synchronizer() {}

void hiros::xsens_mtw::Synchronizer::add(const std::shared_ptr<XsDataPacket>& t_packet)
{
  m_buffer.at(t_packet->deviceId()).push_back(t_packet);

  switch (m_sync_policy) {
    case SyncPolicy::fillPartialFrames:
      fillMissingPackets(t_packet->deviceId());
      break;

    case SyncPolicy::skipPartialFrames:
      break;
  }

  if (!newFrameAvailable()) {
    sync(t_packet);
  }
}

bool hiros::xsens_mtw::Synchronizer::newFrameAvailable()
{
  return (m_frame.size() == m_buffer.size());
}

std::vector<std::shared_ptr<XsDataPacket>> hiros::xsens_mtw::Synchronizer::getLatestFrame()
{
  return m_frame;
}

void hiros::xsens_mtw::Synchronizer::clearLatestFrame()
{
  m_frame.clear();

  switch (m_sync_policy) {
    case SyncPolicy::fillPartialFrames:
      for (auto& pair : m_buffer) {
        long previous_pid = pair.second.front()->packetId();

        pair.second.pop_front();
        if (pair.second.front()->packetId() > previous_pid) {
          m_mtw_callbacks.at(pair.first)->deleteOldestPacket();
        }
      }

      --m_n_synched_frames;
      break;

    case SyncPolicy::skipPartialFrames:
      for (auto& pair : m_buffer) {
        unsigned long n_packets_to_delete = pair.second.size();

        pair.second.clear();
        m_mtw_callbacks.at(pair.first)->deleteOldestPackets(n_packets_to_delete);
      }
      break;
  }
}

void hiros::xsens_mtw::Synchronizer::fillMissingPackets(const XsDeviceId& t_device_id)
{
  if (m_buffer.at(t_device_id).size() > 1) {
    unsigned long n_packets_to_add =
      static_cast<unsigned long>(m_buffer.at(t_device_id).rbegin()->get()->packetId()
                                 - std::next(m_buffer.at(t_device_id).rbegin())->get()->packetId() - 1);

    if (n_packets_to_add > 0) {
      std::vector<std::shared_ptr<XsDataPacket>> interpolated_packets(n_packets_to_add);
      for (unsigned int i = 0; i < interpolated_packets.size(); ++i) {
        interpolated_packets.at(i) = utils::interpolate(*std::next(m_buffer.at(t_device_id).rbegin()),
                                                        m_buffer.at(t_device_id).back(),
                                                        (i + 1) / static_cast<double>(n_packets_to_add + 1));
      }

      m_buffer.at(t_device_id)
        .insert(std::prev(m_buffer.at(t_device_id).end()), interpolated_packets.begin(), interpolated_packets.end());
    }
  }
}

void hiros::xsens_mtw::Synchronizer::sync(std::shared_ptr<XsDataPacket> t_packet)
{
  checkNewFullFrame(t_packet);

  switch (m_sync_policy) {
    case SyncPolicy::fillPartialFrames:
      if (m_new_full_frame) {
        restructureBuffer();
      }

      if (m_n_synched_frames > 0) {
        fillFrame();
      }
      break;

    case SyncPolicy::skipPartialFrames:
      if (m_new_full_frame) {
        fillFrame();
      }
      break;
  }
}

void hiros::xsens_mtw::Synchronizer::checkNewFullFrame(std::shared_ptr<XsDataPacket> t_packet)
{
  if (!m_initialized) {
    for (auto& pair : m_buffer) {
      if (!pair.second.empty()) {
        m_last_full_packet_id = std::min(m_last_full_packet_id, pair.second.front()->packetId() - 1);
      }
    }
  }

  m_new_full_frame = false;
  for (long pid = m_last_full_packet_id + 1; pid <= t_packet->packetId(); ++pid) {
    if (isFullFrame(pid)) {
      m_new_full_frame = true;
      m_last_full_packet_id = pid;
      break;
    }
  }

  if (!m_initialized && m_new_full_frame) {
    clearInitialPackets();
    m_initialized = true;
  }
}

void hiros::xsens_mtw::Synchronizer::restructureBuffer()
{
  auto n_rows = std::min_element(m_buffer.begin(), m_buffer.end(), [](const auto& lhs, const auto& rhs) {
                  return lhs.second.size() < rhs.second.size();
                })->second.size();

  for (unsigned long row = 0; row < n_rows - 1; ++row) {
    if (isDoubleRow(row)) {
      eraseRow(row--);
    }
  }

  m_n_synched_frames = m_buffer.begin()->second.size() - 1;
}

void hiros::xsens_mtw::Synchronizer::fillFrame()
{
  m_frame.clear();

  for (auto& pair : m_buffer) {
    switch (m_sync_policy) {
      case SyncPolicy::fillPartialFrames:
        if (pair.second.size() > 1) {
          m_frame.push_back(pair.second.at(1));
        }
        break;

      case SyncPolicy::skipPartialFrames:
        m_frame.push_back(pair.second.back());
        break;
    }
  }
}

bool hiros::xsens_mtw::Synchronizer::isFullFrame(const long& t_packet_id)
{
  for (auto& pair : m_buffer) {
    if (!containsPacket(pair.first, t_packet_id)) {
      return false;
    }
  }
  return true;
}

bool hiros::xsens_mtw::Synchronizer::containsPacket(const XsDeviceId& t_device_id, const long& t_packet_id)
{
  for (auto it = m_buffer.at(t_device_id).rbegin(); it < m_buffer.at(t_device_id).rend(); ++it) {
    if (it->get()->packetId() == t_packet_id) {
      return true;
    }
  }

  return false;
}

bool hiros::xsens_mtw::Synchronizer::isDoubleRow(const unsigned long& t_row)
{
  for (auto& pair : m_buffer) {
    if (pair.second.size() <= t_row + 1 || pair.second.at(t_row + 1)->packetId() > pair.second.at(t_row)->packetId()) {
      return false;
    }
  }
  return true;
}

void hiros::xsens_mtw::Synchronizer::eraseRow(const unsigned long& t_row)
{
  for (auto& pair : m_buffer) {
    pair.second.erase(pair.second.begin() + static_cast<long>(t_row));
  }
}

void hiros::xsens_mtw::Synchronizer::clearInitialPackets()
{
  for (auto& pair : m_buffer) {
    for (auto& packet : pair.second) {
      if (packet->packetId() < m_last_full_packet_id) {
        pair.second.pop_front();
        m_mtw_callbacks.at(pair.first)->deleteOldestPacket();
      }
    }
  }
}
