// Internal dependencies
#include "xsens_mtw/MtwCallback.h"

hiros::xsens_mtw::MtwCallback::MtwCallback(int t_mtw_index, XsDevice* t_device)
  : m_read_packets(0)
  , m_mtw_index(t_mtw_index)
  , m_device(t_device)
{}

bool hiros::xsens_mtw::MtwCallback::dataAvailable() const
{
  XsMutexLocker lock(m_mutex);
  return !m_packet_buffer.empty();
}

bool hiros::xsens_mtw::MtwCallback::newDataAvailable()
{
  XsMutexLocker lock(m_mutex);
  bool new_data_availabe = (m_packet_buffer.size() > m_read_packets);
  return new_data_availabe;
}

XsDataPacket const* hiros::xsens_mtw::MtwCallback::getLatestPacket()
{
  XsMutexLocker lock(m_mutex);
  ++m_read_packets;
  return &m_packet_buffer.at(m_read_packets - 1);
}

void hiros::xsens_mtw::MtwCallback::deleteOldestPacket()
{
  XsMutexLocker lock(m_mutex);
  m_packet_buffer.pop_front();
  --m_read_packets;
}

void hiros::xsens_mtw::MtwCallback::deleteOldestPackets(const unsigned long& t_n_packets)
{
  XsMutexLocker lock(m_mutex);
  m_packet_buffer.erase(m_packet_buffer.begin(), m_packet_buffer.begin() + static_cast<long>(t_n_packets));
  m_read_packets -= t_n_packets;
}

XsDevice const& hiros::xsens_mtw::MtwCallback::device() const
{
  assert(m_device != 0);
  return *m_device;
}

void hiros::xsens_mtw::MtwCallback::onLiveDataAvailable(XsDevice* t_device, const XsDataPacket* t_packet)
{
  XsMutexLocker lock(m_mutex);
  m_packet_buffer.push_back(*t_packet);
  if (m_packet_buffer.size() > 300) {
    deleteOldestPacket();
  }
}
