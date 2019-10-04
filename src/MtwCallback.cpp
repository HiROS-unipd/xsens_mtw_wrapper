// Internal dependencies
#include "xsens_mtw/MtwCallback.h"

hiros::xsens_mtw::MtwCallback::MtwCallback(int t_mtw_index, XsDevice* t_device)
  : m_mtw_index(t_mtw_index)
  , m_device(t_device)
{}

bool hiros::xsens_mtw::MtwCallback::dataAvailable() const
{
  XsMutexLocker lock(m_mutex);
  return !m_packet_buffer.empty();
}

XsDataPacket const* hiros::xsens_mtw::MtwCallback::getOldestPacket() const
{
  XsMutexLocker lock(m_mutex);
  XsDataPacket const* packet = &m_packet_buffer.front();
  return packet;
}

void hiros::xsens_mtw::MtwCallback::deleteOldestPacket()
{
  XsMutexLocker lock(m_mutex);
  m_packet_buffer.pop_front();
}

XsDevice const& hiros::xsens_mtw::MtwCallback::device() const
{
  assert(m_device != 0);
  return *m_device;
}

void hiros::xsens_mtw::MtwCallback::onLiveDataAvailable(XsDevice* t_device, const XsDataPacket* packet)
{
  XsMutexLocker lock(m_mutex);
  m_packet_buffer.push_back(*packet);
  if (m_packet_buffer.size() > 300) {
    deleteOldestPacket();
  }
}
