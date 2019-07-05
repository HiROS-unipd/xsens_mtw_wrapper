#include <iostream>

#include "xsens_mtw/MtwCallback.h"

mtw::MtwCallback::MtwCallback(int mtwIndex, XsDevice* device)
  : m_mtwIndex(mtwIndex)
  , m_device(device)
{}

bool mtw::MtwCallback::dataAvailable() const
{
  XsMutexLocker lock(m_mutex);
  return !m_packetBuffer.empty();
}

XsDataPacket const* mtw::MtwCallback::getOldestPacket() const
{
  XsMutexLocker lock(m_mutex);
  XsDataPacket const* packet = &m_packetBuffer.front();
  return packet;
}

void mtw::MtwCallback::deleteOldestPacket()
{
  XsMutexLocker lock(m_mutex);
  m_packetBuffer.pop_front();
}

XsDevice const& mtw::MtwCallback::device() const
{
  assert(m_device != 0);
  return *m_device;
}

void mtw::MtwCallback::onLiveDataAvailable(XsDevice*, const XsDataPacket* packet)
{
  XsMutexLocker lock(m_mutex);
  // NOTE: Processing of packets should not be done in this thread.

  m_packetBuffer.push_back(*packet);
  if (m_packetBuffer.size() > 300) {
    std::cout << std::endl;
    deleteOldestPacket();
  }
}
