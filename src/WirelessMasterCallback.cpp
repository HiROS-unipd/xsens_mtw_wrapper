#include "xsens_mtw/WirelessMasterCallback.h"

mtw::utils::XsDeviceSet mtw::WirelessMasterCallback::getWirelessMTWs() const
{
  XsMutexLocker lock(m_mutex);
  return m_connectedMTWs;
}

void mtw::WirelessMasterCallback::onConnectivityChanged(XsDevice* dev, XsConnectivityState newState)
{
  XsMutexLocker lock(m_mutex);
  switch (newState) {
    case XCS_Disconnected: /*!< Device has disconnected, only limited informational functionality is available. */
      mtw::utils::operator<<(std::cout << "EVENT: MTW Disconnected -> ", *dev) << std::endl;
      m_connectedMTWs.erase(dev);
      break;
    case XCS_Rejected: /*!< Device has been rejected and is disconnected, only limited informational functionality is
                          available. */
      mtw::utils::operator<<(std::cout << "EVENT: MTW Rejected -> ", *dev) << std::endl;
      m_connectedMTWs.erase(dev);
      break;
    case XCS_PluggedIn: /*!< Device is connected through a cable. */
      mtw::utils::operator<<(std::cout << "EVENT: MTW PluggedIn -> ", *dev) << std::endl;
      m_connectedMTWs.erase(dev);
      break;
    case XCS_Wireless: /*!< Device is connected wirelessly. */
      mtw::utils::operator<<(std::cout << "EVENT: MTW Connected -> ", *dev) << std::endl;
      m_connectedMTWs.insert(dev);
      break;
    case XCS_File: /*!< Device is reading from a file. */
      mtw::utils::operator<<(std::cout << "EVENT: MTW File -> ", *dev) << std::endl;
      m_connectedMTWs.erase(dev);
      break;
    case XCS_Unknown: /*!< Device is in an unknown state. */
      mtw::utils::operator<<(std::cout << "EVENT: MTW Unknown -> ", *dev) << std::endl;
      m_connectedMTWs.erase(dev);
      break;
    default:
      mtw::utils::operator<<(std::cout << "EVENT: MTW Error -> ", *dev) << std::endl;
      m_connectedMTWs.erase(dev);
      break;
  }
}
