// Standard dependencies
#include <iostream>

// Internal dependencies
#include "xsens_mtw/WirelessMasterCallback.h"

hiros::xsens_mtw::utils::XsDeviceSet hiros::xsens_mtw::WirelessMasterCallback::getWirelessMTWs() const
{
  XsMutexLocker lock(m_mutex);
  return m_connected_mtws;
}

void hiros::xsens_mtw::WirelessMasterCallback::onConnectivityChanged(XsDevice* t_device,
                                                                     XsConnectivityState t_new_state)
{
  XsMutexLocker lock(m_mutex);
  switch (t_new_state) {
    case XCS_Disconnected: /*!< Device has disconnected, only limited informational functionality is available. */
      utils::operator<<(std::cout << "EVENT: MTW Disconnected -> ", *t_device) << std::endl;
      m_connected_mtws.erase(t_device);
      break;
    case XCS_Rejected: /*!< Device has been rejected and is disconnected, only limited informational functionality is
                          available. */
      utils::operator<<(std::cout << "EVENT: MTW Rejected -> ", *t_device) << std::endl;
      m_connected_mtws.erase(t_device);
      break;
    case XCS_PluggedIn: /*!< Device is connected through a cable. */
      utils::operator<<(std::cout << "EVENT: MTW PluggedIn -> ", *t_device) << std::endl;
      m_connected_mtws.erase(t_device);
      break;
    case XCS_Wireless: /*!< Device is connected wirelessly. */
      utils::operator<<(std::cout << "EVENT: MTW Connected -> ", *t_device) << std::endl;
      m_connected_mtws.insert(t_device);
      break;
    case XCS_File: /*!< Device is reading from a file. */
      utils::operator<<(std::cout << "EVENT: MTW File -> ", *t_device) << std::endl;
      m_connected_mtws.erase(t_device);
      break;
    case XCS_Unknown: /*!< Device is in an unknown state. */
      utils::operator<<(std::cout << "EVENT: MTW Unknown -> ", *t_device) << std::endl;
      m_connected_mtws.erase(t_device);
      break;
    default:
      utils::operator<<(std::cout << "EVENT: MTW Error -> ", *t_device) << std::endl;
      m_connected_mtws.erase(t_device);
      break;
  }
}
