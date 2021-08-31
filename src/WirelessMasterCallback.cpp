// Standard dependencies
#include <iostream>

// Internal dependencies
#include "xsens_mtw/WirelessMasterCallback.h"

hiros::xsens_mtw::utils::XsDeviceSet hiros::xsens_mtw::WirelessMasterCallback::getWirelessMTWs() const
{
  XsMutexLocker lock(mutex_);
  return connected_mtws_;
}

void hiros::xsens_mtw::WirelessMasterCallback::onConnectivityChanged(XsDevice* device, XsConnectivityState new_state)
{
  XsMutexLocker lock(mutex_);
  switch (new_state) {
    case XCS_Disconnected: /*!< Device has disconnected, only limited informational functionality is available. */
      utils::operator<<(std::cout << "EVENT: MTW Disconnected -> ", *device) << std::endl;
      connected_mtws_.erase(device);
      break;
    case XCS_Rejected: /*!< Device has been rejected and is disconnected, only limited informational functionality is
                          available. */
      utils::operator<<(std::cout << "EVENT: MTW Rejected -> ", *device) << std::endl;
      connected_mtws_.erase(device);
      break;
    case XCS_PluggedIn: /*!< Device is connected through a cable. */
      utils::operator<<(std::cout << "EVENT: MTW PluggedIn -> ", *device) << std::endl;
      connected_mtws_.erase(device);
      break;
    case XCS_Wireless: /*!< Device is connected wirelessly. */
      utils::operator<<(std::cout << "EVENT: MTW Connected -> ", *device) << std::endl;
      connected_mtws_.insert(device);
      break;
    case XCS_File: /*!< Device is reading from a file. */
      utils::operator<<(std::cout << "EVENT: MTW File -> ", *device) << std::endl;
      connected_mtws_.erase(device);
      break;
    case XCS_Unknown: /*!< Device is in an unknown state. */
      utils::operator<<(std::cout << "EVENT: MTW Unknown -> ", *device) << std::endl;
      connected_mtws_.erase(device);
      break;
    default:
      utils::operator<<(std::cout << "EVENT: MTW Error -> ", *device) << std::endl;
      connected_mtws_.erase(device);
      break;
  }
}
