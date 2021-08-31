// Standard dependencies
#include <iostream>

// Internal dependencies
#include "xsens_mtw/MtwCallback.h"

hiros::xsens_mtw::MtwCallback::MtwCallback(int mtw_index, XsDevice* device)
  : read_packets_(0)
  , mtw_index_(mtw_index)
  , device_(device)
{}

bool hiros::xsens_mtw::MtwCallback::dataAvailable() const
{
  XsMutexLocker lock(mutex_);
  return !packet_buffer_.empty();
}

bool hiros::xsens_mtw::MtwCallback::newDataAvailable()
{
  XsMutexLocker lock(mutex_);
  bool new_data_availabe = (packet_buffer_.size() > static_cast<unsigned long>(read_packets_));
  return new_data_availabe;
}

std::shared_ptr<XsDataPacket> hiros::xsens_mtw::MtwCallback::getLatestPacket()
{
  XsMutexLocker lock(mutex_);
  ++read_packets_;
  return std::make_shared<XsDataPacket>(packet_buffer_.at(read_packets_ - 1));
}

void hiros::xsens_mtw::MtwCallback::deleteOldestPacket()
{
  return deleteOldestPackets(1);
}

void hiros::xsens_mtw::MtwCallback::deleteOldestPackets(const unsigned long& n_packets)
{
  XsMutexLocker lock(mutex_);
  auto n_packets_to_delete = static_cast<long>(std::min(n_packets, packet_buffer_.size()));
  packet_buffer_.erase(packet_buffer_.begin(), packet_buffer_.begin() + n_packets_to_delete);
  read_packets_ = std::max(0, static_cast<int>(read_packets_ - n_packets_to_delete));
}

XsDevice const& hiros::xsens_mtw::MtwCallback::device() const
{
  assert(device_ != 0);
  return *device_;
}

void hiros::xsens_mtw::MtwCallback::onLiveDataAvailable(XsDevice* device, const XsDataPacket* packet)
{
  XsMutexLocker lock(mutex_);
  packet_buffer_.push_back(*packet);
  if (packet_buffer_.size() > max_buffer_size_) {
    std::cout << "MtwCallback... Warning: buffer size > " << max_buffer_size_ << ". Deleting oldest packet"
              << std::endl;
    deleteOldestPacket();
  }
}
