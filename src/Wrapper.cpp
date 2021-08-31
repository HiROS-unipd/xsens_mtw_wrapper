// Standard dependencies
#include <numeric>

// Internal dependencies
#include "xsens_mtw/Wrapper.h"

bool hiros::xsens_mtw::Wrapper::s_request_shutdown = false;

hiros::xsens_mtw::Wrapper::Wrapper()
  : number_of_connected_mtws_(0)
  , xsens_mtw_configured_(false)
  , nh_("~")
  , node_namespace_(nh_.getNamespace())
{
  struct sigaction sig_act;
  memset(&sig_act, 0, sizeof(sig_act));
  sig_act.sa_handler = hiros::xsens_mtw::Wrapper::sighandler;
  sigaction(SIGINT, &sig_act, nullptr);
}

void hiros::xsens_mtw::Wrapper::start()
{
  ROS_INFO_STREAM("Xsens Mtw Wrapper... Starting");

  do {
    if (!xsens_mtw_configured_) {
      if (!configure()) {
        ros::shutdown();
        exit(EXIT_FAILURE);
      }
    }
    else {
      ROS_WARN_STREAM("Xsens Mtw Wrapper... Trying to reduce the update rate");

      stopXsensMtw();

      unsigned long up_rate_index = static_cast<unsigned long>(supported_update_rates_.find(update_rate_));

      if (up_rate_index == supported_update_rates_.size() - 1) {
        ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to go to measurement mode");
        ros::shutdown();
        exit(EXIT_FAILURE);
      }

      mtw_params_.desired_update_rate = supported_update_rates_.at(++up_rate_index);

      configureXsensMtw();
    }

    if (!waitMtwConnection()) {
      ros::shutdown();
      exit(EXIT_FAILURE);
    }

    if (!getMtwsDeviceIstances()) {
      ros::shutdown();
      exit(EXIT_FAILURE);
    }

    attachCallbackHandlers();
  } while (!startMeasurement());

  setupRos();

  if (mtw_params_.reset_initial_orientation) {
    resetInitialOrientation();
  }

  syncInitialPackets();
}

void hiros::xsens_mtw::Wrapper::run()
{
  if (wrapper_params_.enable_external_sync
      && std::find_if(mtw_params_.sync_settings.begin(), mtw_params_.sync_settings.end(), [](const XsSyncSetting& s) {
           return (s.m_line == XSL_In1 || s.m_line == XSL_In2);
         }) != mtw_params_.sync_settings.end()) {
    std::set<XsSyncLine> active_sync_lines;
    for (const auto& sync_setting : mtw_params_.sync_settings) {
      active_sync_lines.emplace(sync_setting.m_line);
    }
    std::string active_sync_lines_str;
    for (const auto& sync_line : active_sync_lines) {
      active_sync_lines_str += " " + sync_line_to_str_map.at(sync_line) + ",";
    }
    active_sync_lines_str.pop_back();
    ROS_WARN_STREAM("Xsens Mtw Wrapper... Waiting for external trigger on line(s)" << active_sync_lines_str);
  }
  else {
    ROS_INFO_STREAM(BASH_MSG_GREEN << "Xsens Mtw Wrapper... RUNNING" << BASH_MSG_RESET);
  }

  std::unique_ptr<Synchronizer> sync;
  if (wrapper_params_.synchronize) {
    sync = std::make_unique<Synchronizer>(mtw_callbacks_, sync_policy_map.at(wrapper_params_.sync_policy_name));
  }

  while (ros::ok() && !s_request_shutdown) {
    for (auto& device : connected_devices_) {
      if (mtw_callbacks_.at(device.first)->newDataAvailable()) {
        if (wrapper_params_.synchronize) {
          sync->add(mtw_callbacks_.at(device.first)->getLatestPacket());
        }
        else {
          publishPacket(mtw_callbacks_.at(device.first)->getLatestPacket());
          mtw_callbacks_.at(device.first)->deleteOldestPacket();
        }
      }

      if (wrapper_params_.synchronize) {
        if (sync->newFrameAvailable()) {
          publishFrame(sync->getLatestFrame());
          sync->clearLatestFrame();
        }
      }
    }
    ros::spinOnce();
  }

  stop();
}

bool hiros::xsens_mtw::Wrapper::configure()
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Configuring");

  if (xsens_mtw_configured_) {
    xsens_mtw_configured_ = false;
    stop();
  }

  configureWrapper();
  xsens_mtw_configured_ = configureXsensMtw();

  if (xsens_mtw_configured_) {
    ROS_DEBUG_STREAM("Xsens Mtw Wrapper... CONFIGURED");
  }

  return xsens_mtw_configured_;
}

void hiros::xsens_mtw::Wrapper::stop()
{
  ROS_INFO_STREAM("Xsens Mtw Wrapper... Stopping");

  stopXsensMtw();
  stopWrapper();

  ROS_INFO_STREAM(BASH_MSG_GREEN << "Xsens Mtw Wrapper... STOPPED" << BASH_MSG_RESET);

  ros::shutdown();
  exit(EXIT_FAILURE);
}

void hiros::xsens_mtw::Wrapper::configureWrapper()
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Configuring Wrapper");

  nh_.getParam("desired_update_rate", mtw_params_.desired_update_rate);
  nh_.getParam("desired_radio_channel", mtw_params_.desired_radio_channel);
  nh_.getParam("fixed_latency", mtw_params_.fixed_latency);
  mtw_params_.fixed_latency = std::max(0., mtw_params_.fixed_latency);
  nh_.getParam("reset_initial_orientation", mtw_params_.reset_initial_orientation);

  nh_.getParam("number_of_mtws", wrapper_params_.number_of_mtws);
  nh_.getParam("tf_prefix", wrapper_params_.tf_prefix);
  nh_.getParam("enable_custom_labeling", wrapper_params_.enable_custom_labeling);
  nh_.getParam("enable_external_sync", wrapper_params_.enable_external_sync);

  nh_.getParam("publish_only_recording", wrapper_params_.publish_only_recording);
  nh_.getParam("synchronize", wrapper_params_.synchronize);
  nh_.getParam("sync_policy", wrapper_params_.sync_policy_name);
  if (sync_policy_map.find(wrapper_params_.sync_policy_name) == sync_policy_map.end()) {
    ROS_WARN_STREAM("Xsens Mtw Wrapper... Sync policy " << wrapper_params_.sync_policy_name << " is not supported");
    std::string supported_policies;
    for (auto& policy : sync_policy_map) {
      supported_policies += (policy.first + ", ");
    }
    ROS_WARN_STREAM(
      "Xsens Mtw Wrapper... Supported policies: " << supported_policies.erase(supported_policies.length() - 2));
    ROS_WARN_STREAM("Xsens Mtw Wrapper... Using " << sync_policy_map.rbegin()->first << " policy");
    wrapper_params_.sync_policy_name = sync_policy_map.rbegin()->first;
  }
  nh_.getParam("publish_mimu_array", wrapper_params_.publish_mimu_array);

  if (wrapper_params_.publish_mimu_array && !wrapper_params_.synchronize) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Cannot publish MIMU array messages when Synchronizer is off. Closing");
    ros::shutdown();
    exit(EXIT_FAILURE);
  }

  nh_.getParam("publish_imu", wrapper_params_.publish_imu);
  nh_.getParam("publish_mag", wrapper_params_.publish_mag);
  nh_.getParam("publish_euler", wrapper_params_.publish_euler);
  nh_.getParam("publish_free_acceleration", wrapper_params_.publish_free_acceleration);
  nh_.getParam("publish_pressure", wrapper_params_.publish_pressure);
  nh_.getParam("publish_tf", wrapper_params_.publish_tf);

  bool nothing_to_publish =
    !(wrapper_params_.publish_imu || wrapper_params_.publish_mag || wrapper_params_.publish_euler
      || wrapper_params_.publish_free_acceleration || wrapper_params_.publish_pressure);

  if (nothing_to_publish && !wrapper_params_.publish_tf) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Nothing to publish. Closing");
    ros::shutdown();
    exit(EXIT_FAILURE);
  }

  if (wrapper_params_.enable_custom_labeling) {
    XmlRpc::XmlRpcValue xml_sensor_labels;
    if (nh_.getParam("sensor_labels", xml_sensor_labels)) {

      for (int i = 0; i < xml_sensor_labels.size(); ++i) {
        ids_to_labels_.emplace(utils::toXsDeviceId(xml_sensor_labels[i]["imu_id"]), xml_sensor_labels[i]["label"]);
        labels_to_ids_.emplace(xml_sensor_labels[i]["label"], utils::toXsDeviceId(xml_sensor_labels[i]["imu_id"]));
      }
    }
  }

  if (wrapper_params_.enable_external_sync) {
    XmlRpc::XmlRpcValue xml_sync_settings;
    if (nh_.getParam("sync_settings", xml_sync_settings)) {
      mtw_params_.sync_settings.reserve(static_cast<unsigned long>(xml_sync_settings.size()));

      for (int i = 0; i < xml_sync_settings.size(); ++i) {
        XsSyncSetting sync_setting;

        sync_setting.m_line = getSetting(xml_sync_settings[i]["line"], sync_line_map);
        sync_setting.m_function = getSetting(xml_sync_settings[i]["function"], sync_function_map);
        sync_setting.m_polarity = getSetting(xml_sync_settings[i]["polarity"], sync_polarity_map);
        sync_setting.m_skipFirst = static_cast<uint16_t>(static_cast<int>(xml_sync_settings[i]["skip_first"]));
        sync_setting.m_pulseWidth = static_cast<uint32_t>(static_cast<int>(xml_sync_settings[i]["pulse_width"]));
        sync_setting.m_skipFactor = static_cast<uint16_t>(static_cast<int>(xml_sync_settings[i]["skip_factor"]));
        sync_setting.m_triggerOnce = static_cast<uint8_t>(static_cast<int>(xml_sync_settings[i]["trigger_once"]));

        mtw_params_.sync_settings.push_back(sync_setting);
      }
    }
  }
}

bool hiros::xsens_mtw::Wrapper::configureXsensMtw()
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Configuring Xsens Mtw");

  bool success = constructControl();
  success = success && findWirelessMaster();
  success = success && openPort();
  success = success && getXsdeviceInstance();
  success = success && setSyncSettings();
  success = success && setConfigMode();
  attachCallbackHandler();
  success = success && getClosestUpdateRate();
  success = success && setUpdateRate();
  success = success && setRadioChannel();

  if (!success) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to configure Xsens Mtw");
  }

  return success;
}

void hiros::xsens_mtw::Wrapper::stopXsensMtw()
{
  if (!setConfigMode()) {
    ros::shutdown();
    exit(EXIT_FAILURE);
  }

  if (!disableRadio()) {
    ros::shutdown();
    exit(EXIT_FAILURE);
  }

  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Closing XsControl");
  control_->close();

  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Deleting MTW callbacks");
  for (auto& mtw_callback : mtw_callbacks_) {
    delete (mtw_callback.second);
  }
  mtw_callbacks_.clear();

  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Clearing MTW devices");
  connected_devices_.clear();
}

void hiros::xsens_mtw::Wrapper::stopWrapper()
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Shutting down ROS publisher(s)");
  if (wrapper_params_.publish_mimu_array) {
    if (data_pub_) {
      data_pub_.shutdown();
    }
  }
  else {
    if (wrapper_params_.publish_imu) {
      for (auto& pub : imu_pubs_) {
        if (pub.second) {
          pub.second.shutdown();
        }
      }
    }

    if (wrapper_params_.publish_mag) {
      for (auto& pub : mag_pubs_) {
        if (pub.second) {
          pub.second.shutdown();
        }
      }
    }

    if (wrapper_params_.publish_euler) {
      for (auto& pub : euler_pubs_) {
        if (pub.second) {
          pub.second.shutdown();
        }
      }
    }

    if (wrapper_params_.publish_free_acceleration) {
      for (auto& pub : free_acceleration_pubs_) {
        if (pub.second) {
          pub.second.shutdown();
        }
      }
    }

    if (wrapper_params_.publish_pressure) {
      for (auto& pub : pressure_pubs_) {
        if (pub.second) {
          pub.second.shutdown();
        }
      }
    }
  }

  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Shutting down ROS service server(s)");
  reset_orientation_srv_.shutdown();
  start_recording_srv_.shutdown();
  stop_recording_srv_.shutdown();

  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Clearing maps");
  ids_to_labels_.clear();
  labels_to_ids_.clear();
}

bool hiros::xsens_mtw::Wrapper::waitMtwConnection()
{
  ROS_INFO_STREAM("Xsens Mtw Wrapper... Waiting for MTW to wirelessly connect");

  if (wrapper_params_.number_of_mtws <= 0) {
    XsTime::msleep(connection_timeout_);
    number_of_connected_mtws_ = wireless_master_callback_.getWirelessMTWs().size();
  }
  else {
    while (number_of_connected_mtws_ < static_cast<unsigned int>(wrapper_params_.number_of_mtws)) {
      XsTime::msleep(500);
      number_of_connected_mtws_ = wireless_master_callback_.getWirelessMTWs().size();
    }
  }

  ROS_INFO_STREAM("Xsens Mtw Wrapper... Number of connected MTWs: " << number_of_connected_mtws_);

  if (number_of_connected_mtws_ == 0) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to connect to MTWs");
    return false;
  }

  return true;
}

bool hiros::xsens_mtw::Wrapper::getMtwsDeviceIstances()
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Getting XsDevice instances for all MTWs");

  for (auto& xs_device_id : control_->deviceIds()) {
    if (xs_device_id.isMtw()) {
      XsDevicePtr xs_device = control_->device(xs_device_id);
      if (xs_device != nullptr) {
        connected_devices_.emplace(xs_device_id, xs_device);
      }
      else {
        ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to create an MTW XsDevice instance");
        return false;
      }
    }
  }

  return true;
}

void hiros::xsens_mtw::Wrapper::attachCallbackHandlers()
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Attaching callback handlers to MTWs");

  int mtw_index = 0;
  for (auto& device : connected_devices_) {
    mtw_callbacks_.emplace(device.first, new MtwCallback(mtw_index++, device.second));
    device.second->addCallbackHandler(mtw_callbacks_.at(device.first));
  }
}

bool hiros::xsens_mtw::Wrapper::startMeasurement()
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Starting measurement");

  if (!wireless_master_device_->gotoMeasurement()) {
    ROS_WARN_STREAM("Xsens Mtw Wrapper... Failed to go to measurement mode");
    return false;
  }

  return true;
}

bool hiros::xsens_mtw::Wrapper::constructControl()
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Constructing XsControl");

  control_ = XsControl::construct();

  if (control_ == nullptr) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to construct XsControl instance");
    return false;
  }

  return true;
}

bool hiros::xsens_mtw::Wrapper::findWirelessMaster()
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Scanning ports");

  detected_devices_ = XsScanner::scanPorts();

  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Finding wireless master");

  wireless_master_port_ = detected_devices_.begin();
  while (wireless_master_port_ != detected_devices_.end() && !wireless_master_port_->deviceId().isWirelessMaster()) {
    ++wireless_master_port_;
  }

  if (wireless_master_port_ == detected_devices_.end()) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... No wireless masters found");
    return false;
  }

  ROS_INFO_STREAM("Xsens Mtw Wrapper... Wireless master found @ " << *wireless_master_port_);
  return true;
}

bool hiros::xsens_mtw::Wrapper::openPort()
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Opening port");

  if (!control_->openPort(wireless_master_port_->portName().toStdString(), wireless_master_port_->baudrate())) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to open port " << *wireless_master_port_);
    return false;
  }

  return true;
}

bool hiros::xsens_mtw::Wrapper::getXsdeviceInstance()
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Getting XsDevice instance for wireless master");

  wireless_master_device_ = control_->device(wireless_master_port_->deviceId());
  if (wireless_master_device_ == nullptr) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to construct XsDevice instance: " << *wireless_master_port_);
    return false;
  }

  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... XsDevice instance created @ " << utils::toString(*wireless_master_device_));
  return true;
}

bool hiros::xsens_mtw::Wrapper::setSyncSettings()
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Setting sync settings for wireless master");

  if (!wireless_master_device_->setSyncSettings(mtw_params_.sync_settings)) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to set sync settings");
    return false;
  }

  return true;
}

bool hiros::xsens_mtw::Wrapper::setConfigMode()
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Setting config mode");

  if (!wireless_master_device_->gotoConfig()) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to go to config mode: " + utils::toString(*wireless_master_device_));
    return false;
  }

  return true;
}

void hiros::xsens_mtw::Wrapper::attachCallbackHandler()
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Attaching callback handler");

  wireless_master_device_->addCallbackHandler(&wireless_master_callback_);
}

bool hiros::xsens_mtw::Wrapper::getClosestUpdateRate()
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Getting the list of the supported update rates");

  supported_update_rates_ = wireless_master_device_->supportedUpdateRates();

  std::string info_str = "Xsens Mtw Wrapper... Supported update rates:";
  for (auto& up_rate : supported_update_rates_) {
    info_str += " " + std::to_string(up_rate);
  }
  ROS_INFO_STREAM(info_str);

  if (supported_update_rates_.empty()) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to get supported update rates");
    return false;
  }

  if (supported_update_rates_.size() == 1) {
    update_rate_ = supported_update_rates_.at(0);
  }
  else {
    int u_rate_dist = -1;
    int closest_update_rate = -1;

    for (auto& up_rate : supported_update_rates_) {
      const int curr_dist = std::abs(up_rate - mtw_params_.desired_update_rate);

      if ((u_rate_dist == -1) || (curr_dist < u_rate_dist)) {
        u_rate_dist = curr_dist;
        closest_update_rate = up_rate;
      }
    }

    update_rate_ = closest_update_rate;
  }

  return true;
}

bool hiros::xsens_mtw::Wrapper::setUpdateRate()
{
  ROS_INFO_STREAM(BASH_MSG_GREEN << "Xsens Mtw Wrapper... Setting update rate to " << update_rate_ << " Hz"
                                 << BASH_MSG_RESET);

  if (!wireless_master_device_->setUpdateRate(update_rate_)) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to set update rate: " << utils::toString(*wireless_master_device_));
    return false;
  }

  return true;
}

bool hiros::xsens_mtw::Wrapper::setRadioChannel()
{
  if (wireless_master_device_->isRadioEnabled()) {
    ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Disabling previously enabled radio channel");
    if (!disableRadio()) {
      return false;
    }
  }

  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Setting radio channel to " << mtw_params_.desired_radio_channel
                                                                    << " and enabling radio");
  if (!wireless_master_device_->enableRadio(mtw_params_.desired_radio_channel)) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to set radio channel: " << utils::toString(*wireless_master_device_));
    return false;
  }

  return true;
}

bool hiros::xsens_mtw::Wrapper::disableRadio()
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Disabling radio");

  if (!wireless_master_device_->disableRadio()) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to disable radio: " << utils::toString(*wireless_master_device_));
    return false;
  }

  return true;
}

void hiros::xsens_mtw::Wrapper::setupRos()
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Setting up ROS");

  reset_orientation_srv_ = nh_.advertiseService("reset_orientation", &Wrapper::resetOrientation, this);
  start_recording_srv_ = nh_.advertiseService("start_recording", &Wrapper::startRecording, this);
  stop_recording_srv_ = nh_.advertiseService("stop_recording", &Wrapper::stopRecording, this);

  if (wrapper_params_.publish_mimu_array) {
    data_pub_ = nh_.advertise<hiros_xsens_mtw_wrapper::MIMUArray>("mimu/data", ros_topic_queue_size_);
  }
  else {
    for (auto& device : connected_devices_) {
      if (wrapper_params_.publish_imu) {
        imu_pubs_.emplace(
          device.first,
          nh_.advertise<sensor_msgs::Imu>(composeTopicPrefix(device.first) + "/imu/data", ros_topic_queue_size_));
      }

      if (wrapper_params_.publish_mag) {
        mag_pubs_.emplace(device.first,
                          nh_.advertise<sensor_msgs::MagneticField>(composeTopicPrefix(device.first) + "/imu/mag",
                                                                    ros_topic_queue_size_));
      }

      if (wrapper_params_.publish_euler) {
        euler_pubs_.emplace(device.first,
                            nh_.advertise<hiros_xsens_mtw_wrapper::Euler>(
                              composeTopicPrefix(device.first) + "/imu/euler", ros_topic_queue_size_));
      }

      if (wrapper_params_.publish_free_acceleration) {
        free_acceleration_pubs_.emplace(
          device.first,
          nh_.advertise<geometry_msgs::Vector3Stamped>(composeTopicPrefix(device.first) + "/filter/free_acceleration",
                                                       ros_topic_queue_size_));
      }

      if (wrapper_params_.publish_pressure) {
        pressure_pubs_.emplace(device.first,
                               nh_.advertise<sensor_msgs::FluidPressure>(composeTopicPrefix(device.first) + "/pressure",
                                                                         ros_topic_queue_size_));
      }
    }
  }
}

void hiros::xsens_mtw::Wrapper::syncInitialPackets()
{
  double delta_t = 1 / static_cast<double>(update_rate_);
  std::map<XsDeviceId, bool> initial_new_packets_received;
  std::map<XsDeviceId, long> initial_packet_ids;
  std::map<XsDeviceId, double> initial_timestamps;
  for (auto& device : connected_devices_) {
    initial_new_packets_received.emplace(device.first, false);
    initial_packet_ids.emplace(device.first, -1);
    initial_timestamps.emplace(device.first, std::numeric_limits<double>::max());
  }
  bool got_all_initial_new_packets = false;

  while (!got_all_initial_new_packets) {
    for (auto& device : connected_devices_) {
      if (!initial_new_packets_received.at(device.first) && mtw_callbacks_.at(device.first)->newDataAvailable()) {
        std::shared_ptr<XsDataPacket> packet = mtw_callbacks_.at(device.first)->getLatestPacket();

        if ((packet->timeOfArrival().secTime() - initial_timestamps.at(device.first)) < (0.5 * delta_t)) {
          // First message
          initial_packet_ids.at(device.first) = packet->packetId();
          initial_timestamps.at(device.first) = packet->timeOfArrival().secTime();
        }
        else {
          // First message with different time of arrival
          initial_packet_ids.at(device.first) = packet->packetId();
          initial_timestamps.at(device.first) += delta_t;

          initial_new_packets_received.at(device.first) = true;
        }

        mtw_callbacks_.at(device.first)->deleteOldestPacket();
      }
    }

    got_all_initial_new_packets =
      std::all_of(initial_new_packets_received.begin(),
                  initial_new_packets_received.end(),
                  [](std::map<XsDeviceId, bool>::const_reference p) { return p.second == true; });
  }

  initial_packet_id_ =
    std::max_element(initial_packet_ids.begin(),
                     initial_packet_ids.end(),
                     [](std::map<XsDeviceId, long>::const_reference p1,
                        std::map<XsDeviceId, long>::const_reference p2) { return p1.second < p2.second; })
      ->second;

  for (auto& device : connected_devices_) {
    initial_timestamps.at(device.first) += ((initial_packet_id_ - initial_packet_ids.at(device.first)) * delta_t);
  }

  initial_timestamp_ =
    ros::Time(std::min_element(initial_timestamps.begin(),
                               initial_timestamps.end(),
                               [](std::map<XsDeviceId, double>::const_reference t1,
                                  std::map<XsDeviceId, double>::const_reference t2) { return t1.second < t2.second; })
                ->second
              - mtw_params_.fixed_latency);
}

bool hiros::xsens_mtw::Wrapper::resetInitialOrientation() const
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Resetting initial orientation");

  bool success = true;

  for (auto& device : connected_devices_) {
    success = device.second->resetOrientation(XRM_Heading) && success;
  }

  if (!success) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to reset initial orientation");
  }

  return success;
}

std::string hiros::xsens_mtw::Wrapper::getDeviceLabel(const XsDeviceId& id) const
{
  return (ids_to_labels_.find(id) != ids_to_labels_.end()) ? ids_to_labels_.at(id) : id.toString().toStdString();
}

XsDeviceId hiros::xsens_mtw::Wrapper::getDeviceId(const std::string label) const
{
  return (labels_to_ids_.find(label) != labels_to_ids_.end()) ? labels_to_ids_.at(label) : utils::toXsDeviceId(label);
}

std::string hiros::xsens_mtw::Wrapper::composeTopicPrefix(const XsDeviceId& id) const
{
  return "/" + node_namespace_ + "/" + getDeviceLabel(id);
}

void hiros::xsens_mtw::Wrapper::publishPacket(std::shared_ptr<XsDataPacket> packet)
{
  if (wrapper_params_.publish_only_recording && !wireless_master_device_->isRecording()) {
    return;
  }

  if (wrapper_params_.publish_imu && packet->containsOrientation() && packet->containsCalibratedGyroscopeData()
      && packet->containsCalibratedAcceleration()) {
    imu_pubs_.at(packet->deviceId()).publish(getImuMsg(packet));
  }

  if (wrapper_params_.publish_mag && packet->containsCalibratedMagneticField()) {
    mag_pubs_.at(packet->deviceId()).publish(getMagMsg(packet));
  }

  if (wrapper_params_.publish_euler && packet->containsOrientation()) {
    euler_pubs_.at(packet->deviceId()).publish(getEulerMsg(packet));
  }

  if (wrapper_params_.publish_free_acceleration && packet->containsFreeAcceleration()) {
    free_acceleration_pubs_.at(packet->deviceId()).publish(getFreeAccelerationMsg(packet));
  }

  if (wrapper_params_.publish_pressure && packet->containsPressure()) {
    pressure_pubs_.at(packet->deviceId()).publish(getPressureMsg(packet));
  }

  if (wrapper_params_.publish_tf && packet->containsOrientation()) {
    tf_broadcaster_.sendTransform(getTf(packet));
  }
}

void hiros::xsens_mtw::Wrapper::publishFrame(const std::vector<std::shared_ptr<XsDataPacket>>& frame)
{
  if (wrapper_params_.publish_only_recording && !wireless_master_device_->isRecording()) {
    return;
  }

  if (wrapper_params_.publish_mimu_array) {
    data_pub_.publish(getMIMUArrayMsg(frame));

    if (wrapper_params_.publish_tf) {
      for (auto& packet : frame) {
        if (packet->containsOrientation()) {
          tf_broadcaster_.sendTransform(getTf(packet));
        }
      }
    }
  }
  else {
    for (auto& packet : frame) {
      publishPacket(packet);
    }
  }
}

std_msgs::Header hiros::xsens_mtw::Wrapper::getHeader(std::shared_ptr<XsDataPacket> packet) const
{
  std_msgs::Header header;

  header.seq = static_cast<unsigned int>(packet->packetId());
  header.stamp =
    initial_timestamp_ + ros::Duration((packet->packetId() - initial_packet_id_) / static_cast<double>(update_rate_));
  header.frame_id = wrapper_params_.tf_prefix + getDeviceLabel(packet->deviceId());

  return header;
}

sensor_msgs::Imu hiros::xsens_mtw::Wrapper::getImuMsg(std::shared_ptr<XsDataPacket> packet) const
{
  sensor_msgs::Imu out_msg;
  out_msg.header = getHeader(packet);

  out_msg.orientation.x = packet->orientationQuaternion().x();
  out_msg.orientation.y = packet->orientationQuaternion().y();
  out_msg.orientation.z = packet->orientationQuaternion().z();
  out_msg.orientation.w = packet->orientationQuaternion().w();
  out_msg.orientation_covariance.front() = 0.0;

  out_msg.angular_velocity.x = packet->calibratedGyroscopeData().at(0);
  out_msg.angular_velocity.y = packet->calibratedGyroscopeData().at(1);
  out_msg.angular_velocity.z = packet->calibratedGyroscopeData().at(2);
  out_msg.angular_velocity_covariance.front() = 0.0;

  out_msg.linear_acceleration.x = packet->calibratedAcceleration().at(0);
  out_msg.linear_acceleration.y = packet->calibratedAcceleration().at(1);
  out_msg.linear_acceleration.z = packet->calibratedAcceleration().at(2);
  out_msg.linear_acceleration_covariance.front() = 0.0;

  return out_msg;
}

sensor_msgs::MagneticField hiros::xsens_mtw::Wrapper::getMagMsg(std::shared_ptr<XsDataPacket> t_packet) const
{
  sensor_msgs::MagneticField out_msg;
  out_msg.header = getHeader(t_packet);

  out_msg.magnetic_field.x = t_packet->calibratedMagneticField().at(0) * 1e-4; // G to T
  out_msg.magnetic_field.y = t_packet->calibratedMagneticField().at(1) * 1e-4; // G to T
  out_msg.magnetic_field.z = t_packet->calibratedMagneticField().at(2) * 1e-4; // G to T
  out_msg.magnetic_field_covariance.front() = 0.0;

  return out_msg;
}

hiros_xsens_mtw_wrapper::Euler hiros::xsens_mtw::Wrapper::getEulerMsg(std::shared_ptr<XsDataPacket> packet) const
{
  hiros_xsens_mtw_wrapper::Euler out_msg;
  out_msg.header = getHeader(packet);

  // roll = atan2(2 * (qw * qx + qy * qz), (1 - 2 * (pow(qx, 2) + pow(qy, 2))))
  out_msg.roll = packet->orientationEuler().roll();
  // pitch = asin(2 * (qw * qy - qz * qx))
  out_msg.pitch = packet->orientationEuler().pitch();
  // yaw = atan2(2 * (qw * qz + qx * qy), (1 - 2 * (pow(qy, 2) + pow(qz, 2))))
  out_msg.yaw = packet->orientationEuler().yaw();

  return out_msg;
}

geometry_msgs::Vector3Stamped
hiros::xsens_mtw::Wrapper::getFreeAccelerationMsg(std::shared_ptr<XsDataPacket> packet) const
{
  geometry_msgs::Vector3Stamped out_msg;
  out_msg.header = getHeader(packet);

  out_msg.vector.x = packet->freeAcceleration().at(0);
  out_msg.vector.y = packet->freeAcceleration().at(1);
  out_msg.vector.z = packet->freeAcceleration().at(2);

  return out_msg;
}

sensor_msgs::FluidPressure hiros::xsens_mtw::Wrapper::getPressureMsg(std::shared_ptr<XsDataPacket> t_packet) const
{
  sensor_msgs::FluidPressure out_msg;
  out_msg.header = getHeader(t_packet);

  out_msg.fluid_pressure = t_packet->pressure().m_pressure;
  out_msg.variance = 0.0;

  return out_msg;
}

hiros_xsens_mtw_wrapper::MIMU hiros::xsens_mtw::Wrapper::getMIMUMsg(std::shared_ptr<XsDataPacket> packet) const
{
  hiros_xsens_mtw_wrapper::MIMU out_msg;

  if (wrapper_params_.publish_imu && packet->containsOrientation() && packet->containsCalibratedGyroscopeData()
      && packet->containsCalibratedAcceleration()) {
    out_msg.imu = getImuMsg(packet);
  }

  if (wrapper_params_.publish_mag && packet->containsCalibratedMagneticField()) {
    out_msg.mag = getMagMsg(packet);
  }

  if (wrapper_params_.publish_euler && packet->containsOrientation()) {
    out_msg.euler = getEulerMsg(packet);
  }

  if (wrapper_params_.publish_free_acceleration && packet->containsFreeAcceleration()) {
    out_msg.free_acceleration = getFreeAccelerationMsg(packet);
  }

  if (wrapper_params_.publish_pressure && packet->containsPressure()) {
    out_msg.pressure = getPressureMsg(packet);
  }

  return out_msg;
}

hiros_xsens_mtw_wrapper::MIMUArray
hiros::xsens_mtw::Wrapper::getMIMUArrayMsg(const std::vector<std::shared_ptr<XsDataPacket>>& frame) const
{
  hiros_xsens_mtw_wrapper::MIMUArray out_msg;

  for (auto& packet : frame) {
    out_msg.mimus.push_back(getMIMUMsg(packet));
  }

  return out_msg;
}

geometry_msgs::TransformStamped hiros::xsens_mtw::Wrapper::getTf(std::shared_ptr<XsDataPacket> packet) const
{
  geometry_msgs::TransformStamped tf;
  tf.header = getHeader(packet);
  tf.header.frame_id = "world";
  tf.child_frame_id = wrapper_params_.tf_prefix + getDeviceLabel(packet->deviceId());

  tf.transform.translation.x = 0.0;
  tf.transform.translation.y = 0.0;
  tf.transform.translation.z = 0.0;
  tf.transform.rotation.x = packet->orientationQuaternion().x();
  tf.transform.rotation.y = packet->orientationQuaternion().y();
  tf.transform.rotation.z = packet->orientationQuaternion().z();
  tf.transform.rotation.w = packet->orientationQuaternion().w();

  return tf;
}

bool hiros::xsens_mtw::Wrapper::resetOrientation(hiros_xsens_mtw_wrapper::ResetOrientation::Request& req,
                                                 hiros_xsens_mtw_wrapper::ResetOrientation::Response& res)
{
  bool success = true;

  if (req.sensors.empty()) {
    ROS_INFO_STREAM(BASH_MSG_GREEN << "Xsens Mtw Wrapper... Resetting orientation of all connected sensors"
                                   << BASH_MSG_RESET);
    for (auto& device : connected_devices_) {
      success = device.second->resetOrientation(XRM_Alignment) && success;
    }
  }
  else {
    for (auto& sensor_label : req.sensors) {
      if (connected_devices_.find(getDeviceId(sensor_label)) != connected_devices_.end()) {
        ROS_INFO_STREAM(BASH_MSG_GREEN << "Xsens Mtw Wrapper... Resetting orientation of '" << sensor_label << "'"
                                       << BASH_MSG_RESET);
        success = connected_devices_.at(getDeviceId(sensor_label))->resetOrientation(XRM_Alignment) && success;
      }
      else {
        ROS_WARN_STREAM("Xsens Mtw Wrapper... Cannot find '" << sensor_label << "'");
        success = false;
      }
    }
  }

  return success;
}

bool hiros::xsens_mtw::Wrapper::startRecording(hiros_xsens_mtw_wrapper::StartRecording::Request& req,
                                               hiros_xsens_mtw_wrapper::StartRecording::Response& res)
{
  if (!wireless_master_device_->startRecording()) {
    ROS_WARN_STREAM("Xsens Mtw Wrapper... Failed to start recording");
    return false;
  }

  ROS_INFO_STREAM(BASH_MSG_GREEN << "Xsens Mtw Wrapper... Start recording @ " << ros::Time::now());
  return true;
}

bool hiros::xsens_mtw::Wrapper::stopRecording(hiros_xsens_mtw_wrapper::StopRecording::Request& req,
                                              hiros_xsens_mtw_wrapper::StopRecording::Response& res)
{
  if (!wireless_master_device_->stopRecording()) {
    ROS_WARN_STREAM("Xsens Mtw Wrapper... Failed to stop recording");
    return false;
  }

  ROS_INFO_STREAM(BASH_MSG_GREEN "Xsens Mtw Wrapper... Stop  recording @ " << ros::Time::now());
  return true;
}
