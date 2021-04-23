// Standard dependencies
#include <numeric>

// Internal dependencies
#include "xsens_mtw/Wrapper.h"

bool hiros::xsens_mtw::Wrapper::s_request_shutdown = false;

hiros::xsens_mtw::Wrapper::Wrapper()
  : m_number_of_connected_mtws(0)
  , m_xsens_mtw_configured(false)
  , m_nh("~")
  , m_node_namespace(m_nh.getNamespace())
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
    if (!m_xsens_mtw_configured) {
      if (!configure()) {
        ros::shutdown();
        exit(EXIT_FAILURE);
      }
    }
    else {
      ROS_WARN_STREAM("Xsens Mtw Wrapper... Trying to reduce the update rate");

      stopXsensMtw();

      unsigned long up_rate_index = static_cast<unsigned long>(m_supported_update_rates.find(m_update_rate));

      if (up_rate_index == m_supported_update_rates.size() - 1) {
        ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to go to measurement mode");
        ros::shutdown();
        exit(EXIT_FAILURE);
      }

      m_mtw_params.desired_update_rate = m_supported_update_rates.at(++up_rate_index);

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

  if (m_mtw_params.reset_initial_orientation) {
    resetInitialOrientation();
  }

  syncInitialPackets();
}

void hiros::xsens_mtw::Wrapper::run()
{
  ROS_INFO_STREAM(BASH_MSG_GREEN << "Xsens Mtw Wrapper... RUNNING" << BASH_MSG_RESET);

  std::unique_ptr<Synchronizer> sync;
  if (m_wrapper_params.synchronize) {
    sync = std::make_unique<Synchronizer>(m_mtw_callbacks, sync_policy_map.at(m_wrapper_params.sync_policy_name));
  }

  while (ros::ok() && !s_request_shutdown) {
    for (auto& device : m_connected_devices) {
      if (m_mtw_callbacks.at(device.first)->newDataAvailable()) {
        if (m_wrapper_params.synchronize) {
          sync->add(m_mtw_callbacks.at(device.first)->getLatestPacket());
        }
        else {
          publishPacket(m_mtw_callbacks.at(device.first)->getLatestPacket());
          m_mtw_callbacks.at(device.first)->deleteOldestPacket();
        }
      }

      if (m_wrapper_params.synchronize) {
        if (sync->newFrameAvailable()) {
          publishFrame(sync->getLatestFrame());
          sync->clearLatestFrame();
        }
      }
    }
  }

  stop();
}

bool hiros::xsens_mtw::Wrapper::configure()
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Configuring");

  if (m_xsens_mtw_configured) {
    m_xsens_mtw_configured = false;
    stop();
  }

  configureWrapper();
  m_xsens_mtw_configured = configureXsensMtw();

  if (m_xsens_mtw_configured) {
    ROS_DEBUG_STREAM("Xsens Mtw Wrapper... CONFIGURED");
  }

  return m_xsens_mtw_configured;
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

  m_nh.getParam("desired_update_rate", m_mtw_params.desired_update_rate);
  m_nh.getParam("desired_radio_channel", m_mtw_params.desired_radio_channel);
  m_nh.getParam("fixed_latency", m_mtw_params.fixed_latency);
  m_mtw_params.fixed_latency = std::max(0., m_mtw_params.fixed_latency);
  m_nh.getParam("reset_initial_orientation", m_mtw_params.reset_initial_orientation);

  m_nh.getParam("number_of_mtws", m_wrapper_params.number_of_mtws);
  m_nh.getParam("tf_prefix", m_wrapper_params.tf_prefix);
  m_nh.getParam("enable_custom_labeling", m_wrapper_params.enable_custom_labeling);

  m_nh.getParam("synchronize", m_wrapper_params.synchronize);
  m_nh.getParam("sync_policy", m_wrapper_params.sync_policy_name);
  if (sync_policy_map.find(m_wrapper_params.sync_policy_name) == sync_policy_map.end()) {
    ROS_WARN_STREAM("Xsens Mtw Wrapper... Sync policy " << m_wrapper_params.sync_policy_name << " is not supported");
    std::string supported_policies;
    for (auto& policy : sync_policy_map) {
      supported_policies += (policy.first + ", ");
    }
    ROS_WARN_STREAM(
      "Xsens Mtw Wrapper... Supported policies: " << supported_policies.erase(supported_policies.length() - 2));
    ROS_WARN_STREAM("Xsens Mtw Wrapper... Using " << sync_policy_map.rbegin()->first << " policy");
    m_wrapper_params.sync_policy_name = sync_policy_map.rbegin()->first;
  }
  m_nh.getParam("publish_mimu_array", m_wrapper_params.publish_mimu_array);

  if (m_wrapper_params.publish_mimu_array && !m_wrapper_params.synchronize) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Cannot publish MIMU array messages when Synchronizer is off. Closing");
    ros::shutdown();
    exit(EXIT_FAILURE);
  }

  m_nh.getParam("publish_imu", m_wrapper_params.publish_imu);
  m_nh.getParam("publish_mag", m_wrapper_params.publish_mag);
  m_nh.getParam("publish_euler", m_wrapper_params.publish_euler);
  m_nh.getParam("publish_free_acceleration", m_wrapper_params.publish_free_acceleration);
  m_nh.getParam("publish_pressure", m_wrapper_params.publish_pressure);
  m_nh.getParam("publish_tf", m_wrapper_params.publish_tf);

  bool nothing_to_publish =
    !(m_wrapper_params.publish_imu || m_wrapper_params.publish_mag || m_wrapper_params.publish_euler
      || m_wrapper_params.publish_free_acceleration || m_wrapper_params.publish_pressure);

  if (nothing_to_publish && !m_wrapper_params.publish_tf) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Nothing to publish. Closing");
    ros::shutdown();
    exit(EXIT_FAILURE);
  }

  if (m_wrapper_params.enable_custom_labeling) {
    XmlRpc::XmlRpcValue xml_sensor_labels;
    if (m_nh.getParam("sensor_labels", xml_sensor_labels)) {

      for (int i = 0; i < xml_sensor_labels.size(); ++i) {
        m_ids_to_labels.emplace(utils::toXsDeviceId(xml_sensor_labels[i]["imu_id"]), xml_sensor_labels[i]["label"]);
        m_labels_to_ids.emplace(xml_sensor_labels[i]["label"], utils::toXsDeviceId(xml_sensor_labels[i]["imu_id"]));
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
  m_control->close();

  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Deleting MTW callbacks");
  for (auto& mtw_callback : m_mtw_callbacks) {
    delete (mtw_callback.second);
  }
  m_mtw_callbacks.clear();

  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Clearing MTW devices");
  m_connected_devices.clear();
}

void hiros::xsens_mtw::Wrapper::stopWrapper()
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Shutting down ROS publisher(s)");
  if (m_wrapper_params.publish_mimu_array) {
    if (m_data_pub) {
      m_data_pub.shutdown();
    }
  }
  else {
    if (m_wrapper_params.publish_imu) {
      for (auto& pub : m_imu_pubs) {
        if (pub.second) {
          pub.second.shutdown();
        }
      }
    }

    if (m_wrapper_params.publish_mag) {
      for (auto& pub : m_mag_pubs) {
        if (pub.second) {
          pub.second.shutdown();
        }
      }
    }

    if (m_wrapper_params.publish_euler) {
      for (auto& pub : m_euler_pubs) {
        if (pub.second) {
          pub.second.shutdown();
        }
      }
    }

    if (m_wrapper_params.publish_free_acceleration) {
      for (auto& pub : m_free_acceleration_pubs) {
        if (pub.second) {
          pub.second.shutdown();
        }
      }
    }

    if (m_wrapper_params.publish_pressure) {
      for (auto& pub : m_pressure_pubs) {
        if (pub.second) {
          pub.second.shutdown();
        }
      }
    }
  }

  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Shutting down ROS service server");
  m_reset_orientation_srv.shutdown();

  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Clearing maps");
  m_ids_to_labels.clear();
  m_labels_to_ids.clear();
}

bool hiros::xsens_mtw::Wrapper::waitMtwConnection()
{
  ROS_INFO_STREAM("Xsens Mtw Wrapper... Waiting for MTW to wirelessly connect");

  if (m_wrapper_params.number_of_mtws <= 0) {
    XsTime::msleep(m_connection_timeout);
    m_number_of_connected_mtws = m_wireless_master_callback.getWirelessMTWs().size();
  }
  else {
    while (m_number_of_connected_mtws < static_cast<unsigned int>(m_wrapper_params.number_of_mtws)) {
      XsTime::msleep(500);
      m_number_of_connected_mtws = m_wireless_master_callback.getWirelessMTWs().size();
    }
  }

  ROS_INFO_STREAM("Xsens Mtw Wrapper... Number of connected MTWs: " << m_number_of_connected_mtws);

  if (m_number_of_connected_mtws == 0) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to connect to MTWs");
    return false;
  }

  return true;
}

bool hiros::xsens_mtw::Wrapper::getMtwsDeviceIstances()
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Getting XsDevice instances for all MTWs");

  for (auto& xs_device_id : m_control->deviceIds()) {
    if (xs_device_id.isMtw()) {
      XsDevicePtr xs_device = m_control->device(xs_device_id);
      if (xs_device != nullptr) {
        m_connected_devices.emplace(xs_device_id, xs_device);
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
  for (auto& device : m_connected_devices) {
    m_mtw_callbacks.emplace(device.first, new MtwCallback(mtw_index++, device.second));
    device.second->addCallbackHandler(m_mtw_callbacks.at(device.first));
  }
}

bool hiros::xsens_mtw::Wrapper::startMeasurement()
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Starting measurement");

  if (!m_wireless_master_device->gotoMeasurement()) {
    ROS_WARN_STREAM("Xsens Mtw Wrapper... Failed to go to measurement mode");
    return false;
  }

  return true;
}

bool hiros::xsens_mtw::Wrapper::constructControl()
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Constructing XsControl");

  m_control = XsControl::construct();

  if (m_control == nullptr) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to construct XsControl instance");
    return false;
  }

  return true;
}

bool hiros::xsens_mtw::Wrapper::findWirelessMaster()
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Scanning ports");

  m_detected_devices = XsScanner::scanPorts();

  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Finding wireless master");

  m_wireless_master_port = m_detected_devices.begin();
  while (m_wireless_master_port != m_detected_devices.end() && !m_wireless_master_port->deviceId().isWirelessMaster()) {
    ++m_wireless_master_port;
  }

  if (m_wireless_master_port == m_detected_devices.end()) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... No wireless masters found");
    return false;
  }

  ROS_INFO_STREAM("Xsens Mtw Wrapper... Wireless master found @ " << *m_wireless_master_port);
  return true;
}

bool hiros::xsens_mtw::Wrapper::openPort()
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Opening port");

  if (!m_control->openPort(m_wireless_master_port->portName().toStdString(), m_wireless_master_port->baudrate())) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to open port " << *m_wireless_master_port);
    return false;
  }

  return true;
}

bool hiros::xsens_mtw::Wrapper::getXsdeviceInstance()
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Getting XsDevice instance for wireless master");

  m_wireless_master_device = m_control->device(m_wireless_master_port->deviceId());
  if (m_wireless_master_device == nullptr) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to construct XsDevice instance: " << *m_wireless_master_port);
    return false;
  }

  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... XsDevice instance created @ " << utils::toString(*m_wireless_master_device));
  return true;
}

bool hiros::xsens_mtw::Wrapper::setConfigMode()
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Setting config mode");

  if (!m_wireless_master_device->gotoConfig()) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to go to config mode: " + utils::toString(*m_wireless_master_device));
    return false;
  }

  return true;
}

void hiros::xsens_mtw::Wrapper::attachCallbackHandler()
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Attaching callback handler");

  m_wireless_master_device->addCallbackHandler(&m_wireless_master_callback);
}

bool hiros::xsens_mtw::Wrapper::getClosestUpdateRate()
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Getting the list of the supported update rates");

  m_supported_update_rates = m_wireless_master_device->supportedUpdateRates();

  std::string info_str = "Xsens Mtw Wrapper... Supported update rates:";
  for (auto& up_rate : m_supported_update_rates) {
    info_str += " " + std::to_string(up_rate);
  }
  ROS_INFO_STREAM(info_str);

  if (m_supported_update_rates.empty()) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to get supported update rates");
    return false;
  }

  if (m_supported_update_rates.size() == 1) {
    m_update_rate = m_supported_update_rates.at(0);
  }
  else {
    int u_rate_dist = -1;
    int closest_update_rate = -1;

    for (auto& up_rate : m_supported_update_rates) {
      const int curr_dist = std::abs(up_rate - m_mtw_params.desired_update_rate);

      if ((u_rate_dist == -1) || (curr_dist < u_rate_dist)) {
        u_rate_dist = curr_dist;
        closest_update_rate = up_rate;
      }
    }

    m_update_rate = closest_update_rate;
  }

  return true;
}

bool hiros::xsens_mtw::Wrapper::setUpdateRate()
{
  ROS_INFO_STREAM(BASH_MSG_GREEN << "Xsens Mtw Wrapper... Setting update rate to " << m_update_rate << " Hz"
                                 << BASH_MSG_RESET);

  if (!m_wireless_master_device->setUpdateRate(m_update_rate)) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to set update rate: " << utils::toString(*m_wireless_master_device));
    return false;
  }

  return true;
}

bool hiros::xsens_mtw::Wrapper::setRadioChannel()
{
  if (m_wireless_master_device->isRadioEnabled()) {
    ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Disabling previously enabled radio channel");
    if (!disableRadio()) {
      return false;
    }
  }

  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Setting radio channel to " << m_mtw_params.desired_radio_channel
                                                                    << " and enabling radio");
  if (!m_wireless_master_device->enableRadio(m_mtw_params.desired_radio_channel)) {
    ROS_FATAL_STREAM(
      "Xsens Mtw Wrapper... Failed to set radio channel: " << utils::toString(*m_wireless_master_device));
    return false;
  }

  return true;
}

bool hiros::xsens_mtw::Wrapper::disableRadio()
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Disabling radio");

  if (!m_wireless_master_device->disableRadio()) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to disable radio: " << utils::toString(*m_wireless_master_device));
    return false;
  }

  return true;
}

void hiros::xsens_mtw::Wrapper::setupRos()
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Setting up ROS");

  m_reset_orientation_srv = m_nh.advertiseService("reset_orientation", &Wrapper::resetOrientation, this);

  if (m_wrapper_params.publish_mimu_array) {
    m_data_pub = m_nh.advertise<hiros_xsens_mtw_wrapper::MIMUArray>("data", m_ros_topic_queue_size);
  }
  else {
    for (auto& device : m_connected_devices) {
      if (m_wrapper_params.publish_imu) {
        m_imu_pubs.emplace(
          device.first,
          m_nh.advertise<sensor_msgs::Imu>(composeTopicPrefix(device.first) + "/imu/data", m_ros_topic_queue_size));
      }

      if (m_wrapper_params.publish_mag) {
        m_mag_pubs.emplace(device.first,
                           m_nh.advertise<sensor_msgs::MagneticField>(composeTopicPrefix(device.first) + "/imu/mag",
                                                                      m_ros_topic_queue_size));
      }

      if (m_wrapper_params.publish_euler) {
        m_euler_pubs.emplace(device.first,
                             m_nh.advertise<hiros_xsens_mtw_wrapper::Euler>(
                               composeTopicPrefix(device.first) + "/imu/euler", m_ros_topic_queue_size));
      }

      if (m_wrapper_params.publish_free_acceleration) {
        m_free_acceleration_pubs.emplace(
          device.first,
          m_nh.advertise<geometry_msgs::Vector3Stamped>(composeTopicPrefix(device.first) + "/filter/free_acceleration",
                                                        m_ros_topic_queue_size));
      }

      if (m_wrapper_params.publish_pressure) {
        m_pressure_pubs.emplace(device.first,
                                m_nh.advertise<sensor_msgs::FluidPressure>(
                                  composeTopicPrefix(device.first) + "/pressure", m_ros_topic_queue_size));
      }
    }
  }
}

void hiros::xsens_mtw::Wrapper::syncInitialPackets()
{
  double delta_t = 1 / static_cast<double>(m_update_rate);
  std::map<XsDeviceId, bool> initial_new_packets_received;
  std::map<XsDeviceId, long> initial_packet_ids;
  std::map<XsDeviceId, double> initial_timestamps;
  for (auto& device : m_connected_devices) {
    initial_new_packets_received.emplace(device.first, false);
    initial_packet_ids.emplace(device.first, -1);
    initial_timestamps.emplace(device.first, std::numeric_limits<double>::max());
  }
  bool got_all_initial_new_packets = false;

  while (!got_all_initial_new_packets) {
    for (auto& device : m_connected_devices) {
      if (!initial_new_packets_received.at(device.first) && m_mtw_callbacks.at(device.first)->newDataAvailable()) {
        std::shared_ptr<XsDataPacket> packet = m_mtw_callbacks.at(device.first)->getLatestPacket();

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

        m_mtw_callbacks.at(device.first)->deleteOldestPacket();
      }
    }

    got_all_initial_new_packets =
      std::all_of(initial_new_packets_received.begin(),
                  initial_new_packets_received.end(),
                  [](std::map<XsDeviceId, bool>::const_reference p) { return p.second == true; });
  }

  m_initial_packet_id =
    std::max_element(initial_packet_ids.begin(),
                     initial_packet_ids.end(),
                     [](std::map<XsDeviceId, long>::const_reference p1,
                        std::map<XsDeviceId, long>::const_reference p2) { return p1.second < p2.second; })
      ->second;

  for (auto& device : m_connected_devices) {
    initial_timestamps.at(device.first) += ((m_initial_packet_id - initial_packet_ids.at(device.first)) * delta_t);
  }

  m_initial_timestamp =
    ros::Time(std::min_element(initial_timestamps.begin(),
                               initial_timestamps.end(),
                               [](std::map<XsDeviceId, double>::const_reference t1,
                                  std::map<XsDeviceId, double>::const_reference t2) { return t1.second < t2.second; })
                ->second
              - m_mtw_params.fixed_latency);
}

bool hiros::xsens_mtw::Wrapper::resetInitialOrientation() const
{
  ROS_DEBUG_STREAM("Xsens Mtw Wrapper... Resetting initial orientation");

  bool success = true;

  for (auto& device : m_connected_devices) {
    success = device.second->resetOrientation(XRM_Heading) && success;
  }

  if (!success) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to reset initial orientation");
  }

  return success;
}

std::string hiros::xsens_mtw::Wrapper::getDeviceLabel(const XsDeviceId& t_id) const
{
  return (m_ids_to_labels.find(t_id) != m_ids_to_labels.end()) ? m_ids_to_labels.at(t_id)
                                                               : t_id.toString().toStdString();
}

XsDeviceId hiros::xsens_mtw::Wrapper::getDeviceId(const std::string t_label) const
{
  return (m_labels_to_ids.find(t_label) != m_labels_to_ids.end()) ? m_labels_to_ids.at(t_label)
                                                                  : utils::toXsDeviceId(t_label);
}

std::string hiros::xsens_mtw::Wrapper::composeTopicPrefix(const XsDeviceId& t_id) const
{
  return "/" + m_node_namespace + "/" + getDeviceLabel(t_id);
}

void hiros::xsens_mtw::Wrapper::publishPacket(std::shared_ptr<XsDataPacket> t_packet)
{
  if (m_wrapper_params.publish_imu && t_packet->containsOrientation() && t_packet->containsCalibratedGyroscopeData()
      && t_packet->containsCalibratedAcceleration()) {
    m_imu_pubs.at(t_packet->deviceId()).publish(getImuMsg(t_packet));
  }

  if (m_wrapper_params.publish_mag && t_packet->containsCalibratedMagneticField()) {
    m_mag_pubs.at(t_packet->deviceId()).publish(getMagMsg(t_packet));
  }

  if (m_wrapper_params.publish_euler && t_packet->containsOrientation()) {
    m_euler_pubs.at(t_packet->deviceId()).publish(getEulerMsg(t_packet));
  }

  if (m_wrapper_params.publish_free_acceleration && t_packet->containsFreeAcceleration()) {
    m_free_acceleration_pubs.at(t_packet->deviceId()).publish(getFreeAccelerationMsg(t_packet));
  }

  if (m_wrapper_params.publish_pressure && t_packet->containsPressure()) {
    m_pressure_pubs.at(t_packet->deviceId()).publish(getPressureMsg(t_packet));
  }

  if (m_wrapper_params.publish_tf && t_packet->containsOrientation()) {
    m_tf_broadcaster.sendTransform(getTf(t_packet));
  }
}

void hiros::xsens_mtw::Wrapper::publishFrame(const std::vector<std::shared_ptr<XsDataPacket>>& t_frame)
{
  if (m_wrapper_params.publish_mimu_array) {
    m_data_pub.publish(getMIMUArrayMsg(t_frame));

    if (m_wrapper_params.publish_tf) {
      for (auto& packet : t_frame) {
        if (packet->containsOrientation()) {
          m_tf_broadcaster.sendTransform(getTf(packet));
        }
      }
    }
  }
  else {
    for (auto& packet : t_frame) {
      publishPacket(packet);
    }
  }

  ros::spinOnce();
}

std_msgs::Header hiros::xsens_mtw::Wrapper::getHeader(std::shared_ptr<XsDataPacket> t_packet) const
{
  std_msgs::Header header;

  header.seq = static_cast<unsigned int>(t_packet->packetId());
  header.stamp = m_initial_timestamp
                 + ros::Duration((t_packet->packetId() - m_initial_packet_id) / static_cast<double>(m_update_rate));
  header.frame_id = m_wrapper_params.tf_prefix + getDeviceLabel(t_packet->deviceId());

  return header;
}

sensor_msgs::Imu hiros::xsens_mtw::Wrapper::getImuMsg(std::shared_ptr<XsDataPacket> t_packet) const
{
  sensor_msgs::Imu out_msg;
  out_msg.header = getHeader(t_packet);

  out_msg.orientation.x = t_packet->orientationQuaternion().x();
  out_msg.orientation.y = t_packet->orientationQuaternion().y();
  out_msg.orientation.z = t_packet->orientationQuaternion().z();
  out_msg.orientation.w = t_packet->orientationQuaternion().w();
  out_msg.orientation_covariance.front() = 0.0;

  out_msg.angular_velocity.x = t_packet->calibratedGyroscopeData().at(0);
  out_msg.angular_velocity.y = t_packet->calibratedGyroscopeData().at(1);
  out_msg.angular_velocity.z = t_packet->calibratedGyroscopeData().at(2);
  out_msg.angular_velocity_covariance.front() = 0.0;

  out_msg.linear_acceleration.x = t_packet->calibratedAcceleration().at(0);
  out_msg.linear_acceleration.y = t_packet->calibratedAcceleration().at(1);
  out_msg.linear_acceleration.z = t_packet->calibratedAcceleration().at(2);
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

hiros_xsens_mtw_wrapper::Euler hiros::xsens_mtw::Wrapper::getEulerMsg(std::shared_ptr<XsDataPacket> t_packet) const
{
  hiros_xsens_mtw_wrapper::Euler out_msg;
  out_msg.header = getHeader(t_packet);

  // roll = atan2(2 * (qw * qx + qy * qz), (1 - 2 * (pow(qx, 2) + pow(qy, 2))))
  out_msg.roll = t_packet->orientationEuler().roll();
  // pitch = asin(2 * (qw * qy - qz * qx))
  out_msg.pitch = t_packet->orientationEuler().pitch();
  // yaw = atan2(2 * (qw * qz + qx * qy), (1 - 2 * (pow(qy, 2) + pow(qz, 2))))
  out_msg.yaw = t_packet->orientationEuler().yaw();

  return out_msg;
}

geometry_msgs::Vector3Stamped
hiros::xsens_mtw::Wrapper::getFreeAccelerationMsg(std::shared_ptr<XsDataPacket> t_packet) const
{
  geometry_msgs::Vector3Stamped out_msg;
  out_msg.header = getHeader(t_packet);

  out_msg.vector.x = t_packet->freeAcceleration().at(0);
  out_msg.vector.y = t_packet->freeAcceleration().at(1);
  out_msg.vector.z = t_packet->freeAcceleration().at(2);

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

hiros_xsens_mtw_wrapper::MIMU hiros::xsens_mtw::Wrapper::getMIMUMsg(std::shared_ptr<XsDataPacket> t_packet) const
{
  hiros_xsens_mtw_wrapper::MIMU out_msg;

  if (m_wrapper_params.publish_imu && t_packet->containsOrientation() && t_packet->containsCalibratedGyroscopeData()
      && t_packet->containsCalibratedAcceleration()) {
    out_msg.imu = getImuMsg(t_packet);
  }

  if (m_wrapper_params.publish_mag && t_packet->containsCalibratedMagneticField()) {
    out_msg.mag = getMagMsg(t_packet);
  }

  if (m_wrapper_params.publish_euler && t_packet->containsOrientation()) {
    out_msg.euler = getEulerMsg(t_packet);
  }

  if (m_wrapper_params.publish_free_acceleration && t_packet->containsFreeAcceleration()) {
    out_msg.free_acceleration = getFreeAccelerationMsg(t_packet);
  }

  if (m_wrapper_params.publish_pressure && t_packet->containsPressure()) {
    out_msg.pressure = getPressureMsg(t_packet);
  }

  return out_msg;
}

hiros_xsens_mtw_wrapper::MIMUArray
hiros::xsens_mtw::Wrapper::getMIMUArrayMsg(const std::vector<std::shared_ptr<XsDataPacket>>& t_frame) const
{
  hiros_xsens_mtw_wrapper::MIMUArray out_msg;

  for (auto& packet : t_frame) {
    out_msg.mimus.push_back(getMIMUMsg(packet));
  }

  return out_msg;
}

geometry_msgs::TransformStamped hiros::xsens_mtw::Wrapper::getTf(std::shared_ptr<XsDataPacket> t_packet) const
{
  geometry_msgs::TransformStamped tf;
  tf.header = getHeader(t_packet);
  tf.header.frame_id = "world";
  tf.child_frame_id = m_wrapper_params.tf_prefix + getDeviceLabel(t_packet->deviceId());

  tf.transform.translation.x = 0.0;
  tf.transform.translation.y = 0.0;
  tf.transform.translation.z = 0.0;
  tf.transform.rotation.x = t_packet->orientationQuaternion().x();
  tf.transform.rotation.y = t_packet->orientationQuaternion().y();
  tf.transform.rotation.z = t_packet->orientationQuaternion().z();
  tf.transform.rotation.w = t_packet->orientationQuaternion().w();

  return tf;
}

bool hiros::xsens_mtw::Wrapper::resetOrientation(hiros_xsens_mtw_wrapper::ResetOrientation::Request& t_req,
                                                 hiros_xsens_mtw_wrapper::ResetOrientation::Response& t_res)
{
  bool success = true;

  if (t_req.sensors.empty()) {
    ROS_INFO_STREAM(BASH_MSG_GREEN << "Xsens Mtw Wrapper... Resetting orientation of all connected sensors"
                                   << BASH_MSG_RESET);
    for (auto& device : m_connected_devices) {
      success = device.second->resetOrientation(XRM_Alignment) && success;
    }
  }
  else {
    for (auto& sensor_label : t_req.sensors) {
      if (m_connected_devices.find(getDeviceId(sensor_label)) != m_connected_devices.end()) {
        ROS_INFO_STREAM(BASH_MSG_GREEN << "Xsens Mtw Wrapper... Resetting orientation of '" << sensor_label << "'"
                                       << BASH_MSG_RESET);
        success = m_connected_devices.at(getDeviceId(sensor_label))->resetOrientation(XRM_Alignment) && success;
      }
      else {
        ROS_WARN_STREAM("Xsens Mtw Wrapper... Cannot find '" << sensor_label << "'");
        success = false;
      }
    }
  }

  return success;
}
