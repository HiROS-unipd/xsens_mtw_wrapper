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

  if (!m_xsens_mtw_configured) {
    if (!configure()) {
      ros::shutdown();
    }
  }

  if (!waitMtwConnection()) {
    ros::shutdown();
  }

  if (!getMtwsDeviceIstances()) {
    ros::shutdown();
  }

  setupRosTopics();
  initializeVectors();

  attachCallbackHandlers();

  if (!startMeasurement()) {
    ros::shutdown();
  }

  ROS_INFO_STREAM("Xsens Mtw Wrapper... RUNNING");
}

void hiros::xsens_mtw::Wrapper::run()
{
  ROS_INFO_STREAM("Xsens Mtw Wrapper... Publishing data. Press ctrl+c to quit");

  while (ros::ok() && !s_request_shutdown) {
    for (size_t i = 0; i < m_number_of_connected_mtws; ++i) {
      if (m_mtw_callbacks.at(i)->dataAvailable()) {
        const XsDataPacket* packet = m_mtw_callbacks.at(i)->getOldestPacket();

        ros::Time sample_time = computeSampleTime(*packet);

        if (m_wrapper_params.publish_imu) {
          m_imu_pub.at(i).publish(getImuMsg(sample_time, *packet));
        }

        if (m_wrapper_params.publish_acceleration) {
          m_acceleration_pub.at(i).publish(getAccelerationMsg(sample_time, *packet));
        }

        if (m_wrapper_params.publish_angular_velocity) {
          m_angular_velocity_pub.at(i).publish(getAngularVelocityMsg(sample_time, *packet));
        }

        if (m_wrapper_params.publish_mag) {
          m_mag_pub.at(i).publish(getMagMsg(sample_time, *packet));
        }

        if (m_wrapper_params.publish_euler) {
          m_euler_pub.at(i).publish(getEulerMsg(sample_time, *packet));
        }

        if (m_wrapper_params.publish_quaternion) {
          m_quaternion_pub.at(i).publish(getQuaternionMsg(sample_time, *packet));
        }

        if (m_wrapper_params.publish_free_acceleration) {
          m_free_acceleration_pub.at(i).publish(getFreeAccelerationMsg(sample_time, *packet));
        }

        if (m_wrapper_params.publish_pressure) {
          m_pressure_pub.at(i).publish(getPressureMsg(sample_time, *packet));
        }

        if (m_wrapper_params.publish_tf) {
          m_tf_broadcaster.sendTransform(getTf(sample_time, *packet));
        }

        m_mtw_callbacks.at(i)->deleteOldestPacket();

        ros::spinOnce();
      }
    }
  }

  stop();
}

void hiros::xsens_mtw::Wrapper::stop()
{
  ROS_INFO_STREAM("Xsens Mtw Wrapper... Stopping");

  if (!setConfigMode()) {
    ros::shutdown();
  }

  if (!disableRadio()) {
    ros::shutdown();
  }

  ROS_INFO_STREAM("Xsens Mtw Wrapper... Closing XsControl");
  m_control->close();

  ROS_INFO_STREAM("Xsens Mtw Wrapper... Deleting MTW callbacks");
  for (auto& mtw_callback : m_mtw_callbacks) {
    delete (mtw_callback);
  }

  ROS_INFO_STREAM("Xsens Mtw Wrapper... Shutting down ROS publishers");

  if (m_wrapper_params.publish_imu) {
    for (auto& pub : m_imu_pub) {
      if (pub) {
        pub.shutdown();
      }
    }
  }

  if (m_wrapper_params.publish_acceleration) {
    for (auto& pub : m_acceleration_pub) {
      if (pub) {
        pub.shutdown();
      }
    }
  }

  if (m_wrapper_params.publish_angular_velocity) {
    for (auto& pub : m_angular_velocity_pub) {
      if (pub) {
        pub.shutdown();
      }
    }
  }

  if (m_wrapper_params.publish_mag) {
    for (auto& pub : m_mag_pub) {
      if (pub) {
        pub.shutdown();
      }
    }
  }

  if (m_wrapper_params.publish_euler) {
    for (auto& pub : m_euler_pub) {
      if (pub) {
        pub.shutdown();
      }
    }
  }

  if (m_wrapper_params.publish_quaternion) {
    for (auto& pub : m_quaternion_pub) {
      if (pub) {
        pub.shutdown();
      }
    }
  }

  if (m_wrapper_params.publish_free_acceleration) {
    for (auto& pub : m_free_acceleration_pub) {
      if (pub) {
        pub.shutdown();
      }
    }
  }

  if (m_wrapper_params.publish_pressure) {
    for (auto& pub : m_pressure_pub) {
      if (pub) {
        pub.shutdown();
      }
    }
  }

  ROS_INFO_STREAM("Xsens Mtw Wrapper... STOPPED");

  ros::shutdown();
}

bool hiros::xsens_mtw::Wrapper::configure()
{
  ROS_INFO_STREAM("Xsens Mtw Wrapper... Configuring");

  if (m_xsens_mtw_configured) {
    m_xsens_mtw_configured = false;
    stop();
  }

  configureWrapper();
  bool success = configureXsensMtw();

  m_xsens_mtw_configured = true;

  if (success) {
    ROS_INFO_STREAM("Xsens Mtw Wrapper... CONFIGURED");
  }

  return success;
}

void hiros::xsens_mtw::Wrapper::configureWrapper()
{
  ROS_INFO_STREAM("Xsens Mtw Wrapper... Configuring Wrapper");

  m_nh.getParam("publish_imu", m_wrapper_params.publish_imu);
  m_nh.getParam("publish_acceleration", m_wrapper_params.publish_acceleration);
  m_nh.getParam("publish_angular_velocity", m_wrapper_params.publish_angular_velocity);
  m_nh.getParam("publish_mag", m_wrapper_params.publish_mag);
  m_nh.getParam("publish_euler", m_wrapper_params.publish_euler);
  m_nh.getParam("publish_quaternion", m_wrapper_params.publish_quaternion);
  m_nh.getParam("publish_free_acceleration", m_wrapper_params.publish_free_acceleration);
  m_nh.getParam("publish_pressure", m_wrapper_params.publish_pressure);
  m_nh.getParam("publish_tf", m_wrapper_params.publish_tf);

  bool nothing_to_publish =
    (!m_wrapper_params.publish_imu && !m_wrapper_params.publish_acceleration
     && !m_wrapper_params.publish_angular_velocity && !m_wrapper_params.publish_mag && !m_wrapper_params.publish_euler
     && !m_wrapper_params.publish_quaternion && !m_wrapper_params.publish_free_acceleration
     && !m_wrapper_params.publish_pressure && !m_wrapper_params.publish_tf);

  if (nothing_to_publish) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Nothing to publish. Closing");
    ros::shutdown();
  }
}

bool hiros::xsens_mtw::Wrapper::configureXsensMtw()
{
  ROS_INFO_STREAM("Xsens Mtw Wrapper... Configuring Xsens Mtw");

  m_nh.getParam("desired_update_rate", m_mtw_params.desired_update_rate);
  m_nh.getParam("desired_radio_channel", m_mtw_params.desired_radio_channel);

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
    return false;
  }

  return true;
}

bool hiros::xsens_mtw::Wrapper::waitMtwConnection()
{
  ROS_INFO_STREAM("Xsens Mtw Wrapper... Waiting for MTW to wirelessly connect");

  for (unsigned int cnt = 0; cnt < m_connection_attempts; ++cnt) {
    XsTime::msleep(100);

    size_t next_count = m_wireless_master_callback.getWirelessMTWs().size();
    if (next_count != m_number_of_connected_mtws) {
      m_number_of_connected_mtws = next_count;
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
  ROS_INFO_STREAM("Xsens Mtw Wrapper... Getting XsDevice instances for all MTWs");

  for (auto& xs_device_id : m_control->deviceIds()) {
    if (xs_device_id.isMtw()) {
      m_mtw_device_ids.push_back(xs_device_id);
    }
  }

  for (auto& mtw_device_id : m_mtw_device_ids) {
    XsDevicePtr mtw_device = m_control->device(mtw_device_id);
    if (mtw_device != nullptr) {
      m_mtw_devices.push_back(mtw_device);
    }
    else {
      ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to create an MTW XsDevice instance");
      return false;
    }
  }

  return true;
}

void hiros::xsens_mtw::Wrapper::setupRosTopics()
{
  ROS_INFO_STREAM("Xsens Mtw Wrapper... Setting up ROS topics");

  for (auto& device_id : m_mtw_device_ids) {
    if (m_wrapper_params.publish_imu) {
      m_imu_pub.push_back(m_nh.advertise<sensor_msgs::Imu>(composeTopicPrefix(device_id) + "/imu/data", 1));
    }

    if (m_wrapper_params.publish_acceleration) {
      m_acceleration_pub.push_back(
        m_nh.advertise<geometry_msgs::Vector3Stamped>(composeTopicPrefix(device_id) + "/imu/acceleration", 1));
    }

    if (m_wrapper_params.publish_angular_velocity) {
      m_angular_velocity_pub.push_back(
        m_nh.advertise<geometry_msgs::Vector3Stamped>(composeTopicPrefix(device_id) + "/imu/angular_velocity", 1));
    }

    if (m_wrapper_params.publish_mag) {
      m_mag_pub.push_back(m_nh.advertise<sensor_msgs::MagneticField>(composeTopicPrefix(device_id) + "/imu/mag", 1));
    }

    if (m_wrapper_params.publish_euler) {
      m_euler_pub.push_back(
        m_nh.advertise<hiros_xsens_mtw_wrapper::Euler>(composeTopicPrefix(device_id) + "/imu/euler", 1));
    }

    if (m_wrapper_params.publish_quaternion) {
      m_quaternion_pub.push_back(
        m_nh.advertise<geometry_msgs::QuaternionStamped>(composeTopicPrefix(device_id) + "/filter/quaternion", 1));
    }

    if (m_wrapper_params.publish_free_acceleration) {
      m_free_acceleration_pub.push_back(
        m_nh.advertise<geometry_msgs::Vector3Stamped>(composeTopicPrefix(device_id) + "/filter/free_acceleration", 1));
    }

    if (m_wrapper_params.publish_pressure) {
      m_pressure_pub.push_back(
        m_nh.advertise<sensor_msgs::FluidPressure>(composeTopicPrefix(device_id) + "/pressure", 1));
    }
  }
}

void hiros::xsens_mtw::Wrapper::initializeVectors()
{
  m_prev_packet_time_of_arrival.resize(m_number_of_connected_mtws, 0);
  m_prev_packet_sample_time.resize(m_number_of_connected_mtws, ros::Time(0));
}

void hiros::xsens_mtw::Wrapper::attachCallbackHandlers()
{
  ROS_INFO_STREAM("Xsens Mtw Wrapper... Attaching callback handlers to MTWs");

  m_mtw_callbacks.resize(m_mtw_devices.size());
  for (size_t i = 0; i < m_mtw_devices.size(); ++i) {
    m_mtw_callbacks.at(i) = new MtwCallback(static_cast<int>(i), m_mtw_devices.at(i));
    m_mtw_devices.at(i)->addCallbackHandler(m_mtw_callbacks.at(i));
  }
}

bool hiros::xsens_mtw::Wrapper::startMeasurement()
{
  ROS_INFO_STREAM("Xsens Mtw Wrapper... Starting measurement");

  if (!m_wireless_master_device->gotoMeasurement()) {
    ROS_FATAL_STREAM(
      "Xsens Mtw Wrapper... Failed to go to measurement mode: " << utils::toString(*m_wireless_master_device));
    return false;
  }

  return true;
}

ros::Time hiros::xsens_mtw::Wrapper::computeSampleTime(const XsDataPacket& t_packet)
{
  ros::Time sample_time;

  size_t mtw_device_index = static_cast<size_t>(std::distance(
    m_mtw_device_ids.begin(), std::find(m_mtw_device_ids.begin(), m_mtw_device_ids.end(), t_packet.deviceId())));

  double delta_time_of_arrival =
    (t_packet.timeOfArrival() - m_prev_packet_time_of_arrival.at(mtw_device_index)).secTime();

  // same timeOfArrival
  if (delta_time_of_arrival < (0.5 / m_update_rate)) {
    sample_time = m_prev_packet_sample_time.at(mtw_device_index) + ros::Duration(1.0 / m_update_rate);
  }
  // new timeOfArrival
  else {
    sample_time = (t_packet.timeOfArrival().secTime() > m_prev_packet_sample_time.at(mtw_device_index).toSec())
                    ? ros::Time(t_packet.timeOfArrival().secTime())
                    : ros::Time(m_prev_packet_sample_time.at(mtw_device_index).toSec() + m_sample_time_epsilon);
  }

  m_prev_packet_time_of_arrival.at(mtw_device_index) = t_packet.timeOfArrival();
  m_prev_packet_sample_time.at(mtw_device_index) = sample_time;

  return sample_time;
}

bool hiros::xsens_mtw::Wrapper::constructControl()
{
  ROS_INFO_STREAM("Xsens Mtw Wrapper... Constructing XsControl");

  m_control = XsControl::construct();

  if (m_control == nullptr) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to construct XsControl instance");
    return false;
  }

  return true;
}

bool hiros::xsens_mtw::Wrapper::findWirelessMaster()
{
  ROS_INFO_STREAM("Xsens Mtw Wrapper... Scanning ports");

  m_detected_devices = XsScanner::scanPorts();

  ROS_INFO_STREAM("Xsens Mtw Wrapper... Finding wireless master");

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
  ROS_INFO_STREAM("Xsens Mtw Wrapper... Opening port");

  if (!m_control->openPort(m_wireless_master_port->portName().toStdString(), m_wireless_master_port->baudrate())) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to open port " << *m_wireless_master_port);
    return false;
  }

  return true;
}

bool hiros::xsens_mtw::Wrapper::getXsdeviceInstance()
{
  ROS_INFO_STREAM("Xsens Mtw Wrapper... Getting XsDevice instance for wireless master");

  m_wireless_master_device = m_control->device(m_wireless_master_port->deviceId());
  if (m_wireless_master_device == nullptr) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to construct XsDevice instance: " << *m_wireless_master_port);
    return false;
  }

  ROS_INFO_STREAM("Xsens Mtw Wrapper... XsDevice instance created @ " << utils::toString(*m_wireless_master_device));
  return true;
}

bool hiros::xsens_mtw::Wrapper::setConfigMode()
{
  ROS_INFO_STREAM("Xsens Mtw Wrapper... Setting config mode");

  if (!m_wireless_master_device->gotoConfig()) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to go to config mode: " + utils::toString(*m_wireless_master_device));
    return false;
  }

  return true;
}

void hiros::xsens_mtw::Wrapper::attachCallbackHandler()
{
  ROS_INFO_STREAM("Xsens Mtw Wrapper... Attaching callback handler");

  m_wireless_master_device->addCallbackHandler(&m_wireless_master_callback);
}

bool hiros::xsens_mtw::Wrapper::getClosestUpdateRate()
{
  ROS_INFO_STREAM("Xsens Mtw Wrapper... Getting the list of the supported update rates");

  const XsIntArray supported_update_rates = m_wireless_master_device->supportedUpdateRates();

  std::string info_str = "Xsens Mtw Wrapper... Supported update rates:";
  for (auto& up_rate : supported_update_rates) {
    info_str += " " + std::to_string(up_rate);
  }
  ROS_INFO_STREAM(info_str);

  if (supported_update_rates.empty()) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to get supported update rates");
    return false;
  }

  if (supported_update_rates.size() == 1) {
    return supported_update_rates.at(0);
  }

  int u_rate_dist = -1;
  int closest_update_rate = -1;

  for (auto& up_rate : supported_update_rates) {
    const int curr_dist = std::abs(up_rate - m_mtw_params.desired_update_rate);

    if ((u_rate_dist == -1) || (curr_dist < u_rate_dist)) {
      u_rate_dist = curr_dist;
      closest_update_rate = up_rate;
    }
  }

  m_update_rate = closest_update_rate;
  m_sample_time_epsilon = (0.5 / m_update_rate);

  return true;
}

bool hiros::xsens_mtw::Wrapper::setUpdateRate()
{
  ROS_INFO_STREAM("Xsens Mtw Wrapper... Setting update rate to " << m_update_rate << " Hz");

  if (!m_wireless_master_device->setUpdateRate(m_update_rate)) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to set update rate: " << utils::toString(*m_wireless_master_device));
    return false;
  }

  return true;
}

bool hiros::xsens_mtw::Wrapper::setRadioChannel()
{
  if (m_wireless_master_device->isRadioEnabled()) {
    ROS_INFO_STREAM("Xsens Mtw Wrapper... Disabling previously enabled radio channel");
    if (!disableRadio()) {
      return false;
    }
  }

  ROS_INFO_STREAM("Xsens Mtw Wrapper... Setting radio channel to " << m_mtw_params.desired_radio_channel
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
  ROS_INFO_STREAM("Xsens Mtw Wrapper... Disabling radio");

  if (!m_wireless_master_device->disableRadio()) {
    ROS_FATAL_STREAM("Xsens Mtw Wrapper... Failed to disable radio: " << utils::toString(*m_wireless_master_device));
    return false;
  }

  return true;
}

sensor_msgs::Imu hiros::xsens_mtw::Wrapper::getImuMsg(const ros::Time& t_sample_time,
                                                      const XsDataPacket& t_packet) const
{
  sensor_msgs::Imu out_msg;
  out_msg.header.stamp = t_sample_time;
  out_msg.header.frame_id = t_packet.deviceId().toString().toStdString();

  if (t_packet.containsOrientation()) {
    out_msg.orientation.x = t_packet.orientationQuaternion().x();
    out_msg.orientation.y = t_packet.orientationQuaternion().y();
    out_msg.orientation.z = t_packet.orientationQuaternion().z();
    out_msg.orientation.w = t_packet.orientationQuaternion().w();
    out_msg.orientation_covariance.front() = -1.0;
  }

  if (t_packet.containsCalibratedGyroscopeData()) {
    out_msg.angular_velocity.x = t_packet.calibratedGyroscopeData().at(0);
    out_msg.angular_velocity.y = t_packet.calibratedGyroscopeData().at(1);
    out_msg.angular_velocity.z = t_packet.calibratedGyroscopeData().at(2);
    out_msg.angular_velocity_covariance.front() = -1.0;
  }

  if (t_packet.containsCalibratedAcceleration()) {
    out_msg.linear_acceleration.x = t_packet.calibratedAcceleration().at(0);
    out_msg.linear_acceleration.y = t_packet.calibratedAcceleration().at(1);
    out_msg.linear_acceleration.z = t_packet.calibratedAcceleration().at(2);
    out_msg.linear_acceleration_covariance.front() = -1.0;
  }

  return out_msg;
}

geometry_msgs::Vector3Stamped hiros::xsens_mtw::Wrapper::getAccelerationMsg(const ros::Time& t_sample_time,
                                                                            const XsDataPacket& t_packet) const
{
  geometry_msgs::Vector3Stamped out_msg;
  out_msg.header.stamp = t_sample_time;
  out_msg.header.frame_id = t_packet.deviceId().toString().toStdString();

  if (t_packet.containsCalibratedAcceleration()) {
    out_msg.vector.x = t_packet.calibratedAcceleration().at(0);
    out_msg.vector.y = t_packet.calibratedAcceleration().at(1);
    out_msg.vector.z = t_packet.calibratedAcceleration().at(2);
  }

  return out_msg;
}

geometry_msgs::Vector3Stamped hiros::xsens_mtw::Wrapper::getAngularVelocityMsg(const ros::Time& t_sample_time,
                                                                               const XsDataPacket& t_packet) const
{
  geometry_msgs::Vector3Stamped out_msg;
  out_msg.header.stamp = t_sample_time;
  out_msg.header.frame_id = t_packet.deviceId().toString().toStdString();

  if (t_packet.containsCalibratedGyroscopeData()) {
    out_msg.vector.x = t_packet.calibratedGyroscopeData().at(0);
    out_msg.vector.y = t_packet.calibratedGyroscopeData().at(1);
    out_msg.vector.z = t_packet.calibratedGyroscopeData().at(2);
  }

  return out_msg;
}

sensor_msgs::MagneticField hiros::xsens_mtw::Wrapper::getMagMsg(const ros::Time& t_sample_time,
                                                                const XsDataPacket& t_packet) const
{
  sensor_msgs::MagneticField out_msg;
  out_msg.header.stamp = t_sample_time;
  out_msg.header.frame_id = t_packet.deviceId().toString().toStdString();

  if (t_packet.containsCalibratedMagneticField()) {
    out_msg.magnetic_field.x = t_packet.calibratedMagneticField().at(0);
    out_msg.magnetic_field.y = t_packet.calibratedMagneticField().at(1);
    out_msg.magnetic_field.z = t_packet.calibratedMagneticField().at(2);
    out_msg.magnetic_field_covariance.front() = 0.0;
  }

  return out_msg;
}

hiros_xsens_mtw_wrapper::Euler hiros::xsens_mtw::Wrapper::getEulerMsg(const ros::Time& t_sample_time,
                                                                      const XsDataPacket& t_packet) const
{
  hiros_xsens_mtw_wrapper::Euler out_msg;
  out_msg.header.stamp = t_sample_time;
  out_msg.header.frame_id = t_packet.deviceId().toString().toStdString();

  if (t_packet.containsOrientation()) {
    // roll = atan2(2 * (qw * qx + qy * qz), (1 - 2 * (pow(qx, 2) + pow(qy, 2))))
    out_msg.roll = t_packet.orientationEuler().roll();
    // pitch = asin(2 * (qw * qy - qz * qx))
    out_msg.pitch = t_packet.orientationEuler().pitch();
    // yaw = atan2(2 * (qw * qz + qx * qy), (1 - 2 * (pow(qy, 2) + pow(qz, 2))))
    out_msg.yaw = t_packet.orientationEuler().yaw();
  }

  return out_msg;
}

geometry_msgs::QuaternionStamped hiros::xsens_mtw::Wrapper::getQuaternionMsg(const ros::Time& t_sample_time,
                                                                             const XsDataPacket& t_packet) const
{
  geometry_msgs::QuaternionStamped out_msg;
  out_msg.header.stamp = t_sample_time;
  out_msg.header.frame_id = t_packet.deviceId().toString().toStdString();

  if (t_packet.containsOrientation()) {
    out_msg.quaternion.x = t_packet.orientationQuaternion().x();
    out_msg.quaternion.y = t_packet.orientationQuaternion().y();
    out_msg.quaternion.z = t_packet.orientationQuaternion().z();
    out_msg.quaternion.w = t_packet.orientationQuaternion().w();
  }

  return out_msg;
}

geometry_msgs::Vector3Stamped hiros::xsens_mtw::Wrapper::getFreeAccelerationMsg(const ros::Time& t_sample_time,
                                                                                const XsDataPacket& t_packet) const
{
  geometry_msgs::Vector3Stamped out_msg;
  out_msg.header.stamp = t_sample_time;
  out_msg.header.frame_id = t_packet.deviceId().toString().toStdString();

  if (t_packet.containsFreeAcceleration()) {
    out_msg.vector.x = t_packet.freeAcceleration().at(0);
    out_msg.vector.y = t_packet.freeAcceleration().at(1);
    out_msg.vector.z = t_packet.freeAcceleration().at(2);
  }

  return out_msg;
}

sensor_msgs::FluidPressure hiros::xsens_mtw::Wrapper::getPressureMsg(const ros::Time& t_sample_time,
                                                                     const XsDataPacket& t_packet) const
{
  sensor_msgs::FluidPressure out_msg;
  out_msg.header.stamp = t_sample_time;
  out_msg.header.frame_id = t_packet.deviceId().toString().toStdString();

  if (t_packet.containsPressure()) {
    out_msg.fluid_pressure = t_packet.pressure().m_pressure;
    out_msg.variance = 0.0;
  }

  return out_msg;
}

geometry_msgs::TransformStamped hiros::xsens_mtw::Wrapper::getTf(const ros::Time& t_sample_time,
                                                                 const XsDataPacket& t_packet) const
{
  geometry_msgs::TransformStamped tf;

  tf.header.stamp = t_sample_time;
  tf.header.frame_id = "world";
  tf.child_frame_id = t_packet.deviceId().toString().toStdString();

  if (t_packet.containsOrientation()) {
    tf.transform.translation.x = 0.0;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = t_packet.orientationQuaternion().x();
    tf.transform.rotation.y = t_packet.orientationQuaternion().y();
    tf.transform.rotation.z = t_packet.orientationQuaternion().z();
    tf.transform.rotation.w = t_packet.orientationQuaternion().w();
  }

  return tf;
}
