#include "xsens_mtw/XsensMtw.h"

mtw::XsensMtw::XsensMtw(int t_desired_update_rate, int t_desired_radio_channel)
  : m_desired_update_rate(t_desired_update_rate)
  , m_desired_radio_channel(t_desired_radio_channel)
  , m_number_of_connected_mtws(0)
  , m_nh("~")
  , m_node_namespace(m_nh.getNamespace())
  , m_xsens_mtw_configured(false)
{}

void mtw::XsensMtw::start()
{
  ROS_INFO_STREAM("XsensMtw... Starting");

  // TODO ros::shutdown() is not enough to fully exit after error occurs
  if (!m_xsens_mtw_configured) {
    if (!configure()) {
      ROS_FATAL_STREAM("XsensMtw::configure() Error... Shutting down");
      ros::shutdown();
    }
  }

  if (!wait_mtw_connection()) {
    ROS_FATAL_STREAM("XsensMtw::wait_mtw_connection() No MTws connected... Shutting down");
    ros::shutdown();
  }

  if (!start_measurement()) {
    ROS_FATAL_STREAM("XsensMtw::start_measurement() Error... Shutting down");
    ros::shutdown();
  }

  if (!get_mtws_device_istances()) {
    ROS_FATAL_STREAM("XsensMtw::get_mtws_device_istances() Error... Shutting down");
    ros::shutdown();
  }

  attach_callback_handlers();

  setup_ros_topics();

  ROS_INFO_STREAM("XsensMtw... RUNNING");
}

void mtw::XsensMtw::run()
{
  std::cout << "\nMain loop. Press ctrl+c to quit\n" << std::endl;

  m_orientation_quaternion.resize(m_mtw_callbacks.size());
  m_raw_acc_data.resize(m_mtw_callbacks.size());
  m_raw_gyro_data.resize(m_mtw_callbacks.size());
  m_raw_mag_data.resize(m_mtw_callbacks.size());
  m_euler_data.resize(m_mtw_callbacks.size());

  std::cout << "Publishing data..." << std::endl;

  while (ros::ok()) {
    XsTime::msleep(0);

    bool newDataAvailable = false;
    for (size_t i = 0; i < m_mtw_callbacks.size(); ++i) {
      if (m_mtw_callbacks.at(i)->dataAvailable()) {
        newDataAvailable = true;

        XsDataPacket const* packet = m_mtw_callbacks.at(i)->getOldestPacket();

        m_orientation_quaternion.at(i) = packet->orientationQuaternion();
        m_raw_acc_data.at(i) = packet->calibratedAcceleration();
        m_raw_gyro_data.at(i) = packet->calibratedGyroscopeData();
        m_raw_mag_data.at(i) = packet->calibratedMagneticField();
        m_euler_data.at(i) = packet->orientationEuler();

        m_mtw_callbacks.at(i)->deleteOldestPacket();
      }
    }

    if (newDataAvailable) {
      for (size_t i = 0; i < m_mtw_callbacks.size(); ++i) {
        m_imu_msg.header.stamp = ros::Time::now();
        m_imu_msg.header.frame_id = m_mtw_callbacks.at(i)->device().deviceId().toString().toStdString();
        m_imu_msg.orientation.w = static_cast<double>(m_orientation_quaternion.at(i).w());
        m_imu_msg.orientation.x = static_cast<double>(m_orientation_quaternion.at(i).x());
        m_imu_msg.orientation.y = static_cast<double>(m_orientation_quaternion.at(i).y());
        m_imu_msg.orientation.z = static_cast<double>(m_orientation_quaternion.at(i).z());
        m_imu_msg.angular_velocity.x = static_cast<double>(m_raw_gyro_data.at(i).at(0));
        m_imu_msg.angular_velocity.y = static_cast<double>(m_raw_gyro_data.at(i).at(1));
        m_imu_msg.angular_velocity.z = static_cast<double>(m_raw_gyro_data.at(i).at(2));
        m_imu_msg.linear_acceleration.x = static_cast<double>(m_raw_acc_data.at(i).at(0));
        m_imu_msg.linear_acceleration.y = static_cast<double>(m_raw_acc_data.at(i).at(1));
        m_imu_msg.linear_acceleration.z = static_cast<double>(m_raw_acc_data.at(i).at(2));

        m_mag_msg.header = m_imu_msg.header;
        m_mag_msg.magnetic_field.x = static_cast<double>(m_raw_mag_data.at(i).at(0));
        m_mag_msg.magnetic_field.y = static_cast<double>(m_raw_mag_data.at(i).at(1));
        m_mag_msg.magnetic_field.z = static_cast<double>(m_raw_mag_data.at(i).at(2));

        m_imu_pub.publish(m_imu_msg);
        m_mag_pub.publish(m_mag_msg);

        ros::spinOnce();
      }
    }
  }

  stop();
}

void mtw::XsensMtw::stop()
{
  ROS_INFO_STREAM("XsensMtw... Stopping");

  if (!set_config_mode()) {
    ROS_FATAL_STREAM("XsensMtw::set_config_mode() Error... Shutting down");
    ros::shutdown();
  }

  if (!disable_radio()) {
    ROS_FATAL_STREAM("XsensMtw::disable_radio() Error... Shutting down");
    ros::shutdown();
  }

  std::cout << "Closing XsControl..." << std::endl;
  m_control->close();

  std::cout << "Deleting mtw callbacks..." << std::endl;
  for (std::vector<MtwCallback*>::iterator i = m_mtw_callbacks.begin(); i != m_mtw_callbacks.end(); ++i) {
    delete (*i);
  }

  std::cout << "Successful exit" << std::endl;
}

bool mtw::XsensMtw::configure()
{
  if (!construct_control()) {
    return false;
  }

  if (!find_wireless_master()) {
    return false;
  }

  if (!open_port()) {
    return false;
  }

  if (!get_xsdevice_instance()) {
    return false;
  }

  if (!set_config_mode()) {
    return false;
  }

  attach_callback_handler();

  if (!get_closest_update_rate()) {
    return false;
  }

  if (!set_update_rate()) {
    return false;
  }

  if (!set_radio_channel()) {
    return false;
  }

  m_xsens_mtw_configured = true;
  return m_xsens_mtw_configured;
}

bool mtw::XsensMtw::construct_control()
{
  std::cout << "Constructing XsControl..." << std::endl;

  m_control = XsControl::construct();

  if (m_control == nullptr) {
    std::cerr << "Failed to construct XsControl instance" << std::endl;
    return false;
  }

  return true;
}

bool mtw::XsensMtw::find_wireless_master()
{
  std::cout << "Scanning ports..." << std::endl;
  m_detected_devices = XsScanner::scanPorts();

  std::cout << "Finding wireless master..." << std::endl;
  m_wireless_master_port = m_detected_devices.begin();
  while (m_wireless_master_port != m_detected_devices.end() && !m_wireless_master_port->deviceId().isWirelessMaster()) {
    ++m_wireless_master_port;
  }

  if (m_wireless_master_port == m_detected_devices.end()) {
    std::cerr << "No wireless masters found" << std::endl;
    return false;
  }

  std::cout << "Wireless master found @ " << *m_wireless_master_port << std::endl;
  return true;
}

bool mtw::XsensMtw::open_port()
{
  std::cout << "Opening port..." << std::endl;
  if (!m_control->openPort(m_wireless_master_port->portName().toStdString(), m_wireless_master_port->baudrate())) {
    std::cerr << "Failed to open port " << *m_wireless_master_port << std::endl;
    return false;
  }

  return true;
}

bool mtw::XsensMtw::get_xsdevice_instance()
{
  std::cout << "Getting XsDevice instance for wireless master..." << std::endl;
  m_wireless_master_device = m_control->device(m_wireless_master_port->deviceId());
  if (m_wireless_master_device == nullptr) {
    std::cerr << "Failed to construct XsDevice instance: " << *m_wireless_master_port << std::endl;
    return false;
  }

  mtw::utils::operator<<(std::cout << "XsDevice instance created @ ", *m_wireless_master_device) << std::endl;
  return true;
}

bool mtw::XsensMtw::set_config_mode()
{
  std::cout << "Setting config mode..." << std::endl;
  if (!m_wireless_master_device->gotoConfig()) {
    mtw::utils::operator<<(std::cerr << "Failed to goto config mode: ", *m_wireless_master_device) << std::endl;
    return false;
  }

  return true;
}

void mtw::XsensMtw::attach_callback_handler()
{
  std::cout << "Attaching callback handler..." << std::endl;
  m_wireless_master_device->addCallbackHandler(&m_wireless_master_callback);
}

bool mtw::XsensMtw::get_closest_update_rate()
{
  std::cout << "Getting the list of the supported update rates..." << std::endl;
  const XsIntArray supportedUpdateRates = m_wireless_master_device->supportedUpdateRates();

  std::cout << "Supported update rates: ";
  for (XsIntArray::const_iterator itUpRate = supportedUpdateRates.begin(); itUpRate != supportedUpdateRates.end();
       ++itUpRate) {
    std::cout << *itUpRate << " ";
  }
  std::cout << std::endl;

  if (supportedUpdateRates.empty()) {
    std::cerr << "Failed to get supported update rates" << std::endl;
    return false;
  }

  if (supportedUpdateRates.size() == 1) {
    return supportedUpdateRates.at(0);
  }

  int uRateDist = -1;
  int closestUpdateRate = -1;

  for (XsIntArray::const_iterator itUpRate = supportedUpdateRates.begin(); itUpRate != supportedUpdateRates.end();
       ++itUpRate) {
    const int currDist = std::abs(*itUpRate - m_desired_update_rate);

    if ((uRateDist == -1) || (currDist < uRateDist)) {
      uRateDist = currDist;
      closestUpdateRate = *itUpRate;
    }
  }

  m_update_rate = closestUpdateRate;

  return true;
}

bool mtw::XsensMtw::set_update_rate()
{
  std::cout << "Setting update rate to " << m_update_rate << " Hz..." << std::endl;
  if (!m_wireless_master_device->setUpdateRate(m_update_rate)) {
    mtw::utils::operator<<(std::cerr << "Failed to set update rate: ", *m_wireless_master_device) << std::endl;
    return false;
  }

  return true;
}

bool mtw::XsensMtw::set_radio_channel()
{
  std::cout << "Disabling radio channel if previously enabled..." << std::endl;
  if (m_wireless_master_device->isRadioEnabled()) {
    if (!m_wireless_master_device->disableRadio()) {
      mtw::utils::operator<<(std::cerr << "Failed to disable radio channel: ", *m_wireless_master_device) << std::endl;
      return false;
    }
  }

  std::cout << "Setting radio channel to " << m_desired_radio_channel << " and enabling radio..." << std::endl;
  if (!m_wireless_master_device->enableRadio(m_desired_radio_channel)) {
    mtw::utils::operator<<(std::cerr << "Failed to set radio channel: ", *m_wireless_master_device) << std::endl;
    return false;
  }

  return true;
}

bool mtw::XsensMtw::wait_mtw_connection() // TODO: fix dirty waiting
{
  std::cout << "Waiting for MTW to wirelessly connect...\n" << std::endl;

  m_number_of_connected_mtws = m_wireless_master_callback.getWirelessMTWs().size();

  int cnt1 = 50;
  int cnt2 = 50;
  do {
    --cnt1;
    XsTime::msleep(100);

    while (cnt2 > 0) {
      --cnt2;
      size_t nextCount = m_wireless_master_callback.getWirelessMTWs().size();
      if (nextCount != m_number_of_connected_mtws) {
        std::cout << "Number of connected MTWs: " << nextCount << std::endl;
        m_number_of_connected_mtws = nextCount;
      }
      else {
        break;
      }
    }
  } while (cnt1 > 0);

  return (m_number_of_connected_mtws != 0);
}

bool mtw::XsensMtw::start_measurement()
{
  std::cout << "Starting measurement..." << std::endl;
  if (!m_wireless_master_device->gotoMeasurement()) {
    mtw::utils::operator<<(std::cerr << "Failed to goto measurement mode: ", *m_wireless_master_device) << std::endl;
    return false;
  }

  return true;
}

bool mtw::XsensMtw::get_mtws_device_istances()
{
  std::cout << "Getting XsDevice instances for all MTWs..." << std::endl;
  m_all_device_ids = m_control->deviceIds();

  for (XsDeviceIdArray::const_iterator i = m_all_device_ids.begin(); i != m_all_device_ids.end(); ++i) {
    if (i->isMtw()) {
      m_mtw_device_ids.push_back(*i);
    }
  }

  for (XsDeviceIdArray::const_iterator i = m_mtw_device_ids.begin(); i != m_mtw_device_ids.end(); ++i) {
    XsDevicePtr mtwDevice = m_control->device(*i);
    if (mtwDevice != nullptr) {
      m_mtw_devices.push_back(mtwDevice);
    }
    else {
      std::cerr << "Failed to create an MTW XsDevice instance" << std::endl;
      return false;
    }
  }

  return true;
}

void mtw::XsensMtw::attach_callback_handlers()
{
  std::cout << "Attaching callback handlers to MTWs..." << std::endl;
  m_mtw_callbacks.resize(m_mtw_devices.size());

  for (size_t i = 0; i < m_mtw_devices.size(); ++i) {
    m_mtw_callbacks.at(i) = new MtwCallback(static_cast<int>(i), m_mtw_devices.at(i));
    m_mtw_devices.at(i)->addCallbackHandler(m_mtw_callbacks.at(i));
  }
}

void mtw::XsensMtw::setup_ros_topics() // TODO: fix for multiple imus
{
  m_imu_pub = m_nh.advertise<sensor_msgs::Imu>("/imu/data_raw", 1000);
  m_mag_pub = m_nh.advertise<sensor_msgs::MagneticField>("/imu/mag", 1000);
}

bool mtw::XsensMtw::disable_radio()
{
  std::cout << "Disabling radio... " << std::endl;

  if (!m_wireless_master_device->disableRadio()) {
    mtw::utils::operator<<(std::cerr << "Failed to disable radio: ", *m_wireless_master_device) << std::endl;
    return false;
  }

  return true;
}
