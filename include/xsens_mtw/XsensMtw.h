#ifndef xsens_mtw_driver_XsensMtw_h
#define xsens_mtw_driver_XsensMtw_h

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"

#include "xsens_mtw/MtwCallback.h"
#include "xsens_mtw/WirelessMasterCallback.h"

namespace mtw {

  class XsensMtw
  {
  public:
    XsensMtw(int t_desired_update_rate = 75, int t_desired_radio_channel = 19);
    ~XsensMtw() {}

    void start();
    void run();
    void stop();

  private:
    bool configure();

    bool construct_control();
    bool find_wireless_master();
    bool open_port();
    bool get_xsdevice_instance();
    bool set_config_mode();
    void attach_callback_handler();
    bool get_closest_update_rate();
    bool set_update_rate();
    bool set_radio_channel();

    bool wait_mtw_connection();
    bool start_measurement();
    bool get_mtws_device_istances();
    void attach_callback_handlers();
    void setup_ros_topics();

    bool disable_radio();

    void find_closest_update_rate();

    int m_desired_update_rate;
    int m_desired_radio_channel;

    WirelessMasterCallback m_wireless_master_callback; // Callback for wireless master
    std::vector<MtwCallback*> m_mtw_callbacks; // Callbacks for mtw devices

    XsControl* m_control;
    XsPortInfoArray m_detected_devices;
    XsPortInfoArray::const_iterator m_wireless_master_port;
    XsDevicePtr m_wireless_master_device;

    XsIntArray m_supported_update_rates;
    int m_update_rate;

    unsigned long m_number_of_connected_mtws;

    XsDeviceIdArray m_all_device_ids;
    XsDeviceIdArray m_mtw_device_ids;
    XsDevicePtrArray m_mtw_devices;

    std::vector<XsQuaternion> m_orientation_quaternion;
    std::vector<XsVector3> m_raw_acc_data;
    std::vector<XsVector3> m_raw_gyro_data;
    std::vector<XsVector3> m_raw_mag_data;
    std::vector<XsEuler> m_euler_data;

    ros::NodeHandle m_nh;
    std::string m_node_namespace;

    ros::Publisher m_imu_pub;
    ros::Publisher m_mag_pub;

    sensor_msgs::Imu m_imu_msg;
    sensor_msgs::MagneticField m_mag_msg;

    bool m_xsens_mtw_configured;
  };

} // namespace mtw

#endif
