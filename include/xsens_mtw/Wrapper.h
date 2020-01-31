#ifndef hiros_xsens_mtw_wrapper_h
#define hiros_xsens_mtw_wrapper_h

// Standard dependencies
#include <csignal>
#include <deque>

// ROS dependencies
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "ros/ros.h"
#include "sensor_msgs/FluidPressure.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "tf2_ros/transform_broadcaster.h"

// Internal dependencies
#include "hiros_xsens_mtw_wrapper/Euler.h"
#include "hiros_xsens_mtw_wrapper/ResetOrientation.h"
#include "xsens_mtw/MtwCallback.h"
#include "xsens_mtw/WirelessMasterCallback.h"

#define BASH_MSG_RESET "\033[0m"
#define BASH_MSG_GREEN "\033[32m"

namespace hiros {
  namespace xsens_mtw {

    struct XsensMtwParameters
    {
      int desired_update_rate;
      int desired_radio_channel;

      bool reset_initial_orientation;
    };

    struct WrapperParameters
    {
      std::string tf_prefix;
      bool enable_custom_labeling;

      bool publish_imu;
      bool publish_acceleration;
      bool publish_angular_velocity;
      bool publish_mag;
      bool publish_euler;
      bool publish_quaternion;
      bool publish_free_acceleration;
      bool publish_pressure;
      bool publish_tf;
    };

    class Wrapper
    {
    public:
      Wrapper();
      ~Wrapper() {}

      void start();
      void run();

    private:
      bool configure();
      void stop();

      void configureWrapper();
      bool configureXsensMtw();

      void stopXsensMtw();
      void stopWrapper();

      bool waitMtwConnection();
      bool getMtwsDeviceIstances();
      void attachCallbackHandlers();
      bool startMeasurement();

      bool constructControl();
      bool findWirelessMaster();
      bool openPort();
      bool getXsdeviceInstance();
      bool setConfigMode();
      void attachCallbackHandler();
      bool getClosestUpdateRate();
      bool setUpdateRate();
      bool setRadioChannel();
      bool disableRadio();

      void setupRos();
      bool resetInitialOrientation() const;
      void syncInitialPackets();

      std::string getDeviceLabel(const XsDeviceId& t_id) const;
      XsDeviceId getDeviceId(const std::string t_label) const;
      std::string composeTopicPrefix(const XsDeviceId& t_id) const;

      void publishData();
      std_msgs::Header getHeader() const;
      sensor_msgs::Imu getImuMsg() const;
      geometry_msgs::Vector3Stamped getAccelerationMsg() const;
      geometry_msgs::Vector3Stamped getAngularVelocityMsg() const;
      sensor_msgs::MagneticField getMagMsg() const;
      hiros_xsens_mtw_wrapper::Euler getEulerMsg() const;
      geometry_msgs::QuaternionStamped getQuaternionMsg() const;
      geometry_msgs::Vector3Stamped getFreeAccelerationMsg() const;
      sensor_msgs::FluidPressure getPressureMsg() const;
      geometry_msgs::TransformStamped getTf() const;

      bool resetOrientation(hiros_xsens_mtw_wrapper::ResetOrientation::Request& t_req,
                            hiros_xsens_mtw_wrapper::ResetOrientation::Response& t_res);

      static inline void sighandler(int t_sig) { s_request_shutdown = (t_sig == SIGINT); };

      XsensMtwParameters m_mtw_params;
      WrapperParameters m_wrapper_params;

      WirelessMasterCallback m_wireless_master_callback;
      std::map<XsDeviceId, MtwCallback*> m_mtw_callbacks;

      XsControl* m_control;
      XsPortInfoArray m_detected_devices;
      XsPortInfoArray::const_iterator m_wireless_master_port;
      XsDevicePtr m_wireless_master_device;

      XsIntArray m_supported_update_rates;
      int m_update_rate;

      const unsigned int m_connection_timeout = 5000; // [ms]
      unsigned long m_number_of_connected_mtws;

      std::map<XsDeviceId, XsDevicePtr> m_connected_devices;
      std::map<XsDeviceId, std::string> m_ids_to_labels;
      std::map<std::string, XsDeviceId> m_labels_to_ids;

      long m_initial_packet_id;
      ros::Time m_initial_timestamp;
      std::shared_ptr<XsDataPacket> m_latest_packet;

      bool m_xsens_mtw_configured;

      ros::NodeHandle m_nh;
      std::string m_node_namespace;

      ros::ServiceServer m_reset_orientation_srv;
      std::map<XsDeviceId, ros::Publisher> m_imu_pubs;
      std::map<XsDeviceId, ros::Publisher> m_acceleration_pubs;
      std::map<XsDeviceId, ros::Publisher> m_angular_velocity_pubs;
      std::map<XsDeviceId, ros::Publisher> m_mag_pubs;
      std::map<XsDeviceId, ros::Publisher> m_euler_pubs;
      std::map<XsDeviceId, ros::Publisher> m_quaternion_pubs;
      std::map<XsDeviceId, ros::Publisher> m_free_acceleration_pubs;
      std::map<XsDeviceId, ros::Publisher> m_pressure_pubs;
      tf2_ros::TransformBroadcaster m_tf_broadcaster;

      static bool s_request_shutdown;
    };

  } // namespace xsens_mtw
} // namespace hiros

#endif
