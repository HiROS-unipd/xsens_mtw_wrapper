#ifndef hiros_xsens_mtw_wrapper_h
#define hiros_xsens_mtw_wrapper_h

// Standard dependencies
#include <csignal>

// ROS dependencies
#include "geometry_msgs/QuaternionStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/FluidPressure.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/TimeReference.h"
#include "tf2_ros/transform_broadcaster.h"

// Internal dependencies
#include "hiros_xsens_mtw_wrapper/Euler.h"
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
    };

    struct WrapperParameters
    {
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
      void stop();
      bool configure();

      void configureWrapper();
      bool configureXsensMtw();

      bool waitMtwConnection();
      bool getMtwsDeviceIstances();
      void setupRosTopics();
      void initializeVectors();
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

      std::string getDeviceLabel(const XsDeviceId& t_id) const;
      std::string composeTopicPrefix(const XsDeviceId& t_id) const;
      void computeSampleTime();

      sensor_msgs::Imu getImuMsg() const;
      geometry_msgs::Vector3Stamped getAccelerationMsg() const;
      geometry_msgs::Vector3Stamped getAngularVelocityMsg() const;
      sensor_msgs::MagneticField getMagMsg() const;
      hiros_xsens_mtw_wrapper::Euler getEulerMsg() const;
      geometry_msgs::QuaternionStamped getQuaternionMsg() const;
      geometry_msgs::Vector3Stamped getFreeAccelerationMsg() const;
      sensor_msgs::FluidPressure getPressureMsg() const;
      geometry_msgs::TransformStamped getTf() const;

      static inline void sighandler(int t_sig) { s_request_shutdown = (t_sig == SIGINT); };

      XsensMtwParameters m_mtw_params;
      WrapperParameters m_wrapper_params;

      WirelessMasterCallback m_wireless_master_callback;
      std::vector<MtwCallback*> m_mtw_callbacks;

      XsControl* m_control;
      XsPortInfoArray m_detected_devices;
      XsPortInfoArray::const_iterator m_wireless_master_port;
      XsDevicePtr m_wireless_master_device;

      XsIntArray m_supported_update_rates;
      int m_update_rate;
      double m_sample_time_epsilon;

      const unsigned int m_connection_attempts = 50;
      unsigned long m_number_of_connected_mtws;
      std::map<std::string, std::string> m_ids_to_labels;

      XsDeviceIdArray m_all_device_ids;
      XsDeviceIdArray m_mtw_device_ids;
      XsDevicePtrArray m_mtw_devices;

      bool m_xsens_mtw_configured;

      ros::NodeHandle m_nh;
      std::string m_node_namespace;

      const XsDataPacket* m_packet;
      ros::Time m_sample_time;

      std::vector<XsTimeStamp> m_prev_packet_time_of_arrival;
      std::vector<ros::Time> m_prev_packet_sample_time;

      std::vector<ros::Publisher> m_imu_pub;
      std::vector<ros::Publisher> m_acceleration_pub;
      std::vector<ros::Publisher> m_angular_velocity_pub;
      std::vector<ros::Publisher> m_mag_pub;
      std::vector<ros::Publisher> m_euler_pub;
      std::vector<ros::Publisher> m_quaternion_pub;
      std::vector<ros::Publisher> m_free_acceleration_pub;
      std::vector<ros::Publisher> m_pressure_pub;
      tf2_ros::TransformBroadcaster m_tf_broadcaster;

      static bool s_request_shutdown;
    };

  } // namespace xsens_mtw
} // namespace hiros

#endif
