#ifndef hiros_xsens_mtw_wrapper_h
#define hiros_xsens_mtw_wrapper_h

// Standard dependencies
#include <csignal>

// ROS dependencies
#include "geometry_msgs/Vector3Stamped.h"
#include "ros/ros.h"
#include "sensor_msgs/FluidPressure.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "std_msgs/Header.h"
#include "tf2_ros/transform_broadcaster.h"

// Internal dependencies
#include "hiros_xsens_mtw_wrapper/Euler.h"
#include "hiros_xsens_mtw_wrapper/MIMU.h"
#include "hiros_xsens_mtw_wrapper/MIMUArray.h"
#include "hiros_xsens_mtw_wrapper/ResetOrientation.h"
#include "hiros_xsens_mtw_wrapper/StartRecording.h"
#include "hiros_xsens_mtw_wrapper/StopRecording.h"
#include "xsens_mtw/MtwCallback.h"
#include "xsens_mtw/Synchronizer.h"
#include "xsens_mtw/WirelessMasterCallback.h"

#define BASH_MSG_RESET "\033[0m"
#define BASH_MSG_GREEN "\033[32m"

namespace hiros {
  namespace xsens_mtw {

    static const std::map<std::string, Synchronizer::SyncPolicy> sync_policy_map = {
      {"fill_partial_frames", Synchronizer::SyncPolicy::fillPartialFrames},
      {"skip_partial_frames", Synchronizer::SyncPolicy::skipPartialFrames}};

    static const std::map<std::string, XsSyncLine> sync_line_map = {{"XSL_In1", XSL_In1},
                                                                    {"XSL_In2", XSL_In2},
                                                                    {"XSL_Out1", XSL_Out1},
                                                                    {"XSL_Out2", XSL_Out2}};

    static const std::map<XsSyncLine, std::string> sync_line_to_str_map = {{XSL_In1, "XSL_In1"},
                                                                           {XSL_In2, "XSL_In2"},
                                                                           {XSL_Out1, "XSL_Out1"},
                                                                           {XSL_Out2, "XSL_Out2"}};

    static const std::map<std::string, XsSyncFunction> sync_function_map = {
      {"XSF_StartRecording", XSF_StartRecording},
      {"XSF_StopRecording", XSF_StopRecording},
      {"XSF_ResetTimer", XSF_ResetTimer},
      {"XSF_TriggerIndication", XSF_TriggerIndication},
      {"XSF_IntervalTransitionMeasurement", XSF_IntervalTransitionMeasurement},
      {"XSF_IntervalTransitionRecording", XSF_IntervalTransitionRecording},
      {"XSF_GotoOperational", XSF_GotoOperational},
      {"XSF_Invalid", XSF_Invalid},
      {"XSF_Count", XSF_Count}};

    static const std::map<std::string, XsSyncPolarity> sync_polarity_map = {{"XSP_None", XSP_None},
                                                                            {"XSP_RisingEdge", XSP_RisingEdge},
                                                                            {"XSP_PositivePulse", XSP_PositivePulse},
                                                                            {"XSP_FallingEdge", XSP_FallingEdge},
                                                                            {"XSP_NegativePulse", XSP_NegativePulse},
                                                                            {"XSP_Both", XSP_Both}};

    struct XsensMtwParameters
    {
      int desired_update_rate;
      int desired_radio_channel;
      double fixed_latency;

      XsSyncSettingArray sync_settings;

      bool reset_initial_orientation;
    };

    struct WrapperParameters
    {
      int number_of_mtws;
      std::string tf_prefix;
      bool enable_custom_labeling;
      bool enable_external_sync;

      bool publish_only_recording;
      bool synchronize;
      std::string sync_policy_name;
      bool publish_mimu_array;

      bool publish_imu;
      bool publish_mag;
      bool publish_euler;
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
      bool setSyncSettings();
      bool setConfigMode();
      void attachCallbackHandler();
      bool getClosestUpdateRate();
      bool setUpdateRate();
      bool setRadioChannel();
      bool disableRadio();

      template <class T>
      T getSetting(const std::string& setting_name, const std::map<std::string, T>& setting_map) const
      {
        if (setting_map.count(setting_name) == 0) {
          ROS_FATAL_STREAM("Xsens Mtw Wrapper... Unsupported setting '" << setting_name << "'.");
          ROS_FATAL_STREAM("Xsens Mtw Wrapper... Supported settings are:");
          for (const auto& pair : setting_map) {
            ROS_FATAL_STREAM("Xsens Mtw Wrapper...     - " << pair.first);
          }
          ROS_FATAL_STREAM("Xsens Mtw Wrapper... Closing");
          ros::shutdown();
          exit(EXIT_FAILURE);
        }
        return setting_map.at(setting_name);
      }

      void setupRos();
      bool resetInitialOrientation() const;
      void syncInitialPackets();

      std::string getDeviceLabel(const XsDeviceId& id) const;
      XsDeviceId getDeviceId(const std::string label) const;
      std::string composeTopicPrefix(const XsDeviceId& id) const;

      void publishPacket(std::shared_ptr<XsDataPacket> packet);
      void publishFrame(const std::vector<std::shared_ptr<XsDataPacket>>& frame);

      std_msgs::Header getHeader(std::shared_ptr<XsDataPacket> packet) const;
      sensor_msgs::Imu getImuMsg(std::shared_ptr<XsDataPacket> packet) const;
      sensor_msgs::MagneticField getMagMsg(std::shared_ptr<XsDataPacket> packet) const;
      hiros_xsens_mtw_wrapper::Euler getEulerMsg(std::shared_ptr<XsDataPacket> packet) const;
      geometry_msgs::Vector3Stamped getFreeAccelerationMsg(std::shared_ptr<XsDataPacket> packet) const;
      sensor_msgs::FluidPressure getPressureMsg(std::shared_ptr<XsDataPacket> packet) const;
      hiros_xsens_mtw_wrapper::MIMU getMIMUMsg(std::shared_ptr<XsDataPacket> packet) const;
      hiros_xsens_mtw_wrapper::MIMUArray getMIMUArrayMsg(const std::vector<std::shared_ptr<XsDataPacket>>& frame) const;
      geometry_msgs::TransformStamped getTf(std::shared_ptr<XsDataPacket> packet) const;

      bool resetOrientation(hiros_xsens_mtw_wrapper::ResetOrientation::Request& req,
                            hiros_xsens_mtw_wrapper::ResetOrientation::Response& res);
      bool startRecording(hiros_xsens_mtw_wrapper::StartRecording::Request& req,
                          hiros_xsens_mtw_wrapper::StartRecording::Response& res);
      bool stopRecording(hiros_xsens_mtw_wrapper::StopRecording::Request& req,
                         hiros_xsens_mtw_wrapper::StopRecording::Response& res);

      static inline void sighandler(int sig) { s_request_shutdown = (sig == SIGINT); }

      XsensMtwParameters mtw_params_;
      WrapperParameters wrapper_params_;

      WirelessMasterCallback wireless_master_callback_;
      std::map<XsDeviceId, MtwCallback*> mtw_callbacks_;

      XsControl* control_;
      XsPortInfoArray detected_devices_;
      XsPortInfoArray::const_iterator wireless_master_port_;
      XsDevicePtr wireless_master_device_;

      XsIntArray supported_update_rates_;
      int update_rate_;

      const unsigned int connection_timeout_ = 5000; // [ms]
      unsigned long number_of_connected_mtws_;

      std::map<XsDeviceId, XsDevicePtr> connected_devices_;
      std::map<XsDeviceId, std::string> ids_to_labels_;
      std::map<std::string, XsDeviceId> labels_to_ids_;

      long initial_packet_id_;
      ros::Time initial_timestamp_;

      bool xsens_mtw_configured_;

      ros::NodeHandle nh_;
      std::string node_namespace_;

      const unsigned int ros_topic_queue_size_ = 10;

      ros::ServiceServer reset_orientation_srv_;
      ros::ServiceServer start_recording_srv_;
      ros::ServiceServer stop_recording_srv_;
      ros::Publisher data_pub_;
      std::map<XsDeviceId, ros::Publisher> imu_pubs_;
      std::map<XsDeviceId, ros::Publisher> mag_pubs_;
      std::map<XsDeviceId, ros::Publisher> euler_pubs_;
      std::map<XsDeviceId, ros::Publisher> free_acceleration_pubs_;
      std::map<XsDeviceId, ros::Publisher> pressure_pubs_;
      tf2_ros::TransformBroadcaster tf_broadcaster_;

      static bool s_request_shutdown;
    };

  } // namespace xsens_mtw
} // namespace hiros

#endif
