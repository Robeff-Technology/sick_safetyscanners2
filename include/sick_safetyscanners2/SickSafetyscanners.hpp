// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------

/*!
*  Copyright (C) 2023, Lowpad, Bleskensgraaf, Netherlands*
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*    http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.

*/

// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!
 * \file SickSafetyscanners.h
 *
 * \author  Rein Appeldoorn <rein.appeldoorn@lowpad.com>
 * \date    2023-09-07
 */
//----------------------------------------------------------------------

#pragma once

#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

#include <std_msgs/msg/bool.hpp>

#include <sick_safetyscanners_base/SickSafetyscanners.h>

#include <sick_safetyscanners2_interfaces/srv/field_data.hpp>
#include <sick_safetyscanners2_interfaces/srv/status_overview.hpp>

#include <sick_safetyscanners2_interfaces/msg/contamination_level.hpp>
#include <sick_safetyscanners2_interfaces/msg/contamination_measurement.hpp>
#include <sick_safetyscanners2_interfaces/msg/contamination_scan.hpp>

#include <sick_safetyscanners2/utils/Conversions.h>
#include <sick_safetyscanners2/utils/MessageCreator.h>

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace sick {

/**
 * Sick safety scanners base class that implements the shared functionality
 * between the Node (Ros2) and LifCycle node (LifeCycle)
 */
class SickSafetyscanners {
public:
  using DiagnosedLaserScanPublisher =
      diagnostic_updater::DiagnosedPublisher<sensor_msgs::msg::LaserScan>;

  /**
   * Sick safety scanner configuration
   */
  struct Config {
    void setupMsgCreator() {
      m_msg_creator = std::make_unique<sick::MessageCreator>(
          m_frame_id, m_time_offset, m_range_min, m_range_max, m_angle_offset,
          m_min_intensities);
    }

    boost::asio::ip::address_v4 m_sensor_ip;
    boost::asio::ip::address_v4 m_interface_ip;
    std::string m_frame_id;
    double m_time_offset = 0.0;
    double m_range_min = 0.0;
    double m_range_max;
    double m_frequency_tolerance = 0.1;
    double m_expected_frequency = 34.0;
    double m_timestamp_min_acceptable = -1.0;
    double m_timestamp_max_acceptable = 1.0;
    double m_min_intensities = 0.0; /*!< min intensities for laser points */
    bool m_use_sick_angles;
    float m_angle_offset = -90.0;
    bool m_use_pers_conf = false;

    sick::types::port_t m_tcp_port = 2122;

    datastructure::ConfigMetadata m_metadata;

    datastructure::FirmwareVersion m_firmware_version;

    sick::datastructure::CommSettings m_communications_settings;

    std::unique_ptr<sick::MessageCreator> m_msg_creator;
  };

  Config m_config;

  /**
   * Initialize parameters for this node
   *
   * This method is a template function because the node and lifecycle node
   * are not sharing the same implementation but have the same interface.
   *
   * @tparam NodeT Templated node input
   * @param node Node
   */
  template <typename NodeT> static void initializeParameters(NodeT &node) {
    node.template declare_parameter<std::string>("frame_id", "scan");
    node.template declare_parameter<std::string>("sensor_ip", "192.168.1.11");
    node.template declare_parameter<std::string>("host_ip", "192.168.1.9");
    node.template declare_parameter<std::string>("interface_ip", "0.0.0.0");
    node.template declare_parameter<int>("host_udp_port", 0);
    node.template declare_parameter<int>("channel", 0);
    node.template declare_parameter<bool>("channel_enabled", true);
    node.template declare_parameter<int>("skip", 0);
    node.template declare_parameter<double>("angle_start", 0.0);
    node.template declare_parameter<double>("angle_end", 0.0);
    node.template declare_parameter<double>("time_offset", 0.0);
    node.template declare_parameter<bool>("general_system_state", true);
    node.template declare_parameter<bool>("derived_settings", true);
    node.template declare_parameter<bool>("measurement_data", true);
    node.template declare_parameter<bool>("intrusion_data", true);
    node.template declare_parameter<bool>("application_io_data", true);
    node.template declare_parameter<bool>("use_persistent_config", false);
    node.template declare_parameter<float>("min_intensities", 0.f);

    node.template declare_parameter<double>("expected_frequency", 34);
    node.template declare_parameter<double>("frequency_tolerance", 0.1);
    node.template declare_parameter<double>("timestamp_min_acceptable", -1);
    node.template declare_parameter<double>("timestamp_max_acceptable", 1);
    node.template declare_parameter<int>("contamination_sectors", 12);
    node.template declare_parameter<double>("contamination_filter_alpha",
                                            0.02);
    node.template declare_parameter<std::string>("device_variant", "nanoScan3");
    node.template declare_parameter<double>("contamination_poll_period", 5.0);
    node.template declare_parameter<bool>("contamination_per_beam", false);
  }

  /**
   * Load the initial parameters
   *
   * This method is a template function because the node and lifecycle node
   * are not sharing the same implementation but have the same interface.
   *
   * @tparam NodeT Templated node input
   * @param node Node
   */
  template <typename NodeT> void loadParameters(NodeT &node) {
    node.template get_parameter<std::string>("frame_id", m_config.m_frame_id);
    RCLCPP_INFO(getLogger(), "frame_id: %s", m_config.m_frame_id.c_str());

    std::string sensor_ip;
    node.template get_parameter<std::string>("sensor_ip", sensor_ip);
    RCLCPP_INFO(getLogger(), "sensor_ip: %s", sensor_ip.c_str());
    m_config.m_sensor_ip = boost::asio::ip::address_v4::from_string(sensor_ip);

    std::string interface_ip;
    node.template get_parameter<std::string>("interface_ip", interface_ip);
    RCLCPP_INFO(getLogger(), "interface_ip: %s", interface_ip.c_str());
    m_config.m_interface_ip =
        boost::asio::ip::address_v4::from_string(interface_ip);

    std::string host_ip;
    node.template get_parameter<std::string>("host_ip", host_ip);
    RCLCPP_INFO(getLogger(), "host_ip: %s", host_ip.c_str());
    // TODO check if valid IP?
    m_config.m_communications_settings.host_ip =
        boost::asio::ip::address_v4::from_string(host_ip);

    int host_udp_port;
    node.template get_parameter<int>("host_udp_port", host_udp_port);
    RCLCPP_INFO(getLogger(), "host_udp_port: %i", host_udp_port);
    m_config.m_communications_settings.host_udp_port = host_udp_port;

    int channel;
    node.template get_parameter<int>("channel", channel);
    RCLCPP_INFO(getLogger(), "channel: %i", channel);
    m_config.m_communications_settings.channel = channel;

    bool enabled;
    node.template get_parameter<bool>("channel_enabled", enabled);
    RCLCPP_INFO(getLogger(), "channel_enabled: %s", btoa(enabled).c_str());
    m_config.m_communications_settings.enabled = enabled;

    int skip;
    node.template get_parameter<int>("skip", skip);
    RCLCPP_INFO(getLogger(), "skip: %i", skip);
    m_config.m_communications_settings.publishing_frequency =
        skipToPublishFrequency(skip);

    float angle_start;
    node.template get_parameter<float>("angle_start", angle_start);
    RCLCPP_INFO(getLogger(), "angle_start: %f", angle_start);

    float angle_end;
    node.template get_parameter<float>("angle_end", angle_end);
    RCLCPP_INFO(getLogger(), "angle_end: %f", angle_end);

    // Included check before calculations to prevent rounding errors while
    // calculating
    if (angle_start == angle_end) {
      m_config.m_communications_settings.start_angle = sick::radToDeg(0);
      m_config.m_communications_settings.end_angle = sick::radToDeg(0);
    } else {
      m_config.m_communications_settings.start_angle =
          sick::radToDeg(angle_start) - m_config.m_angle_offset;
      m_config.m_communications_settings.end_angle =
          sick::radToDeg(angle_end) - m_config.m_angle_offset;
    }

    node.template get_parameter<double>("time_offset", m_config.m_time_offset);
    RCLCPP_INFO(getLogger(), "time_offset: %f", m_config.m_time_offset);

    // Features
    bool general_system_state;
    node.template get_parameter<bool>("general_system_state",
                                      general_system_state);
    RCLCPP_INFO(getLogger(), "general_system_state: %s",
                btoa(general_system_state).c_str());

    bool derived_settings;
    node.template get_parameter<bool>("derived_settings", derived_settings);
    RCLCPP_INFO(getLogger(), "derived_settings: %s",
                btoa(derived_settings).c_str());

    bool measurement_data;
    node.template get_parameter<bool>("measurement_data", measurement_data);
    RCLCPP_INFO(getLogger(), "measurement_data: %s",
                btoa(measurement_data).c_str());

    bool intrusion_data;
    node.template get_parameter<bool>("intrusion_data", intrusion_data);
    RCLCPP_INFO(getLogger(), "intrusion_data: %s",
                btoa(intrusion_data).c_str());

    bool application_io_data;
    node.template get_parameter<bool>("application_io_data",
                                      application_io_data);
    RCLCPP_INFO(getLogger(), "application_io_data: %s",
                btoa(application_io_data).c_str());

    m_config.m_communications_settings.features =
        sick::SensorDataFeatures::toFeatureFlags(
            general_system_state, derived_settings, measurement_data,
            intrusion_data, application_io_data);

    node.template get_parameter<bool>("use_persistent_config",
                                      m_config.m_use_pers_conf);
    RCLCPP_INFO(getLogger(), "use_persistent_config: %s",
                btoa(m_config.m_use_pers_conf).c_str());

    node.template get_parameter<double>("min_intensities",
                                        m_config.m_min_intensities);
    RCLCPP_INFO(getLogger(), "min_intensities: %f", m_config.m_min_intensities);

    node.template get_parameter<double>("expected_frequency",
                                        m_config.m_expected_frequency);
    RCLCPP_INFO(getLogger(), "expected_frequency: %f",
                m_config.m_expected_frequency);

    node.template get_parameter<double>("frequency_tolerance",
                                        m_config.m_frequency_tolerance);
    RCLCPP_INFO(getLogger(), "frequency_tolerance: %f",
                m_config.m_frequency_tolerance);

    node.template get_parameter<double>("timestamp_min_acceptable",
                                        m_config.m_timestamp_min_acceptable);
    RCLCPP_INFO(getLogger(), "timestamp_min_acceptable: %f",
                m_config.m_timestamp_min_acceptable);

    node.template get_parameter<double>("timestamp_max_acceptable",
                                        m_config.m_timestamp_max_acceptable);
    RCLCPP_INFO(getLogger(), "timestamp_max_acceptable: %f",
                m_config.m_timestamp_max_acceptable);

    node.template get_parameter<int>("contamination_sectors",
                                     m_contamination_sectors);
    if (m_contamination_sectors < 1) {
      RCLCPP_WARN(getLogger(),
                  "contamination_sectors must be >= 1, got %i, using 1",
                  m_contamination_sectors);
      m_contamination_sectors = 1;
    }
    RCLCPP_INFO(getLogger(), "contamination_sectors: %i",
                m_contamination_sectors);

    node.template get_parameter<double>("contamination_filter_alpha",
                                        m_contamination_filter_alpha);
    if (m_contamination_filter_alpha <= 0.0 ||
        m_contamination_filter_alpha > 1.0) {
      RCLCPP_WARN(getLogger(),
                  "contamination_filter_alpha must be in (0, 1], got %f, "
                  "using 0.02",
                  m_contamination_filter_alpha);
      m_contamination_filter_alpha = 0.02;
    }
    RCLCPP_INFO(getLogger(), "contamination_filter_alpha: %f",
                m_contamination_filter_alpha);

    std::string device_variant;
    node.template get_parameter<std::string>("device_variant", device_variant);
    if (device_variant == "microScan3") {
      m_device_variant = sick::datastructure::ContaminationVariant::MICRO_SCAN3;
    } else if (device_variant == "outdoorScan3") {
      m_device_variant =
          sick::datastructure::ContaminationVariant::OUTDOOR_SCAN3;
    } else {
      if (device_variant != "nanoScan3") {
        RCLCPP_WARN(getLogger(),
                    "unknown device_variant '%s', using nanoScan3",
                    device_variant.c_str());
      }
      m_device_variant = sick::datastructure::ContaminationVariant::NANO_SCAN3;
    }
    RCLCPP_INFO(getLogger(), "device_variant: %s", device_variant.c_str());

    node.template get_parameter<double>("contamination_poll_period",
                                        m_contamination_poll_period);
    RCLCPP_INFO(getLogger(), "contamination_poll_period: %f",
                m_contamination_poll_period);

    node.template get_parameter<bool>("contamination_per_beam",
                                      m_contamination_per_beam);
    RCLCPP_INFO(getLogger(), "contamination_per_beam: %s",
                btoa(m_contamination_per_beam).c_str());
  }

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      m_param_callback;
  rcl_interfaces::msg::SetParametersResult
  parametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Diagnostics
  std::shared_ptr<diagnostic_updater::Updater> m_diagnostic_updater;
  std::shared_ptr<DiagnosedLaserScanPublisher> m_diagnosed_laser_scan_publisher;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
      m_contamination_warning_publisher;
  rclcpp::Publisher<
      sick_safetyscanners2_interfaces::msg::ContaminationLevel>::SharedPtr
      m_contamination_level_publisher;

  // Contamination level filter state. Negative means "not yet initialized",
  // so the first scan seeds the EMA instead of ramping up from zero.
  int m_contamination_sectors = 12;
  double m_contamination_filter_alpha = 0.02;
  double m_contamination_ema_warning = -1.0;
  double m_contamination_ema_contamination = -1.0;

  // Device contamination measurement, read over CoLa2 on a timer.
  rclcpp::Publisher<
      sick_safetyscanners2_interfaces::msg::ContaminationMeasurement>::SharedPtr
      m_contamination_measurement_publisher;
  rclcpp::TimerBase::SharedPtr m_contamination_measurement_timer;
  sick::datastructure::ContaminationVariant m_device_variant =
      sick::datastructure::ContaminationVariant::NANO_SCAN3;
  double m_contamination_poll_period = 5.0;
  bool m_contamination_per_beam = false;
  rclcpp::Publisher<
      sick_safetyscanners2_interfaces::msg::ContaminationScan>::SharedPtr
      m_contamination_scan_publisher;

  // Device and Communication
  std::unique_ptr<sick::AsyncSickSafetyScanner> m_device;

  /**
   * Setup the device communication
   * @param callback Callback that fires when a new UDP packet is received
   */
  void setupCommunication(
      std::function<void(const sick::datastructure::Data &)> callback);

  /**
   * Start the sensor communication by receiving UDP packets
   */
  template <typename NodeT>
  void startCommunication(
      NodeT *node,
      rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher) {
    // Set-up diagnostics
    m_diagnostic_updater = std::make_shared<diagnostic_updater::Updater>(node);
    m_diagnostic_updater->setHardwareID(m_config.m_sensor_ip.to_string());

    diagnostic_updater::FrequencyStatusParam frequency_param(
        &m_config.m_expected_frequency, &m_config.m_expected_frequency,
        m_config.m_frequency_tolerance);

    diagnostic_updater::TimeStampStatusParam timestamp_param(
        m_config.m_timestamp_min_acceptable,
        m_config.m_timestamp_max_acceptable);

    m_diagnosed_laser_scan_publisher =
        std::make_shared<DiagnosedLaserScanPublisher>(
            publisher, *m_diagnostic_updater, frequency_param, timestamp_param);

    m_diagnostic_updater->add("State", this,
                              &SickSafetyscanners::sensorDiagnostics);

    // Contamination boolean publisher
    m_contamination_warning_publisher =
      node->template create_publisher<std_msgs::msg::Bool>(
        "sick_contamination_warning", 1);

    // Continuous contamination level publisher
    m_contamination_level_publisher = node->template create_publisher<
        sick_safetyscanners2_interfaces::msg::ContaminationLevel>(
        "sick_contamination_level", 1);

    // Device contamination measurement, polled over CoLa2. Contamination
    // builds up over minutes to hours, so a slow poll is enough and keeps the
    // CoLa2 session free for everything else.
    m_contamination_measurement_publisher = node->template create_publisher<
        sick_safetyscanners2_interfaces::msg::ContaminationMeasurement>(
        "sick_contamination_measurement", 1);
    if (m_contamination_per_beam) {
      m_contamination_scan_publisher = node->template create_publisher<
          sick_safetyscanners2_interfaces::msg::ContaminationScan>(
          "sick_contamination_scan", 1);
    }
    if (m_contamination_poll_period > 0.0) {
      m_contamination_measurement_timer = node->create_wall_timer(
          std::chrono::duration<double>(m_contamination_poll_period),
          [this, node]() {
            const rclcpp::Time now = node->now();
            publishContaminationMeasurement(now);
            publishContaminationScan(now);
          });
    }

    // Start async receiving and processing of sensor data
    RCLCPP_INFO(getLogger(), "Run");
    m_device->run();
    m_device->changeSensorSettings(m_config.m_communications_settings);
  }

  /**
   * Stop the sensor communication
   */
  void stopCommunication();

  /**
   * Build the continuous contamination level message from the per-beam
   * contamination bits of a scan, updating the EMA filter state.
   *
   * @param data Sensor data of a single scan
   * @param now Timestamp for the message header
   */
  sick_safetyscanners2_interfaces::msg::ContaminationLevel
  createContaminationLevelMsg(const sick::datastructure::Data &data,
                              const rclcpp::Time &now);

  /*!
   * Read the device's own contamination measurement over CoLa2 and publish it.
   *
   * @param now Timestamp for the message header
   */
  void publishContaminationMeasurement(const rclcpp::Time &now);

  /*!
   * Read the device's contamination measurement projected onto every
   * measurement beam over CoLa2 and publish it. Does nothing unless the
   * contamination_per_beam parameter is set.
   *
   * @param now Timestamp for the message header
   */
  void publishContaminationScan(const rclcpp::Time &now);

  // Diagnostics
  sick_safetyscanners2_interfaces::msg::RawMicroScanData m_last_raw_msg;
  void sensorDiagnostics(
      diagnostic_updater::DiagnosticStatusWrapper &diagnostic_status);

  // Methods Triggering COLA2 calls towards the sensor
  bool getFieldData(
      const std::shared_ptr<
          sick_safetyscanners2_interfaces::srv::FieldData::Request>
          request,
      std::shared_ptr<sick_safetyscanners2_interfaces::srv::FieldData::Response>
          response);
  bool getStatusOverview(
      const std::shared_ptr<
          sick_safetyscanners2_interfaces::srv::StatusOverview::Request>
          request,
      std::shared_ptr<
          sick_safetyscanners2_interfaces::srv::StatusOverview::Response>
          response);

  void readPersistentConfig();
  void readTypeCodeSettings();
  void readMetadata();
  void readFirmwareVersion();

private:
  rclcpp::Logger getLogger() {
    return rclcpp::get_logger("SickSafetyscanners");
  }
};
} // namespace sick
