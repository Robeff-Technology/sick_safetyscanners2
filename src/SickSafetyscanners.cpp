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

#include <sick_safetyscanners2/SickSafetyscanners.hpp>

#include <algorithm>
#include <vector>

namespace sick {
rcl_interfaces::msg::SetParametersResult SickSafetyscanners::parametersCallback(
    std::vector<rclcpp::Parameter> parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "";

  bool update_sensor_config = false;

  for (const auto &param : parameters) {
    if (param.get_name().rfind("diagnostic_updater.", 0) == 0) {
      continue;
    }

    std::stringstream ss;
    ss << "{" << param.get_name() << ", " << param.value_to_string() << "}";
    RCLCPP_INFO(getLogger(), "Got parameter: '%s'", ss.str().c_str());

    if (param.get_name() == "frame_id") {
      m_config.m_frame_id = param.value_to_string();
    } else if (param.get_name() == "host_ip") {
      m_config.m_communications_settings.host_ip =
          boost::asio::ip::address_v4::from_string(param.value_to_string());
      update_sensor_config = true;
    } else if (param.get_name() == "host_udp_port") {
      m_config.m_communications_settings.host_udp_port = param.as_int();
      update_sensor_config = true;
    } else if (param.get_name() == "channel") {
      m_config.m_communications_settings.channel = param.as_int();
      update_sensor_config = true;
    } else if (param.get_name() == "channel_enabled") {
      m_config.m_communications_settings.enabled = param.as_bool();
      update_sensor_config = true;
    } else if (param.get_name() == "skip") {
      m_config.m_communications_settings.publishing_frequency =
          skipToPublishFrequency(param.as_int());
      update_sensor_config = true;
    } else if (param.get_name() == "angle_start") {
      m_config.m_communications_settings.start_angle =
          sick::radToDeg(param.as_double()) - m_config.m_angle_offset;
      update_sensor_config = true;
    } else if (param.get_name() == "angle_end") {
      m_config.m_communications_settings.end_angle =
          sick::radToDeg(param.as_double()) - m_config.m_angle_offset;
      update_sensor_config = true;
    } else if (param.get_name() == "time_offset") {
      m_config.m_time_offset = param.as_double();
    } else if (param.get_name() == "general_system_state") {
      // TODO improve
      m_config.m_communications_settings.features =
          (m_config.m_communications_settings.features & ~(1UL << 0)) |
          (param.as_bool() << 0);
      update_sensor_config = true;
    } else if (param.get_name() == "derived_settings") {
      m_config.m_communications_settings.features =
          (m_config.m_communications_settings.features & ~(1UL << 1)) |
          (param.as_bool() << 1);
      update_sensor_config = true;
    } else if (param.get_name() == "measurement_data") {
      m_config.m_communications_settings.features =
          (m_config.m_communications_settings.features & ~(1UL << 2)) |
          (param.as_bool() << 2);
      update_sensor_config = true;
    } else if (param.get_name() == "intrusion_data") {
      m_config.m_communications_settings.features =
          (m_config.m_communications_settings.features & ~(1UL << 3)) |
          (param.as_bool() << 3);
      update_sensor_config = true;
    } else if (param.get_name() == "application_io_data") {
      m_config.m_communications_settings.features =
          (m_config.m_communications_settings.features & ~(1UL << 4)) |
          (param.as_bool() << 4);
      update_sensor_config = true;
    } else if (param.get_name() == "min_intensities") {
      m_config.m_min_intensities = param.as_double();
    } else {
      throw std::runtime_error("Parameter is not dynamic reconfigurable");
    }
  }

  if (update_sensor_config) {
    m_device->changeSensorSettings(m_config.m_communications_settings);
  }

  m_config.setupMsgCreator();

  return result;
}

void SickSafetyscanners::setupCommunication(
    std::function<void(const sick::datastructure::Data &)> callback) {
  // Create a sensor instance
  if (m_config.m_communications_settings.host_ip.is_multicast()) {
    m_device = std::make_unique<sick::AsyncSickSafetyScanner>(
        m_config.m_sensor_ip, m_config.m_tcp_port,
        m_config.m_communications_settings, m_config.m_interface_ip, callback);
  } else {
    m_device = std::make_unique<sick::AsyncSickSafetyScanner>(
        m_config.m_sensor_ip, m_config.m_tcp_port,
        m_config.m_communications_settings, callback);
  }

  RCLCPP_INFO(getLogger(), "Communication to Sensor set up");

  // Read sensor specific configurations
  readTypeCodeSettings();
  readMetadata();
  readFirmwareVersion();

  if (m_config.m_use_pers_conf) {
    readPersistentConfig();
  }

  m_config.setupMsgCreator();
}

void SickSafetyscanners::stopCommunication() {
  m_device->stop();
  m_diagnosed_laser_scan_publisher.reset();
  m_diagnostic_updater.reset();
  m_contamination_warning_publisher.reset();
  m_contamination_level_publisher.reset();
  m_contamination_measurement_timer.reset();
  m_contamination_measurement_publisher.reset();
  m_contamination_scan_publisher.reset();
  m_contamination_ema_warning = -1.0;
  m_contamination_ema_contamination = -1.0;
}

sick_safetyscanners2_interfaces::msg::ContaminationLevel
SickSafetyscanners::createContaminationLevelMsg(
    const sick::datastructure::Data &data, const rclcpp::Time &now) {
  sick_safetyscanners2_interfaces::msg::ContaminationLevel msg;
  msg.header.stamp = now;
  msg.header.frame_id = m_config.m_frame_id;

  const std::vector<sick::datastructure::ScanPoint> scan_points =
      data.getMeasurementDataPtr()->getScanPointsVector();

  const auto sectors = static_cast<size_t>(m_contamination_sectors);
  std::vector<uint32_t> sector_warning(sectors, 0);
  std::vector<uint32_t> sector_total(sectors, 0);

  if (!scan_points.empty()) {
    msg.sector_angle_min = scan_points.front().getAngle() + m_config.m_angle_offset;
    msg.sector_angle_max = scan_points.back().getAngle() + m_config.m_angle_offset;
  }

  // The scan points are ordered by angle, so the index already carries the
  // angular information; bucketing by index keeps the sectors evenly sized
  // regardless of the configured angular resolution.
  for (size_t i = 0; i < scan_points.size(); ++i) {
    const sick::datastructure::ScanPoint &scan_point = scan_points[i];

    // Invalid beams carry no contamination information. Counting them would
    // make the ratio depend on what the sensor happens to be looking at.
    if (!scan_point.getValidBit()) {
      continue;
    }

    ++msg.valid_beams;

    const size_t sector = std::min(i * sectors / scan_points.size(), sectors - 1);
    ++sector_total[sector];

    if (scan_point.getContaminationWarningBit()) {
      ++msg.warning_beams;
      ++sector_warning[sector];
    }
    if (scan_point.getContaminationBit()) {
      ++msg.contamination_beams;
    }
  }

  if (msg.valid_beams > 0) {
    msg.instant_warning = 100.f * static_cast<float>(msg.warning_beams) /
                          static_cast<float>(msg.valid_beams);
    msg.instant_contamination = 100.f *
                                static_cast<float>(msg.contamination_beams) /
                                static_cast<float>(msg.valid_beams);
  }

  // A single scan is noisy; contamination builds up over minutes to hours, so
  // the filtered value is what a threshold should be applied to. The first
  // scan seeds the filter to avoid a slow ramp-up from zero after startup.
  const double alpha = m_contamination_filter_alpha;
  if (msg.valid_beams == 0) {
    // Nothing to learn from this scan, hold the previous value.
  } else if (m_contamination_ema_warning < 0.0) {
    m_contamination_ema_warning = msg.instant_warning;
    m_contamination_ema_contamination = msg.instant_contamination;
  } else {
    m_contamination_ema_warning = (1.0 - alpha) * m_contamination_ema_warning +
                                  alpha * msg.instant_warning;
    m_contamination_ema_contamination =
        (1.0 - alpha) * m_contamination_ema_contamination +
        alpha * msg.instant_contamination;
  }
  // The filter state uses a negative sentinel for "not yet seeded"; never let
  // that reach the message, the published level is always a valid percentage.
  msg.level_warning = m_contamination_ema_warning < 0.0
                          ? msg.instant_warning
                          : static_cast<float>(m_contamination_ema_warning);
  msg.level_contamination =
      m_contamination_ema_contamination < 0.0
          ? msg.instant_contamination
          : static_cast<float>(m_contamination_ema_contamination);

  msg.sector_warning.resize(sectors);
  for (size_t i = 0; i < sectors; ++i) {
    msg.sector_warning[i] =
        sector_total[i] > 0 ? 100.f * static_cast<float>(sector_warning[i]) /
                                  static_cast<float>(sector_total[i])
                            : 0.f;
  }

  if (!data.getGeneralSystemStatePtr()->isEmpty()) {
    msg.device_warning =
        data.getGeneralSystemStatePtr()->getContaminationWarning();
    msg.device_error = data.getGeneralSystemStatePtr()->getContaminationError();
  }

  return msg;
}

void SickSafetyscanners::publishContaminationMeasurement(
    const rclcpp::Time &now) {
  if (!m_contamination_measurement_publisher || !m_device) {
    return;
  }

  sick::datastructure::ContaminationInfo info;
  if (!m_device->requestContaminationInfo(m_device_variant, info)) {
    // The base class has no node handle, so throttle against a local clock.
    static rclcpp::Clock throttle_clock(RCL_STEADY_TIME);
    RCLCPP_WARN_THROTTLE(
        getLogger(), throttle_clock, 60000,
        "Could not read the contamination measurement from the device. The "
        "configured device_variant may be wrong, or the firmware may not "
        "support this variable.");
    return;
  }

  sick_safetyscanners2_interfaces::msg::ContaminationMeasurement msg;
  msg.header.stamp = now;
  msg.header.frame_id = m_config.m_frame_id;
  msg.processing_state = info.getProcessingState();
  msg.valid = info.isValid();
  msg.attention_threshold =
      static_cast<float>(m_contamination_attention_threshold);

  const std::vector<sick::datastructure::ContaminationSector> &sectors =
      info.getSectors();

  double level_sum = 0.0;
  for (const auto &sector : sectors) {
    msg.sector_level.push_back(sector.level);
    msg.sector_warning.push_back(sector.warning);
    msg.sector_error.push_back(sector.error);
    msg.sector_angle_start.push_back(sector.angle_start);
    msg.sector_angle_end.push_back(sector.angle_end);

    msg.level_max = std::max(msg.level_max, sector.level);
    level_sum += sector.level;
    msg.warning = msg.warning || sector.warning;
    msg.error = msg.error || sector.error;
  }
  if (!sectors.empty()) {
    msg.level_mean = static_cast<float>(level_sum / sectors.size());
  }

  m_contamination_measurement_publisher->publish(msg);
}

void SickSafetyscanners::publishContaminationScan(const rclcpp::Time &now) {
  if (!m_contamination_scan_publisher || !m_device) {
    return;
  }

  sick::datastructure::ContaminationScan scan;
  if (!m_device->requestContaminationScan(scan)) {
    static rclcpp::Clock throttle_clock(RCL_STEADY_TIME);
    RCLCPP_WARN_THROTTLE(
        getLogger(), throttle_clock, 60000,
        "Could not read the per beam contamination from the device. This "
        "variable needs firmware 1.66 (nanoScan3) or 1.7x (microScan3, "
        "outdoorScan3); set contamination_per_beam to false if the device "
        "does not support it.");
    return;
  }

  sick_safetyscanners2_interfaces::msg::ContaminationScan msg;
  msg.header.stamp = now;
  msg.header.frame_id = m_config.m_frame_id;
  msg.valid = scan.isValid();

  const std::vector<sick::datastructure::ContaminationBeam> &beams =
      scan.getBeams();
  msg.num_beams = static_cast<uint32_t>(beams.size());
  msg.level.reserve(beams.size());
  msg.warning.reserve(beams.size());
  msg.error.reserve(beams.size());

  double level_sum = 0.0;
  for (const auto &beam : beams) {
    msg.level.push_back(beam.level);
    msg.warning.push_back(beam.warning);
    msg.error.push_back(beam.error);

    msg.level_max = std::max(msg.level_max, beam.level);
    level_sum += beam.level;
  }
  if (!beams.empty()) {
    msg.level_mean = static_cast<float>(level_sum / beams.size());
  }

  m_contamination_scan_publisher->publish(msg);
}

std::string boolToString(bool b) { return b ? "true" : "false"; }

void SickSafetyscanners::sensorDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper &diagnostic_status) {
  const sick_safetyscanners2_interfaces::msg::DataHeader &header =
      m_last_raw_msg.header;
  if (header.timestamp_time == 0 && header.timestamp_date == 0) {
    diagnostic_status.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE,
                              "Could not get sensor state");
    return;
  }

  diagnostic_status.addf("Version version", "%c", header.version_version);
  diagnostic_status.addf("Version major version", "%u",
                         header.version_major_version);
  diagnostic_status.addf("Version minor version", "%u",
                         header.version_minor_version);
  diagnostic_status.addf("Version release", "%u", header.version_release);
  diagnostic_status.addf(
      "Firmware version", "%s",
      m_config.m_firmware_version.getFirmwareVersion().c_str());
  diagnostic_status.addf("Serial number of device", "%u",
                         header.serial_number_of_device);
  diagnostic_status.addf("Serial number of channel plug", "%u",
                         header.serial_number_of_channel_plug);
  diagnostic_status.addf("App checksum", "%08X",
                         m_config.m_metadata.getAppChecksum());
  diagnostic_status.addf("Overall checksum", "%08X",
                         m_config.m_metadata.getOverallChecksum());
  diagnostic_status.addf("Channel number", "%u", header.channel_number);
  diagnostic_status.addf("Sequence number", "%u", header.sequence_number);
  diagnostic_status.addf("Scan number", "%u", header.scan_number);
  diagnostic_status.addf("Timestamp date", "%u", header.timestamp_date);
  diagnostic_status.addf("Timestamp time", "%u", header.timestamp_time);

  const sick_safetyscanners2_interfaces::msg::GeneralSystemState &state =
      m_last_raw_msg.general_system_state;
  diagnostic_status.add("Run mode active", boolToString(state.run_mode_active));
  diagnostic_status.add("Standby mode active",
                        boolToString(state.standby_mode_active));
  diagnostic_status.add("Contamination warning",
                        boolToString(state.contamination_warning));
  diagnostic_status.add("Contamination error",
                        boolToString(state.contamination_error));
  diagnostic_status.add("Reference contour status",
                        boolToString(state.reference_contour_status));
  diagnostic_status.add("Manipulation status",
                        boolToString(state.manipulation_status));
  diagnostic_status.addf("Current monitoring case no table 1", "%u",
                         state.current_monitoring_case_no_table_1);
  diagnostic_status.addf("Current monitoring case no table 2", "%u",
                         state.current_monitoring_case_no_table_2);
  diagnostic_status.addf("Current monitoring case no table 3", "%u",
                         state.current_monitoring_case_no_table_3);
  diagnostic_status.addf("Current monitoring case no table 4", "%u",
                         state.current_monitoring_case_no_table_4);
  diagnostic_status.add("Application error",
                        boolToString(state.application_error));
  diagnostic_status.add("Device error", boolToString(state.device_error));

  if (state.device_error) {
    diagnostic_status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                              "Device error");
  } else if (state.application_error) {
    diagnostic_status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                              "Application error");
  } else if (state.contamination_error) {
    diagnostic_status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                              "Contamination error");
  } else if (state.contamination_warning) {
    diagnostic_status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                              "Contamination warning");
  } else {
    diagnostic_status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
  }
}

bool SickSafetyscanners::getFieldData(
    const std::shared_ptr<
        sick_safetyscanners2_interfaces::srv::FieldData::Request>
        request,
    std::shared_ptr<sick_safetyscanners2_interfaces::srv::FieldData::Response>
        response) {
  // Suppress warning of unused request variable due to empty request fields
  (void)request;

  std::vector<sick::datastructure::FieldData> fields;
  m_device->requestFieldData(fields);

  for (size_t i = 0; i < fields.size(); i++) {
    sick::datastructure::FieldData field = fields.at(i);
    sick_safetyscanners2_interfaces::msg::Field field_msg;

    field_msg.start_angle =
        degToRad(field.getStartAngle() + m_config.m_angle_offset);
    field_msg.angular_resolution = degToRad(field.getAngularBeamResolution());
    field_msg.protective_field = field.getIsProtectiveField();

    std::vector<uint16_t> ranges = field.getBeamDistances();
    for (size_t j = 0; j < ranges.size(); j++) {
      field_msg.ranges.push_back(static_cast<float>(ranges.at(j)) * 1e-3);
    }

    response->fields.push_back(field_msg);
  }

  datastructure::DeviceName device_name;
  m_device->requestDeviceName(device_name);
  response->device_name = device_name.getDeviceName();

  std::vector<sick::datastructure::MonitoringCaseData> monitoring_cases;
  m_device->requestMonitoringCases(monitoring_cases);

  for (const auto &monitoring_case : monitoring_cases) {
    sick_safetyscanners2_interfaces::msg::MonitoringCase monitoring_case_msg;

    monitoring_case_msg.monitoring_case_number =
        monitoring_case.getMonitoringCaseNumber();
    std::vector<uint16_t> mon_fields = monitoring_case.getFieldIndices();
    std::vector<bool> mon_fields_valid = monitoring_case.getFieldsValid();
    for (size_t j = 0; j < mon_fields.size(); j++) {
      monitoring_case_msg.fields.push_back(mon_fields.at(j));
      monitoring_case_msg.fields_valid.push_back(mon_fields_valid.at(j));
    }
    response->monitoring_cases.push_back(monitoring_case_msg);
  }

  return true;
}

bool SickSafetyscanners::getStatusOverview(
    const std::shared_ptr<
        sick_safetyscanners2_interfaces::srv::StatusOverview::Request>
        request,
    std::shared_ptr<
        sick_safetyscanners2_interfaces::srv::StatusOverview::Response>
        response) {
  (void)request;

  sick::datastructure::StatusOverview d;
  m_device->requestStatusOverview(d);

  response->version_c_version = d.getVersionCVersion();
  response->version_major_version_number = d.getVersionMajorVersionNumber();
  response->version_minor_version_number = d.getVersionMinorVersionNumber();
  response->version_release_number = d.getVersionReleaseNumber();

  response->device_state = d.getDeviceState();
  response->config_state = d.getConfigState();
  response->application_state = d.getApplicationState();
  response->current_time_power_on_count = d.getCurrentTimePowerOnCount();

  response->current_time_time = d.getCurrentTimeTime();
  response->current_time_date = d.getCurrentTimeDate();

  response->error_info_code = d.getErrorInfoCode();

  response->error_info_time = d.getErrorInfoTime();
  response->error_info_time_date = d.getErrorInfoDate();

  return true;
}

void SickSafetyscanners::readTypeCodeSettings() {
  RCLCPP_INFO(getLogger(), "Reading Type code settings");
  sick::datastructure::TypeCode type_code;
  m_device->requestTypeCode(type_code);
  m_config.m_communications_settings.e_interface_type =
      type_code.getInterfaceType();
  m_config.m_range_min = 0.1;
  m_config.m_range_max = type_code.getMaxRange();
}

void SickSafetyscanners::readPersistentConfig() {
  RCLCPP_INFO(getLogger(), "Reading Persistent Configuration");
  sick::datastructure::ConfigData config_data;
  m_device->requestPersistentConfig(config_data);
  m_config.m_communications_settings.start_angle = config_data.getStartAngle();
  m_config.m_communications_settings.end_angle = config_data.getEndAngle();
}

void SickSafetyscanners::readMetadata() {
  RCLCPP_INFO(getLogger(), "Reading Metadata");
  m_device->requestConfigMetadata(m_config.m_metadata);
}

void SickSafetyscanners::readFirmwareVersion() {
  RCLCPP_INFO(getLogger(), "Reading firmware version");
  m_device->requestFirmwareVersion(m_config.m_firmware_version);
}
} // namespace sick
