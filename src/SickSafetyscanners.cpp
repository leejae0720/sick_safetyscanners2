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
 * \file SickSafetyscanners.cpp
 *
 * \author  Rein Appeldoorn <rein.appeldoorn@lowpad.com>
 * \date    2023-09-07
 */
//----------------------------------------------------------------------

#include <sick_safetyscanners2/SickSafetyscanners.hpp>

namespace {

std::mutex g_comm_mtx;                 // 통신 리스타트 동기화용
std::atomic<bool> g_restarting{false}; // 재시작 중 플래그
// --- 파일 로컬(전역) 워치독 상태: 헤더 수정 없이 동작하도록 설계 ---
std::atomic<bool> g_watchdog_running{false};
std::thread       g_watchdog_thread;
std::atomic<int64_t> g_last_rx_ns{0};
constexpr int kTimeoutSec = 5; // 무응답 허용 시간(초)

// 재시작 시 필요한 컨텍스트: 현재 인스턴스와 래핑된 콜백
sick::SickSafetyscanners* g_self = nullptr;
std::function<void(const sick::datastructure::Data&)> g_wrapped_cb;

inline void mark_rx_now() {
  auto now = std::chrono::steady_clock::now().time_since_epoch();
  g_last_rx_ns.store(
    std::chrono::duration_cast<std::chrono::nanoseconds>(now).count(),
    std::memory_order_relaxed);
}

inline void stop_watchdog() {
  if (!g_watchdog_running.exchange(false)) return;
  if (g_watchdog_thread.joinable()) g_watchdog_thread.join();
}

// 통신 (재)설정
inline void restart_comm() {
  if (!g_self) return;
  auto& self = *g_self;

  // 1) 이전 디바이스 안전 종료 (예외 무시)
  try {
    if (self.m_device) self.m_device->stop();
  } catch (const std::exception& e) {
    spdlog::warn("stop() during restart raised: {}", e.what());
  } catch (...) {
    spdlog::warn("stop() during restart raised: unknown error");
  }

  // (주의) 퍼블리셔/업데이터는 여기서 reset하지 않음 — 재시작 중 콜백 경합 위험 방지 목적이면
  // 콜백 쪽에서 가드하거나(재시작 플래그) stop 시에만 정리.

  // 2) 디바이스 재생성 + run + 설정 적용
  try {
    if (self.m_config.m_communications_settings.host_ip.is_multicast()) {
      self.m_device = std::make_unique<sick::AsyncSickSafetyScanner>(
          self.m_config.m_sensor_ip, self.m_config.m_tcp_port,
          self.m_config.m_communications_settings, self.m_config.m_interface_ip,
          g_wrapped_cb);
    } else {
      self.m_device = std::make_unique<sick::AsyncSickSafetyScanner>(
          self.m_config.m_sensor_ip, self.m_config.m_tcp_port,
          self.m_config.m_communications_settings, g_wrapped_cb);
    }

    // 재시작 시에도 run()과 설정 적용 필요
    self.m_device->run();
    self.m_device->changeSensorSettings(self.m_config.m_communications_settings);

    spdlog::info("Communication to Sensor set up (re)started");
    // 성공시 수신 타임스탬프 리셋
    mark_rx_now();
  } catch (const std::exception& e) {
      spdlog::error("LiDAR restart failed: {}", e.what());
      return; // 다음 워치독 주기에 재시도
  } catch (...) {
      spdlog::error("LiDAR restart failed: unknown error");
      return;
  }

  // 재연결 시에는 소켓만 재생성하면 충분 (메타정보 읽기는 초기 1회만)
  self.m_config.setupMsgCreator();
}

inline void start_watchdog() {
  if (g_watchdog_running.exchange(true)) return;
  mark_rx_now();
  g_watchdog_thread = std::thread([](){
    using namespace std::chrono_literals;
    while (g_watchdog_running.load()) {
      std::this_thread::sleep_for(1s);
      const auto last = g_last_rx_ns.load(std::memory_order_relaxed);
      const auto now  = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::steady_clock::now().time_since_epoch()).count();
      const auto diff_ms = (now - last) / 1'000'000;
      if (diff_ms > kTimeoutSec * 1000) {
        if (g_self) {
          spdlog::warn("No LiDAR data for {} ms (> {} s). Restarting driver...",
                       (long long)diff_ms, kTimeoutSec);
        }
        restart_comm();
      }
    }
  });
}

} // anonymous namespace

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
    spdlog::info("Got parameter: '{}'", ss.str());

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
  // 1) 수신 콜백 래핑: 데이터 들어올 때마다 최근시각 기록
  g_self = this;
  g_wrapped_cb = [callback](const sick::datastructure::Data& d) {
    mark_rx_now();
    if (callback) callback(d);
  };

  // 2) (초기) 통신 설정: 디바이스 재생성(소켓 준비)만 수행
  restart_comm();

  // 3) 초기 1회: 센서 메타정보/설정 읽기
  readTypeCodeSettings();
  readMetadata();
  readFirmwareVersion();
  if (m_config.m_use_pers_conf) {
    readPersistentConfig();
  }
  m_config.setupMsgCreator();  // 메타정보 반영해서 메시지 생성기 구성

  spdlog::info("Communication to Sensor set up");

  // 4) 워치독 시작 (1초마다 체크, 5초 무응답 시 restart_comm())
  start_watchdog();
}

void SickSafetyscanners::stopCommunication() {
  // 워치독 먼저 종료
  stop_watchdog();
  try {
    if (m_device) m_device->stop();
  } catch (const std::exception& e) {
    spdlog::warn("stop() raised during shutdown: {}", e.what());
  } catch (...) {
    spdlog::warn("stop() raised unknown during shutdown");
  }
  m_diagnosed_laser_scan_publisher.reset();
  m_diagnostic_updater.reset();
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
  spdlog::info("Reading Type code settings");
  sick::datastructure::TypeCode type_code;
  m_device->requestTypeCode(type_code);
  m_config.m_communications_settings.e_interface_type =
      type_code.getInterfaceType();
  m_config.m_range_min = 0.1;
  m_config.m_range_max = type_code.getMaxRange();
}

void SickSafetyscanners::readPersistentConfig() {
  spdlog::info("Reading Persistent Configuration");
  sick::datastructure::ConfigData config_data;
  m_device->requestPersistentConfig(config_data);
  m_config.m_communications_settings.start_angle = config_data.getStartAngle();
  m_config.m_communications_settings.end_angle = config_data.getEndAngle();
}

void SickSafetyscanners::readMetadata() {
  spdlog::info("Reading Metadata");
  m_device->requestConfigMetadata(m_config.m_metadata);
}

void SickSafetyscanners::readFirmwareVersion() {
  spdlog::info("Reading firmware version");
  m_device->requestFirmwareVersion(m_config.m_firmware_version);
}

} // namespace sick
