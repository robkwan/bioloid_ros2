// Copyright 2020 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "dynamixel_hardware/dynamixel_hardware.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <string>
#include <vector>
#include <thread>
#include <chrono>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
//#include "rclcpp/rclcpp.hpp"

#define MOVING_SPEED 100  // Further reduce to 100 from 300 Reduce the moving speed from 0 (Max speed) to 1/3 of it, 300 (33.3 rpm) 

namespace dynamixel_hardware
{
constexpr const char * kDynamixelHardware = "DynamixelHardware";
constexpr uint8_t kGoalPositionIndex = 0;
constexpr uint8_t kGoalVelocityIndex = 1;
constexpr uint8_t kPresentPositionIndex = 0;
constexpr uint8_t kPresentVelocityIndex = 1;
constexpr uint8_t kPresentCurrentIndex = 2;

void DynamixelHardware::start_service_executor()
{
  if (!node_) {
    RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware),
                "start_service_executor() called without node_");
    return;
  }
  if (executor_) {
    return;  // already running
  }

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  executor_running_.store(true);

  executor_thread_ = std::thread([this]() {
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware),
                "Service executor thread started");
    while (executor_running_.load() && rclcpp::ok()) {
      executor_->spin_some(std::chrono::milliseconds(100));
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware),
                "Service executor thread stopped");
  });
}

void DynamixelHardware::stop_service_executor()
{
  if (!executor_) return;
  executor_running_.store(false);
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
  try {
    executor_->remove_node(node_);
  } catch (...) {
    // ignore if remove_node not supported
  }
  executor_.reset();
}

DynamixelHardware::~DynamixelHardware()
{
  stop_service_executor();
}

CallbackReturn DynamixelHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "configure");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  joints_.resize(info_.joints.size(), Joint());
  joint_ids_.resize(info_.joints.size(), 0);

  for (uint i = 0; i < info_.joints.size(); i++) {
    joint_ids_[i] = std::stoi(info_.joints[i].parameters.at("id"));
    joints_[i].state.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].state.effort = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.position = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.velocity = std::numeric_limits<double>::quiet_NaN();
    joints_[i].command.effort = std::numeric_limits<double>::quiet_NaN();
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "joint_id %d: %d", i, joint_ids_[i]);
  }

  if (
    info_.hardware_parameters.find("usb_port") != info_.hardware_parameters.end()) {
    usb_port_ = info_.hardware_parameters.at("usb_port");
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "usb_port: %s", usb_port_.c_str());
  } else if (
    info_.hardware_parameters.find("port") != info_.hardware_parameters.end()) {
    usb_port_ = info_.hardware_parameters.at("port");
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "port: %s", usb_port_.c_str());
  } else {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "usb_port or port parameter is required");
    return CallbackReturn::ERROR;
  }

  if (
    info_.hardware_parameters.find("baud_rate") != info_.hardware_parameters.end()) {
    baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "baud_rate: %d", baud_rate_);
  } else {
    baud_rate_ = 57600;
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "baud_rate: %d (default)", baud_rate_);
  }

  // Check for protocol version parameter
  use_protocol_1_0_ = false;
  if (info_.hardware_parameters.find("protocol_version") != info_.hardware_parameters.end()) {
    std::string protocol_version = info_.hardware_parameters["protocol_version"];
    if (protocol_version == "1.0" || protocol_version == "1") {
      use_protocol_1_0_ = true;
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Using Protocol 1.0 mode (8-bit checksum)");
    } else {
      RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Using Protocol 2.0 mode (16-bit CRC)");
    }
  } else {
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "No protocol_version specified, using Protocol 2.0 (16-bit CRC, default)");
  }

  use_dummy_ = false;
  if (
    info_.hardware_parameters.find("use_dummy") != info_.hardware_parameters.end() &&
    info_.hardware_parameters.at("use_dummy") == "true") {
    use_dummy_ = true;
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "dummy mode");
    return CallbackReturn::SUCCESS;
  }

  using namespace std::placeholders;

  if (!node_) {
    node_ = std::make_shared<rclcpp::Node>("dynamixel_hardware_services");
  }

  // allow override via parameters (optional)
  std::string get_srv_name = "/dynamixel_hardwareInterface/get_dxl_data";
  std::string set_srv_name = "/dynamixel_hardwareInterface/set_dxl_data";
  if (info_.hardware_parameters.find("get_dynamixel_data_srv_name") != info_.hardware_parameters.end()) {
    get_srv_name = info_.hardware_parameters.at("get_dynamixel_data_srv_name");
  }
  if (info_.hardware_parameters.find("set_dynamixel_data_srv_name") != info_.hardware_parameters.end()) {
    set_srv_name = info_.hardware_parameters.at("set_dynamixel_data_srv_name");
  }

  // create services (these callbacks will use dynamixel_workbench_.itemRead / itemWrite)
  get_dxl_data_srv_ = node_->create_service<dynamixel_interfaces::srv::GetDataFromDxl>(
    get_srv_name,
    std::bind(&DynamixelHardware::get_dxl_data_srv_callback, this,
              std::placeholders::_1, std::placeholders::_2));

  set_dxl_data_srv_ = node_->create_service<dynamixel_interfaces::srv::SetDataToDxl>(
    set_srv_name,
    std::bind(&DynamixelHardware::set_dxl_data_srv_callback, this,
              std::placeholders::_1, std::placeholders::_2));

  // Start background executor thread so services can be processed
  start_service_executor();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DynamixelHardware::export_state_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].state.position));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].state.velocity));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joints_[i].state.effort));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DynamixelHardware::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), "export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].command.position));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].command.velocity));
  }

  return command_interfaces;
}

CallbackReturn DynamixelHardware::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "configure");
  if (use_dummy_) {
    return CallbackReturn::SUCCESS;
  }

  const char * log = nullptr;

  // Initialize workbench with appropriate protocol
  //float protocol_version = use_protocol_1_0_ ? 1.0f : 2.0f;
  if (!dynamixel_workbench_.init(usb_port_.c_str(), baud_rate_, &log)) {
    RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "Failed to init : %s", log);
    return CallbackReturn::ERROR;
  }

  for (uint i = 0; i < info_.joints.size(); ++i) {
    uint8_t dxl_id = joint_ids_[i];
    bool result = false;

    // Ping the servo
    uint16_t model_number = 0;
    result = dynamixel_workbench_.ping(dxl_id, &model_number, &log);
    if (result == false) {
      RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "Failed to ping : %s", log);
      return CallbackReturn::ERROR;
    }

    // Set joint mode based on protocol version
    if (use_protocol_1_0_) {
      // For Protocol 1.0 (AX-12A), just set to joint mode (position control)
      result = dynamixel_workbench_.jointMode(dxl_id, MOVING_SPEED, 0, &log);
    } else {
      // For Protocol 2.0, use position control mode
      result = dynamixel_workbench_.setPositionControlMode(dxl_id, &log);
    }

    if (result == false) {
      RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "Failed to set position control mode : %s", log);
      return CallbackReturn::ERROR;
    }

    // Add sync write items for Protocol 2.0
    if (!use_protocol_1_0_) {
      result = dynamixel_workbench_.addSyncWriteHandler(dxl_id, "Goal_Position", &log);
      if (result == false) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "Failed to add sync write handler : %s", log);
        return CallbackReturn::ERROR;
      }

      result = dynamixel_workbench_.addSyncReadHandler(dxl_id,
                                                        "Present_Position",
                                                        &log);
      if (result == false) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "Failed to add sync read handler : %s", log);
        return CallbackReturn::ERROR;
      }

      result = dynamixel_workbench_.addSyncReadHandler(dxl_id,
                                                        "Present_Velocity",
                                                        &log);
      if (result == false) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "Failed to add sync read handler : %s", log);
        return CallbackReturn::ERROR;
      }

      result = dynamixel_workbench_.addSyncReadHandler(dxl_id,
                                                        "Present_Current",
                                                        &log);
      if (result == false) {
        RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "Failed to add sync read handler : %s", log);
        return CallbackReturn::ERROR;
      }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Successful configuring");
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "activate");
  if (use_dummy_) {
    for (uint i = 0; i < info_.joints.size(); i++) {
      joints_[i].state.position = 0.0;
      joints_[i].state.velocity = 0.0;
      joints_[i].state.effort = 0.0;
      joints_[i].command.position = 0.0;
    }
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Successful dummy activating: %d", use_dummy_);
    return CallbackReturn::SUCCESS;
  }

  const char * log = nullptr;

  // Enable torque for all servos
  for (uint i = 0; i < info_.joints.size(); ++i) {
    uint8_t dxl_id = joint_ids_[i];
    bool result = dynamixel_workbench_.torqueOn(dxl_id, &log);
    if (result == false) {
      RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "Failed to enable torque : %s", log);
      return CallbackReturn::ERROR;
    }

    // Initialize command with current position
    int32_t present_position = 0;
    result = dynamixel_workbench_.itemRead(dxl_id, "Present_Position", &present_position, &log);
    if (result == false) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "Failed to read present position : %s", log);
      joints_[i].command.position = 0.0;
    } else {
      joints_[i].command.position = dynamixel_workbench_.convertValue2Radian(dxl_id, present_position);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Successful activating");
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "deactivate");
  stop_service_executor();
  if (use_dummy_) {
    RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Successful dummy deactivating");
    return CallbackReturn::SUCCESS;
  }

  const char * log = nullptr;

  // Disable torque for all servos
  for (uint i = 0; i < info_.joints.size(); ++i) {
    uint8_t dxl_id = joint_ids_[i];
    bool result = dynamixel_workbench_.torqueOff(dxl_id, &log);
    if (result == false) {
      RCLCPP_FATAL(rclcpp::get_logger(kDynamixelHardware), "Failed to disable torque : %s", log);
      return CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware), "Successful deactivating");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DynamixelHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  /*if (!dxl_request_queue_.empty()) {
    use_dummy_ = true;
  } else {
    use_dummy_ = false;
  }*/

  // Try to lock with simple try_lock
  std::unique_lock<std::timed_mutex> lock(service_mutex_, std::defer_lock);
  if (!lock.try_lock_for(std::chrono::milliseconds(10))) {
    RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), 
                 "Read skipped - service call in progress");
    return hardware_interface::return_type::OK;
  }

  if (use_dummy_) {
    return hardware_interface::return_type::OK;
  }

  if (use_protocol_1_0_) {
    return read_protocol_1_0();
  } else {
    return read_protocol_2_0();
  }
}

hardware_interface::return_type DynamixelHardware::read_protocol_1_0()
{
  const char * log = nullptr;

  for (uint i = 0; i < info_.joints.size(); ++i) {
    uint8_t dxl_id = joint_ids_[i];
    int32_t present_position = 0;
    int32_t present_velocity = 0;
    int32_t present_current = 0;

    // Read present position
    bool result = dynamixel_workbench_.itemRead(dxl_id, "Present_Position", &present_position, &log);
    if (result == false) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), 
        "Failed to read present position for joint %d: %s", dxl_id, log);
      continue;
    }

    // Read present velocity (speed)
    result = dynamixel_workbench_.itemRead(dxl_id, "Present_Speed", &present_velocity, &log);
    if (result == false) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), 
        "Failed to read present velocity for joint %d: %s", dxl_id, log);
      present_velocity = 0;  // Set to 0 if read fails
    }

    // For Protocol 1.0, we can read present load as effort approximation
    result = dynamixel_workbench_.itemRead(dxl_id, "Present_Load", &present_current, &log);
    if (result == false) {
      present_current = 0;  // Set to 0 if read fails
    }

    // Convert values to standard units
    joints_[i].state.position = dynamixel_workbench_.convertValue2Radian(dxl_id, present_position);
    joints_[i].state.velocity = dynamixel_workbench_.convertValue2Velocity(dxl_id, present_velocity);
    joints_[i].state.effort = present_current * 0.001;  // Approximate conversion for load

    // Add small delay for Protocol 1.0 timing
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynamixelHardware::read_protocol_2_0()
{
  std::vector<uint8_t> ids(info_.joints.size(), 0);
  std::vector<int32_t> positions(info_.joints.size(), 0);
  std::vector<int32_t> velocities(info_.joints.size(), 0);
  std::vector<int32_t> currents(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());

  const char * log = nullptr;

  if (!dynamixel_workbench_.getSyncReadData(
        kPresentPositionIndex, &ids[0], ids.size(), &positions[0], &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "groupSyncRead getdata failed: %s", log);
  }

  if (!dynamixel_workbench_.getSyncReadData(
        kPresentVelocityIndex, &ids[0], ids.size(), &velocities[0], &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "groupSyncRead getdata failed: %s", log);
  }

  if (!dynamixel_workbench_.getSyncReadData(
        kPresentCurrentIndex, &ids[0], ids.size(), &currents[0], &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "groupSyncRead getdata failed: %s", log);
  }

  for (uint i = 0; i < ids.size(); ++i) {
    joints_[i].state.position = dynamixel_workbench_.convertValue2Radian(ids[i], positions[i]);
    joints_[i].state.velocity = dynamixel_workbench_.convertValue2Velocity(ids[i], velocities[i]);
    joints_[i].state.effort = dynamixel_workbench_.convertValue2Current(currents[i]);
  }

  return hardware_interface::return_type::OK;
}



hardware_interface::return_type DynamixelHardware::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{

  /*if (!dxl_request_queue_.empty()) {
    use_dummy_ = true;
  } else {
    use_dummy_ = false;
  }*/

  // Try to lock with simple try_lock
  std::unique_lock<std::timed_mutex> lock(service_mutex_, std::defer_lock);
  if (!lock.try_lock_for(std::chrono::milliseconds(10))) {
    RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware), 
                 "Write skipped - service call in progress");
    return hardware_interface::return_type::OK;
  }

  if (use_dummy_) {
    for (auto i = 0ul; i < info_.joints.size(); i++) {
      joints_[i].state.position = joints_[i].command.position;
    }
    return hardware_interface::return_type::OK;
  }

  if (use_protocol_1_0_) {
    return write_protocol_1_0();
  } else {
    return write_protocol_2_0();
  }


}

hardware_interface::return_type DynamixelHardware::write_protocol_1_0()
{
  const char * log = nullptr;

  for (uint i = 0; i < info_.joints.size(); ++i) {
    uint8_t dxl_id = joint_ids_[i];
    
    // Convert radian command to servo value
    int32_t goal_position = dynamixel_workbench_.convertRadian2Value(dxl_id, joints_[i].command.position);

    // Write goal position individually for each joint
    bool result = dynamixel_workbench_.itemWrite(dxl_id, "Goal_Position", goal_position, &log);
    if (result == false) {
      RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), 
        "Failed to set goal position for joint %d: %s", dxl_id, log);
      continue;
    }

    // Add small delay for Protocol 1.0 timing
    std::this_thread::sleep_for(std::chrono::microseconds(500));
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynamixelHardware::write_protocol_2_0()
{
  std::vector<uint8_t> ids(info_.joints.size(), 0);
  std::vector<int32_t> commands(info_.joints.size(), 0);

  std::copy(joint_ids_.begin(), joint_ids_.end(), ids.begin());

  for (uint i = 0; i < ids.size(); ++i) {
    commands[i] = dynamixel_workbench_.convertRadian2Value(ids[i], joints_[i].command.position);
  }

  const char * log = nullptr;
  if (!dynamixel_workbench_.syncWrite(kGoalPositionIndex, &ids[0], ids.size(), &commands[0], 1, &log)) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), "groupSyncWrite addparam failed: %s", log);
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

void DynamixelHardware::get_dxl_data_srv_callback(
  const std::shared_ptr<dynamixel_interfaces::srv::GetDataFromDxl::Request> request,
  std::shared_ptr<dynamixel_interfaces::srv::GetDataFromDxl::Response> response)
{
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware),
              "get_dxl_data_srv_callback called: id=%u item=%s",
              request->id, request->item_name.c_str());

  // Lock to prevent control loop interference
  std::lock_guard<std::timed_mutex> lock(service_mutex_);
  
  int32_t data = 0;
  const char* log = nullptr;
  bool success = dynamixel_workbench_.itemRead(request->id, request->item_name.c_str(), &data, &log);
  
  if (success) {
    response->item_data = data;
    response->result = true;
    RCLCPP_DEBUG(rclcpp::get_logger(kDynamixelHardware),
                 "Service read success: id=%u item=%s value=%d",
                 request->id, request->item_name.c_str(), data);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware), 
                 "Failed to read %s from ID %d: %s", 
                 request->item_name.c_str(), request->id, log ? log : "unknown error");
    response->item_data = 0;
    response->result = false;
  }
}

void DynamixelHardware::set_dxl_data_srv_callback(
  const std::shared_ptr<dynamixel_interfaces::srv::SetDataToDxl::Request> request,
  std::shared_ptr<dynamixel_interfaces::srv::SetDataToDxl::Response> response)
{
  RCLCPP_INFO(rclcpp::get_logger(kDynamixelHardware),
              "set_dxl_data_srv_callback called: id=%u item=%s value=%u",
              request->id, request->item_name.c_str(), request->item_data);

  // Try to acquire service_mutex_ with retry
  std::unique_lock<std::timed_mutex> lock(service_mutex_, std::defer_lock);

  bool got_lock = false;
  for (int retries = 0; retries < 10; ++retries) {  // retry up to ~1000ms total
    if (lock.try_lock_for(std::chrono::milliseconds(100))) {
      got_lock = true;
      break;
    }
    RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware),
                "Port busy, retrying write for ID %u...", request->id);
  }

  if (!got_lock) {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware),
                 "Timeout: could not acquire port lock for ID %u", request->id);
    response->result = false;
    return;
  }

  const char* log = nullptr;
  bool success = dynamixel_workbench_.itemWrite(request->id, request->item_name.c_str(), request->item_data, &log);
  
  if (success) {
    // Add a small delay to allow EEPROM/RAM to settle
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    int32_t verify_data = 0;
    bool verify_ok = dynamixel_workbench_.itemRead(
      request->id, request->item_name.c_str(),
      &verify_data, &log);

    if (verify_ok && verify_data == request->item_data) {
      response->result = true;
    } else {
      RCLCPP_WARN(rclcpp::get_logger(kDynamixelHardware),
        "%s write verification failed for ID %d: expected %d, got %d",
        request->item_name.c_str(), request->id,
        request->item_data, verify_data);
        response->result = false;
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(kDynamixelHardware),
      "Failed to write %s=%d to ID %d",
      request->item_name.c_str(),
      request->item_data, request->id);
      response->result = false;
    }
}

}  // namespace dynamixel_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynamixel_hardware::DynamixelHardware, hardware_interface::SystemInterface)

