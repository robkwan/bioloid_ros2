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

#ifndef DYNAMIXEL_HARDWARE__DYNAMIXEL_HARDWARE_HPP_
#define DYNAMIXEL_HARDWARE__DYNAMIXEL_HARDWARE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <atomic>
#include <thread>


#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "dynamixel_interfaces/srv/get_data_from_dxl.hpp"
#include "dynamixel_interfaces/srv/set_data_to_dxl.hpp"

#include "std_srvs/srv/set_bool.hpp"

#include <queue>
#include <mutex>
#include <future>

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

struct DxlRequest
{
    uint8_t id;
    std::string item_name;
//    int32_t data;  // For set requests
//    bool is_set;    // true=set, false=get
//    std::promise<bool> done;
};

namespace dynamixel_hardware
{
struct Joint
{
  double command_position;
  double state_position;
  double state_velocity;
  double state_effort;
};

class DynamixelHardware : public hardware_interface::SystemInterface
{
public:
  ~DynamixelHardware();
  
  RCLCPP_SHARED_PTR_DEFINITIONS(DynamixelHardware)

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Joint
  {
    struct State
    {
      double position{0.0};
      double velocity{0.0};
      double effort{0.0};
    };
    
    struct Command
    {
      double position{0.0};
      double velocity{0.0};
      double effort{0.0};
    };
    
    State state;
    Command command;
  };

  DynamixelWorkbench dynamixel_workbench_;
  std::map<const char * const, const ControlItem *> control_items_;
  std::vector<Joint> joints_;
  std::vector<uint8_t> joint_ids_;
  bool use_dummy_;
  std::string usb_port_;
  int baud_rate_;
 
  // Protocol 1.0 support
  bool use_protocol_1_0_;
  
  // Protocol-specific methods
  return_type read_protocol_1_0();
  return_type read_protocol_2_0();
  return_type write_protocol_1_0();
  return_type write_protocol_2_0();

  std::timed_mutex service_mutex_;  // Protect against control loop interference
  //std::queue<DxlRequest> dxl_request_queue_;
  std::mutex dxl_queue_mutex_;
  std::deque<DxlRequest> dxl_request_queue_;
  std::unordered_map<uint8_t, int32_t> dxl_last_read_;  // store last read values

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Service<dynamixel_interfaces::srv::GetDataFromDxl>::SharedPtr get_dxl_data_srv_;
  rclcpp::Service<dynamixel_interfaces::srv::SetDataToDxl>::SharedPtr set_dxl_data_srv_;

  void get_dxl_data_srv_callback(
    const std::shared_ptr<dynamixel_interfaces::srv::GetDataFromDxl::Request> request,
    std::shared_ptr<dynamixel_interfaces::srv::GetDataFromDxl::Response> response);

  void set_dxl_data_srv_callback(
    const std::shared_ptr<dynamixel_interfaces::srv::SetDataToDxl::Request> request,
    std::shared_ptr<dynamixel_interfaces::srv::SetDataToDxl::Response> response);
  
  // Executor + thread for spinning the service node
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
  std::atomic_bool executor_running_{false};

  // Helpers to manage executor lifecycle
  void start_service_executor();
  void stop_service_executor();
    
};

}  // namespace dynamixel_hardware

#endif  // DYNAMIXEL_HARDWARE__DYNAMIXEL_HARDWARE_HPP_

