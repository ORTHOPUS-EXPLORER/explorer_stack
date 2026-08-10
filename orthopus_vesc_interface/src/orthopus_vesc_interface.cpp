#include "orthopus_vesc_interface/orthopus_vesc_interface.hpp"

#include <chrono>
#include <sstream>  // for from_str, below

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;
using namespace hardware_interface;

namespace orthopus_ros
{

template <typename T>
T from_str(const std::string& str, const T& def_v)
{
  T out;
  std::istringstream ss(str);
  ss >> out;
  return ss.fail() ? def_v : out;
}

const auto qos_pub = rclcpp::SystemDefaultsQoS();

// See https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/hardware_info.hpp
CallbackReturn VESCInterface::on_init(const HardwareInfo& info)
{
  if (ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;

  name_ = info.name;
  node_ = std::make_unique<rclcpp::Node>(name_);
  // CAN Port
  auto it = info.hardware_parameters.find("can_port");
  if (it == info.hardware_parameters.end() || it->second.empty())
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("VESCInterface"), " Can't spawn VESCHost, can_port is not defined");
    exit(0);
  }
  auto can_port = it->second;
  // Virtual can communication used if can port starts with letter 'v'
  is_virtual_can_used_ = can_port.at(0) == 'v';

  if (!vesc_host_)
  {
    vesc_host_ = orthopus::VESCHost::get_instance();

    if (!vesc_host_)
    {
      spdlog::cfg::load_env_levels();
      // Load parameters
      // Host ID
      it = info.hardware_parameters.find("host_id");
      if (it == info.hardware_parameters.end())
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("VESCInterface"), " Can't spawn VESCHost, host_id is not defined");
        exit(0);
      }
      auto host_id =
        (vescpp::VESC::BoardId)(from_str<unsigned int>(it->second, vescpp::VESC::InvalidBoardId)) &
        0xFF;
      if (host_id == vescpp::VESC::InvalidBoardId)
      {
        RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"), "Invalid host_id in HardwareInfo, abort");
        return CallbackReturn::ERROR;
      }
      // Stream rate
      it = info.hardware_parameters.find("rt_stream_rate");
      if (it == info.hardware_parameters.end())
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("VESCInterface"),
          " Can't spawn VESCHost, rt_stream_rate is not defined");
        exit(0);
      }
      auto rt_stream_rate_hz = from_str<unsigned>(it->second, 250);
      // Aux servo rate
      it = info.hardware_parameters.find("aux_servo_stream_rate");
      if (it == info.hardware_parameters.end())
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("VESCInterface"),
          " Can't spawn VESCHost, aux_servo_stream_rate is not defined");
        exit(0);
      }
      auto aux_servo_stream_rate_hz = from_str<unsigned>(it->second, 50);
      // Aux config rate
      it = info.hardware_parameters.find("aux_config_stream_rate");
      if (it == info.hardware_parameters.end())
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("VESCInterface"),
          " Can't spawn VESCHost, aux_config_stream_rate is not defined");
        exit(0);
      }
      auto aux_config_stream_rate_hz = from_str<unsigned>(it->second, 10);

      RCLCPP_INFO(
        rclcpp::get_logger("VESCInterface"), " => Use CAN port '%s' with Host ID '%d'",
        can_port.c_str(), host_id);
      auto can = std::make_shared<vescpp::comm::CAN>(can_port);
      vesc_host_ = orthopus::VESCHost::spawn_instance(
        host_id, can, rt_stream_rate_hz, aux_servo_stream_rate_hz, aux_config_stream_rate_hz,
        is_virtual_can_used_);

      vesc_host_->scanCAN(true, 100ms);
      RCLCPP_DEBUG(
        rclcpp::get_logger("VESCInterface"), " => Spawn VESCHost: %p", (void*)vesc_host_.get());
    }
  }

  // Read default_mode parameter (optional, defaults to "off" for backward compatibility)
  {
    auto it = info.hardware_parameters.find("default_mode");
    if (it != info.hardware_parameters.end())
    {
      default_mode_ = it->second;
      RCLCPP_INFO(
        rclcpp::get_logger("VESCInterface"), " => Default mode set to '%s'", default_mode_.c_str());
    }
  }

  auto board_id = vescpp::VESC::InvalidBoardId;
  {
    auto it = info.hardware_parameters.find("can_id");
    if (it == info.hardware_parameters.end())
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("VESCInterface"), "'can_id' not found in HardwareInfo, abort");
      return CallbackReturn::ERROR;
    }
    board_id =
      (vescpp::VESC::BoardId)(from_str<unsigned int>(it->second, vescpp::VESC::InvalidBoardId)) &
      0xFF;
  }
  if (board_id == vescpp::VESC::InvalidBoardId)
  {
    RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"), "Invalid 'can_id' in HardwareInfo, abort");
    return CallbackReturn::ERROR;
  }

  // Add target (check firmware version only when using real can communication)
  vesc_dev_ = vesc_host_->add_target(board_id, !is_virtual_can_used_);
  if (!vesc_dev_)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("VESCInterface"), "Timeout waiting for VESC '%d'. Abort", board_id);
    return CallbackReturn::ERROR;
  }
  vesc_dev_->print_hdlr_ = [&](const std::string& s) -> void
  {
    const auto now = rclcpp::Clock().now();

    if (print_buf_duration_.seconds() > 0.0)
    {
      if (now < print_buf_start_ + print_buf_duration_)
      {
        print_buf_.emplace_back(printMsg_t{now, s});
      }
      else
        print_buf_duration_ *= 0.0;
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] <= %s", name_.c_str(), s.c_str());
  };

  RCLCPP_DEBUG(
    rclcpp::get_logger("VESCInterface"), "[%s] Init with BoardID '%d'", name_.c_str(), board_id);
  //spdlog::debug("==> {:np}", spdlog::to_hex(_vesc_dev->fw()->uuid));
  //print_hardware_info(info);

  auto j_sz = vesc_dev_->joints.size();
  if (info_.joints.size() > j_sz)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("VESCInterface"),
      "Target '%d' doesn't have enough Joints. Expected '%ld', got '%ld'. Abort", board_id,
      info.joints.size(), j_sz);
    return CallbackReturn::ERROR;
  }
  j_sz = std::min(j_sz, info_.joints.size());

  // FIXME: Get Joint names from VESC, well build the whole joints map from VESC data.
  //        Not supported in firmware YET
  //
  //        Please note this trick is REALLY FRAGILE !!!
  //        It depends on how the joints are defined in the URDF and how they are added to the
  //        unordered_map (reverse order in which they are written in the .hpp file)
  {
    //for(const auto& j: _vesc_dev->joints)
    //{
    //  RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"),"Target '%d', Joint '%s' found.", board_id, j.name.c_str());
    //  for(const auto& [r, _]:  j.refs)
    //    RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"),"  Ref: %s", r.c_str());
    //}
    auto it = vesc_dev_->joints.begin();
    for (const auto& cfg_j : info.joints)
    {
      it->name = cfg_j.name;
      if (++it == vesc_dev_->joints.end()) break;
    }
    //for(const auto& j: _vesc_dev->joints)
    //{
    //  RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"),"Target '%d', Joint '%s' found.", board_id, j.name.c_str());
    //  for(const auto& [r, _]:  j.refs)
    //    RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"),"  Ref: %s", r.c_str());
    //}
  }

  for (const auto& cfg_j : info.joints)
  {
    auto j = vesc_dev_->get_joint_from_name(cfg_j.name);
    if (j == nullptr)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("VESCInterface"), "Target '%d', Joint '%s' not found. Abort", board_id,
        cfg_j.name.c_str());
      return CallbackReturn::ERROR;
    }

    j->in_use = true;

    //RCLCPP_DEBUG(rclcpp::get_logger("VESCInterface"),"Target '%d', Joint '%s' found.", board_id, j->name.//c_str());
    //for(const auto& [r, _]: j->refs)
    //  RCLCPP_DEBUG(rclcpp::get_logger("VESCInterface"),"  Ref: %s", r.c_str());

    // FIXME: Not ideal: The callback is called directly on new state (ie: from the receiving thread in orthopus_vesc/VESCHost)
    //        DO NOT do anything crazy in there, and consider it "hard RT"-domain
    //        Another approach would be to poll the status in VESCInterface::read() and handle things there.
    // FIXME: Not required for SERVO joint. Also, the capture of j is not that great.
    j->status_changed_cb = [j, this](orthopus::VESCTarget::joint_t& j_data, uint16_t)
    {
      const std::string& sstr = orthopus::state_to_text(j_data.status);
      const std::string& estr = orthopus::error_to_text(j_data.status);
      const std::string& mstr = orthopus::mode_to_text(j_data.status);
      RCLCPP_INFO(
        rclcpp::get_logger("VESCInterface"),
        "[%s] Got State: 0x%04X: State: '%s' Error '%s' Mode '%s'", name_.c_str(), j_data.status,
        sstr.c_str(), estr.c_str(), mstr.c_str());
      if (!state_rtpub_) return;
      state_rtpub_->lock();
      state_rtpub_->msg_.timestamp = rclcpp::Clock().now();
      state_rtpub_->msg_.joint_name = j->name;  // FIXME: Get real joint name

      state_rtpub_->msg_.mode = mstr;
      state_rtpub_->unlockAndPublish();
    };

    for (const auto& cif : cfg_j.command_interfaces)
    {
      const auto it = j->refs.find(cif.name);
      if (it == j->refs.end())
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("VESCInterface"),
          "Target '%d', Joint '%s', Interface '%s' not found in available refs. Abort", board_id,
          j->name.c_str(), cif.name.c_str());
        return CallbackReturn::ERROR;
      }
    }
    for (const auto& sif : cfg_j.state_interfaces)
    {
      auto it = j->meas.find(sif.name);
      if (it == j->meas.end())
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("VESCInterface"),
          "Target '%d', Joint '%s', Interface '%s' not found in available meas. Abort", board_id,
          j->name.c_str(), sif.name.c_str());
        return CallbackReturn::ERROR;
      }
      state_interfaces_.emplace_back(
        hardware_interface::StateInterface(j->name, sif.name, &it->second.v));
    }
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> VESCInterface::export_state_interfaces()
{
  return state_interfaces_;
}

std::vector<hardware_interface::CommandInterface> VESCInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> _command_interfaces;
  for (auto& j : vesc_dev_->joints)
  {
    if (!j.in_use) continue;
    for (auto& [cif_name, cif_v] : j.refs)
    {
      _command_interfaces.emplace_back(j.name, cif_name, &cif_v.v);
    }
  }
  return _command_interfaces;
}

CallbackReturn VESCInterface::on_configure(
  [[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("VESCInterface"), "[on_configure][%s] Successfully configured!",
    name_.c_str());

  dev_srv_ = node_->create_service<orthopus_vesc_interfaces::srv::Dev>(
    "~/dev",
    [this](
      const std::shared_ptr<orthopus_vesc_interfaces::srv::Dev::Request> req,
      std::shared_ptr<orthopus_vesc_interfaces::srv::Dev::Response> resp)
    {
      (void)req;
      RCLCPP_INFO(
        rclcpp::get_logger("VESCInterface"), "VESCHost: %p - %s", (void*)vesc_host_.get(),
        this->name_.c_str());
      resp->help = "Hello World";
    });

  set_mode_srv_ = node_->create_service<orthopus_vesc_interfaces::srv::SetMode>(
    "~/mode",
    [this](
      const std::shared_ptr<orthopus_vesc_interfaces::srv::SetMode::Request> req,
      std::shared_ptr<orthopus_vesc_interfaces::srv::SetMode::Response> resp)
    {
      const auto& j_name = req->joint_name;
      auto j = vesc_dev_->get_joint_from_name(j_name);
      if (j == nullptr)
      {
        RCLCPP_ERROR(
          rclcpp::get_logger("VESCInterface"), "[setMode] Joint %s not found, abort",
          j_name.c_str());
        resp->ret = false;
        return;
      }
      const auto& req_mode = req->mode;

      auto new_ctrl = j->ctrl & ~orthopus::ORTHOPUS_CTRL_MODE_MSK;  // Clear last command
      // FIXME: For each mode, check wether the required interfaces are "in use" (which j->refs["NAME"].in_use) and act accordingly
      if (req_mode == "custom")
      {
        new_ctrl |= orthopus::ORTHOPUS_CTRL_MODE_CST;
        RCLCPP_INFO(
          rclcpp::get_logger("VESCInterface"), "[%s] Switch to CuSTom mode", j_name.c_str());
      }
      else if (req_mode == "impedance")
      {
        new_ctrl |= orthopus::ORTHOPUS_CTRL_MODE_IMP;
        RCLCPP_INFO(
          rclcpp::get_logger("VESCInterface"), "[%s] Switch to IMPedance mode", j_name.c_str());
      }
      else if (req_mode == "effort")
      {
        new_ctrl |= orthopus::ORTHOPUS_CTRL_MODE_TRQ;
        RCLCPP_INFO(
          rclcpp::get_logger("VESCInterface"), "[%s] Switch to ToRQue mode", j_name.c_str());
      }
      else if (req_mode == "velocity")
      {
        new_ctrl |= orthopus::ORTHOPUS_CTRL_MODE_VEL;
        RCLCPP_INFO(
          rclcpp::get_logger("VESCInterface"), "[%s] Switch to VELocity mode", j_name.c_str());
      }
      else if (req_mode == "position")
      {
        new_ctrl |= orthopus::ORTHOPUS_CTRL_MODE_POS;
        RCLCPP_INFO(
          rclcpp::get_logger("VESCInterface"), "[%s] Switch to POSition mode", j_name.c_str());
      }
      else
      {
        new_ctrl |= orthopus::ORTHOPUS_CTRL_MODE_OFF;  // Useless
        RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Switch to OFF mode", j_name.c_str());
      }

      if (new_ctrl != j->ctrl)
      {
        j->ctrl = new_ctrl;
        if (!j->stream)
        {
          RCLCPP_INFO(
            rclcpp::get_logger("VESCInterface"),
            "[%s] Enable stream, with ctrlWord 0x%04x, posMeas: %f posRef: %f", j_name.c_str(),
            j->ctrl, j->meas.at("position").v, j->refs.at("position").v);
          j->stream = true;
        }
        const std::string& state_str = orthopus::state_to_text(j->ctrl);
        const std::string& error_str = orthopus::error_to_text(j->ctrl);
        const std::string& mode_str = orthopus::mode_to_text(j->ctrl);
        RCLCPP_INFO(
          rclcpp::get_logger("VESCInterface"),
          "[%s] Set Ctrl: 0x%04X: State: '%s' Error '%s' Mode '%s'", name_.c_str(), j->ctrl,
          state_str.c_str(), error_str.c_str(), mode_str.c_str());
        resp->ret = true;
      }
    });

  cmd_srv_ = node_->create_service<orthopus_vesc_interfaces::srv::Cmd>(
    "~/command",
    [this](
      const std::shared_ptr<orthopus_vesc_interfaces::srv::Cmd::Request> req,
      std::shared_ptr<orthopus_vesc_interfaces::srv::Cmd::Response> resp)
    {
      const auto wait_ms = std::chrono::milliseconds(req->wait_for_ms);
      //const auto now = vescpp::Time::now();
      print_buf_.clear();

      print_buf_duration_ = rclcpp::Duration(wait_ms);
      print_buf_start_ = rclcpp::Clock().now();
      vesc_dev_->sendCmd(req->cmd, wait_ms);  // It waits an appropriate time for us, so we're good
      for (const auto& s : print_buf_) resp->ret += s.str + "\n";
      print_buf_.clear();
    });

  config_sub_ = node_->create_subscription<orthopus_vesc_interfaces::msg::Config>(
    "~/config", 10, [this](orthopus_vesc_interfaces::msg::Config msg) { callback_config_(msg); });

  state_pub_ = node_->create_publisher<orthopus_vesc_interfaces::msg::State>("~/state", qos_pub);
  state_rtpub_ =
    std::make_unique<realtime_tools::RealtimePublisher<orthopus_vesc_interfaces::msg::State>>(
      state_pub_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn VESCInterface::on_activate(
  [[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  // Wait can data ony when using real can connection / hardware
  if (!is_virtual_can_used_)
  {
    auto can_data_received = wait_can_data_();
    if (can_data_received != CallbackReturn::SUCCESS)
    {
      return can_data_received;
    }
    // Meas are a-okay. Init interfaces refs !
    init_refs_();
  }

  // Apply default mode if specified (skip if "off" for backward compatibility)
  if (default_mode_ != "off")
  {
    for (auto& j : vesc_dev_->joints)
    {
      if (!j.in_use) continue;

      auto new_ctrl = j.ctrl & ~orthopus::ORTHOPUS_CTRL_MODE_MSK;  // Clear current mode

      if (default_mode_ == "position")
        new_ctrl |= orthopus::ORTHOPUS_CTRL_MODE_POS;
      else if (default_mode_ == "velocity")
        new_ctrl |= orthopus::ORTHOPUS_CTRL_MODE_VEL;
      else if (default_mode_ == "effort")
        new_ctrl |= orthopus::ORTHOPUS_CTRL_MODE_TRQ;
      else if (default_mode_ == "impedance")
        new_ctrl |= orthopus::ORTHOPUS_CTRL_MODE_IMP;
      else if (default_mode_ == "custom")
        new_ctrl |= orthopus::ORTHOPUS_CTRL_MODE_CST;

      if (new_ctrl != j.ctrl)
      {
        j.ctrl = new_ctrl;
        if (!j.stream)
        {
          j.stream = true;
        }
        RCLCPP_INFO(
          rclcpp::get_logger("VESCInterface"),
          "[on_activate][%s] Set default mode '%s' (ctrl: 0x%04X)", j.name.c_str(),
          default_mode_.c_str(), j.ctrl);
      }
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn VESCInterface::on_deactivate(
  [[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  //RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Deactivating ...please wait...", name_.c_str());
  //RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Successfully deactivated!", name_.c_str());
  return CallbackReturn::SUCCESS;
}

ORTHOPUS_ROS_PUBLIC
return_type VESCInterface::prepare_command_mode_switch(
  [[maybe_unused]] const std::vector<std::string>& start_if,
  [[maybe_unused]] const std::vector<std::string>& stop_if)
{
  /*RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Preparing Command mode switch...", name_.c_str());
  RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "  Stopping Interfaces:");
  // FIXME: Check that we can actually perform the requested command mode switch
  RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Command mode switch prepared!", name_.c_str());
  */
  return return_type::OK;
}

ORTHOPUS_ROS_PUBLIC
return_type VESCInterface::perform_command_mode_switch(
  [[maybe_unused]] const std::vector<std::string>& start_if,
  [[maybe_unused]] const std::vector<std::string>& stop_if)
{
  auto enable_intf = [this](const std::vector<std::string>& _ifs, bool enable)
  {
    for (const auto& st_if : _ifs)
    {
      for (auto& j : this->vesc_dev_->joints)
      {
        // st_if format is JOINT_NAME/INTERFACE
        if (
          auto idx = st_if.find(j.name, 0);
          idx == 0)  // Match beginning of the name with joint name
        {
          // FIXME: Check st_if length first
          const auto& intf =
            st_if.substr(j.name.length() + 1);  // Then match the end with supported interfaces
          if (auto it = j.refs.find(intf); it != j.refs.end())
          {
            it->second.in_use = enable;
            RCLCPP_DEBUG(rclcpp::get_logger("VESCInterface"), "   - %s", st_if.c_str());
          }
        }
      }
    }
  };

  RCLCPP_INFO(
    rclcpp::get_logger("VESCInterface"), "[%s] Performing Command mode switch...", name_.c_str());
  RCLCPP_DEBUG(rclcpp::get_logger("VESCInterface"), "  Stopping Interfaces:");
  enable_intf(stop_if, false);
  RCLCPP_DEBUG(rclcpp::get_logger("VESCInterface"), "  Starting Interfaces:");
  enable_intf(start_if, true);
  RCLCPP_INFO(
    rclcpp::get_logger("VESCInterface"), "[%s] Command mode switch performed!", name_.c_str());
  return return_type::OK;
}

return_type VESCInterface::read(
  [[maybe_unused]] const rclcpp::Time& time, [[maybe_unused]] const rclcpp::Duration& period)
{
  // Async, Measures are streamed by the devices, directly to orthopus::VESCTarget
  // TODO: Sanity checks (trigger error if delay since last meas reached a timeout for instance)
  // TODO: Make sure spin_some does not slow down the RT loop (event when processing srv/pub/sub/...)
  if (node_ && rclcpp::ok()) rclcpp::spin_some(node_->get_node_base_interface());
  return return_type::OK;
}

return_type VESCInterface::write(
  [[maybe_unused]] const rclcpp::Time& time, [[maybe_unused]] const rclcpp::Duration& period)
{
  // Async, Refs are sent in another Thread, managed by orthopus::VESCHost
  // TODO: Sanity checks: Make sure the refs are not completely out of range, for instance
  return return_type::OK;
}

void VESCInterface::print_parameters_(const std::unordered_map<std::string, std::string>& params)
{
  const auto& log = rclcpp::get_logger("VESCInterface");
  for (const auto& [name, v] : params)
  {
    RCLCPP_INFO(log, "        - '%s': '%s'", name.c_str(), v.c_str());
  }
}

void VESCInterface::print_interface_info_(const InterfaceInfo& info, size_t i)
{
  const auto& log = rclcpp::get_logger("VESCInterface");
  RCLCPP_INFO(log, "        - %ld", i);
  RCLCPP_INFO(log, "          Name: '%s'", info.name.c_str());
  RCLCPP_INFO(log, "          Min : '%s'", info.min.c_str());
  RCLCPP_INFO(log, "          Max : '%s'", info.max.c_str());
  RCLCPP_INFO(log, "          Init: '%s'", info.initial_value.c_str());
  RCLCPP_INFO(log, "          Type: '%s'", info.data_type.c_str());
  RCLCPP_INFO(log, "          Size: '%d'", info.size);
}
void VESCInterface::print_transmission_info_(const TransmissionInfo& info, size_t i)
{
  const auto& log = rclcpp::get_logger("VESCInterface");
  RCLCPP_INFO(log, "        - %ld", i);
  RCLCPP_INFO(log, "          Name: '%s'", info.name.c_str());
  RCLCPP_INFO(log, "          Type: '%s'", info.type.c_str());
  RCLCPP_INFO(log, "          Joints: TODO");
  RCLCPP_INFO(log, "          Actuators: TODO");
  RCLCPP_INFO(log, "          Parameters:");
  print_parameters_(info.parameters);
}

void VESCInterface::print_component_info_(const ComponentInfo& info, size_t i)
{
  const auto& log = rclcpp::get_logger("VESCInterface");
  RCLCPP_INFO(log, "    - %ld", i);
  RCLCPP_INFO(log, "      Name  : '%s'", info.name.c_str());
  RCLCPP_INFO(log, "      Type  : '%s'", info.type.c_str());
  RCLCPP_INFO(log, "      Command Interfaces:");
  size_t j = 0;
  for (const auto& intf : info.command_interfaces) print_interface_info_(intf, j++);
  RCLCPP_INFO(log, "      State Interfaces:");
  j = 0;
  for (const auto& intf : info.state_interfaces) print_interface_info_(intf, j++);
  RCLCPP_INFO(log, "      Parameters:");
  print_parameters_(info.parameters);
};

void VESCInterface::print_hardware_info_(const HardwareInfo& info)
{
  const auto& log = rclcpp::get_logger("VESCInterface");
  RCLCPP_INFO(log, "Hardware Info for: '%s'", info.name.c_str());
  RCLCPP_INFO(log, "  Hardware parameters:");
  print_parameters_(info.hardware_parameters);
  size_t i = 0;
  RCLCPP_INFO(log, "  Joints:");
  for (auto& joint : info.joints) print_component_info_(joint, i++);
  i = 0;
  RCLCPP_INFO(log, "  Sensors:");
  for (auto& sensors : info.sensors) print_component_info_(sensors, i++);
  i = 0;
  RCLCPP_INFO(log, "  GPIOs:");
  for (auto& gpio : info.gpios) print_component_info_(gpio, i++);
  i = 0;
  RCLCPP_INFO(log, "  Transmissions:");
  for (auto& transmission : info.transmissions) print_transmission_info_(transmission, i++);
}

void VESCInterface::callback_config_(const orthopus_vesc_interfaces::msg::Config& msg)
{
  vesc_dev_->acquire_joint().impedance_control_damping = msg.impedance_control_damping;
  vesc_dev_->acquire_joint().impedance_control_stiffness = msg.impedance_control_stiffness;
}

CallbackReturn VESCInterface::wait_can_data_()
{
  const auto now = vescpp::Time::now();
  while (true)
  {
    if ((now - vesc_dev_->_meas_last_tp) < 10ms)
    {
      //RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Got a fresh Meas! Marching on !",name_.c_str());
      break;
    }
    else if (vescpp::Time::now() - now >= 1000ms)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("VESCInterface"),
        "[on_activate][%s] Timeout waiting for a fresh Meas. Abort,", name_.c_str());
      return CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(10us);
  }
  return CallbackReturn::SUCCESS;
}

void VESCInterface::init_refs_()
{
  // FIXME: What do we do with servo, since it does not have any pos streaming ? Currently init at 0.5 from orthopus::VESCTarget
  auto& j = vesc_dev_->acquire_joint();
  for (auto& [ifn, if_v] : j.refs)
  {
    if_v.v = 0.0;         // 0.0 is the default
    if_v.in_use = false;  // Free the interface
    // FIXME: Keep this ?
    if (ifn == "position")
    {
      // Set init value to measure only when a meas was received (if so, in_use is true)
      if (auto it = j.meas.find(ifn); it != j.meas.end() && it->second.in_use)
      {
        if_v.v = it->second.v;  // Set ref to last/current position
        RCLCPP_INFO(
          rclcpp::get_logger("VESCInterface"),
          "[on_activate][%s][%s] Init POS ref with value: % 7.4f", name_.c_str(), j.name.c_str(),
          if_v.v);
      }
    }
  }
}

}  // namespace orthopus_ros

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(orthopus_ros::VESCInterface, ActuatorInterface)
