#include "orthopus_ros_interface/hw_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace hardware_interface;

namespace orthopus_ros_interface
{

HwInterfaceComm::HwInterfaceComm(HwInterface& parent)
  : rclcpp::Node(std::string(parent._name+"_comm"))
  , _parent(parent)
{
  // To ros_explorer_bridge
  _expl_ref_pub = this->create_publisher<control_msgs::msg::DynamicJointState>("/explorer_ref", rclcpp::SystemDefaultsQoS());

    // Prepare pub message
  _expl_ref_msg.joint_names.emplace_back(_parent._joint_name);
  _expl_ref_msg.interface_values.resize(1);
  auto& ifv = _expl_ref_msg.interface_values[0];
  for (const auto& intf: {HW_IF_POSITION}) // FIXME! Support more interfaces
  {
    ifv.interface_names.emplace_back(intf);
    ifv.values.emplace_back(std::numeric_limits<double>::quiet_NaN());
  }

  // From ros_explorer_bridge
  _expl_meas_sub = this->create_subscription<control_msgs::msg::DynamicJointState>("/explorer_meas", rclcpp::SystemDefaultsQoS(),
  [this](const std::shared_ptr<control_msgs::msg::DynamicJointState> msg) -> void
  {
    //RCLCPP_INFO(rclcpp::get_logger(get_name()), "Got /explorer_meas");
    const auto& _expl_meas_msg = *(msg.get());
    const auto jsz = _expl_meas_msg.joint_names.size();
    for(size_t i=0;i<jsz;i++)
    {
      auto& jname = _expl_meas_msg.joint_names[i];
      if(jname != _parent._joint_name)
        continue;
      auto& ifv = _expl_meas_msg.interface_values[i];
      const auto& ifsz = ifv.interface_names.size();
      for(size_t j=0;j<ifsz;j++)
      {
        auto& ifval = ifv.values[j];
        const auto& ifn = ifv.interface_names[j];
        if(ifn == HW_IF_POSITION)
          _parent.hw_pos_state_ = ifval;
        // FIXME! Support more interfaces
      }
    }
  });

  // Create a ROS timer for publishing refs
  _expl_ref_pub_tmr = this->create_wall_timer(std::chrono::milliseconds(100), [this](void)
  {
    // Skip if value hasn't changed
    if(!_parent._expl_ref_pub_new)
      return;
    // Pub
    //RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "[%s] Publish !", _parent._name.c_str());
    auto& ifv = _expl_ref_msg.interface_values[0];
    const auto sz = ifv.interface_names.size();
    for (size_t i=0;i<sz;i++)
    {
      const auto& intf_name = ifv.interface_names[i];
      auto& intf_val = ifv.values[i];
      if(intf_name == HW_IF_POSITION)
        intf_val = _parent.hw_pos_command_;
      // FIXME! Support more interfaces
    }
    _parent._expl_ref_pub_new = false;
    _expl_ref_pub->publish(_expl_ref_msg);
  });

}

HwInterface::~HwInterface()
{
  if(_comm_th.joinable())
    _comm_th.join();
}

CallbackReturn HwInterface::on_init(const HardwareInfo& info)
{
  if (ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  hw_pos_state_ = std::numeric_limits<double>::quiet_NaN();
  hw_vel_state_ = std::numeric_limits<double>::quiet_NaN();
  hw_acc_state_ = std::numeric_limits<double>::quiet_NaN();
  hw_eff_state_ = std::numeric_limits<double>::quiet_NaN();
  hw_pos_command_ = std::numeric_limits<double>::quiet_NaN();
  hw_vel_command_ = std::numeric_limits<double>::quiet_NaN();
  hw_acc_command_ = std::numeric_limits<double>::quiet_NaN();
  hw_eff_command_ = std::numeric_limits<double>::quiet_NaN();

  // Hw
  _name = info_.name;

  // Joint
  if(info_.joints.size() > 1)
  {
    RCLCPP_FATAL(rclcpp::get_logger("HwInterface"), "[%s] Can't control more than 1 joint, got %ld", _name.c_str(), info_.joints.size());
    return CallbackReturn::ERROR;
  }
  const auto& joint = info_.joints[0];
  _joint_name = joint.name;

  // Command interfaces
  // _command_interfaces can't be a class item, else build fails on export_command_interfaces
  //_command_interfaces.emplace_back(hardware_interface::CommandInterface(_joint_name, hardware_interface::HW_IF_POSITION, &hw_pos_command_));
  //_command_interfaces.emplace_back(hardware_interface::CommandInterface(_joint_name, hardware_interface::HW_IF_VELOCITY, &hw_vel_command_));
  //_command_interfaces.emplace_back(hardware_interface::CommandInterface(_joint_name, hardware_interface::HW_IF_ACCELERATION, &hw_acc_command_));
  //_command_interfaces.emplace_back(hardware_interface::CommandInterface(_joint_name, hardware_interface::HW_IF_EFFORT, &hw_eff_command_));

  if (joint.command_interfaces.size() > 4)
  {
    RCLCPP_FATAL(rclcpp::get_logger("HwInterface"),"Joint '%s' has %zu command interfaces found. max 4 expected.", joint.name.c_str(),joint.command_interfaces.size());
    return CallbackReturn::ERROR;
  }
  for(const auto& intf: joint.command_interfaces)
  {
    if (intf.name != HW_IF_POSITION && intf.name != HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(rclcpp::get_logger("HwInterface"),"[%s][%s] Unsupported command interface '%s'", _name.c_str(), _joint_name.c_str(), intf.name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  // State interfaces
  _state_interfaces.emplace_back(hardware_interface::StateInterface(_joint_name, hardware_interface::HW_IF_POSITION, &hw_pos_state_));
  _state_interfaces.emplace_back(hardware_interface::StateInterface(_joint_name, hardware_interface::HW_IF_VELOCITY, &hw_vel_state_));
  _state_interfaces.emplace_back(hardware_interface::StateInterface(_joint_name, hardware_interface::HW_IF_ACCELERATION, &hw_acc_state_));
  _state_interfaces.emplace_back(hardware_interface::StateInterface(_joint_name, hardware_interface::HW_IF_EFFORT, &hw_eff_state_));

  if (joint.state_interfaces.size() > _state_interfaces.size())
  {
    RCLCPP_FATAL(rclcpp::get_logger("HwInterface"),"Joint '%s' has %zu state interfaces found. max %ld expected.", joint.name.c_str(),joint.state_interfaces.size(), _state_interfaces.size());
    return CallbackReturn::ERROR;
  }
  for(const auto& intf: joint.state_interfaces)
  {
    if (intf.name != HW_IF_POSITION && intf.name != HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(rclcpp::get_logger("HwInterface"),"[%s][%s] Unsupported state interface '%s'", _name.c_str(), _joint_name.c_str(), intf.name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  // Comms !
  _comm = std::make_shared<HwInterfaceComm>(*this);
  _executor.add_node(_comm);
  _comm_th = std::thread([this]() { this->_executor.spin(); });

  RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "[%s][%s] Successfully initialized!", _name.c_str(), _joint_name.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn HwInterface::on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "[%s] Successfully configured!", _name.c_str());
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> HwInterface::export_state_interfaces()
{
  return _state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HwInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> _command_interfaces;
  _command_interfaces.emplace_back(hardware_interface::CommandInterface(_joint_name, hardware_interface::HW_IF_POSITION, &hw_pos_command_));
  _command_interfaces.emplace_back(hardware_interface::CommandInterface(_joint_name, hardware_interface::HW_IF_VELOCITY, &hw_vel_command_));
  _command_interfaces.emplace_back(hardware_interface::CommandInterface(_joint_name, hardware_interface::HW_IF_ACCELERATION, &hw_acc_command_));
  _command_interfaces.emplace_back(hardware_interface::CommandInterface(_joint_name, hardware_interface::HW_IF_EFFORT, &hw_eff_command_));

  return _command_interfaces;
}

ORTHOPUS_ROS_PUBLIC
return_type HwInterface::prepare_command_mode_switch([[maybe_unused]] const std::vector<std::string>&start_if, [[maybe_unused]] const std::vector<std::string> &stop_if)
{
  RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "[%s] Preparing Command mode switch...", _name.c_str());
  RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "  Starting Interfaces:");
  const auto jnsz = _joint_name.length();
  for(const auto& st_if: start_if)
  {
    if(!st_if.compare(0,jnsz,_joint_name)) // Check only for us
      RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "   - %s",st_if.c_str());
  }
  RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "  Stopping Interfaces:");
  for(const auto& st_if: stop_if)
  {
    if(!st_if.compare(0,jnsz,_joint_name)) // Check only for us
      RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "   - %s",st_if.c_str());
  }
  RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "[%s] Command mode switch prepared!", _name.c_str());
  return return_type::OK;
}

ORTHOPUS_ROS_PUBLIC
return_type HwInterface::perform_command_mode_switch([[maybe_unused]] const std::vector<std::string>&start_if, [[maybe_unused]] const std::vector<std::string> &stop_if)
{
  RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "[%s] Performing Command mode switch...", _name.c_str());
  RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "  Starting Interfaces:");
  const auto jnsz = _joint_name.length();
  for(const auto& st_if: start_if)
  {
    if(!st_if.compare(0,jnsz,_joint_name)) // Check only for us
      RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "   - %s",st_if.c_str());
  }
  RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "  Stopping Interfaces:");
  for(const auto& st_if: stop_if)
  {
    if(!st_if.compare(0,jnsz,_joint_name)) // Check only for us
      RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "   - %s",st_if.c_str());
  }
  RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "[%s] Command mode switch performed!", _name.c_str());
  return return_type::OK;
}

CallbackReturn HwInterface::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "[%s] Activating ...please wait...", _name.c_str());
  if (std::isnan(hw_pos_state_))
  {
    hw_pos_state_ = 0;
    hw_vel_state_ = 0;
    hw_acc_state_ = 0;
    hw_eff_state_ = 0;
    hw_pos_command_ = 0;
    hw_vel_command_ = 0;
    hw_acc_command_ = 0;
    hw_eff_command_ = 0;
  }
  RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "[%s] Successfully activated!", _name.c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn HwInterface::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "[%s] Deactivating ...please wait...", _name.c_str());
  RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "[%s] Successfully deactivated!", _name.c_str());

  return CallbackReturn::SUCCESS;
}

return_type HwInterface::read([[maybe_unused]] const rclcpp::Time& time, [[maybe_unused]] const rclcpp::Duration& period)
{
  // Done in HwInterfaceComm
  return return_type::OK;
}

return_type HwInterface::write([[maybe_unused]] const rclcpp::Time& time, [[maybe_unused]] const rclcpp::Duration& period)
{

  if(!std::isnan(hw_pos_command_) && hw_pos_command_ != prev_hw_pos_command_)
  {
    //RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "[%s] Writing new pos command: %f...", _name.c_str(), hw_pos_command_);
    prev_hw_pos_command_ = hw_pos_command_;
    _expl_ref_pub_new = true;
  }
  if(!std::isnan(hw_vel_command_) && hw_vel_command_ != prev_hw_vel_command_)
  {
    //RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "[%s] Writing new vel command: %f...", _name.c_str(), hw_vel_command_);
    prev_hw_vel_command_ = hw_vel_command_;
  }
  if(!std::isnan(hw_acc_command_) && hw_acc_command_ != prev_hw_acc_command_)
  {
    //RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "[%s] Writing new acc command: %f...", _name.c_str(), hw_acc_command_);
    prev_hw_acc_command_ = hw_acc_command_;
  }
  if(!std::isnan(hw_eff_command_) && hw_eff_command_ != prev_hw_eff_command_)
  {
    //RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "[%s] Writing new eff command: %f...", _name.c_str(), hw_eff_command_);
    prev_hw_eff_command_ = hw_eff_command_;
  }

  return return_type::OK;
}

}  // namespace orthopus_ros_interface


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(orthopus_ros_interface::HwInterface, ActuatorInterface)
