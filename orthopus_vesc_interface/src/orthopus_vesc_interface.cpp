#include "orthopus_vesc_interface/orthopus_vesc_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace hardware_interface;

namespace orthopus_ros
{

VESCInterface::~VESCInterface()
{

}

void VESCInterface::printParameters(const std::unordered_map<std::string, std::string>& params)
{
  const auto& log = rclcpp::get_logger("VESCInterface");
  size_t j=0;
  for(const auto& [name,v]: params)
  {
    RCLCPP_INFO(log,  "        - '%s': '%s'", name.c_str(), v.c_str());
    j++;
  }
}

void VESCInterface::printInterfaceInfo(const InterfaceInfo& info, size_t i)
{
  const auto& log = rclcpp::get_logger("VESCInterface");
  RCLCPP_INFO(log,  "        - %ld",i);
  RCLCPP_INFO(log,  "          Name: '%s'", info.name.c_str());
  RCLCPP_INFO(log,  "          Min : '%s'", info.min.c_str());
  RCLCPP_INFO(log,  "          Max : '%s'", info.max.c_str());
  RCLCPP_INFO(log,  "          Init: '%s'", info.initial_value.c_str());
  RCLCPP_INFO(log,  "          Type: '%s'", info.data_type.c_str());
  RCLCPP_INFO(log,  "          Size: '%d'", info.size);
}
void VESCInterface::printTransmissionInfo(const TransmissionInfo& info, size_t i)
{
  const auto& log = rclcpp::get_logger("VESCInterface");
  RCLCPP_INFO(log,  "        - %ld",i);
  RCLCPP_INFO(log,  "          Name: '%s'", info.name.c_str());
  RCLCPP_INFO(log,  "          Type: '%s'", info.type.c_str());
  RCLCPP_INFO(log,  "          Joints: TODO");
  RCLCPP_INFO(log,  "          Actuators: TODO");
  RCLCPP_INFO(log,  "          Parameters:");
  printParameters(info.parameters);
}

void VESCInterface::printComponentInfo(const ComponentInfo& info, size_t i)
{
  const auto& log = rclcpp::get_logger("VESCInterface");
  RCLCPP_INFO(log,    "    - %ld",i);
  RCLCPP_INFO(log,    "      Name  : '%s'", info.name.c_str()); 
  RCLCPP_INFO(log,    "      Type  : '%s'", info.type.c_str()); 
  RCLCPP_INFO(log,    "      Command Interfaces:"); 
  size_t j=0;
  for(const auto& intf: info.command_interfaces)
    printInterfaceInfo(intf, j++);
  RCLCPP_INFO(log,    "      State Interfaces:"); 
  j=0;
  for(const auto& intf: info.command_interfaces)
    printInterfaceInfo(intf, j++);
  RCLCPP_INFO(log,    "      Parameters:"); 
  printParameters(info.parameters);

};

void VESCInterface::printHardwareInfo(const HardwareInfo& info)
{
  const auto& log = rclcpp::get_logger("VESCInterface");
  RCLCPP_INFO(log,      "Hardware Info for: '%s'", info.name.c_str());
  RCLCPP_INFO(log,      "  Hardware parameters:");
  printParameters(info.hardware_parameters);
  size_t i=0;
  RCLCPP_INFO(log,      "  Joints:");
  for(auto& joint: info.joints)
    printComponentInfo(joint, i++);
  i=0;
  RCLCPP_INFO(log,      "  Sensors:");
  for(auto& sensors: info.sensors)
    printComponentInfo(sensors, i++);
  i=0;
  RCLCPP_INFO(log,      "  GPIOs:");
  for(auto& gpio: info.gpios)
    printComponentInfo(gpio, i++);
  i=0;
  RCLCPP_INFO(log,      "  Transmissions:");
  for(auto& transmission: info.transmissions)
    printTransmissionInfo(transmission, i++);
}

// See https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/hardware_info.hpp
CallbackReturn VESCInterface::on_init(const HardwareInfo& info)
{
  if (ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  if(!_vesc)
  {
    _vesc = orthopus::VESCHost::getInstance();
    RCLCPP_INFO(rclcpp::get_logger("VESCInterface"),"VESCHost: %p", (void*)_vesc.get());
    if(!_vesc)
    {
      auto can = std::make_shared<vescpp::comm::CAN>("vxcan1");
      _vesc = orthopus::VESCHost::spawnInstance(1, can);
      RCLCPP_INFO(rclcpp::get_logger("VESCInterface")," => Spawn VESCHost: %p", (void*)_vesc.get());
    }
  }

  printHardwareInfo(info);
  const auto* name = info_.name.c_str();
  RCLCPP_INFO(rclcpp::get_logger("VESCInterface"),  "[VESCInterface::on_init] Init '%s'", name);
  for(auto& joint: info_.joints)
  {
    const auto* jname = joint.name.c_str();
    if (joint.command_interfaces.size() > 4)
    {
      RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"),"[%s] Joint '%s' has %zu command interfaces. Expected Max 4", name, jname, joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }
    for(const auto& intf: joint.command_interfaces)
    {
      const auto* iname = intf.name.c_str();
      if(    intf.name != HW_IF_POSITION
          && intf.name != HW_IF_VELOCITY
          && intf.name != HW_IF_ACCELERATION
          && intf.name != HW_IF_EFFORT
        )
      {
        RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"),"[%s][%s] Unsupported command interface '%s'", name, jname, iname);
        return CallbackReturn::ERROR;
      }
    }
    for(const auto& intf: joint.state_interfaces)
    {
      const auto* iname = intf.name.c_str();
      if(    intf.name != HW_IF_POSITION
          && intf.name != HW_IF_VELOCITY
          && intf.name != HW_IF_ACCELERATION
          && intf.name != HW_IF_EFFORT
        )
      {
        RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"),"[%s][%s] Unsupported command interface '%s'", name, jname, iname);
        return CallbackReturn::ERROR;
      }
    }
/*
    // State interfaces
    _state_interfaces.emplace_back(hardware_interface::StateInterface(_name, hardware_interface::HW_IF_POSITION, &_qm));
    _state_interfaces.emplace_back(hardware_interface::StateInterface(_name, hardware_interface::HW_IF_VELOCITY, &_dqm));
    _state_interfaces.emplace_back(hardware_interface::StateInterface(_name, hardware_interface::HW_IF_ACCELERATION, &_ddqm));
    _state_interfaces.emplace_back(hardware_interface::StateInterface(_name, hardware_interface::HW_IF_EFFORT, &_taum));

    if (joint.state_interfaces.size() > _state_interfaces.size())
    {
      RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"),"[%s] Joint '%s' has %zu state interfaces found. max %ld expected.", name, jname, joint.state_interfaces.size(), _state_interfaces.size());
      return CallbackReturn::ERROR;
    }
    for(const auto& intf: joint.state_interfaces)
    {
      if(    intf.name != HW_IF_POSITION
          && intf.name != HW_IF_VELOCITY
          && intf.name != HW_IF_ACCELERATION
          && intf.name != HW_IF_EFFORT
        )
      {
        RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"),"[%s][%s] Unsupported state interface '%s'", name, name, intf.name.c_str());
        return CallbackReturn::ERROR;
      }
    }
*/
  }

  //RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Successfully initialized!", name);
  /*
  auto& _hw_joints = *(VESCInterfaceComm::getInstance().getJoints());
  for(auto& joint: _hw_joints)
  {
    if(!(joint.owner == this && joint.enable))
      continue;
    RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s][%s] Successfully initialized!", name,joint.name.c_str());
  }
    */

  return CallbackReturn::SUCCESS;
}

CallbackReturn VESCInterface::on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Successfully configured!", _name.c_str());
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> VESCInterface::export_state_interfaces()
{
  return _state_interfaces;
}

std::vector<hardware_interface::CommandInterface> VESCInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> _command_interfaces;
  /*auto& _hw_joints = *(VESCInterfaceComm::getInstance().getJoints());
  for(auto& joint: _hw_joints)
  {
    if(!(joint.owner == this && joint.enable))
      continue;
    const auto& joint_name = joint.name;
    _command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_name, hardware_interface::HW_IF_POSITION, &joint.ref.pos));
    _command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_name, hardware_interface::HW_IF_VELOCITY, &joint.ref.vel));
    _command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_name, hardware_interface::HW_IF_ACCELERATION, &joint.ref.acc));
    _command_interfaces.emplace_back(hardware_interface::CommandInterface(joint_name, hardware_interface::HW_IF_EFFORT, &joint.ref.eff));
  }
    */
  return _command_interfaces;
}

ORTHOPUS_ROS_PUBLIC
return_type VESCInterface::prepare_command_mode_switch([[maybe_unused]] const std::vector<std::string>&start_if, [[maybe_unused]] const std::vector<std::string> &stop_if)
{
  /*
  auto& _hw_joints = *(VESCInterfaceComm::getInstance().getJoints());
  RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Preparing Command mode switch...", _name.c_str());
  RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "  Starting Interfaces:");
  for(const auto& st_if: start_if)
  {
    for(const auto& joint: _hw_joints)
    {
      if(!(joint.owner == this && joint.enable))
        continue;
      const auto jnsz = joint.name.length();
      if(!st_if.compare(0,jnsz,joint.name)) // Check only for us
        RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "   - %s",st_if.c_str());
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "  Stopping Interfaces:");
  for(const auto& st_if: stop_if)
  {
    for(const auto& joint: _hw_joints)
    {
      if(!(joint.owner == this && joint.enable))
        continue;
      const auto jnsz = joint.name.length();
      if(!st_if.compare(0,jnsz,joint.name)) // Check only for us
        RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "   - %s",st_if.c_str());
    }
  }
  */
  RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Command mode switch prepared!", _name.c_str());
  return return_type::OK;
}

ORTHOPUS_ROS_PUBLIC
return_type VESCInterface::perform_command_mode_switch([[maybe_unused]] const std::vector<std::string>&start_if, [[maybe_unused]] const std::vector<std::string> &stop_if)
{
  /*
  auto& _hw_joints = *(VESCInterfaceComm::getInstance().getJoints());
  RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Performing Command mode switch...", _name.c_str());
  RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "  Starting Interfaces:");
  for(const auto& st_if: start_if)
  {
    for(const auto& joint: _hw_joints)
    {
      if(!(joint.owner == this && joint.enable))
        continue;
      const auto jnsz = joint.name.length();
      if(!st_if.compare(0,jnsz,joint.name)) // Check only for us
        RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "   - %s",st_if.c_str());
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "  Stopping Interfaces:");
  for(const auto& st_if: stop_if)
  {
    for(const auto& joint: _hw_joints)
    {
      if(!(joint.owner == this && joint.enable))
        continue;
      const auto jnsz = joint.name.length();
      if(!st_if.compare(0,jnsz,joint.name)) // Check only for us
        RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "   - %s",st_if.c_str());
    }
  }
  */
  RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Command mode switch performed!", _name.c_str());
  return return_type::OK;
}

CallbackReturn VESCInterface::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  /*
  auto& _hw_joints = *(VESCInterfaceComm::getInstance().getJoints());
  RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Activating ...please wait...", _name.c_str());
  for(auto& joint: _hw_joints)
  {
    if(!(joint.owner == this && joint.enable))
      continue;
    joint.state.pos = 0.0;
    joint.state.vel = 0.0;
    joint.state.acc = 0.0;
    joint.state.eff = 0.0;

    joint.ref.pos = 0.0;
    joint.ref.vel = 0.0;
    joint.ref.acc = 0.0;
    joint.ref.eff = 0.0;
  }
  */
  RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Successfully activated!", _name.c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn VESCInterface::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Deactivating ...please wait...", _name.c_str());
  RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Successfully deactivated!", _name.c_str());

  return CallbackReturn::SUCCESS;
}

return_type VESCInterface::read([[maybe_unused]] const rclcpp::Time& time, [[maybe_unused]] const rclcpp::Duration& period)
{
  // Done in VESCInterfaceComm
  return return_type::OK;
}

return_type VESCInterface::write([[maybe_unused]] const rclcpp::Time& time, [[maybe_unused]] const rclcpp::Duration& period)
{
/*  for(auto& joint: _hw_joints)
  {
    if(!(joint.owner == this && joint.enable))
      continue;
    auto& hw_ref = joint.ref;
    if(!std::isnan(hw_ref.pos))// && hw_ref.pos != hw_ref.ppos)
    {
      //RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s][%s] Writing new pos command: %f...", _name.c_str(), joint.name.c_str(), hw_ref.pos);
      //hw_ref.ppos = hw_ref.pos;
      //hw_ref.nwpos = true;
    }
    if(!std::isnan(hw_ref.vel))// && hw_ref.vel != hw_ref.pvel)
    {
      //RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s][%s] Writing new vel command: %f...", _name.c_str(), joint.name.c_str(), hw_ref.vel);
      //hw_ref.pvel = hw_ref.vel;
      //hw_ref.nwvel = true;
    }
    if(!std::isnan(hw_ref.acc))// && hw_ref.acc != hw_ref.pacc)
    {
      //RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s][%s] Writing new acc command: %f...", _name.c_str(), joint.name.c_str(), hw_ref.acc);
      //hw_ref.pacc = hw_ref.acc;
      //hw_ref.nwacc = true;
    }
    if(!std::isnan(hw_ref.eff))// && hw_ref.eff != hw_ref.peff)
    {
      //RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s][%s] Writing new eff command: %f...", _name.c_str(), joint.name.c_str(), hw_ref.eff);
      //hw_ref.peff = hw_ref.eff;
      //hw_ref.nweff = true;
    }
  }
    */
  return return_type::OK;
}

}  // namespace orthopus_ros

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(orthopus_ros::VESCInterface, ActuatorInterface)
