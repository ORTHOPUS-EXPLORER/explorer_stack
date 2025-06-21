#include <sstream>  // for from_str, below

#include "orthopus_vesc_interface/orthopus_vesc_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace hardware_interface;

namespace orthopus_ros
{

template<typename T>
T from_str(const std::string& str, const T& def_v)
{
  T out;
  std::istringstream ss(str);
  ss >> out;
  return ss.fail() ? def_v : out;
}

// See https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/hardware_info.hpp
CallbackReturn VESCInterface::on_init(const HardwareInfo& info)
{
  if (ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  _name = info.name;

  if(!_vesc_host)
  {
    _vesc_host = orthopus::VESCHost::getInstance();
    RCLCPP_INFO(rclcpp::get_logger("VESCInterface"),"VESCHost: %p", (void*)_vesc_host.get());
    if(!_vesc_host)
    {
      spdlog::cfg::load_env_levels();
      // Load parameters
        // CAN Port
      auto it = info.hardware_parameters.find("can_port");
      if(it == info.hardware_parameters.end())
      {
        RCLCPP_FATAL(rclcpp::get_logger("VESCInterface")," Can't spawn VESCHost, can_port is not defined");  
        exit(0);
      }
      auto can_port = it->second;
        // Host ID
      it = info.hardware_parameters.find("host_id");
      if(it == info.hardware_parameters.end())
      {
        RCLCPP_FATAL(rclcpp::get_logger("VESCInterface")," Can't spawn VESCHost, host_id is not defined");  
        exit(0);
      }
      auto host_id = (vescpp::VESC::BoardId)(from_str<unsigned int>(it->second, vescpp::VESC::InvalidBoardId))&0xFF;
      if(host_id == vescpp::VESC::InvalidBoardId)
      {
        RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"),"Invalid host_id in HardwareInfo, abort");  
        return CallbackReturn::ERROR;
      }
        // Stream rate
      it = info.hardware_parameters.find("rt_stream_rate");
      if(it == info.hardware_parameters.end())
      {
        RCLCPP_FATAL(rclcpp::get_logger("VESCInterface")," Can't spawn VESCHost, rt_stream_rate is not defined");  
        exit(0);
      }
      auto rt_stream_rate_hz = from_str<double>(it->second, 250);
        // Aux rate
      it = info.hardware_parameters.find("aux_stream_rate");
      if(it == info.hardware_parameters.end())
      {
        RCLCPP_FATAL(rclcpp::get_logger("VESCInterface")," Can't spawn VESCHost, aux_stream_rate is not defined");  
        exit(0);
      }
      auto aux_stream_rate_hz = from_str<double>(it->second, 10);
      

      RCLCPP_INFO(rclcpp::get_logger("VESCInterface")," => Use CAN port '%s' with Host ID '%d'", can_port.c_str(), host_id);
      auto can = std::make_shared<vescpp::comm::CAN>(can_port);
      _vesc_host = orthopus::VESCHost::spawnInstance(host_id, can);
      _vesc_host->setRTStreamRate(rt_stream_rate_hz);
      _vesc_host->setAuxStreamRate(aux_stream_rate_hz);
      
      _vesc_host->scanCAN(true, 100ms);
      RCLCPP_INFO(rclcpp::get_logger("VESCInterface")," => Spawn VESCHost: %p", (void*)_vesc_host.get());
    }
  }

  auto board_id = vescpp::VESC::InvalidBoardId;
  {
    auto it = info.hardware_parameters.find("can_id");
    if(it == info.hardware_parameters.end())
    {
      RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"),"can_id not found in HardwareInfo, abort");  
      return CallbackReturn::ERROR;
    }
    board_id = (vescpp::VESC::BoardId)(from_str<unsigned int>(it->second, vescpp::VESC::InvalidBoardId))&0xFF;
  }
  if(board_id == vescpp::VESC::InvalidBoardId)
  {
    RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"),"Invalid can_id in HardwareInfo, abort");  
    return CallbackReturn::ERROR;
  }

  // Not quite necessary since we fetch the FwVersion when adding the Device just below
  //if(!_vesc_host->pingCAN(board_id, 200ms))
  //{
  //  RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"),"Timeout waiting for CAN PONG from '%d'. Abort", board_id);  
  //  return CallbackReturn::ERROR;
  //}

  _vesc_dev = _vesc_host->addTarget(board_id);
  if(!_vesc_dev || !_vesc_dev->fw())
  {
    RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"),"Timeout waiting for VESC '%d'. Abort", board_id);  
    return CallbackReturn::ERROR;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("VESCInterface"),  "[%s]] Init with BoardID '%d'", _name.c_str(), board_id);
  //spdlog::debug("==> {:np}", spdlog::to_hex(_vesc_dev->fw()->uuid));
  //printHardwareInfo(info);
  
  auto j_sz = _vesc_dev->joints.size();
  if(info_.joints.size() > j_sz)
  {
    RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"),"Target '%d' doesn't have enough Joints. Expected '%ld', got '%ld'. Abort", board_id, info.joints.size(), j_sz);  
    return CallbackReturn::ERROR;
  }
  j_sz = std::min(j_sz, info_.joints.size());
  
  // FIXME: Get Joint names from VESC, well build the whole joints map from VESC data
  //        Please note this trick is REALLY FRAGILE !!! 
  //        It depends on how the joints are defined in the URDF and how they are added to the 
  //        unordered_map (reverse order in which they are written in the .hpp file)
  {
    //for(const auto& [j_name, tgt_j]: _vesc_dev->joints)
    //{
    //  RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"),"Target '%d', Joint '%s' found.", board_id, j_name.c_str());  
    //  for(const auto& [r, _]: tgt_j.refs)
    //    RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"),"  Ref: %s", r.c_str());   
    //}

    auto it = _vesc_dev->joints.begin();
    for(const auto& cfg_j: info.joints)
    {
      auto v = _vesc_dev->joints.extract(it);
      v.key() = cfg_j.name;
      _vesc_dev->joints.insert(it, std::move(v));
      if(++it == _vesc_dev->joints.end())
        break;
    }
    //for(const auto& [j_name, tgt_j]: _vesc_dev->joints)
    //{
    //  RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"),"Target '%d', Joint '%s' found.", board_id, j_name.c_str());  
    //  for(const auto& [r, _]: tgt_j.refs)
    //    RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"),"  Ref: %s", r.c_str());   
    //}
  }
  size_t j = 0;
  for(const auto& cfg_j: info.joints)
  {
    auto j_it = _vesc_dev->joints.find(cfg_j.name);
    if(j_it == _vesc_dev->joints.end())
    {
      RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"),"Target '%d', Joint '%s' not found. Abort", board_id, cfg_j.name.c_str());  
      return CallbackReturn::ERROR;
    }
    
    const auto& j_name = j_it->first;
    auto& tgt_j = j_it->second;
    tgt_j.in_use = true;

    //RCLCPP_DEBUG(rclcpp::get_logger("VESCInterface"),"Target '%d', Joint '%s' found.", board_id, j_name.//c_str());  
    //for(const auto& [r, _]: tgt_j.refs)
    //  RCLCPP_DEBUG(rclcpp::get_logger("VESCInterface"),"  Ref: %s", r.c_str());  
    
    for(const auto& cif: cfg_j.command_interfaces)
    {
      const auto it = tgt_j.refs.find(cif.name);
      if(it == tgt_j.refs.end())
      {
        RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"),"Target '%d', Joint '%s', Interface '%s' not found in available refs. Abort", board_id, j_name.c_str(), cif.name.c_str());  
        return CallbackReturn::ERROR;
      }
    }
    for(const auto& sif: cfg_j.state_interfaces)
    {
      auto it = tgt_j.meas.find(sif.name);
      if(it == tgt_j.meas.end())
      {
        RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"),"Target '%d', Joint '%s', Interface '%s' not found in available meas. Abort", board_id, j_name.c_str(), sif.name.c_str());  
        return CallbackReturn::ERROR;
      }
      _state_interfaces.emplace_back(hardware_interface::StateInterface(j_name, sif.name, &it->second));
    }
    j_it++;
    j++;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> VESCInterface::export_state_interfaces()
{
  return _state_interfaces;
}

std::vector<hardware_interface::CommandInterface> VESCInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> _command_interfaces;
  for(auto& [j_name, tgt_j]: _vesc_dev->joints)
  {
    if(!tgt_j.in_use)
      continue;
    for(auto& [cif_name, cif_v]: tgt_j.refs)
    {
      _command_interfaces.emplace_back(hardware_interface::CommandInterface(j_name, cif_name, &cif_v));
    }
  }
  return _command_interfaces;
}


CallbackReturn VESCInterface::on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Successfully configured!", _name.c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn VESCInterface::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  //RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Activating ...please wait...", _name.c_str());
  // TODO: Figure out what we should do about the servo/2nd joint.
  //       Currently, we only check the main/first joint (the actuator)
  for(auto& [j_name, j]: _vesc_dev->joints)
  {
    // Wait for a a fresh meas
    const auto now = vescpp::Time::now();
    while(true)
    {
      if((now - _vesc_dev->_meas_last_tp) < 10ms)
      {
        RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s][%s] Got a fresh Meas! Marching on !",_name.c_str(), j_name.c_str());
        break;
      }
      else if(vescpp::Time::now()-now >= 1000ms)
      {
        RCLCPP_ERROR(rclcpp::get_logger("VESCInterface"), "[%s][%s] Timeout waiting for a fresh Meas. Abort,",_name.c_str(), j_name.c_str());
        return CallbackReturn::ERROR;
      } 
      std::this_thread::sleep_for(10us);
    }
    // Meas are a-okay. Init refs !
    for(auto& [ifn,if_v]: j.refs)
    {
      if_v = 0.0; // 0.0 is the default
      if(ifn == "position")
      {
        if(auto it=j.meas.find(ifn); it != j.meas.end())
        {
          if_v = it->second; // Set ref to last/current position
          RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s][%s] Init POS ref with value: % 7.4f", _name.c_str(), j_name.c_str(), if_v);
        }
      }
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Successfully activated!", _name.c_str());
  // FIXME: Do this somewhere else ffs
  _vesc_host->startStreaming();
  return CallbackReturn::SUCCESS;
}

CallbackReturn VESCInterface::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  //RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Deactivating ...please wait...", _name.c_str());
  //RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Successfully deactivated!", _name.c_str());

  return CallbackReturn::SUCCESS;
}

ORTHOPUS_ROS_PUBLIC
return_type VESCInterface::prepare_command_mode_switch([[maybe_unused]] const std::vector<std::string>&start_if, [[maybe_unused]] const std::vector<std::string> &stop_if)
{
  // TODO: Sanity checks: Make sure we're not trying to enable POS + VEL + TRQ at the same time.
  /*RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Preparing Command mode switch...", _name.c_str());
  RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "  Stopping Interfaces:");
  for(const auto& st_if: stop_if)
  {
    for(const auto& [j_name, _]: _vesc_dev->joints)
    {
      if(auto id = st_if.find(j_name,0); id == 0)
      {
        RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "   - %s",st_if.c_str());
      }
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "  Starting Interfaces:");
  for(const auto& st_if: start_if)
  {
    for(const auto& [j_name, _]: _vesc_dev->joints)
    {
      if(auto id = st_if.find(j_name,0); id == 0)
      {
        RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "   - %s",st_if.c_str());
      }
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Command mode switch prepared!", _name.c_str());
  */
  return return_type::OK;
}

ORTHOPUS_ROS_PUBLIC
return_type VESCInterface::perform_command_mode_switch([[maybe_unused]] const std::vector<std::string>&start_if, [[maybe_unused]] const std::vector<std::string> &stop_if)
{

  /*RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Performing Command mode switch...", _name.c_str());
  RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "  Stopping Interfaces:");
  for(const auto& st_if: stop_if)
  {
    for(const auto& [j_name, _]: _vesc_dev->joints)
    {
      if(auto id = st_if.find(j_name,0); id == 0)
      {
        RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "   - %s",st_if.c_str());
      }
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "  Starting Interfaces:");
  */
  for(const auto& st_if: start_if)
  {
    for(const auto& [j_name, j_data]: _vesc_dev->joints)
    {
      // st_if format is JOINT_NAME/INTERFACE 
      if(auto idx = st_if.find(j_name,0); idx == 0)   // Match beginning of the name with joint name
      {
        // FIXME: Check st_if length first
        const auto& intf = st_if.substr(j_name.length()+1);  // Then match the end with supported interfaces
        if(auto it = j_data.refs.find(intf); it != j_data.refs.end())
        {
          auto pctrl = j_data.ctrl;
          j_data.ctrl &= ~orthopus::ORTHOPUS_CTRL_MODE_MSK;  // Clear last command

          if(intf == "position" )
          {
            j_data.ctrl |= orthopus::ORTHOPUS_CTRL_MODE_POS;
            RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Switch to POS mode", j_name.c_str());
          } 
          else if(intf == "velocity")
          {
            j_data.ctrl |= orthopus::ORTHOPUS_CTRL_MODE_VEL;
            RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Switch to VEL mode", j_name.c_str());
          } 
          else if(intf == "effort")
          {
            j_data.ctrl |= orthopus::ORTHOPUS_CTRL_MODE_TRQ;
            RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Switch to TRQ mode", j_name.c_str());
          } 
          else
            RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Switch to OFF mode", j_name.c_str());
          
          if(pctrl != j_data.ctrl)
          {
            if(!j_data.stream)
            {
              RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Enable stream, with ctrlWord 0x%04x, posMeas: %f posRef: %f", j_name.c_str(), j_data.ctrl, j_data.meas.at("position"), j_data.refs.at("position"));
              j_data.stream = true;
            }
          }

          //RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "   - %s",st_if.c_str());
          break; // We're done for this joint
        }
      }
    }
  }
    
  //RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Command mode switch performed!", _name.c_str());
  return return_type::OK;
}


return_type VESCInterface::read([[maybe_unused]] const rclcpp::Time& time, [[maybe_unused]] const rclcpp::Duration& period)
{
  // Async, Measures are streamed by the devices, directly to orthopus::VESCTarget
  // TODO: Sanity checks (trigger error if delay since last meas reached a timeout for instance)
  return return_type::OK;
}

return_type VESCInterface::write([[maybe_unused]] const rclcpp::Time& time, [[maybe_unused]] const rclcpp::Duration& period)
{
  // Async, Refs are sent in another Thread, managed by orthopus::VESCHost (started by startStreaming)
  // TODO: Sanity checks: Make sure the refs are not completely out of range, for instance
  return return_type::OK;
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


}  // namespace orthopus_ros

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(orthopus_ros::VESCInterface, ActuatorInterface)
