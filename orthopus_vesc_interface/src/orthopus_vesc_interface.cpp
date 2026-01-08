#include <chrono>
#include <sstream>  // for from_str, below

#include "orthopus_vesc_interface/orthopus_vesc_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

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

const auto qos_services = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_ALL, 1))
                            .reliable()
                            .durability_volatile();

const auto qos_pub = rclcpp::SystemDefaultsQoS();

// See https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/include/hardware_interface/hardware_info.hpp
CallbackReturn VESCInterface::on_init(const HardwareInfo& info)
{
  if (ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  _name = info.name;
  _node = std::make_unique<rclcpp::Node>(_name);

  if(!_vesc_host)
  {
    _vesc_host = orthopus::VESCHost::getInstance();
    //RCLCPP_DEBUG(rclcpp::get_logger("VESCInterface"),"VESCHost: %p", (void*)_vesc_host.get());
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
      RCLCPP_DEBUG(rclcpp::get_logger("VESCInterface")," => Spawn VESCHost: %p", (void*)_vesc_host.get());
    }
  }

  auto board_id = vescpp::VESC::InvalidBoardId;
  {
    auto it = info.hardware_parameters.find("can_id");
    if(it == info.hardware_parameters.end())
    {
      RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"),"'can_id' not found in HardwareInfo, abort");  
      return CallbackReturn::ERROR;
    }
    board_id = (vescpp::VESC::BoardId)(from_str<unsigned int>(it->second, vescpp::VESC::InvalidBoardId))&0xFF;
  }
  if(board_id == vescpp::VESC::InvalidBoardId)
  {
    RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"),"Invalid 'can_id' in HardwareInfo, abort");  
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
  _vesc_dev->_print_hdlr = [&](const std::string& s) -> void
  {
    const auto now = rclcpp::Clock().now();
    
    if(_print_buf_duration.seconds() > 0.0)
    {
      if(now < _print_buf_start + _print_buf_duration)
      {
        _print_buf.emplace_back(printMsg_t{now, s});
      }
      else
        _print_buf_duration *= 0.0;
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("VESCInterface"),  "[%s] <= %s", _name.c_str(), s.c_str());
  };
  
  RCLCPP_DEBUG(rclcpp::get_logger("VESCInterface"),  "[%s] Init with BoardID '%d'", _name.c_str(), board_id);
  //spdlog::debug("==> {:np}", spdlog::to_hex(_vesc_dev->fw()->uuid));
  //printHardwareInfo(info);
  
  auto j_sz = _vesc_dev->joints.size();
  if(info_.joints.size() > j_sz)
  {
    RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"),"Target '%d' doesn't have enough Joints. Expected '%ld', got '%ld'. Abort", board_id, info.joints.size(), j_sz);  
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
    auto it = _vesc_dev->joints.begin();
    for(const auto& cfg_j: info.joints)
    {
      it->name = cfg_j.name;
      if(++it == _vesc_dev->joints.end())
        break;
    }
    //for(const auto& j: _vesc_dev->joints)
    //{
    //  RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"),"Target '%d', Joint '%s' found.", board_id, j.name.c_str());  
    //  for(const auto& [r, _]:  j.refs)
    //    RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"),"  Ref: %s", r.c_str());   
    //}
  }

  for(const auto& cfg_j: info.joints)
  {
    auto j = _vesc_dev->getJoint(cfg_j.name);
    if(j == nullptr)
    {
      RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"),"Target '%d', Joint '%s' not found. Abort", board_id, cfg_j.name.c_str());  
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
      const std::string& sstr = orthopus::State2Text(j_data.status);
      const std::string& estr = orthopus::Err2Text(j_data.status);
      const std::string& mstr = orthopus::Mode2Text(j_data.status);
      RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Got State: 0x%04X: State: '%s' Error '%s' Mode '%s'", _name.c_str(), j_data.status, sstr.c_str(), estr.c_str(), mstr.c_str());
      if(!_state_rtpub)
        return;
      _state_rtpub->lock();
      _state_rtpub->msg_.timestamp = rclcpp::Clock().now();
      _state_rtpub->msg_.joint_name = j->name;  // FIXME: Get real joint name
      
      _state_rtpub->msg_.mode = mstr;
      _state_rtpub->unlockAndPublish();
    };
    
    for(const auto& cif: cfg_j.command_interfaces)
    {
      const auto it =  j->refs.find(cif.name);
      if(it ==  j->refs.end())
      {
        RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"),"Target '%d', Joint '%s', Interface '%s' not found in available refs. Abort", board_id, j->name.c_str(), cif.name.c_str());  
        return CallbackReturn::ERROR;
      }
    }
    for(const auto& sif: cfg_j.state_interfaces)
    {
      auto it =  j->meas.find(sif.name);
      if(it ==  j->meas.end())
      {
        RCLCPP_FATAL(rclcpp::get_logger("VESCInterface"),"Target '%d', Joint '%s', Interface '%s' not found in available meas. Abort", board_id, j->name.c_str(), sif.name.c_str());  
        return CallbackReturn::ERROR;
      }
      _state_interfaces.emplace_back(hardware_interface::StateInterface(j->name, sif.name, &it->second.v));
    }
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
  for(auto& j: _vesc_dev->joints)
  {
    if(!j.in_use)
      continue;
    for(auto& [cif_name, cif_v]:  j.refs)
    {
      _command_interfaces.emplace_back(hardware_interface::CommandInterface(j.name, cif_name, &cif_v.v));
    }
  }
  return _command_interfaces;
}

CallbackReturn VESCInterface::on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_DEBUG(rclcpp::get_logger("VESCInterface"), "[on_configure][%s] Successfully configured!", _name.c_str());

  _dev_srv = _node->create_service<orthopus_vesc_interfaces::srv::Dev>("~/dev", [this]
    (const std::shared_ptr<orthopus_vesc_interfaces::srv::Dev::Request> req,
        std::shared_ptr<orthopus_vesc_interfaces::srv::Dev::Response> resp)
  {
    (void)req;
    RCLCPP_INFO(rclcpp::get_logger("VESCInterface"),"VESCHost: %p - %s", (void*)_vesc_host.get(), this->_name.c_str());
    resp->help = "Hello World";
  });

  _set_mode_srv = _node->create_service<orthopus_vesc_interfaces::srv::SetMode>("~/mode", [this]
    (const std::shared_ptr<orthopus_vesc_interfaces::srv::SetMode::Request> req,
        std::shared_ptr<orthopus_vesc_interfaces::srv::SetMode::Response> resp)
  {
    const auto& j_name = req->joint_name;
    auto j = _vesc_dev->getJoint(j_name); 
    if(j == nullptr)
    {
      RCLCPP_ERROR(rclcpp::get_logger("VESCInterface"), "[setMode] Joint %s not found, abort", j_name.c_str());
      resp->ret = false;
      return;
    }
    const auto& req_mode = req->mode;

    auto new_ctrl = j->ctrl &~orthopus::ORTHOPUS_CTRL_MODE_MSK; // Clear last command
    // FIXME: For each mode, check wether the required interfaces are "in use" (which j->refs["NAME"].in_use) and act accordingly
    if(req_mode =="custom")
    {
      new_ctrl |= orthopus::ORTHOPUS_CTRL_MODE_CST;
      RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Switch to CuSTom mode", j_name.c_str());
    }
    else if(req_mode =="impedence")
    {
      new_ctrl |= orthopus::ORTHOPUS_CTRL_MODE_IMP;
      RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Switch to IMPedence mode", j_name.c_str());
    }
    else if(req_mode =="effort")
    {
      new_ctrl |= orthopus::ORTHOPUS_CTRL_MODE_TRQ;
      RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Switch to ToRQue mode", j_name.c_str());
    }
    else if(req_mode =="velocity")
    {
      new_ctrl |= orthopus::ORTHOPUS_CTRL_MODE_VEL;
      RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Switch to VELocity mode", j_name.c_str());
    }
    else if(req_mode =="position")
    {
      new_ctrl |= orthopus::ORTHOPUS_CTRL_MODE_POS;
      RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Switch to POSition mode", j_name.c_str());
    }
    else
    {
      new_ctrl |= orthopus::ORTHOPUS_CTRL_MODE_OFF; // Useless
      RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Switch to OFF mode", j_name.c_str());
    }

    if(new_ctrl != j->ctrl)
    {
      j->ctrl = new_ctrl;
      if(!j->stream)
      {
        RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Enable stream, with ctrlWord 0x%04x, posMeas: %f posRef: %f", j_name.c_str(), j->ctrl, j->meas.at("position").v, j->refs.at("position").v);
        j->stream = true;
      }
      const std::string& sstr = orthopus::State2Text(j->ctrl);
      const std::string& estr = orthopus::Err2Text(j->ctrl);
      const std::string& mstr = orthopus::Mode2Text(j->ctrl);
      RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Set Ctrl: 0x%04X: State: '%s' Error '%s' Mode '%s'", _name.c_str(), j->ctrl, sstr.c_str(), estr.c_str(), mstr.c_str());
      resp->ret = true;
    }
  });

  _cmd_srv = _node->create_service<orthopus_vesc_interfaces::srv::Cmd>("~/command", [this]
    (const std::shared_ptr<orthopus_vesc_interfaces::srv::Cmd::Request> req,
        std::shared_ptr<orthopus_vesc_interfaces::srv::Cmd::Response> resp)
  {
    const auto wait_ms = std::chrono::milliseconds(req->wait_for_ms);
    //const auto now = vescpp::Time::now();
    _print_buf.clear();

    _print_buf_duration = rclcpp::Duration(wait_ms);
    _print_buf_start    = rclcpp::Clock().now();
    _vesc_dev->sendCmd(req->cmd, wait_ms); // It waits an appropriate time for us, so we're good
    for(const auto& s: _print_buf)
      resp->ret += s.str+"\n";        
    _print_buf.clear();
  });

  _state_pub   = _node->create_publisher<orthopus_vesc_interfaces::msg::State>("~/state", qos_pub);
  _state_rtpub = std::make_unique<realtime_tools::RealtimePublisher<orthopus_vesc_interfaces::msg::State>>(_state_pub);
   
  return CallbackReturn::SUCCESS;
}

CallbackReturn VESCInterface::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  //RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Activating ...please wait...", _name.c_str());
  
  // Wait for a a fresh meas from device
  const auto now = vescpp::Time::now();
  while(true)
  {
    if((now - _vesc_dev->_meas_last_tp) < 10ms)
    {
      //RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Got a fresh Meas! Marching on !",_name.c_str());
      break;
    }
    else if(vescpp::Time::now()-now >= 1000ms)
    {
      RCLCPP_ERROR(rclcpp::get_logger("VESCInterface"), "[on_activate][%s] Timeout waiting for a fresh Meas. Abort,",_name.c_str());
      return CallbackReturn::ERROR;
    } 
    std::this_thread::sleep_for(10us);
  }

  // Meas are a-okay. Init interfaces refs !
  // FIXME: What do we do with servo, since it does not have any pos streaming ? Currently init at 0.5 from orthopus::VESCTarget
  auto& j = _vesc_dev->joint;
  for(auto& [ifn,if_v]: j.refs)
  {
    if_v.v = 0.0;         // 0.0 is the default
    if_v.in_use = false;  // Free the interface
    // FIXME: Keep this ?
    if(ifn == "position")
    {
      // Set init value to measure only when a meas was received (if so, in_use is true)
      if(auto it=j.meas.find(ifn); it != j.meas.end() && it->second.in_use)
      {
        if_v.v = it->second.v; // Set ref to last/current position
        RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[on_activate][%s][%s] Init POS ref with value: % 7.4f", _name.c_str(), j.name.c_str(), if_v.v);
      }
    }
  }
  //RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Successfully activated!", _name.c_str());
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
  /*RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Preparing Command mode switch...", _name.c_str());
  RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "  Stopping Interfaces:");
  // FIXME: Check that we can actually perform the requested command mode switch
  RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Command mode switch prepared!", _name.c_str());
  */
  return return_type::OK;
}

ORTHOPUS_ROS_PUBLIC
return_type VESCInterface::perform_command_mode_switch([[maybe_unused]] const std::vector<std::string>&start_if, [[maybe_unused]] const std::vector<std::string> &stop_if)
{
  auto enable_intf = [this](const std::vector<std::string>& _ifs, bool enable)
  {
    for(const auto& st_if: _ifs)
    {
      for(auto& j: this->_vesc_dev->joints)
      {
        // st_if format is JOINT_NAME/INTERFACE 
        if(auto idx = st_if.find(j.name,0); idx == 0)   // Match beginning of the name with joint name
        {
          // FIXME: Check st_if length first
          const auto& intf = st_if.substr(j.name.length()+1);  // Then match the end with supported interfaces
          if(auto it = j.refs.find(intf); it != j.refs.end())
          {
            it->second.in_use = enable;
            RCLCPP_DEBUG(rclcpp::get_logger("VESCInterface"), "   - %s",st_if.c_str());
          }
        }
      }
    }
  };

  RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Performing Command mode switch...", _name.c_str());
  RCLCPP_DEBUG(rclcpp::get_logger("VESCInterface"), "  Stopping Interfaces:");
  enable_intf(stop_if, false);
  RCLCPP_DEBUG(rclcpp::get_logger("VESCInterface"), "  Starting Interfaces:");
  enable_intf(start_if, true);
  RCLCPP_INFO(rclcpp::get_logger("VESCInterface"), "[%s] Command mode switch performed!", _name.c_str());
  return return_type::OK;
}


return_type VESCInterface::read([[maybe_unused]] const rclcpp::Time& time, [[maybe_unused]] const rclcpp::Duration& period)
{
  // Async, Measures are streamed by the devices, directly to orthopus::VESCTarget
  // TODO: Sanity checks (trigger error if delay since last meas reached a timeout for instance)
  // TODO: Make sure spin_some does not slow down the RT loop (event when processing srv/pub/sub/...)
  if(_node)
    rclcpp::spin_some(_node->get_node_base_interface());
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
  for(const auto& intf: info.state_interfaces)
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
