#include "orthopus_ros_interface/hw_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace hardware_interface;

namespace orthopus_ros_interface
{

class HwInterfaceComm
    : public rclcpp::Node
{
public:
    // Data
  typedef struct
  {
    double pos;
    double vel;
    double acc;
    double eff;
  } hw_state_t;

  using hw_ref_t = hw_state_t;

  typedef struct
  {
    HwInterface* owner;
    std::string name;
    bool enable;
    hw_state_t state;
    hw_ref_t ref;
  } hw_joint_t;

  using hw_joints_t = std::vector<hw_joint_t>;
  std::mutex _lck;

  static HwInterfaceComm& getInstance();
  std::shared_ptr<hw_joints_t> getJoints();
  HwInterfaceComm::hw_joint_t& getJoint(HwInterface* owner, const std::string& joint_name);
private:
  HwInterfaceComm();
  std::shared_ptr<HwInterfaceComm> _this_comm;
  rclcpp::executors::SingleThreadedExecutor _executor; //Executor needed to subscriber
  std::thread _comm_th;

  //HwInterface &_parent;

  hw_joints_t _hw_joints;

  // From ros_explorer_bridge
  rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr _expl_meas_sub;
  //realtime_tools::RealtimeBuffer<std::shared_ptr<control_msgs::msg::DynamicJointState>>  _expl_meas_msg_buf;
  //std::shared_ptr<control_msgs::msg::DynamicJointState>                                  _expl_meas_msg;

  // To ros_explorer_bridge
  rclcpp::Publisher<control_msgs::msg::DynamicJointState>::SharedPtr _expl_ref_pub;
  rclcpp::Time _expl_ref_pub_time;
  control_msgs::msg::DynamicJointState _expl_ref_msg;
  rclcpp::TimerBase::SharedPtr _expl_ref_pub_tmr;
  };

HwInterfaceComm::HwInterfaceComm()
  : rclcpp::Node("orthopus_hw_interface_comm")
{
  // To ros_explorer_bridge
  _expl_ref_pub = this->create_publisher<control_msgs::msg::DynamicJointState>("/explorer_ref", rclcpp::SystemDefaultsQoS());

  // From ros_explorer_bridge
  _expl_meas_sub = this->create_subscription<control_msgs::msg::DynamicJointState>("/explorer_meas", rclcpp::SystemDefaultsQoS(),
  [this](const std::shared_ptr<control_msgs::msg::DynamicJointState> msg) -> void
  {
    //RCLCPP_INFO(rclcpp::get_logger(get_name()), "Got /explorer_meas");
    const auto& _expl_meas_msg = *(msg.get());
    const auto jsz = _expl_meas_msg.joint_names.size();
    for(size_t i=0;i<jsz;i++)
    {
      const auto& jname = _expl_meas_msg.joint_names[i];
      const auto hjsz = _hw_joints.size();
      size_t j;
      for(j=0;j<hjsz;j++)
      {
        const auto& hwjname = _hw_joints[j].name;
        if(jname == hwjname)
          break;
      }
      if(j >= hjsz)
      {
        //RCLCPP_ERROR(rclcpp::get_logger("HwInterface"), "[%s] Unknown joint: %s", _name.c_str(), jname.c_str());
        continue;
      }
      auto& hwjst = _hw_joints[j].state;

      auto& ifv = _expl_meas_msg.interface_values[i];
      const auto& ifsz = ifv.interface_names.size();
      for(size_t j=0;j<ifsz;j++)
      {
        auto& ifval = ifv.values[j];
        const auto& ifn = ifv.interface_names[j];
        if(ifn == HW_IF_POSITION)
          hwjst.pos = ifval;
        else if(ifn == HW_IF_VELOCITY)
          hwjst.vel = ifval;
        else if(ifn == HW_IF_ACCELERATION)
          hwjst.acc = ifval;
        else if(ifn == HW_IF_EFFORT)
          hwjst.eff = ifval;
        else
        {
          //RCLCPP_ERROR(rclcpp::get_logger("HwInterface"), "[%s][%s] Unknown interface: %s", _name.c_str(), _hw_joints[j].name.c_str(),ifn.c_str());
        }
      }
    }
  });

  // Create a ROS timer for publishing refs
  _expl_ref_pub_tmr = this->create_wall_timer(std::chrono::milliseconds(10), [this](void)
  {
    //RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "This: %p", this);
    bool pub = false;
    rclcpp::Time start = this->get_clock()->now();
    //RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "Début : %f s", start.seconds());
    size_t j = 0;
    for(auto& ifv: _expl_ref_msg.interface_values)
    {
      _expl_ref_msg.header.stamp.sec = start.seconds();
      _expl_ref_msg.header.stamp.nanosec = start.nanoseconds();

      auto& hw_ref = _hw_joints[j].ref;
      const auto sz = ifv.interface_names.size();
      for (size_t i=0;i<sz;i++)
      {
        const auto& intf_name = ifv.interface_names[i];
        auto& intf_val = ifv.values[i];
        bool jpub = true;
        if(intf_name == HW_IF_POSITION)// && hw_ref.nwpos)
        {
          intf_val = hw_ref.pos;
          //hw_ref.nwpos = false;
        }
        else if(intf_name == HW_IF_VELOCITY)// && hw_ref.nwvel)
        {
          intf_val = hw_ref.vel;
          //hw_ref.nwvel = false;
        }
        else if(intf_name == HW_IF_ACCELERATION)// && hw_ref.nwacc)
        {
          intf_val = hw_ref.acc;
          //hw_ref.nwacc = false;
        }
        else if(intf_name == HW_IF_EFFORT)// && hw_ref.nweff)
        {
          intf_val = hw_ref.eff;
          //hw_ref.nweff = false;
        }
        else
          jpub = false;
        pub = pub || jpub;
      }
      j++;
    }
    if(pub)
    {
      rclcpp::Time end = this->get_clock()->now();
      auto dure = end-start;
      //RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "Fin : %f s", end.seconds());
      //RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "[%s] Publish !", _name.c_str());
      _expl_ref_pub->publish(_expl_ref_msg);
    }
  });

  _this_comm = std::shared_ptr<HwInterfaceComm>(this,[](HwInterfaceComm*){});
  _executor.add_node(_this_comm);
  _comm_th = std::thread([this]() { this->_executor.spin(); });

}

HwInterfaceComm& HwInterfaceComm::getInstance()
{
  static HwInterfaceComm _comm;
  return _comm;
}

HwInterfaceComm::hw_joint_t& HwInterfaceComm::getJoint(HwInterface* owner, const std::string& joint_name)
{
  for(auto& hwj: _hw_joints)
  {
    if(hwj.owner == owner && hwj.name == joint_name)
      return hwj;
  }
  auto& hwj = _hw_joints.emplace_back(hw_joint_t{
    /*.owner   = */owner,
    /*.name    = */joint_name,
    /*.enable  = */false,
    /*.state   = */{
      /* .pos = */0.0,//std::numeric_limits<double>::quiet_NaN(),
      /* .vel = */0.0,//std::numeric_limits<double>::quiet_NaN(),
      /* .acc = */0.0,//std::numeric_limits<double>::quiet_NaN(),
      /* .eff = */0.0,//std::numeric_limits<double>::quiet_NaN(),
    },
    /*.ref = */{
      /* .pos = */std::numeric_limits<double>::quiet_NaN(),
      /* .vel = */std::numeric_limits<double>::quiet_NaN(),
      /* .acc = */std::numeric_limits<double>::quiet_NaN(),
      /* .eff = */std::numeric_limits<double>::quiet_NaN(),
    },
  });

  _expl_ref_msg.joint_names.emplace_back(joint_name);

  const std::array<std::string,4> intfs{
    HW_IF_POSITION,
    HW_IF_VELOCITY,
    HW_IF_ACCELERATION,
    HW_IF_EFFORT
  };

  auto& ifv = _expl_ref_msg.interface_values.emplace_back(control_msgs::msg::InterfaceValue{});
  for (const auto& intf: intfs)
  {
    ifv.interface_names.emplace_back(intf);
    ifv.values.emplace_back(std::numeric_limits<double>::quiet_NaN());
  }

  return hwj;
}

std::shared_ptr<HwInterfaceComm::hw_joints_t> HwInterfaceComm::getJoints()
{
  _lck.lock();
  return std::shared_ptr<hw_joints_t>(&_hw_joints, [this](hw_joints_t*) { _lck.unlock(); });
}

HwInterface::~HwInterface()
{

}

CallbackReturn HwInterface::on_init(const HardwareInfo& info)
{
  if (ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  for(auto& joint: info_.joints)
  {
    auto& hw_j = HwInterfaceComm::getInstance().getJoint(this, joint.name);
    hw_j.enable = true;

    // Command interfaces
    // _command_interfaces can't be a class item, else build fails on export_command_interfaces

    if (joint.command_interfaces.size() > 4)
    {
      RCLCPP_FATAL(rclcpp::get_logger("HwInterface"),"Joint '%s' has %zu command interfaces found. max 4 expected.", hw_j.name.c_str(),joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }
    for(const auto& intf: joint.command_interfaces)
    {
      if(    intf.name != HW_IF_POSITION
          && intf.name != HW_IF_VELOCITY
          && intf.name != HW_IF_ACCELERATION
          && intf.name != HW_IF_EFFORT
        )
      {
        RCLCPP_FATAL(rclcpp::get_logger("HwInterface"),"[%s][%s] Unsupported command interface '%s'", _name.c_str(), hw_j.name.c_str(), intf.name.c_str());
        return CallbackReturn::ERROR;
      }
    }

    // State interfaces
    _state_interfaces.emplace_back(hardware_interface::StateInterface(hw_j.name, hardware_interface::HW_IF_POSITION, &hw_j.state.pos));
    _state_interfaces.emplace_back(hardware_interface::StateInterface(hw_j.name, hardware_interface::HW_IF_VELOCITY, &hw_j.state.vel));
    _state_interfaces.emplace_back(hardware_interface::StateInterface(hw_j.name, hardware_interface::HW_IF_ACCELERATION, &hw_j.state.acc));
    _state_interfaces.emplace_back(hardware_interface::StateInterface(hw_j.name, hardware_interface::HW_IF_EFFORT, &hw_j.state.eff));

    if (joint.state_interfaces.size() > _state_interfaces.size())
    {
      RCLCPP_FATAL(rclcpp::get_logger("HwInterface"),"Joint '%s' has %zu state interfaces found. max %ld expected.", hw_j.name.c_str(),joint.state_interfaces.size(), _state_interfaces.size());
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
        RCLCPP_FATAL(rclcpp::get_logger("HwInterface"),"[%s][%s] Unsupported state interface '%s'", _name.c_str(), hw_j.name.c_str(), intf.name.c_str());
        return CallbackReturn::ERROR;
      }
    }
  }

  //RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "[%s] Successfully initialized!", _name.c_str());
  auto& _hw_joints = *(HwInterfaceComm::getInstance().getJoints());
  for(auto& joint: _hw_joints)
  {
    if(!(joint.owner == this && joint.enable))
      continue;
    RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "[%s][%s] Successfully initialized!", _name.c_str(),joint.name.c_str());
  }

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
  auto& _hw_joints = *(HwInterfaceComm::getInstance().getJoints());
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
  return _command_interfaces;
}

ORTHOPUS_ROS_PUBLIC
return_type HwInterface::prepare_command_mode_switch([[maybe_unused]] const std::vector<std::string>&start_if, [[maybe_unused]] const std::vector<std::string> &stop_if)
{
  auto& _hw_joints = *(HwInterfaceComm::getInstance().getJoints());
  RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "[%s] Preparing Command mode switch...", _name.c_str());
  RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "  Starting Interfaces:");
  for(const auto& st_if: start_if)
  {
    for(const auto& joint: _hw_joints)
    {
      if(!(joint.owner == this && joint.enable))
        continue;
      const auto jnsz = joint.name.length();
      if(!st_if.compare(0,jnsz,joint.name)) // Check only for us
        RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "   - %s",st_if.c_str());
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "  Stopping Interfaces:");
  for(const auto& st_if: stop_if)
  {
    for(const auto& joint: _hw_joints)
    {
      if(!(joint.owner == this && joint.enable))
        continue;
      const auto jnsz = joint.name.length();
      if(!st_if.compare(0,jnsz,joint.name)) // Check only for us
        RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "   - %s",st_if.c_str());
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "[%s] Command mode switch prepared!", _name.c_str());
  return return_type::OK;
}

ORTHOPUS_ROS_PUBLIC
return_type HwInterface::perform_command_mode_switch([[maybe_unused]] const std::vector<std::string>&start_if, [[maybe_unused]] const std::vector<std::string> &stop_if)
{
  auto& _hw_joints = *(HwInterfaceComm::getInstance().getJoints());
  RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "[%s] Performing Command mode switch...", _name.c_str());
  RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "  Starting Interfaces:");
  for(const auto& st_if: start_if)
  {
    for(const auto& joint: _hw_joints)
    {
      if(!(joint.owner == this && joint.enable))
        continue;
      const auto jnsz = joint.name.length();
      if(!st_if.compare(0,jnsz,joint.name)) // Check only for us
        RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "   - %s",st_if.c_str());
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "  Stopping Interfaces:");
  for(const auto& st_if: stop_if)
  {
    for(const auto& joint: _hw_joints)
    {
      if(!(joint.owner == this && joint.enable))
        continue;
      const auto jnsz = joint.name.length();
      if(!st_if.compare(0,jnsz,joint.name)) // Check only for us
        RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "   - %s",st_if.c_str());
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "[%s] Command mode switch performed!", _name.c_str());
  return return_type::OK;
}

CallbackReturn HwInterface::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  auto& _hw_joints = *(HwInterfaceComm::getInstance().getJoints());
  RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "[%s] Activating ...please wait...", _name.c_str());
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
  auto& _hw_joints = *(HwInterfaceComm::getInstance().getJoints());
  for(auto& joint: _hw_joints)
  {
    if(!(joint.owner == this && joint.enable))
      continue;
    auto& hw_ref = joint.ref;
    if(!std::isnan(hw_ref.pos))// && hw_ref.pos != hw_ref.ppos)
    {
      //RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "[%s][%s] Writing new pos command: %f...", _name.c_str(), joint.name.c_str(), hw_ref.pos);
      //hw_ref.ppos = hw_ref.pos;
      //hw_ref.nwpos = true;
    }
    if(!std::isnan(hw_ref.vel))// && hw_ref.vel != hw_ref.pvel)
    {
      //RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "[%s][%s] Writing new vel command: %f...", _name.c_str(), joint.name.c_str(), hw_ref.vel);
      //hw_ref.pvel = hw_ref.vel;
      //hw_ref.nwvel = true;
    }
    if(!std::isnan(hw_ref.acc))// && hw_ref.acc != hw_ref.pacc)
    {
      //RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "[%s][%s] Writing new acc command: %f...", _name.c_str(), joint.name.c_str(), hw_ref.acc);
      //hw_ref.pacc = hw_ref.acc;
      //hw_ref.nwacc = true;
    }
    if(!std::isnan(hw_ref.eff))// && hw_ref.eff != hw_ref.peff)
    {
      //RCLCPP_INFO(rclcpp::get_logger("HwInterface"), "[%s][%s] Writing new eff command: %f...", _name.c_str(), joint.name.c_str(), hw_ref.eff);
      //hw_ref.peff = hw_ref.eff;
      //hw_ref.nweff = true;
    }
  }
  return return_type::OK;
}

}  // namespace orthopus_ros_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(orthopus_ros_interface::HwInterface, ActuatorInterface)
