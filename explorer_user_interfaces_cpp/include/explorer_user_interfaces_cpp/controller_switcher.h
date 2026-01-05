#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"

namespace space_control
{

  class ControllerSwitcher
  {
    public:
      explicit ControllerSwitcher(const rclcpp::Node::SharedPtr& node);
    
      bool switch_controller(
        const std::vector<std::string>& stop,
        const std::vector<std::string>& start);
    
    private:
      rclcpp::Node::SharedPtr node_;
      rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr client_;
  };

}  