#include <controller_manager_msgs/srv/detail/list_controllers__struct.hpp>
#include <string>
#include <vector>

#include "controller_manager_msgs/srv/list_controllers.h"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "rclcpp/rclcpp.hpp"

namespace space_control
{

class ControllerManagerWrapper
{
public:
  explicit ControllerManagerWrapper(rclcpp::Node::SharedPtr node);

  std::future<bool> switch_controller_async(
    const std::vector<std::string>& stop, const std::vector<std::string>& start);
  std::future<std::vector<std::string>> get_controller_list(
    const std::optional<std::string> &state_filter, std::optional<bool> with_command_interface,
    std::optional<bool> with_state_interface);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
    switch_controller_client_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_client_;
};

}  // namespace space_control