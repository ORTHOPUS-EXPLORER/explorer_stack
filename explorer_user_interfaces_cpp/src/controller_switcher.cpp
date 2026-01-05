#include "explorer_user_interfaces_cpp/controller_switcher.h"

namespace space_control
{

  ControllerSwitcher::ControllerSwitcher(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    client_ = node_->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller");

    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_INFO(node_->get_logger(), "wait for switch_controller...");
    }
  }

  bool ControllerSwitcher::switch_controller(const std::vector<std::string>& stop, const std::vector<std::string>& start)
  {
    auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  
    req->deactivate_controllers = stop;
    req->activate_controllers   = start;
    req->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
    req->activate_asap = true;
    req->timeout = rclcpp::Duration::from_seconds(5.0);
  
    client_->async_send_request(req, [this](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future)
      {
        auto response = future.get();
  
        if (!response->ok) {
          RCLCPP_ERROR(node_->get_logger(), "Controller switch failed");
        } else {
          RCLCPP_INFO(node_->get_logger(), "Controller switch successful");
        }
      });
  
    return true;
  }
}  