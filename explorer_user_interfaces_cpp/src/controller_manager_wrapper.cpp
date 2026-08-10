#include "explorer_user_interfaces_cpp/controller_manager_wrapper.h"

#include <controller_manager_msgs/msg/controller_state.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <utility>

namespace space_control
{

ControllerManagerWrapper::ControllerManagerWrapper(rclcpp::Node::SharedPtr node)
: node_(std::move(node))
{
  switch_controller_client_ = node_->create_client<controller_manager_msgs::srv::SwitchController>(
    "/controller_manager/switch_controller");
  list_controllers_client_ = node_->create_client<controller_manager_msgs::srv::ListControllers>(
    "/controller_manager/list_controllers");

  while (rclcpp::ok() && !switch_controller_client_->wait_for_service(std::chrono::seconds(1)) &&
         !list_controllers_client_->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000, "Waiting for controller_manager services...");
  }
}

std::future<bool> ControllerManagerWrapper::switch_controller_async(
  const std::vector<std::string>& stop, const std::vector<std::string>& start)
{
  auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
  request->deactivate_controllers = stop;
  request->activate_controllers = start;
  request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
  request->activate_asap = true;
  request->timeout = rclcpp::Duration::from_seconds(5.0);

  auto promise = std::make_shared<std::promise<bool>>();
  auto future_result = promise->get_future();

  switch_controller_client_->async_send_request(
    request,
    [this,
     promise](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future)
    {
      const auto& response = future.get();
      if (response->ok)
      {
        RCLCPP_INFO(node_->get_logger(), "Controller switch successful");
        promise->set_value(true);
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Controller switch failed");
        promise->set_value(false);
      }
    });

  return future_result;
}

std::future<std::vector<std::string>> ControllerManagerWrapper::get_controller_list(
  const std::optional<std::string>& state_filter, std::optional<bool> with_command_interface,
  std::optional<bool> with_state_interface)
{
  auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  auto promise = std::make_shared<std::promise<std::vector<std::string>>>();
  auto future_result = promise->get_future();

  list_controllers_client_->async_send_request(
    request,
    [promise, state_filter, with_command_interface, with_state_interface](
      rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future)
    {
      std::vector<std::string> result;
      auto controller_list = future.get()->controller;
      for (const auto& controller : controller_list)
      {
        if (
          (!state_filter.has_value() || state_filter.value() == controller.state) &&
          (!with_command_interface.has_value() ||
           with_command_interface.value() == (controller.required_command_interfaces.size() > 0)) &&
          (!with_state_interface.has_value() ||
           with_state_interface.value() == (controller.required_state_interfaces.size() > 0)))
        {
          result.push_back(controller.name);
        }
      }
      promise->set_value(result);
    });

  return future_result;
}

}  // namespace space_control
