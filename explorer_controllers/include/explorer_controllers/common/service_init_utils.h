#ifndef SERVICE_INIT_UTILS_H
#define SERVICE_INIT_UTILS_H

#include <algorithm>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>

namespace space_control
{
// Blocks until a service call returns a response with `code_error == 0`, or throws
// std::runtime_error if `timeout` elapses first. Must be called before the
// node's executor starts spinning (e.g. from a constructor, before main()
// calls rclcpp::spin), since it spins `node` itself while waiting.
template <typename ServiceT>
typename ServiceT::Response::SharedPtr wait_and_call_init_service(
  const rclcpp::Node::SharedPtr& node, const typename rclcpp::Client<ServiceT>::SharedPtr& client,
  const typename ServiceT::Request::SharedPtr& request,
  std::chrono::seconds timeout = std::chrono::seconds(30),
  std::chrono::milliseconds retry_period = std::chrono::milliseconds(200))
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  auto time_left = [&deadline]() { return deadline - std::chrono::steady_clock::now(); };
  const std::string service_name = client->get_service_name();

  while (!client->wait_for_service(std::min(
    time_left(),
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::seconds(1)))))
  {
    if (!rclcpp::ok())
    {
      throw std::runtime_error("Interrupted while waiting for service '" + service_name + "'");
    }
    if (time_left() <= std::chrono::steady_clock::duration::zero())
    {
      throw std::runtime_error(
        "Timed out waiting for service '" + service_name + "' to become available");
    }
    RCLCPP_INFO_THROTTLE(
      node->get_logger(), *node->get_clock(), 5000, "Service '%s' not available, waiting...",
      service_name.c_str());
  }

  while (true)
  {
    auto future = client->async_send_request(request);
    auto rc = rclcpp::spin_until_future_complete(node, future, time_left());
    if (rc == rclcpp::FutureReturnCode::SUCCESS)
    {
      auto response = future.get();
      if (response->code_error == 0)
      {
        return response;
      }
    }
    else if (rc == rclcpp::FutureReturnCode::INTERRUPTED)
    {
      throw std::runtime_error("Interrupted while calling service '" + service_name + "'");
    }

    if (time_left() <= std::chrono::steady_clock::duration::zero())
    {
      throw std::runtime_error(
        "Service '" + service_name + "' never became ready before timeout");
    }
    RCLCPP_INFO_THROTTLE(
      node->get_logger(), *node->get_clock(), 2000, "Service '%s' not ready yet, retrying...",
      service_name.c_str());
    rclcpp::sleep_for(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::min(
        retry_period, std::chrono::duration_cast<std::chrono::milliseconds>(time_left()))));
  }
}
}  // namespace space_control
#endif
