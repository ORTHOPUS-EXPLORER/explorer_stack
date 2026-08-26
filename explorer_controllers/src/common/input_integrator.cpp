#include "explorer_controllers/common/input_integrator.h"

#include "explorer_controllers/common/service_init_utils.h"

namespace space_control
{
InputIntegrator::InputIntegrator(rclcpp::Node::SharedPtr n)
: n_(n), x_desired_(), x_init_(), dx_desired_(), x_current_()
{
  auto debug_enabled = n_->get_parameter("debug").as_bool();
  if (debug_enabled)
  {
    if (rcutils_logging_set_logger_level(n_->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG) != RCUTILS_RET_OK)
    {
      throw std::runtime_error("Couldn't set logger level to DEBUG.");
    }
  }
  //init settings
  max_vel_ = 0.025;
  max_vel_orientation_ = 0.5;
  sampling_period_ = 0.01;

  go_home_ = false;
  go_zero_ = false;
  go_J1_zero_ = false;
  go_J2_zero_ = false;
  go_J3_zero_ = false;
  go_J4_zero_ = false;
  go_J5_zero_ = false;
  go_J6_zero_ = false;

  x_des_updated_.data = false;

  //init variables
  x_desired_.position.x() = 0.0;
  x_desired_.position.y() = 0.0;
  x_desired_.position.z() = 0.0;
  x_desired_.orientation.x() = 0.0;
  x_desired_.orientation.y() = 0.0;
  x_desired_.orientation.z() = 0.0;
  x_desired_.orientation.w() = 1.0;

  x_init_.position.x() = 0.0;
  x_init_.position.y() = 0.0;
  x_init_.position.z() = 0.0;
  x_init_.orientation.x() = 0.0;
  x_init_.orientation.y() = 0.0;
  x_init_.orientation.z() = 0.0;
  x_init_.orientation.w() = 1.0;

  x_current_.position.x() = 0.0;
  x_current_.position.y() = 0.0;
  x_current_.position.z() = 0.0;
  x_current_.orientation.x() = 0.0;
  x_current_.orientation.y() = 0.0;
  x_current_.orientation.z() = 0.0;
  x_current_.orientation.w() = 1.0;

  //init suscribers
  input_sub_ = n_->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/explorer_user_interfaces/rqt_armcontrol/input_device_velocity", 10,
    std::bind(&InputIntegrator::callback_input_, this, std::placeholders::_1));

  linear_speed_sub_ = n_->create_subscription<std_msgs::msg::Float64>(
    "/explorer_user_interfaces/rqt_armcontrol/max_linear_speed", 10,
    std::bind(&InputIntegrator::callback_linear_speed_, this, std::placeholders::_1));
  angular_speed_sub_ = n_->create_subscription<std_msgs::msg::Float64>(
    "/explorer_user_interfaces/rqt_armcontrol/max_angular_speed", 10,
    std::bind(&InputIntegrator::callback_angular_speed_, this, std::placeholders::_1));
  x_current_sub_ = n_->create_subscription<geometry_msgs::msg::Pose>(
    "/explorer_controllers/qp_solving/x_current", 10,
    std::bind(&InputIntegrator::callback_x_current_, this, std::placeholders::_1));
  home_released_sub_ = n_->create_subscription<std_msgs::msg::Bool>(
    "/explorer_user_interfaces/rqt_armcontrol/home_released", 10,
    std::bind(&InputIntegrator::callback_home_released_, this, std::placeholders::_1));
  home_pressed_sub_ = n_->create_subscription<std_msgs::msg::Bool>(
    "/explorer_user_interfaces/rqt_armcontrol/home_pressed", 10,
    std::bind(&InputIntegrator::callback_home_pressed_, this, std::placeholders::_1));
  zero_released_sub_ = n_->create_subscription<std_msgs::msg::Bool>(
    "/explorer_user_interfaces/rqt_armcontrol/zero_released", 10,
    std::bind(&InputIntegrator::callback_zero_released_, this, std::placeholders::_1));
  zero_pressed_sub_ = n_->create_subscription<std_msgs::msg::Bool>(
    "/explorer_user_interfaces/rqt_armcontrol/zero_pressed", 10,
    std::bind(&InputIntegrator::callback_zero_pressed_, this, std::placeholders::_1));
  J1_zero_released_sub_ = n_->create_subscription<std_msgs::msg::Bool>(
    "/explorer_user_interfaces/rqt_armcontrol/J1_zero_released", 10,
    std::bind(&InputIntegrator::callback_J1_zero_released_, this, std::placeholders::_1));
  J1_zero_pressed_sub_ = n_->create_subscription<std_msgs::msg::Bool>(
    "/explorer_user_interfaces/rqt_armcontrol/J1_zero_pressed", 10,
    std::bind(&InputIntegrator::callback_J1_zero_pressed_, this, std::placeholders::_1));
  J2_zero_released_sub_ = n_->create_subscription<std_msgs::msg::Bool>(
    "/explorer_user_interfaces/rqt_armcontrol/J2_zero_released", 10,
    std::bind(&InputIntegrator::callback_J2_zero_released_, this, std::placeholders::_1));
  J2_zero_pressed_sub_ = n_->create_subscription<std_msgs::msg::Bool>(
    "/explorer_user_interfaces/rqt_armcontrol/J2_zero_pressed", 10,
    std::bind(&InputIntegrator::callback_J2_zero_pressed_, this, std::placeholders::_1));
  J3_zero_released_sub_ = n_->create_subscription<std_msgs::msg::Bool>(
    "/explorer_user_interfaces/rqt_armcontrol/J3_zero_released", 10,
    std::bind(&InputIntegrator::callback_J3_zero_released_, this, std::placeholders::_1));
  J3_zero_pressed_sub_ = n_->create_subscription<std_msgs::msg::Bool>(
    "/explorer_user_interfaces/rqt_armcontrol/J3_zero_pressed", 10,
    std::bind(&InputIntegrator::callback_J3_zero_pressed_, this, std::placeholders::_1));
  J4_zero_released_sub_ = n_->create_subscription<std_msgs::msg::Bool>(
    "/explorer_user_interfaces/rqt_armcontrol/J4_zero_released", 10,
    std::bind(&InputIntegrator::callback_J4_zero_released_, this, std::placeholders::_1));
  J4_zero_pressed_sub_ = n_->create_subscription<std_msgs::msg::Bool>(
    "/explorer_user_interfaces/rqt_armcontrol/J4_zero_pressed", 10,
    std::bind(&InputIntegrator::callback_J4_zero_pressed_, this, std::placeholders::_1));
  J5_zero_released_sub_ = n_->create_subscription<std_msgs::msg::Bool>(
    "/explorer_user_interfaces/rqt_armcontrol/J5_zero_released", 10,
    std::bind(&InputIntegrator::callback_J5_zero_released_, this, std::placeholders::_1));
  J5_zero_pressed_sub_ = n_->create_subscription<std_msgs::msg::Bool>(
    "/explorer_user_interfaces/rqt_armcontrol/J5_zero_pressed", 10,
    std::bind(&InputIntegrator::callback_J5_zero_pressed_, this, std::placeholders::_1));
  J6_zero_released_sub_ = n_->create_subscription<std_msgs::msg::Bool>(
    "/explorer_user_interfaces/rqt_armcontrol/J6_zero_released", 10,
    std::bind(&InputIntegrator::callback_J6_zero_released_, this, std::placeholders::_1));
  J6_zero_pressed_sub_ = n_->create_subscription<std_msgs::msg::Bool>(
    "/explorer_user_interfaces/rqt_armcontrol/J6_zero_pressed", 10,
    std::bind(&InputIntegrator::callback_J6_zero_pressed_, this, std::placeholders::_1));

  x_init_client_ =
    n_->create_client<explorer_msgs::srv::Pose>("/explorer_controllers/qp_solving/x_init");

  //init publishers
  dx_desired_pub_ = n_->create_publisher<geometry_msgs::msg::Pose>(
    "/explorer_controllers/input_integrator/dx_desired", 10);
  x_desired_pub_ = n_->create_publisher<geometry_msgs::msg::Pose>(
    "/explorer_controllers/input_integrator/x_desired", 10);
  x_des_updated_pub_ = n_->create_publisher<std_msgs::msg::Bool>(
    "/explorer_controllers/input_integrator/x_des_updated", 10);

  auto request = std::make_shared<explorer_msgs::srv::Pose::Request>();
  request->ready = true;

  auto response = wait_and_call_init_service<explorer_msgs::srv::Pose>(n_, x_init_client_, request);

  x_init_pose_ = response->pose;
  x_init_.position.x() = x_init_pose_.position.x;
  x_init_.position.y() = x_init_pose_.position.y;
  x_init_.position.z() = x_init_pose_.position.z;
  x_init_.orientation.w() = x_init_pose_.orientation.w;
  x_init_.orientation.x() = x_init_pose_.orientation.x;
  x_init_.orientation.y() = x_init_pose_.orientation.y;
  x_init_.orientation.z() = x_init_pose_.orientation.z;
  RCLCPP_INFO(n_->get_logger(), "input_integrator_initialized");

  //init x_desired with the simulation
  x_desired_ = x_init_;
  send_input_();

  timer_ = n_->create_wall_timer(10ms, std::bind(&InputIntegrator::timer_callback_, this));
}

void InputIntegrator::callback_input_(const geometry_msgs::msg::TwistStamped& msg)
{
  dx_input_ = msg;
}

void InputIntegrator::callback_linear_speed_(const std_msgs::msg::Float64& msg)
{
  max_vel_ = msg.data;
}

void InputIntegrator::callback_angular_speed_(const std_msgs::msg::Float64& msg)
{
  max_vel_orientation_ = msg.data;
}

void InputIntegrator::callback_x_current_(const geometry_msgs::msg::Pose& msg)
{
  x_current_.position.x() = msg.position.x;
  x_current_.position.y() = msg.position.y;
  x_current_.position.z() = msg.position.z;
  x_current_.orientation.w() = msg.orientation.w;
  x_current_.orientation.x() = msg.orientation.x;
  x_current_.orientation.y() = msg.orientation.y;
  x_current_.orientation.z() = msg.orientation.z;
}

void InputIntegrator::callback_home_released_(const std_msgs::msg::Bool& msg)
{
  if (msg.data == true)
  {
    x_desired_ = x_current_;
    x_des_updated_.data = true;
    go_home_ = false;
  }
}

void InputIntegrator::callback_home_pressed_(const std_msgs::msg::Bool& msg)
{
  if (msg.data == true)
  {
    go_home_ = true;
    x_des_updated_.data = false;
    x_des_updated_pub_->publish(x_des_updated_);
  }
}

void InputIntegrator::callback_zero_released_(const std_msgs::msg::Bool& msg)
{
  if (msg.data == true)
  {
    x_desired_ = x_current_;
    x_des_updated_.data = true;
    go_zero_ = false;
  }
}

void InputIntegrator::callback_zero_pressed_(const std_msgs::msg::Bool& msg)
{
  if (msg.data == true)
  {
    go_zero_ = true;
    x_des_updated_.data = false;
    x_des_updated_pub_->publish(x_des_updated_);
  }
}

void InputIntegrator::callback_J1_zero_released_(const std_msgs::msg::Bool& msg)
{
  if (msg.data == true)
  {
    x_desired_ = x_current_;
    x_des_updated_.data = true;
    go_J1_zero_ = false;
  }
}

void InputIntegrator::callback_J1_zero_pressed_(const std_msgs::msg::Bool& msg)
{
  if (msg.data == true)
  {
    go_J1_zero_ = true;
    x_des_updated_.data = false;
    x_des_updated_pub_->publish(x_des_updated_);
  }
}

void InputIntegrator::callback_J2_zero_released_(const std_msgs::msg::Bool& msg)
{
  if (msg.data == true)
  {
    x_desired_ = x_current_;
    x_des_updated_.data = true;
    go_J2_zero_ = false;
  }
}

void InputIntegrator::callback_J2_zero_pressed_(const std_msgs::msg::Bool& msg)
{
  if (msg.data == true)
  {
    go_J2_zero_ = true;
    x_des_updated_.data = false;
    x_des_updated_pub_->publish(x_des_updated_);
  }
}

void InputIntegrator::callback_J3_zero_released_(const std_msgs::msg::Bool& msg)
{
  if (msg.data == true)
  {
    x_desired_ = x_current_;
    x_des_updated_.data = true;
    go_J3_zero_ = false;
  }
}

void InputIntegrator::callback_J3_zero_pressed_(const std_msgs::msg::Bool& msg)
{
  if (msg.data == true)
  {
    go_J3_zero_ = true;
    x_des_updated_.data = false;
    x_des_updated_pub_->publish(x_des_updated_);
  }
}

void InputIntegrator::callback_J4_zero_released_(const std_msgs::msg::Bool& msg)
{
  if (msg.data == true)
  {
    x_desired_ = x_current_;
    x_des_updated_.data = true;
    go_J4_zero_ = false;
  }
}

void InputIntegrator::callback_J4_zero_pressed_(const std_msgs::msg::Bool& msg)
{
  if (msg.data == true)
  {
    go_J4_zero_ = true;
    x_des_updated_.data = false;
    x_des_updated_pub_->publish(x_des_updated_);
  }
}
void InputIntegrator::callback_J5_zero_released_(const std_msgs::msg::Bool& msg)
{
  if (msg.data == true)
  {
    x_desired_ = x_current_;
    x_des_updated_.data = true;
    go_J5_zero_ = false;
  }
}

void InputIntegrator::callback_J5_zero_pressed_(const std_msgs::msg::Bool& msg)
{
  if (msg.data == true)
  {
    go_J5_zero_ = true;
    x_des_updated_.data = false;
    x_des_updated_pub_->publish(x_des_updated_);
  }
}

void InputIntegrator::callback_J6_zero_released_(const std_msgs::msg::Bool& msg)
{
  if (msg.data == true)
  {
    x_desired_ = x_current_;
    x_des_updated_.data = true;
    go_J6_zero_ = false;
  }
}

void InputIntegrator::callback_J6_zero_pressed_(const std_msgs::msg::Bool& msg)
{
  if (msg.data == true)
  {
    go_J6_zero_ = true;
    x_des_updated_.data = false;
    x_des_updated_pub_->publish(x_des_updated_);
  }
}

void InputIntegrator::timer_callback_()
{
  tf2::Quaternion q_orig, q_rot, q_new;

  if (
    !go_home_ && !go_zero_ && !go_J1_zero_ && !go_J2_zero_ && !go_J3_zero_ && !go_J4_zero_ &&
    !go_J5_zero_ && !go_J6_zero_)
  {
    dx_desired_.position.x() = (dx_input_.twist.linear.x * max_vel_);
    dx_desired_.position.y() = (dx_input_.twist.linear.y * max_vel_);
    dx_desired_.position.z() = (dx_input_.twist.linear.z * max_vel_);
    dx_desired_.orientation.w() = (0.0);
    dx_desired_.orientation.x() = (dx_input_.twist.angular.x * max_vel_orientation_);
    dx_desired_.orientation.y() = (dx_input_.twist.angular.y * max_vel_orientation_);
    dx_desired_.orientation.z() = (dx_input_.twist.angular.z * max_vel_orientation_);

    //Integrates cartesian velocities to get cartesian positions
    x_desired_.position.x() =
      (x_desired_.position.x() + dx_desired_.position.x() * sampling_period_);
    x_desired_.position.y() =
      (x_desired_.position.y() + dx_desired_.position.y() * sampling_period_);
    x_desired_.position.z() =
      (x_desired_.position.z() + dx_desired_.position.z() * sampling_period_);

    //converts in quaternion
    q_orig[0] = x_desired_.orientation.x();
    q_orig[1] = x_desired_.orientation.y();
    q_orig[2] = x_desired_.orientation.z();
    q_orig[3] = x_desired_.orientation.w();

    q_rot.setRPY(
      dx_desired_.orientation.x() * sampling_period_,
      dx_desired_.orientation.y() * sampling_period_,
      dx_desired_.orientation.z() * sampling_period_);

    q_new = q_rot * q_orig;
    q_new.normalize();

    x_desired_.orientation.x() = (q_new[0]);
    x_desired_.orientation.y() = (q_new[1]);
    x_desired_.orientation.z() = (q_new[2]);
    x_desired_.orientation.w() = (q_new[3]);

    send_input_();

    x_des_updated_pub_->publish(x_des_updated_);
  }
}

void InputIntegrator::send_input_()
{
  geometry_msgs::msg::Pose x_desired_pose;
  x_desired_pose.position.x = x_desired_.position.x();
  x_desired_pose.position.y = x_desired_.position.y();
  x_desired_pose.position.z = x_desired_.position.z();
  x_desired_pose.orientation.w = x_desired_.orientation.w();
  x_desired_pose.orientation.x = x_desired_.orientation.x();
  x_desired_pose.orientation.y = x_desired_.orientation.y();
  x_desired_pose.orientation.z = x_desired_.orientation.z();

  geometry_msgs::msg::Pose dx_desired_pose;
  dx_desired_pose.position.x = dx_desired_.position.x();
  dx_desired_pose.position.y = dx_desired_.position.y();
  dx_desired_pose.position.z = dx_desired_.position.z();
  dx_desired_pose.orientation.w = dx_desired_.orientation.w();
  dx_desired_pose.orientation.x = dx_desired_.orientation.x();
  dx_desired_pose.orientation.y = dx_desired_.orientation.y();
  dx_desired_pose.orientation.z = dx_desired_.orientation.z();

  dx_desired_pub_->publish(dx_desired_pose);
  x_desired_pub_->publish(x_desired_pose);
}
}  // namespace space_control

using namespace space_control;
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto n = rclcpp::Node::make_shared("input_integrator", node_options);

  InputIntegrator input_integrator(n);

  rclcpp::spin(n);
  rclcpp::shutdown();
  return 0;
}
