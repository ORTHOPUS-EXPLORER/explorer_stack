#include <algorithm>
#include <cstdlib>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <sstream>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace
{
std::string run_xacro(const std::string& xacro_path)
{
  const std::string output_path = "/tmp/explorer_gravity_compensation.urdf";
  const std::string command = std::string("xacro ") + xacro_path + " > " + output_path;

  const int status = std::system(command.c_str());
  if (status != 0) {
    throw std::runtime_error("Failed to process Xacro file via 'xacro': " + xacro_path);
  }
  return output_path;
}

std::string resolve_input_path(const std::string& input_path)
{
  if (input_path.empty()) {
    throw std::runtime_error("No input path supplied.");
  }

  const std::string extension = input_path.substr(input_path.find_last_of('.') + 1);
  if (extension == "xacro") {
    return run_xacro(input_path);
  }
  return input_path;
}
}  // namespace

class GravityCompensationNode : public rclcpp::Node
{
public:
  GravityCompensationNode()
  : Node("gravity_compensation_node")
  {
    this->declare_parameter("urdf_path", std::string("/root/explorer_ws/explorer_stack/explorer_description/urdf/explorer.urdf.xacro"));
    this->declare_parameter("publish_topic", std::string("/explorer_controllers/gravity_compensation/torque"));
    this->declare_parameter("joint_state_topic", std::string("/joint_states"));

    std::string urdf_path;
    std::string joint_state_topic;
    this->get_parameter("urdf_path", urdf_path);
    this->get_parameter("joint_state_topic", joint_state_topic);

    if (!initialize_model(urdf_path)) {
      return;
    }

    latest_q_ = Eigen::VectorXd::Zero(model_.nq);
    has_received_joint_state_ = false;

    std::string publish_topic;
    this->get_parameter("publish_topic", publish_topic);
    torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(publish_topic, 10);
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic, 10,
      std::bind(&GravityCompensationNode::joint_state_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&GravityCompensationNode::publish_gravity_torque, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Gravity compensation node ready. Loaded model with %d joints and %d positions.",
      model_.njoints, model_.nq);
  }

private:
  bool initialize_model(const std::string& urdf_path)
  {
    if (urdf_path.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No URDF path supplied. Set the 'urdf_path' parameter.");
      return false;
    }

    try {
      const std::string resolved_path = resolve_input_path(urdf_path);
      load_model_from_urdf(resolved_path);
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(this->get_logger(), "Unable to load Pinocchio model: %s", ex.what());
      return false;
    }

    if (model_.nq <= 0 || model_.njoints <= 0) {
      RCLCPP_ERROR(this->get_logger(), "Loaded Pinocchio model is empty or invalid.");
      return false;
    }

    return true;
  }

  void load_model_from_urdf(const std::string& urdf_path)
  {
    pinocchio::urdf::buildModel(urdf_path, model_, false);
    data_ = pinocchio::Data(model_);
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    latest_q_ = Eigen::VectorXd::Zero(model_.nq);
    has_received_joint_state_ = true;

    if (msg->position.empty()) {
      return;
    }

    if (msg->name.empty()) {
      const Eigen::Index count = std::min<Eigen::Index>(model_.nq, static_cast<Eigen::Index>(msg->position.size()));
      for (Eigen::Index i = 0; i < count; ++i) {
        latest_q_[i] = msg->position[static_cast<std::size_t>(i)];
      }
      return;
    }

    bool mapped_any_joint = false;
    std::size_t q_index = 0;
    for (const auto & joint_name : model_.names) {
      if (q_index >= static_cast<std::size_t>(model_.nq)) {
        break;
      }

      const auto it = std::find(msg->name.begin(), msg->name.end(), joint_name);
      if (it != msg->name.end()) {
        const std::size_t position_index = std::distance(msg->name.begin(), it);
        if (position_index < msg->position.size()) {
          latest_q_[static_cast<Eigen::Index>(q_index)] = msg->position[position_index];
          ++q_index;
          mapped_any_joint = true;
        }
      }
    }

    if (!mapped_any_joint) {
      const Eigen::Index count = std::min<Eigen::Index>(model_.nq, static_cast<Eigen::Index>(msg->position.size()));
      for (Eigen::Index i = 0; i < count; ++i) {
        latest_q_[i] = msg->position[static_cast<std::size_t>(i)];
      }
    }
  }

  void publish_gravity_torque()
  {
    if (latest_q_.size() != model_.nq) {
      latest_q_ = Eigen::VectorXd::Zero(model_.nq);
    }

    if (!has_received_joint_state_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for joint states; using zero joint configuration.");
    }

    pinocchio::computeGeneralizedGravity(model_, data_, latest_q_);

    const Eigen::Index joint_count = std::min<Eigen::Index>(6, data_.g.size());
    std::vector<double> filtered_torques;
    filtered_torques.reserve(static_cast<size_t>(joint_count));
    for (Eigen::Index i = 0; i < joint_count; ++i) {
      filtered_torques.push_back(data_.g[i]);
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data = filtered_torques;

    torque_pub_->publish(msg);

    std::ostringstream joint_names_stream;
    const std::size_t names_to_log = std::min<std::size_t>(6, model_.names.size());
    for (std::size_t i = 0; i < names_to_log; ++i) {
      if (i > 0) {
        joint_names_stream << ", ";
      }
      joint_names_stream << model_.names[i];
    }

    RCLCPP_INFO_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "gravity torque for joints [" << joint_names_stream.str() << "]: "
      << Eigen::Map<const Eigen::VectorXd>(filtered_torques.data(), static_cast<Eigen::Index>(filtered_torques.size())).transpose());
  }

  pinocchio::Model model_;
  pinocchio::Data data_;
  Eigen::VectorXd latest_q_;
  bool has_received_joint_state_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GravityCompensationNode>());
  rclcpp::shutdown();
  return 0;
}
