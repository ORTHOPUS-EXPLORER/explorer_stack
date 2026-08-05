#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <sstream>

#include <Eigen/Dense>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace
{
// Only these joints are actuated by the effort controller; every other joint found in the
// URDF (gripper, tool frames, etc.) is locked out of the model so Pinocchio never computes
// gravity terms for them.
const std::vector<std::string> kControlledJointNames = {
  "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"
};

std::string run_xacro(const std::string& xacro_path)
{
  const std::string output_path = "/tmp/explorer_gravity_compensation.urdf";
  const std::string command = std::string("xacro ") + xacro_path + " use_POC2:=true" + " > " + output_path;

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
    this->declare_parameter("publish_topic", std::string("/explorer_custom_controller/effort/commands"));
    this->declare_parameter("joint_state_topic", std::string("/joint_states"));
    this->declare_parameter("external_wrench_topic", std::string("/explorer_controllers/gravity_compensation/external_wrench"));
    this->declare_parameter("default_external_wrench_frame", std::string("tool0"));

    std::string urdf_path;
    std::string joint_state_topic;
    this->get_parameter("urdf_path", urdf_path);
    this->get_parameter("joint_state_topic", joint_state_topic);
    this->get_parameter("default_external_wrench_frame", default_external_wrench_frame_);

    if (!initialize_model(urdf_path)) {
      return;
    }

    latest_q_ = Eigen::VectorXd::Zero(model_.nq);
    has_received_joint_state_ = false;
    latest_wrench_.setZero();
    has_external_wrench_ = false;
    latest_wrench_frame_ = default_external_wrench_frame_;

    std::string publish_topic;
    std::string external_wrench_topic;
    this->get_parameter("publish_topic", publish_topic);
    this->get_parameter("external_wrench_topic", external_wrench_topic);
    torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(publish_topic, 10);
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic, 10,
      std::bind(&GravityCompensationNode::joint_state_callback, this, std::placeholders::_1));
    external_wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      external_wrench_topic, 10,
      std::bind(&GravityCompensationNode::external_wrench_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&GravityCompensationNode::publish_gravity_torque, this));

    /* RCLCPP_INFO(
      this->get_logger(),
      "Gravity compensation node ready. Loaded model with %d joints and %d positions.",
      model_.njoints, model_.nq); */
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
    pinocchio::Model full_model;
    pinocchio::urdf::buildModel(urdf_path, full_model, false);

    // Lock every joint that is not one of joint_1..joint_6 (e.g. the gripper) so Pinocchio
    // only keeps and computes gravity terms for the 6 controlled joints.
    std::vector<pinocchio::JointIndex> joints_to_lock;
    for (pinocchio::JointIndex joint_id = 1; joint_id < static_cast<pinocchio::JointIndex>(full_model.njoints); ++joint_id) {
      const bool is_controlled = std::find(
        kControlledJointNames.begin(), kControlledJointNames.end(), full_model.names[joint_id]) != kControlledJointNames.end();
      if (!is_controlled) {
        joints_to_lock.push_back(joint_id);
      }
    }

    pinocchio::buildReducedModel(full_model, joints_to_lock, pinocchio::neutral(full_model), model_);

    if (static_cast<std::size_t>(model_.njoints - 1) != kControlledJointNames.size()) {
      throw std::runtime_error("Expected joint_1..joint_6 in the URDF, found a different set of controlled joints.");
    }

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

  // A WrenchStamped's header.frame_id names the frame the wrench is applied at (defaulting to
  // "tool0" when left empty); force/torque are expressed in that frame with axes aligned to the
  // world (LOCAL_WORLD_ALIGNED), so e.g. a hanging payload of mass m is simply force.z = -m*9.81.
  // The wrench is the force exerted ON the robot by the environment, so it works equally for a
  // payload weight or any other virtual Cartesian force.
  void external_wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    latest_wrench_frame_ = msg->header.frame_id.empty() ? default_external_wrench_frame_ : msg->header.frame_id;
    latest_wrench_ << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
      msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
    has_external_wrench_ = true;
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
    Eigen::VectorXd tau = data_.g;

    if (has_external_wrench_) {
      if (model_.existFrame(latest_wrench_frame_)) {
        pinocchio::Data::Matrix6x J(6, model_.nv);
        J.setZero();
        pinocchio::computeFrameJacobian(
          model_, data_, latest_q_, model_.getFrameId(latest_wrench_frame_),
          pinocchio::LOCAL_WORLD_ALIGNED, J);
        tau.noalias() -= J.transpose() * latest_wrench_;
      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Unknown external wrench frame '%s'; ignoring external wrench.", latest_wrench_frame_.c_str());
      }
    }

    std::vector<double> torques;
    torques.reserve(static_cast<size_t>(tau.size()));
    for (Eigen::Index i = 0; i < tau.size(); ++i) {
      torques.push_back(tau[i]);
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data = torques;

    torque_pub_->publish(msg);

    // model_.names[0] is the implicit "universe" root joint (no torque associated with it);
    // skip it so the printed names line up 1:1 with the published torques.
    std::ostringstream joint_names_stream;
    for (std::size_t i = 1; i < model_.names.size(); ++i) {
      if (i > 1) {
        joint_names_stream << ", ";
      }
      joint_names_stream << model_.names[i];
    }

    RCLCPP_INFO_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "gravity torque for joints [" << joint_names_stream.str() << "]: "
      << Eigen::Map<const Eigen::VectorXd>(torques.data(), static_cast<Eigen::Index>(torques.size())).transpose());
  }

  pinocchio::Model model_;
  pinocchio::Data data_;
  Eigen::VectorXd latest_q_;
  bool has_received_joint_state_;
  Eigen::Matrix<double, 6, 1> latest_wrench_;
  std::string latest_wrench_frame_;
  std::string default_external_wrench_frame_;
  bool has_external_wrench_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr external_wrench_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GravityCompensationNode>());
  rclcpp::shutdown();
  return 0;
}
