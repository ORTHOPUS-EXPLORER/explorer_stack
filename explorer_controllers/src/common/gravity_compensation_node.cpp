#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>
#include <sstream>

#include <Eigen/Dense>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <orthopus_vesc_interfaces/msg/config.hpp>

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

    // Cartesian impedance -> per-joint stiffness/damping (see publish_cartesian_impedance_()).
    // Disabled by default so behaviour is unchanged unless explicitly enabled.
    this->declare_parameter("cartesian_impedance_publish_enable", false);
    // damping = cartesian_impedance_damping_ratio_ * stiffness for every joint. 0.75 matches the
    // rough damping/stiffness ratio of the previous fixed per-joint values in
    // explorer_custom_controller.yaml.
    this->declare_parameter("cartesian_impedance_damping_ratio", 0.75);
    // Target cartesian stiffness driving every joint's computed stiffness. Defaults to ~1.4, the
    // average of the fixed per-joint impedance_stiffness values in explorer_custom_controller.yaml
    // (2.0, 2.5, 2.0, 1.0, 0.8, 0.3), so that at the reference configuration the computed per-joint
    // stiffnesses are close to the previous fixed ones.
    this->declare_parameter("cartesian_impedance_stiffness", 1.4);
    // Frame the cartesian stiffness is expressed at / the Jacobian is computed for. Not
    // reconfigurable at runtime (only read once here).
    this->declare_parameter("cartesian_impedance_end_effector_frame", std::string("tool0"));

    std::string urdf_path;
    std::string joint_state_topic;
    this->get_parameter("urdf_path", urdf_path);
    this->get_parameter("joint_state_topic", joint_state_topic);
    this->get_parameter("default_external_wrench_frame", default_external_wrench_frame_);
    this->get_parameter("cartesian_impedance_publish_enable", cartesian_impedance_enable_);
    this->get_parameter("cartesian_impedance_damping_ratio", cartesian_impedance_damping_ratio_);
    this->get_parameter("cartesian_impedance_stiffness", cartesian_impedance_stiffness_);
    this->get_parameter("cartesian_impedance_end_effector_frame", cartesian_impedance_end_effector_frame_);
    cartesian_impedance_reference_captured_ = false;
    cartesian_impedance_reference_mean_lever_arm_sq_ = 0.0;

    if (!initialize_model(urdf_path)) {
      return;
    }

    if (!model_.existFrame(cartesian_impedance_end_effector_frame_)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Cartesian impedance: unknown end effector frame '%s'; cartesian impedance publishing will stay disabled.",
        cartesian_impedance_end_effector_frame_.c_str());
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

    for (std::size_t i = 0; i < cartesian_impedance_config_pub_.size(); ++i) {
      const std::string topic = "/explorer_joint_" + std::to_string(i + 1) + "/config";
      cartesian_impedance_config_pub_[i] = this->create_publisher<orthopus_vesc_interfaces::msg::Config>(topic, 10);
    }

    // Allow the three cartesian impedance params above to be changed live (e.g. `ros2 param set`)
    // without restarting the node.
    on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&GravityCompensationNode::on_parameter_change_, this, std::placeholders::_1));

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

    if (cartesian_impedance_enable_) {
      publish_cartesian_impedance_();
    }

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

  // --- Cartesian impedance -> per-joint stiffness/damping -------------------------------------
  //
  // The joints' stiffness/damping used to be fixed (see explorer_custom_controller.yaml), meaning
  // the *cartesian* impedance felt at the end effector varies a lot with the arm configuration (a
  // given joint stiffness moves the end effector a lot when the joint is far from it, and little
  // when it's close). When enabled, this instead derives each joint's stiffness from a single
  // target cartesian stiffness and the current configuration, so the cartesian impedance stays
  // approximately uniform.
  //
  // For a small joint deflection dq_i, the resulting end-effector displacement is dx_i = J_i * dq_i,
  // where J_i is the i-th column of the translational Jacobian (already computed the same way as
  // the external wrench handling above, via computeFrameJacobian). Equating the joint-space and
  // cartesian-space elastic energy for a diagonal cartesian stiffness k_cart
  // (1/2 * K_i * dq_i^2 == 1/2 * k_cart * ||dx_i||^2) gives:
  //      K_i = k_cart * ||J_i||^2
  // ||J_i||^2 is the squared lever arm of joint i in the current configuration. To keep the overall
  // magnitude consistent with the previous fixed per-joint values (so
  // cartesian_impedance_stiffness_'s default is "close to the actual stiffness obtained with the
  // actual fixed joint stiffness", as opposed to a raw physical N/m value), K_i is additionally
  // normalized by the mean lever arm captured once, at the first configuration this is evaluated on:
  //      K_i(q) = k_cart * ||J_i(q)||^2 / mean_j(||J_j(q_ref)||^2)
  // so that at q == q_ref, the per-joint stiffnesses average out to k_cart. The result is clamped to
  // [kJointStiffnessMin_, kJointStiffnessMax_] as a safety net independent of
  // cartesian_impedance_stiffness_, and published on /explorer_joint_1/config ..
  // /explorer_joint_6/config.
  void publish_cartesian_impedance_()
  {
    if (!has_received_joint_state_ || !model_.existFrame(cartesian_impedance_end_effector_frame_)) {
      return;
    }

    pinocchio::Data::Matrix6x jacobian(6, model_.nv);
    jacobian.setZero();
    pinocchio::computeFrameJacobian(
      model_, data_, latest_q_, model_.getFrameId(cartesian_impedance_end_effector_frame_),
      pinocchio::LOCAL_WORLD_ALIGNED, jacobian);

    // Per-joint lever arm weight: squared norm of the translational part of that joint's Jacobian
    // column, i.e. how much a small rotation of that joint moves the end effector (m^2/rad^2).
    std::array<double, 6> lever_arm_sq{};
    for (int i = 0; i < 6; ++i) {
      lever_arm_sq[static_cast<std::size_t>(i)] = jacobian.block<3, 1>(0, i).squaredNorm();
    }

    if (!cartesian_impedance_reference_captured_) {
      const double sum = std::accumulate(lever_arm_sq.begin(), lever_arm_sq.end(), 0.0);
      cartesian_impedance_reference_mean_lever_arm_sq_ = sum / 6.0;
      if (cartesian_impedance_reference_mean_lever_arm_sq_ < 1e-9) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "Cartesian impedance: degenerate/singular reference configuration, skipping this cycle.");
        return;
      }
      cartesian_impedance_reference_captured_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "Cartesian impedance: captured reference mean squared lever arm = %.4f at first evaluation.",
        cartesian_impedance_reference_mean_lever_arm_sq_);
    }

    for (int i = 0; i < 6; ++i) {
      double stiffness = cartesian_impedance_stiffness_ *
        (lever_arm_sq[static_cast<std::size_t>(i)] / cartesian_impedance_reference_mean_lever_arm_sq_);
      stiffness = std::clamp(stiffness, kJointStiffnessMin_, kJointStiffnessMax_);
      const double damping = cartesian_impedance_damping_ratio_ * stiffness;

      orthopus_vesc_interfaces::msg::Config command;
      command.timestamp = this->now();
      command.impedance_control_stiffness = static_cast<float>(stiffness);
      command.impedance_control_damping = static_cast<float>(damping);
      cartesian_impedance_config_pub_[static_cast<std::size_t>(i)]->publish(command);
    }
  }

  rcl_interfaces::msg::SetParametersResult on_parameter_change_(const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : parameters) {
      if (param.get_name() == "cartesian_impedance_publish_enable") {
        cartesian_impedance_enable_ = param.as_bool();
      } else if (param.get_name() == "cartesian_impedance_damping_ratio") {
        const double ratio = param.as_double();
        if (ratio < 0.0) {
          result.successful = false;
          result.reason = "cartesian_impedance_damping_ratio must be >= 0";
          continue;
        }
        cartesian_impedance_damping_ratio_ = ratio;
      } else if (param.get_name() == "cartesian_impedance_stiffness") {
        const double stiffness = param.as_double();
        if (stiffness <= 0.0) {
          result.successful = false;
          result.reason = "cartesian_impedance_stiffness must be > 0";
          continue;
        }
        cartesian_impedance_stiffness_ = stiffness;
      }
    }
    return result;
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

  /*!< Publishes updated per-joint stiffness/damping on /explorer_joint_1/config ..
       /explorer_joint_6/config every publish_gravity_torque() cycle when true. Disabled by
       default so behaviour is unchanged unless explicitly enabled. */
  bool cartesian_impedance_enable_;
  /*!< damping = cartesian_impedance_damping_ratio_ * stiffness for every joint. */
  double cartesian_impedance_damping_ratio_;
  /*!< Target cartesian stiffness driving every joint's computed stiffness (see
       publish_cartesian_impedance_() for the full derivation). */
  double cartesian_impedance_stiffness_;
  /*!< Frame the cartesian stiffness is expressed at / the Jacobian is computed for. */
  std::string cartesian_impedance_end_effector_frame_;
  /*!< Mean squared lever arm captured at the first configuration this is evaluated on, used to
       normalize the computed stiffness. */
  double cartesian_impedance_reference_mean_lever_arm_sq_;
  bool cartesian_impedance_reference_captured_;
  /*!< Computed per-joint stiffness is clamped to this range before being published, as a safety
       net independent of cartesian_impedance_stiffness_. */
  static constexpr double kJointStiffnessMin_ = 0.5;
  static constexpr double kJointStiffnessMax_ = 10.0;
  std::array<rclcpp::Publisher<orthopus_vesc_interfaces::msg::Config>::SharedPtr, 6> cartesian_impedance_config_pub_;
  /*!< Handle for the dynamic parameter callback (must be kept alive) allowing the three
       cartesian_impedance_* params above to be changed at runtime, e.g. via `ros2 param set`,
       without restarting the node. */
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GravityCompensationNode>());
  rclcpp::shutdown();
  return 0;
}
