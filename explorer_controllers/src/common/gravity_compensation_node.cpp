#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <iomanip>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>
#include <sstream>

#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <orthopus_vesc_interfaces/msg/config.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
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

geometry_msgs::msg::Point point_from(const Eigen::Vector3d& v)
{
  geometry_msgs::msg::Point p;
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
  return p;
}

std_msgs::msg::ColorRGBA make_color(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
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

    // --- Visualization (see publish_visualization_markers_()) ---------------------------------
    // TF frame the markers are expressed/drawn in; must match the root of the URDF used to build
    // the Pinocchio model (explorer.urdf.xacro's root link is "world").
    this->declare_parameter("cartesian_impedance_marker_frame_id", std::string("world"));
    this->declare_parameter("cartesian_impedance_marker_topic", std::string("/explorer_controllers/gravity_compensation/cartesian_impedance_markers"));
    // This is a real force estimate in Newtons (see publish_visualization_markers_()'s comment),
    // so a smaller default scale (5cm/N) is more likely to fit the arm's workspace for typical
    // light-payload forces; tune by eye in rviz.
    this->declare_parameter("cartesian_impedance_external_force_marker_scale", 0.05);

    std::string urdf_path;
    std::string joint_state_topic;
    std::string marker_topic;
    this->get_parameter("urdf_path", urdf_path);
    this->get_parameter("joint_state_topic", joint_state_topic);
    this->get_parameter("default_external_wrench_frame", default_external_wrench_frame_);
    this->get_parameter("cartesian_impedance_publish_enable", cartesian_impedance_enable_);
    this->get_parameter("cartesian_impedance_damping_ratio", cartesian_impedance_damping_ratio_);
    this->get_parameter("cartesian_impedance_stiffness", cartesian_impedance_stiffness_);
    this->get_parameter("cartesian_impedance_end_effector_frame", cartesian_impedance_end_effector_frame_);
    this->get_parameter("cartesian_impedance_marker_frame_id", marker_frame_id_);
    this->get_parameter("cartesian_impedance_marker_topic", marker_topic);
    this->get_parameter("cartesian_impedance_external_force_marker_scale", cartesian_impedance_external_force_marker_scale_);
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
    latest_effort_ = Eigen::VectorXd::Zero(model_.nq);
    has_received_joint_state_ = false;
    has_received_effort_ = false;
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

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, 10);

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
      // No names to match effort against either; fall back the same way, if present.
      const Eigen::Index effort_count = std::min<Eigen::Index>(model_.nq, static_cast<Eigen::Index>(msg->effort.size()));
      for (Eigen::Index i = 0; i < effort_count; ++i) {
        latest_effort_[i] = msg->effort[static_cast<std::size_t>(i)];
      }
      has_received_effort_ = effort_count > 0;
      return;
    }

    bool mapped_any_joint = false;
    bool mapped_any_effort = false;
    std::size_t q_index = 0;
    for (const auto & joint_name : model_.names) {
      if (q_index >= static_cast<std::size_t>(model_.nq)) {
        break;
      }

      const auto it = std::find(msg->name.begin(), msg->name.end(), joint_name);
      if (it != msg->name.end()) {
        // position/velocity/effort share the same index as name for a given joint, per the
        // sensor_msgs/JointState convention.
        const std::size_t index = std::distance(msg->name.begin(), it);
        if (index < msg->position.size()) {
          latest_q_[static_cast<Eigen::Index>(q_index)] = msg->position[index];
          mapped_any_joint = true;
        }
        if (index < msg->effort.size()) {
          latest_effort_[static_cast<Eigen::Index>(q_index)] = msg->effort[index];
          mapped_any_effort = true;
        }
        ++q_index;
      }
    }
    has_received_effort_ = mapped_any_effort;

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
    // Pristine copy, taken before the (optional, known) external_wrench_topic adjustment below --
    // publish_visualization_markers_() needs pure gravity torque to estimate an *unknown* external
    // force from the residual against the measured effort, see its comment.
    const Eigen::VectorXd gravity_torque = tau;

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

    publish_visualization_markers_(tau, gravity_torque);

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

  // --- RViz visualization: measured external force, per-joint effort ---------------------------
  //
  // - "measured external force" (magenta arrow, real Newtons): assuming, quasi-statically, that
  //   the only unmodeled torque is a wrench applied at cartesian_impedance_end_effector_frame_,
  //   tau_measured = gravity_torque - J^T * F_ext, so F_ext = pinv(J^T) * (gravity_torque -
  //   tau_measured), solved at the actual tool position. Requires real measured effort (see
  //   joint_state_callback()); does nothing if only the "(cmd)" fallback is available, since the
  //   residual would then trivially be ~0 (or just re-derive the known external_wrench_topic input,
  //   if any -- this estimates an *unknown* wrench, independent of that topic).
  // - per-joint effort: one sphere (sized by |effort_i|, blue = positive / orange = negative, text
  //   always shows the sign too) and one text label per joint, placed at that joint's origin.
  //   Prefers the *measured* effort from /joint_states (msg->effort) when the driver populates it;
  //   otherwise falls back to the gravity(+external wrench) compensation torque this node commands
  //   (labelled "(cmd)" so the two aren't confused), computed above in publish_gravity_torque().
  //
  // Note: the "controlled point" (where the tool should be given the last position command) is
  // published directly by joint_output_integrator.cpp instead of from here, since it already has
  // that command and doesn't need to depend on another node guessing the right topic to read it
  // back from; see its publish_controlled_point_marker_().
  void publish_visualization_markers_(const Eigen::VectorXd& tau, const Eigen::VectorXd& gravity_torque)
  {
    if (!has_received_joint_state_ || !model_.existFrame(cartesian_impedance_end_effector_frame_)) {
      return;
    }
    const auto ee_frame_id = model_.getFrameId(cartesian_impedance_end_effector_frame_);

    // Re-run FK explicitly so this doesn't depend on what publish_cartesian_impedance_() did (or
    // didn't do, if disabled) earlier this cycle.
    pinocchio::forwardKinematics(model_, data_, latest_q_);
    pinocchio::updateFramePlacements(model_, data_);
    const Eigen::Vector3d x_actual = data_.oMf[ee_frame_id].translation();

    visualization_msgs::msg::MarkerArray markers;
    const rclcpp::Time stamp = this->now();

    if (has_received_effort_) {
      pinocchio::Data::Matrix6x jacobian(6, model_.nv);
      jacobian.setZero();
      pinocchio::computeFrameJacobian(
        model_, data_, latest_q_, ee_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, jacobian);

      // J is square here (6 cartesian dof x 6 joints), so this is normally an exact solve;
      // completeOrthogonalDecomposition degrades gracefully to a least-norm solution instead of
      // blowing up near singular configurations (rather than e.g. a plain 6x6 inverse).
      const Eigen::VectorXd residual = gravity_torque - latest_effort_;
      const Eigen::VectorXd f_ext = jacobian.transpose().completeOrthogonalDecomposition().solve(residual);
      const Eigen::Vector3d f_ext_force = f_ext.head<3>();

      visualization_msgs::msg::Marker ext_force_arrow;
      ext_force_arrow.header.frame_id = marker_frame_id_;
      ext_force_arrow.header.stamp = stamp;
      ext_force_arrow.ns = "external_force_estimate";
      ext_force_arrow.id = 0;
      ext_force_arrow.type = visualization_msgs::msg::Marker::ARROW;
      ext_force_arrow.action = visualization_msgs::msg::Marker::ADD;
      ext_force_arrow.points.push_back(point_from(x_actual));
      ext_force_arrow.points.push_back(
        point_from(x_actual + cartesian_impedance_external_force_marker_scale_ * f_ext_force));
      ext_force_arrow.scale.x = 0.015;  // shaft diameter
      ext_force_arrow.scale.y = 0.03;   // head diameter
      ext_force_arrow.scale.z = 0.0;    // head length: 0 -> rviz picks a default
      ext_force_arrow.color = make_color(1.0f, 0.0f, 1.0f, 1.0f);  // magenta
      markers.markers.push_back(ext_force_arrow);

      visualization_msgs::msg::Marker ext_force_text;
      ext_force_text.header.frame_id = marker_frame_id_;
      ext_force_text.header.stamp = stamp;
      ext_force_text.ns = "external_force_estimate_text";
      ext_force_text.id = 0;
      ext_force_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      ext_force_text.action = visualization_msgs::msg::Marker::ADD;
      ext_force_text.pose.position = point_from(x_actual + Eigen::Vector3d(0.0, 0.0, 0.06));
      ext_force_text.pose.orientation.w = 1.0;
      ext_force_text.scale.z = 0.04;
      ext_force_text.color = make_color(1.0f, 0.0f, 1.0f, 1.0f);
      std::ostringstream ext_force_text_stream;
      ext_force_text_stream << "F_ext: " << std::fixed << std::setprecision(2) << f_ext_force.norm() << " N";
      ext_force_text.text = ext_force_text_stream.str();
      markers.markers.push_back(ext_force_text);
    }

    for (int i = 0; i < 6; ++i) {
      // data_.oMi is indexed by pinocchio joint id: 0 is the implicit "universe" root, so joint_i
      // (0-based i here) is at index i+1, consistent with model_.names used elsewhere.
      const Eigen::Vector3d joint_origin = data_.oMi[static_cast<std::size_t>(i + 1)].translation();
      const double effort = has_received_effort_ ? latest_effort_[i] : tau[i];

      visualization_msgs::msg::Marker effort_sphere;
      effort_sphere.header.frame_id = marker_frame_id_;
      effort_sphere.header.stamp = stamp;
      effort_sphere.ns = "joint_effort";
      effort_sphere.id = i;
      effort_sphere.type = visualization_msgs::msg::Marker::SPHERE;
      effort_sphere.action = visualization_msgs::msg::Marker::ADD;
      effort_sphere.pose.position = point_from(joint_origin);
      effort_sphere.pose.orientation.w = 1.0;
      const double diameter = std::clamp(0.03 + 0.02 * std::abs(effort), 0.03, 0.15);
      effort_sphere.scale.x = effort_sphere.scale.y = effort_sphere.scale.z = diameter;
      effort_sphere.color = effort >= 0.0 ? make_color(0.1f, 0.4f, 1.0f, 0.8f) : make_color(1.0f, 0.5f, 0.0f, 0.8f);
      markers.markers.push_back(effort_sphere);

      visualization_msgs::msg::Marker effort_text;
      effort_text.header.frame_id = marker_frame_id_;
      effort_text.header.stamp = stamp;
      effort_text.ns = "joint_effort_text";
      effort_text.id = i;
      effort_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      effort_text.action = visualization_msgs::msg::Marker::ADD;
      effort_text.pose.position = point_from(joint_origin + Eigen::Vector3d(0.0, 0.0, 0.08));
      effort_text.pose.orientation.w = 1.0;
      effort_text.scale.z = 0.04;
      effort_text.color = make_color(1.0f, 1.0f, 1.0f, 1.0f);
      std::ostringstream text_stream;
      // showpos: always print the sign so it's readable without cross-referencing the sphere
      // color (blue/orange, set above from the same sign).
      text_stream << model_.names[static_cast<std::size_t>(i + 1)] << ": " << std::fixed
                  << std::showpos << std::setprecision(2) << effort;
      if (!has_received_effort_) {
        text_stream << " (cmd)";
      }
      effort_text.text = text_stream.str();
      markers.markers.push_back(effort_text);
    }

    if (!has_received_effort_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Cartesian impedance markers: '%s' never carried a populated 'effort' field -- per-joint "
        "effort markers show the commanded gravity(+wrench) compensation torque instead (tagged "
        "'(cmd)' in rviz), not a hardware measurement.",
        joint_state_sub_->get_topic_name());
    }

    marker_pub_->publish(markers);
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
      } else if (param.get_name() == "cartesian_impedance_external_force_marker_scale") {
        const double scale = param.as_double();
        if (scale <= 0.0) {
          result.successful = false;
          result.reason = "cartesian_impedance_external_force_marker_scale must be > 0";
          continue;
        }
        cartesian_impedance_external_force_marker_scale_ = scale;
      }
    }
    return result;
  }

  pinocchio::Model model_;
  pinocchio::Data data_;
  Eigen::VectorXd latest_q_;
  bool has_received_joint_state_;
  /*!< Measured effort from /joint_states (msg->effort), aligned to model_.names/latest_q_ the same
       way latest_q_ is. Used for the per-joint effort markers when available (see
       publish_visualization_markers_()); some hardware/sim interfaces never populate this field,
       hence has_received_effort_. */
  Eigen::VectorXd latest_effort_;
  bool has_received_effort_;
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
  /*!< Handle for the dynamic parameter callback (must be kept alive) allowing the reconfigurable
       cartesian_impedance_* params above to be changed at runtime, e.g. via `ros2 param set`,
       without restarting the node. */
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  // --- Visualization (see publish_visualization_markers_()) -------------------------------------
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::string marker_frame_id_;
  double cartesian_impedance_external_force_marker_scale_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GravityCompensationNode>());
  rclcpp::shutdown();
  return 0;
}
