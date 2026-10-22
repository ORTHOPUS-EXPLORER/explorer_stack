#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <controller_interface/controller_interface_params.hpp>
#include <explorer_command_controllers/explorer_position_saturation_controller.hpp>
#include <functional>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>
#include <string_view>
#include <vector>

using explorer_command_controllers::PositionSaturationController;
using lifecycle_msgs::msg::State;

namespace
{
constexpr std::string_view kControllerName = "test_saturation_controller";
constexpr double kEpsilon = 1e-12;
const rclcpp::Time kTime(0, 0, RCL_ROS_TIME);
const rclcpp::Duration kPeriod = rclcpp::Duration::from_seconds(0.002);

hardware_interface::InterfaceDescription make_description(
  const std::string& prefix, const std::string& interface_name)
{
  hardware_interface::InterfaceInfo info;
  info.name = interface_name;
  return hardware_interface::InterfaceDescription(prefix, info);
}
}  // namespace

class PositionSaturationControllerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override { controller_ = std::make_unique<PositionSaturationController>(); }

  void TearDown() override
  {
    executor_.reset();
    test_node_.reset();
    controller_.reset();
  }

  // Init the controller with default test parameters, extra ones override them
  controller_interface::return_type init_controller_(
    const std::vector<rclcpp::Parameter>& extra_parameters = {})
  {
    std::vector<rclcpp::Parameter> parameters = {
      rclcpp::Parameter("joints", joints_),
      rclcpp::Parameter("position_error_saturation_enable", true),
      rclcpp::Parameter("position_error_saturation_threshold", 0.2),
      rclcpp::Parameter("position_error_saturation_persistent", false),
    };
    parameters.insert(parameters.end(), extra_parameters.begin(), extra_parameters.end());

    controller_interface::ControllerInterfaceParams params;
    params.controller_name = std::string(kControllerName);
    params.update_rate = 500;
    params.controller_manager_update_rate = 500;
    params.node_options = controller_->define_custom_node_options();
    params.node_options.parameter_overrides(parameters);
    return controller_->init(params);
  }

  // Configure, export references and loan interfaces as the controller manager would do
  void configure_and_assign_interfaces_()
  {
    ASSERT_EQ(controller_->configure().id(), State::PRIMARY_STATE_INACTIVE);
    reference_interfaces_ = controller_->export_reference_interfaces();

    std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
    std::vector<hardware_interface::LoanedStateInterface> loaned_states;
    for (const auto& joint : joints_)
    {
      auto command = std::make_shared<hardware_interface::CommandInterface>(
        make_description("explorer_custom_controller/" + joint, "qp_position"));
      auto state =
        std::make_shared<hardware_interface::StateInterface>(make_description(joint, "position"));
      ASSERT_TRUE(state->set_value(0.0));

      loaned_commands.emplace_back(command, nullptr);
      loaned_states.emplace_back(state, nullptr);
      command_interfaces_.push_back(command);
      state_interfaces_.push_back(state);
    }
    controller_->assign_interfaces(std::move(loaned_commands), std::move(loaned_states));
  }

  void activate_()
  {
    ASSERT_EQ(controller_->get_node()->activate().id(), State::PRIMARY_STATE_ACTIVE);
  }

  void deactivate_()
  {
    ASSERT_EQ(controller_->get_node()->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  }

  // Init, configure and activate, optionally in chained mode (set before activation, like the
  // controller manager)
  void start_controller_(bool chained, const std::vector<rclcpp::Parameter>& parameters = {})
  {
    ASSERT_EQ(init_controller_(parameters), controller_interface::return_type::OK);
    configure_and_assign_interfaces_();
    ASSERT_TRUE(controller_->set_chained_mode(chained));
    activate_();
  }

  void set_reference_(std::size_t joint, double value)
  {
    ASSERT_TRUE(reference_interfaces_[joint]->set_value(value));
  }

  void set_measured_(std::size_t joint, double value)
  {
    ASSERT_TRUE(state_interfaces_[joint]->set_value(value));
  }

  [[nodiscard]] double command_(std::size_t joint) const
  {
    return command_interfaces_[joint]->get_optional().value();
  }

  void update_()
  {
    ASSERT_EQ(controller_->update(kTime, kPeriod), controller_interface::return_type::OK);
  }

  // Create a publisher on the controller topic and an executor spinning both nodes
  void setup_topic_publisher_()
  {
    test_node_ = std::make_shared<rclcpp::Node>("test_position_publisher");
    publisher_ = test_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/" + std::string(kControllerName) + "/position/commands", rclcpp::SystemDefaultsQoS());
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(controller_->get_node()->get_node_base_interface());
    executor_->add_node(test_node_);
  }

  // Publish `data` and run the control loop until `done` is true or the timeout expires
  bool publish_and_update_until_(
    const std::vector<double>& data, const std::function<bool()>& done,
    std::chrono::milliseconds timeout = std::chrono::seconds(5))
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = data;
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
      publisher_->publish(msg);
      executor_->spin_some(std::chrono::milliseconds(10));
      if (controller_->update(kTime, kPeriod) != controller_interface::return_type::OK)
      {
        return false;
      }
      if (done())
      {
        return true;
      }
    }
    return false;
  }

  const std::vector<std::string> joints_ = {"joint_1", "joint_2"};
  std::unique_ptr<PositionSaturationController> controller_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> reference_interfaces_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces_;
  std::vector<hardware_interface::StateInterface::SharedPtr> state_interfaces_;

  rclcpp::Node::SharedPtr test_node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

// on_init : the controller refuses to start without any joint to saturate
TEST_F(PositionSaturationControllerTest, InitFailsWithoutJoints)
{
  // Arrange
  const std::vector<rclcpp::Parameter> parameters = {
    rclcpp::Parameter("joints", std::vector<std::string>{})};

  // Act
  const auto result = init_controller_(parameters);

  // Assert
  EXPECT_EQ(result, controller_interface::return_type::ERROR);
}

// ~/position/commands callback : in chained mode, commands received on the topic are ignored
TEST_F(PositionSaturationControllerTest, TopicIsIgnoredInChainedMode)
{
  // Arrange: reference written by the upstream controller
  start_controller_(true);
  setup_topic_publisher_();
  set_measured_(0, 0.0);
  set_reference_(0, 0.1);
  set_reference_(1, 0.1);
  update_();
  // Wait until the controller subscription is matched so published messages are delivered
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (publisher_->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline)
  {
    executor_->spin_some(std::chrono::milliseconds(10));
  }
  ASSERT_GT(publisher_->get_subscription_count(), 0u);

  // Act: topic commands would give 0.15 if they were applied
  const bool topic_applied = publish_and_update_until_(
    {0.15, 0.15}, [this] { return std::abs(command_(0) - 0.15) < kEpsilon; },
    std::chrono::milliseconds(500));

  // Assert
  EXPECT_FALSE(topic_applied);
}

// ~/position/commands callback : a topic command with fewer values than joints is ignored
TEST_F(PositionSaturationControllerTest, TopicCommandTooShortIsDropped)
{
  // Arrange
  start_controller_(false);
  setup_topic_publisher_();

  // Act
  const bool short_message_applied = publish_and_update_until_(
    {0.1}, [this] { return !std::isnan(command_(0)); }, std::chrono::milliseconds(200));

  // Assert: a valid message is then applied, so messages were delivered
  EXPECT_FALSE(short_message_applied);
  ASSERT_TRUE(publish_and_update_until_({0.1, 0.1}, [this] { return !std::isnan(command_(0)); }));
}

// ~/position/commands callback : a command containing a NaN is ignored, valid values included
TEST_F(PositionSaturationControllerTest, TopicCommandWithNaNIsDropped)
{
  // Arrange
  start_controller_(false);
  setup_topic_publisher_();

  // Act: joint_1 value is valid, joint_2 is NaN
  const bool nan_message_applied = publish_and_update_until_(
    {0.1, std::numeric_limits<double>::quiet_NaN()}, [this] { return !std::isnan(command_(0)); },
    std::chrono::milliseconds(200));

  // Assert: a valid message is then applied, so messages were delivered
  EXPECT_FALSE(nan_message_applied);
  ASSERT_TRUE(publish_and_update_until_({0.1, 0.1}, [this] { return !std::isnan(command_(0)); }));
}

// command_interface_configuration : by default, commands go to explorer_custom_controller
TEST_F(PositionSaturationControllerTest, ClaimsCustomControllerCommandInterfacesByDefault)
{
  // Arrange
  ASSERT_EQ(init_controller_(), controller_interface::return_type::OK);
  ASSERT_EQ(controller_->configure().id(), State::PRIMARY_STATE_INACTIVE);

  // Act
  const auto config = controller_->command_interface_configuration();

  // Assert
  EXPECT_EQ(
    config.names, (std::vector<std::string>{
                    "explorer_custom_controller/joint_1/qp_position",
                    "explorer_custom_controller/joint_2/qp_position"}));
}

// command_interface_configuration : downstream controller and command interface are configurable
TEST_F(PositionSaturationControllerTest, ClaimsConfiguredCommandInterfaces)
{
  // Arrange
  ASSERT_EQ(
    init_controller_({
      rclcpp::Parameter("chained_controller_name", "downstream"),
      rclcpp::Parameter("command_interface_name", "target"),
    }),
    controller_interface::return_type::OK);
  ASSERT_EQ(controller_->configure().id(), State::PRIMARY_STATE_INACTIVE);

  // Act
  const auto config = controller_->command_interface_configuration();

  // Assert
  EXPECT_EQ(
    config.names,
    (std::vector<std::string>{"downstream/joint_1/target", "downstream/joint_2/target"}));
}

// state_interface_configuration : by default, the measure is read from joint position states
TEST_F(PositionSaturationControllerTest, ClaimsPositionStateInterfacesByDefault)
{
  // Arrange
  ASSERT_EQ(init_controller_(), controller_interface::return_type::OK);
  ASSERT_EQ(controller_->configure().id(), State::PRIMARY_STATE_INACTIVE);

  // Act
  const auto config = controller_->state_interface_configuration();

  // Assert
  EXPECT_EQ(config.names, (std::vector<std::string>{"joint_1/position", "joint_2/position"}));
}

// state_interface_configuration : the state interface used as measured position can be configured
TEST_F(PositionSaturationControllerTest, ClaimsConfiguredStateInterfaces)
{
  // Arrange
  ASSERT_EQ(
    init_controller_({rclcpp::Parameter("state_interface_name", "measure")}),
    controller_interface::return_type::OK);
  ASSERT_EQ(controller_->configure().id(), State::PRIMARY_STATE_INACTIVE);

  // Act
  const auto config = controller_->state_interface_configuration();

  // Assert
  EXPECT_EQ(config.names, (std::vector<std::string>{"joint_1/measure", "joint_2/measure"}));
}

// on_export_reference_interfaces : one qp_position reference interface is exported per joint
TEST_F(PositionSaturationControllerTest, ExportsOneReferenceInterfacePerJoint)
{
  // Arrange
  ASSERT_EQ(init_controller_(), controller_interface::return_type::OK);
  ASSERT_EQ(controller_->configure().id(), State::PRIMARY_STATE_INACTIVE);

  // Act
  const auto references = controller_->export_reference_interfaces();

  // Assert
  ASSERT_EQ(references.size(), joints_.size());
  EXPECT_EQ(references[0]->get_name(), std::string(kControllerName) + "/joint_1/qp_position");
  EXPECT_EQ(references[1]->get_name(), std::string(kControllerName) + "/joint_2/qp_position");
}

// on_activate : after reactivation, nothing is commanded until a new reference is written
TEST_F(PositionSaturationControllerTest, ReactivationClearsReference)
{
  // Arrange
  start_controller_(true);
  set_measured_(0, 0.0);
  set_reference_(0, 0.1);
  update_();

  // Act
  deactivate_();
  activate_();
  update_();

  // Assert
  EXPECT_TRUE(std::isnan(command_(0)));
}

// on_activate : after reactivation, the shift from an earlier persistent clamping is gone
TEST_F(PositionSaturationControllerTest, ReactivationClearsPersistentShift)
{
  // Arrange: reference 0.5 clamped to 0.2 (shift of -0.3)
  start_controller_(true, {rclcpp::Parameter("position_error_saturation_persistent", true)});
  set_measured_(0, 0.0);
  set_reference_(0, 0.5);
  update_();

  // Act
  deactivate_();
  activate_();
  set_reference_(0, 0.1);
  update_();

  // Assert: 0.1 is within 0.0 +/- 0.2 (0.1 - 0.3 = -0.2 if the shift were still applied)
  EXPECT_NEAR(command_(0), 0.1, kEpsilon);
}

// update_reference_from_subscribers : when not chained, topic commands are saturated
TEST_F(PositionSaturationControllerTest, TopicCommandIsSaturatedWhenNotChained)
{
  // Arrange
  start_controller_(false);
  setup_topic_publisher_();
  set_measured_(0, 0.0);

  // Act
  const bool received =
    publish_and_update_until_({0.5, 0.0}, [this] { return !std::isnan(command_(0)); });

  // Assert: 0.0 measured + 0.2 threshold
  ASSERT_TRUE(received) << "No command received on ~/position/commands before timeout";
  EXPECT_NEAR(command_(0), 0.2, kEpsilon);
}

// update_and_write_commands : a new threshold set while running is applied on the next cycle
TEST_F(PositionSaturationControllerTest, ThresholdCanBeChangedLive)
{
  // Arrange
  start_controller_(true);
  set_measured_(0, 0.0);
  set_reference_(0, 0.5);

  // Act
  const auto result = controller_->get_node()->set_parameter(
    rclcpp::Parameter("position_error_saturation_threshold", 0.05));
  update_();

  // Assert: 0.0 measured + 0.05 new threshold (0.2 with the previous one)
  ASSERT_TRUE(result.successful);
  EXPECT_NEAR(command_(0), 0.05, kEpsilon);
}

// position_error_saturation_threshold parameter : setting a negative threshold is refused
TEST_F(PositionSaturationControllerTest, NegativeThresholdParameterIsRejected)
{
  // Arrange
  start_controller_(true);

  // Act
  const auto result = controller_->get_node()->set_parameter(
    rclcpp::Parameter("position_error_saturation_threshold", -0.1));

  // Assert
  EXPECT_FALSE(result.successful);
}

// update_and_write_commands : after a refused threshold, the previous one is still applied
TEST_F(PositionSaturationControllerTest, RejectedThresholdKeepsPreviousOne)
{
  // Arrange
  start_controller_(true);
  set_measured_(0, 0.0);
  set_reference_(0, 0.5);
  ASSERT_FALSE(controller_->get_node()
                 ->set_parameter(rclcpp::Parameter("position_error_saturation_threshold", -0.1))
                 .successful);

  // Act
  update_();

  // Assert: 0.0 measured + 0.2 previous threshold
  EXPECT_NEAR(command_(0), 0.2, kEpsilon);
}

// update_and_write_commands : nothing is commanded (NaN) until a reference is written
TEST_F(PositionSaturationControllerTest, CommandsNaNUntilReferenceIsWritten)
{
  // Arrange
  start_controller_(true);

  // Act
  update_();

  // Assert
  EXPECT_TRUE(std::isnan(command_(0)));
  EXPECT_TRUE(std::isnan(command_(1)));
}

// update_and_write_commands : a reference too far from the measured position is clamped
TEST_F(PositionSaturationControllerTest, ChainedReferenceIsClamped)
{
  // Arrange
  start_controller_(true);
  set_measured_(0, 0.0);
  set_reference_(0, 0.5);

  // Act
  update_();

  // Assert: 0.0 measured + 0.2 threshold
  EXPECT_NEAR(command_(0), 0.2, kEpsilon);
}

// update_and_write_commands : with persistence, later commands keep the clamping shift
TEST_F(PositionSaturationControllerTest, PersistentParameterShiftsLaterCommands)
{
  // Arrange: reference 0.5 clamped to 0.2
  start_controller_(true, {rclcpp::Parameter("position_error_saturation_persistent", true)});
  set_measured_(0, 0.0);
  set_reference_(0, 0.5);
  update_();

  // Act: upstream moves +0.01, robot measured at the previous command
  set_reference_(0, 0.51);
  set_measured_(0, 0.2);
  update_();

  // Assert: 0.51 - 0.3 shift, previous command + 0.01 (0.4 if the shift were lost)
  EXPECT_NEAR(command_(0), 0.21, kEpsilon);
}
