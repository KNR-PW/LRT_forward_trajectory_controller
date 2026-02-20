#include "test_trajectory_controller_utils.hpp"

using lifecycle_msgs::msg::State;
using test_trajectory_controllers::TrajectoryControllerTest;
using test_trajectory_controllers::TrajectoryControllerTestParameterized;


TEST_P(TrajectoryControllerTestParameterized, activate)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  SetUpTrajectoryController(executor);

  auto state = traj_controller_->get_node()->configure();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);

  auto cmd_if_conf = traj_controller_->command_interface_configuration();
  ASSERT_EQ(cmd_if_conf.names.size(), joint_names_.size() * command_interface_types_.size());
  EXPECT_EQ(cmd_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);

  auto state_if_conf = traj_controller_->state_interface_configuration();
  ASSERT_EQ(state_if_conf.names.size(), joint_names_.size() * state_interface_types_.size());
  EXPECT_EQ(state_if_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);

  state = ActivateTrajectoryController();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_ACTIVE);

  executor.cancel();
}

TEST_P(TrajectoryControllerTestParameterized, check_interface_names_with_command_joints)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  // set command_joints parameter different than joint_names_
  const rclcpp::Parameter command_joint_names_param("command_joints", command_joint_names_);
  SetUpTrajectoryController(executor, {command_joint_names_param});

  const auto state = traj_controller_->get_node()->configure();
  ASSERT_EQ(state.id(), State::PRIMARY_STATE_INACTIVE);

  compare_joints(joint_names_, command_joint_names_);
}


INSTANTIATE_TEST_SUITE_P(
  TestCorrectInterfaceConfig, TrajectoryControllerTestParameterized,
  ::testing::Values(
    std::make_tuple(
      std::vector<std::string>({"position"}), std::vector<std::string>({"position", "velocity","effort"})),
    std::make_tuple(
      std::vector<std::string>({"position","velocity"}), std::vector<std::string>({"position", "velocity","effort"})),
    std::make_tuple(
      std::vector<std::string>({"position","velocity","effort"}), std::vector<std::string>({"position", "velocity","effort"})),
    std::make_tuple(
      std::vector<std::string>({"position","effort"}),
      std::vector<std::string>({"position", "velocity", "effort"}))));

/**
 * @brief see if parameter validation is correct
 *
 * Note: generate_parameter_library validates parameters itself during on_init() method, but
 * combinations of parameters are checked from JTC during on_configure()
 */
TEST_F(TrajectoryControllerTest, incorrect_initialization_using_interface_parameters)
{
  state_interface_types_ = {"position","velocity","effort"};
  // command interfaces: empty
  command_interface_types_ = {};
  EXPECT_EQ(SetUpTrajectoryControllerLocal(), controller_interface::return_type::OK);
  auto state = traj_controller_->get_node()->configure();
  EXPECT_EQ(state.id(), State::PRIMARY_STATE_UNCONFIGURED);

  // command interfaces: bad_name
  command_interface_types_ = {"bad_name"};
  EXPECT_EQ(SetUpTrajectoryControllerLocal(), controller_interface::return_type::ERROR);

  // command interfaces: position not present
  command_interface_types_ = { "effort"};
  EXPECT_EQ(SetUpTrajectoryControllerLocal(), controller_interface::return_type::ERROR);

  // command interfaces: velocity - position not present
  command_interface_types_ = {"velocity", "effort"};
  EXPECT_EQ(SetUpTrajectoryControllerLocal(), controller_interface::return_type::ERROR);

  command_interface_types_ = {"effort"};
  EXPECT_EQ(SetUpTrajectoryControllerLocal(), controller_interface::return_type::ERROR);

  command_interface_types_ = {"position"};
  // state interfaces: empty
  state_interface_types_ = {};
  EXPECT_EQ(SetUpTrajectoryControllerLocal(), controller_interface::return_type::ERROR);


  // state interfaces: bad name
  state_interface_types_ = {"bad_name"};
  EXPECT_EQ(SetUpTrajectoryControllerLocal(), controller_interface::return_type::ERROR);

  state_interface_types_ = {"position"};
  EXPECT_EQ(SetUpTrajectoryControllerLocal(), controller_interface::return_type::ERROR);
  state_interface_types_ = {"velocity"};
  EXPECT_EQ(SetUpTrajectoryControllerLocal(), controller_interface::return_type::ERROR);
  state_interface_types_ = {"effort"};
  EXPECT_EQ(SetUpTrajectoryControllerLocal(), controller_interface::return_type::ERROR);
  state_interface_types_ = {"position","velocity"};
  EXPECT_EQ(SetUpTrajectoryControllerLocal(), controller_interface::return_type::ERROR);
  state_interface_types_ = {"position","effort"};
  EXPECT_EQ(SetUpTrajectoryControllerLocal(), controller_interface::return_type::ERROR);
  state_interface_types_ = {"velocity","effort"};
  EXPECT_EQ(SetUpTrajectoryControllerLocal(), controller_interface::return_type::ERROR);
}
