// Copyright 2024 LRT
// Licensed under the Apache License, Version 2.0

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <vector>

#include "gtest/gtest.h"
#include "test_trajectory_controller_utils.hpp"

using lifecycle_msgs::msg::State;
using test_trajectory_controllers::TrajectoryControllerTest;
using test_trajectory_controllers::TrajectoryControllerTestParameterized;

namespace
{
constexpr double EPS = 0.001;
constexpr int NUM_SAMPLES = 100;
}  // namespace

//=============================================================================
// Custom test fixture with logging capabilities
//=============================================================================
class TrajectoryInterpolationTest : public TrajectoryControllerTest
{
public:
  void SetUp() override
  {
    TrajectoryControllerTest:: SetUp();
    logged_data_. clear();
  }

  struct TrajectoryLogEntry
  {
    double time_sec;
    std::vector<double> reference_positions;
    std::vector<double> reference_velocities;
    std::vector<double> reference_efforts;
    std::vector<double> feedback_positions;
    std::vector<double> feedback_velocities;
    std::vector<double> feedback_efforts;
    std:: vector<double> command_positions;
    std::vector<double> command_velocities;
    std::vector<double> command_efforts;
    std:: vector<double> error_positions;
    std::vector<double> error_velocities;
  };

  std::vector<TrajectoryLogEntry> logged_data_;
  std::vector<std::vector<double>> input_positions_;
  std::vector<std::vector<double>> input_velocities_;
  std:: vector<std::vector<double>> input_efforts_;
  std:: vector<double> input_times_;
  std::string test_config_name_;
  std::string interpolation_method_;

  void log_current_state(double time_sec)
  {
    TrajectoryLogEntry entry;
    entry.time_sec = time_sec;

    auto state_ref = traj_controller_->get_state_reference();
    auto state_fb = traj_controller_->get_state_feedback();
    auto state_err = traj_controller_->get_state_error();

    entry.reference_positions = state_ref.positions;
    entry.reference_velocities = state_ref. velocities;
    entry.reference_efforts = state_ref.effort;

    entry.feedback_positions = state_fb.positions;
    entry.feedback_velocities = state_fb.velocities;
    entry.feedback_efforts = state_fb.effort;

    entry.command_positions = joint_pos_;
    entry.command_velocities = joint_vel_;
    entry.command_efforts = joint_eff_;

    entry.error_positions = state_err.positions;
    entry.error_velocities = state_err.velocities;

    logged_data_.push_back(entry);
  }

  void print_log_header()
  {
    std::cout << "\n";
    std::cout << "========================================\n";
    std:: cout << "=== TRAJECTORY INTERPOLATION LOG =======\n";
    std::cout << "========================================\n";
    std::cout << "CONFIG:  " << test_config_name_ << "\n";
    std:: cout << "INTERPOLATION: " << interpolation_method_ << "\n";
    std::cout << "========================================\n";
  }

  void print_input_trajectory()
  {
    std::cout << "--- INPUT_TRAJECTORY_START ---\n";
    std::cout << "NUM_JOINTS: " << joint_names_.size() << "\n";
    std::cout << "JOINT_NAMES: ";
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      if (i > 0) std::cout << ",";
      std::cout << joint_names_[i];
    }
    std::cout << "\n";

    std::cout << "NUM_POINTS: " << input_positions_.size() << "\n";

    for (size_t p = 0; p < input_positions_.size(); ++p)
    {
      std::cout << "POINT " << p << ":\n";
      std::cout << "  TIME: " << std::fixed << std::setprecision(4) << input_times_[p] << "\n";

      std::cout << "  POSITIONS:  ";
      for (size_t j = 0; j < input_positions_[p].size(); ++j)
      {
        if (j > 0) std::cout << ",";
        std:: cout << std::fixed << std::setprecision(6) << input_positions_[p][j];
      }
      std::cout << "\n";

      if (p < input_velocities_.size() && ! input_velocities_[p]. empty())
      {
        std::cout << "  VELOCITIES: ";
        for (size_t j = 0; j < input_velocities_[p]. size(); ++j)
        {
          if (j > 0) std::cout << ",";
          std::cout << std::fixed << std::setprecision(6) << input_velocities_[p][j];
        }
        std::cout << "\n";
      }

      if (p < input_efforts_.size() && !input_efforts_[p].empty())
      {
        std::cout << "  EFFORTS: ";
        for (size_t j = 0; j < input_efforts_[p].size(); ++j)
        {
          if (j > 0) std::cout << ",";
          std::cout << std::fixed << std::setprecision(6) << input_efforts_[p][j];
        }
        std::cout << "\n";
      }
    }
    std::cout << "--- INPUT_TRAJECTORY_END ---\n";
  }

  void print_sampled_trajectory()
  {
    std::cout << "--- SAMPLED_TRAJECTORY_START ---\n";
    std::cout << "NUM_SAMPLES: " << logged_data_.size() << "\n";

    for (size_t s = 0; s < logged_data_.size(); ++s)
    {
      const auto & entry = logged_data_[s];
      std::cout << "SAMPLE " << s << ":\n";
      std::cout << "  TIME: " << std:: fixed << std::setprecision(4) << entry.time_sec << "\n";

      // Reference position
      std::cout << "  POS: ";
      for (size_t j = 0; j < entry.reference_positions.size(); ++j)
      {
        if (j > 0) std::cout << ",";
        std::cout << std::fixed << std::setprecision(6) << entry.reference_positions[j];
      }
      std::cout << "\n";

      // Reference velocity
      if (!entry.reference_velocities. empty())
      {
        std::cout << "  VEL: ";
        for (size_t j = 0; j < entry.reference_velocities.size(); ++j)
        {
          if (j > 0) std::cout << ",";
          std::cout << std::fixed << std::setprecision(6) << entry.reference_velocities[j];
        }
        std::cout << "\n";
      }

      // Reference effort
      if (!entry.reference_efforts.empty())
      {
        std::cout << "  EFF: ";
        for (size_t j = 0; j < entry.reference_efforts.size(); ++j)
        {
          if (j > 0) std::cout << ",";
          std::cout << std::fixed << std::setprecision(6) << entry.reference_efforts[j];
        }
        std:: cout << "\n";
      }

      // Command position
      std::cout << "  CMD_POS: ";
      for (size_t j = 0; j < entry.command_positions.size(); ++j)
      {
        if (j > 0) std::cout << ",";
        std::cout << std::fixed << std::setprecision(6) << entry.command_positions[j];
      }
      std::cout << "\n";

      // Command velocity
      std::cout << "  CMD_VEL: ";
      for (size_t j = 0; j < entry.command_velocities.size(); ++j)
      {
        if (j > 0) std::cout << ",";
        std::cout << std:: fixed << std::setprecision(6) << entry.command_velocities[j];
      }
      std::cout << "\n";

      // Command effort
      std::cout << "  CMD_EFF: ";
      for (size_t j = 0; j < entry.command_efforts. size(); ++j)
      {
        if (j > 0) std::cout << ",";
        std::cout << std::fixed << std::setprecision(6) << entry.command_efforts[j];
      }
      std::cout << "\n";

      // Feedback position
      std::cout << "  FB_POS: ";
      for (size_t j = 0; j < entry.feedback_positions.size(); ++j)
      {
        if (j > 0) std::cout << ",";
        std::cout << std:: fixed << std::setprecision(6) << entry.feedback_positions[j];
      }
      std::cout << "\n";

      // Position error
      std::cout << "  ERR_POS: ";
      for (size_t j = 0; j < entry. error_positions.size(); ++j)
      {
        if (j > 0) std::cout << ",";
        std::cout << std::fixed << std::setprecision(6) << entry.error_positions[j];
      }
      std::cout << "\n";
    }
    std::cout << "--- SAMPLED_TRAJECTORY_END ---\n";
  }

  void print_log_footer()
  {
    std::cout << "========================================\n";
    std::cout << "=== END LOG ============================\n";
    std:: cout << "========================================\n\n";
  }

  void run_trajectory_with_logging(
    rclcpp:: Executor & executor,
    double total_duration_sec,
    int num_samples)
  {
    double dt = total_duration_sec / num_samples;
    rclcpp::Time start_time(0, 0, RCL_STEADY_TIME);
    rclcpp::Duration update_period = rclcpp::Duration:: from_seconds(0.001);

    for (int i = 0; i <= num_samples; ++i)
    {
      double current_time = i * dt;
      rclcpp::Time sample_time = start_time + rclcpp::Duration:: from_seconds(current_time);

      traj_controller_->update(sample_time, update_period);
      log_current_state(current_time);
    }
  }
};

//=============================================================================
// TEST 1: Two-point trajectory (basic motion)
//=============================================================================
TEST_F(TrajectoryInterpolationTest, two_point_trajectory)
{
  command_interface_types_ = {"position", "velocity"};
  state_interface_types_ = {"position", "velocity", "effort"};
  test_config_name_ = "TWO_POINT_SPLINE";
  interpolation_method_ = "splines";

  rclcpp::executors::MultiThreadedExecutor executor;

  std::vector<rclcpp::Parameter> params;
  params.emplace_back("open_loop_control", true);

  SetUpAndActivateTrajectoryController(executor, params);

  input_times_ = {0.0, 1.0};
  input_positions_ = {
    {0.0, 0.0, 0.0},
    {1.0, 1.0, 1.0}
  };
  input_velocities_ = {
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0}
  };
  input_efforts_ = {};  // No specified efforts

  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.joint_names = joint_names_;
  traj_msg. header.stamp = rclcpp::Time(0, 0);

  for (size_t i = 0; i < input_positions_.size(); ++i)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = input_positions_[i];
    point.velocities = input_velocities_[i];
    point.time_from_start = rclcpp::Duration::from_seconds(input_times_[i]);
    traj_msg.points.push_back(point);
  }

  trajectory_publisher_->publish(traj_msg);
  traj_controller_->wait_for_trajectory(executor);

  print_log_header();
  print_input_trajectory();
  run_trajectory_with_logging(executor, 1.2, NUM_SAMPLES);
  print_sampled_trajectory();
  print_log_footer();

  EXPECT_NEAR(1.0, logged_data_.back().command_positions[0], EPS);
  executor.cancel();
}

//=============================================================================
// TEST 2: Three-point trajectory with specified velocities
//=============================================================================
TEST_F(TrajectoryInterpolationTest, three_point_with_velocities)
{
  command_interface_types_ = {"position", "velocity"};
  state_interface_types_ = {"position", "velocity", "effort"};
  test_config_name_ = "THREE_POINT_WITH_VEL";
  interpolation_method_ = "splines";

  rclcpp::executors::MultiThreadedExecutor executor;

  std::vector<rclcpp::Parameter> params;
  params.emplace_back("open_loop_control", true);

  SetUpAndActivateTrajectoryController(executor, params);

  input_times_ = {0.0, 0.5, 1.0};
  input_positions_ = {
    {0.0, 0.0, 0.0},
    {0.8, 0.8, 0.8},
    {1.0, 1.0, 1.0}
  };
  input_velocities_ = {
    {0.0, 0.0, 0.0},
    {1.5, 1.5, 1.5},  // Specified velocity at midpoint
    {0.0, 0.0, 0.0}
  };
  input_efforts_ = {};

  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg. joint_names = joint_names_;
  traj_msg.header.stamp = rclcpp:: Time(0, 0);

  for (size_t i = 0; i < input_positions_.size(); ++i)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = input_positions_[i];
    point.velocities = input_velocities_[i];
    point.time_from_start = rclcpp::Duration::from_seconds(input_times_[i]);
    traj_msg.points.push_back(point);
  }

  trajectory_publisher_->publish(traj_msg);
  traj_controller_->wait_for_trajectory(executor);

  print_log_header();
  print_input_trajectory();
  run_trajectory_with_logging(executor, 1.2, NUM_SAMPLES);
  print_sampled_trajectory();
  print_log_footer();

  executor.cancel();
}

//=============================================================================
// TEST 3: Trajectory with specified efforts at waypoints
//=============================================================================
TEST_F(TrajectoryInterpolationTest, trajectory_with_efforts)
{
  command_interface_types_ = {"position", "velocity", "effort"};
  state_interface_types_ = {"position", "velocity", "effort"};
  test_config_name_ = "WITH_SPECIFIED_EFFORTS";
  interpolation_method_ = "splines";

  rclcpp::executors::MultiThreadedExecutor executor;

  std::vector<rclcpp::Parameter> params;
  params.emplace_back("open_loop_control", true);

  SetUpAndActivateTrajectoryController(executor, params);

  input_times_ = {0.0, 0.5, 1.0, 1.5};
  input_positions_ = {
    {0.0, 0.0, 0.0},
    {0.5, 0.5, 0.5},
    {1.0, 1.0, 1.0},
    {1.0, 1.0, 1.0}  // Hold position
  };
  input_velocities_ = {
    {0.0, 0.0, 0.0},
    {2.0, 2.0, 2.0},
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0}
  };
  // Specified efforts at waypoints (e.g., feedforward torque)
  input_efforts_ = {
    {0.0, 0.0, 0.0},      // Zero effort at start
    {5.0, 5.0, 5.0},      // Positive effort during acceleration
    {-2.0, -2.0, -2.0},   // Negative effort during deceleration
    {0.5, 0.5, 0.5}       // Small holding effort
  };

  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.joint_names = joint_names_;
  traj_msg.header.stamp = rclcpp::Time(0, 0);

  for (size_t i = 0; i < input_positions_.size(); ++i)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = input_positions_[i];
    point.velocities = input_velocities_[i];
    if (i < input_efforts_.size() && !input_efforts_[i].empty())
    {
      point.effort = input_efforts_[i];
    }
    point.time_from_start = rclcpp::Duration::from_seconds(input_times_[i]);
    traj_msg.points.push_back(point);
  }

  trajectory_publisher_->publish(traj_msg);
  traj_controller_->wait_for_trajectory(executor);

  print_log_header();
  print_input_trajectory();
  run_trajectory_with_logging(executor, 1.7, 120);
  print_sampled_trajectory();
  print_log_footer();

  executor.cancel();
}

//=============================================================================
// TEST 4: Multi-segment trajectory with all specified values
//=============================================================================
TEST_F(TrajectoryInterpolationTest, multi_segment_full_spec)
{
  command_interface_types_ = {"position", "velocity", "effort"};
  state_interface_types_ = {"position", "velocity", "effort"};
  test_config_name_ = "MULTI_SEGMENT_FULL";
  interpolation_method_ = "splines";

  rclcpp::executors::MultiThreadedExecutor executor;

  std::vector<rclcpp:: Parameter> params;
  params. emplace_back("open_loop_control", true);

  SetUpAndActivateTrajectoryController(executor, params);

  input_times_ = {0.0, 0.5, 1.0, 1.5, 2.0};
  input_positions_ = {
    {0.0, 0.0, 0.0},
    {1.0, 0.5, 0.25},
    {1.5, 1.5, 0.5},
    {1.0, 1.0, 0.75},
    {2.0, 2.0, 1.0}
  };
  input_velocities_ = {
    {0.0, 0.0, 0.0},
    {2.0, 1.5, 0.5},
    {0.5, 0.0, 0.5},
    {1.0, 1.0, 0.5},
    {0.0, 0.0, 0.0}
  };
  input_efforts_ = {
    {0.0, 0.0, 0.0},
    {10.0, 8.0, 3.0},    // High effort during fast motion
    {2.0, 0.0, 2.0},     // Lower effort at peak
    {5.0, 5.0, 2.5},
    {0.0, 0.0, 0.0}
  };

  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.joint_names = joint_names_;
  traj_msg.header.stamp = rclcpp::Time(0, 0);

  for (size_t i = 0; i < input_positions_.size(); ++i)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = input_positions_[i];
    point.velocities = input_velocities_[i];
    if (i < input_efforts_.size() && !input_efforts_[i].empty())
    {
      point.effort = input_efforts_[i];
    }
    point.time_from_start = rclcpp::Duration:: from_seconds(input_times_[i]);
    traj_msg.points.push_back(point);
  }

  trajectory_publisher_->publish(traj_msg);
  traj_controller_->wait_for_trajectory(executor);

  print_log_header();
  print_input_trajectory();
  run_trajectory_with_logging(executor, 2.2, 150);
  print_sampled_trajectory();
  print_log_footer();

  executor.cancel();
}

//=============================================================================
// TEST 5: Linear interpolation (position only)
//=============================================================================
TEST_F(TrajectoryInterpolationTest, linear_interpolation)
{
  command_interface_types_ = {"position"};
  state_interface_types_ = {"position", "velocity", "effort"};
  test_config_name_ = "LINEAR_INTERPOLATION";
  interpolation_method_ = "linear";

  rclcpp::executors::MultiThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(executor);

  input_times_ = {0.0, 0.5, 1.0};
  input_positions_ = {
    {0.0, 0.0, 0.0},
    {0.8, 0.8, 0.8},
    {1.0, 1.0, 1.0}
  };
  input_velocities_ = {};
  input_efforts_ = {};

  trajectory_msgs::msg:: JointTrajectory traj_msg;
  traj_msg.joint_names = joint_names_;
  traj_msg. header.stamp = rclcpp::Time(0, 0);

  for (size_t i = 0; i < input_positions_.size(); ++i)
  {
    trajectory_msgs:: msg::JointTrajectoryPoint point;
    point.positions = input_positions_[i];
    point.time_from_start = rclcpp::Duration::from_seconds(input_times_[i]);
    traj_msg.points. push_back(point);
  }

  trajectory_publisher_->publish(traj_msg);
  traj_controller_->wait_for_trajectory(executor);

  print_log_header();
  print_input_trajectory();
  run_trajectory_with_logging(executor, 1.2, NUM_SAMPLES);
  print_sampled_trajectory();
  print_log_footer();

  executor.cancel();
}

//=============================================================================
// TEST 6: S-curve motion profile
//=============================================================================
TEST_F(TrajectoryInterpolationTest, s_curve_motion)
{
  command_interface_types_ = {"position", "velocity"};
  state_interface_types_ = {"position", "velocity", "effort"};
  test_config_name_ = "S_CURVE_MOTION";
  interpolation_method_ = "splines";

  rclcpp::executors::MultiThreadedExecutor executor;

  std::vector<rclcpp::Parameter> params;
  params.emplace_back("open_loop_control", true);

  SetUpAndActivateTrajectoryController(executor, params);

  input_times_ = {0.0, 0.5, 1.0};
  input_positions_ = {
    {0.0, 0.0, 0.0},
    {0.5, 0.5, 0.5},
    {1.0, 1.0, 1.0}
  };
  input_velocities_ = {
    {0.0, 0.0, 0.0},
    {2.0, 2.0, 2.0},
    {0.0, 0.0, 0.0}
  };
  input_efforts_ = {};

  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.joint_names = joint_names_;
  traj_msg.header.stamp = rclcpp::Time(0, 0);

  for (size_t i = 0; i < input_positions_.size(); ++i)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = input_positions_[i];
    point. velocities = input_velocities_[i];
    point.time_from_start = rclcpp::Duration::from_seconds(input_times_[i]);
    traj_msg.points. push_back(point);
  }

  trajectory_publisher_->publish(traj_msg);
  traj_controller_->wait_for_trajectory(executor);

  print_log_header();
  print_input_trajectory();
  run_trajectory_with_logging(executor, 1.2, 120);
  print_sampled_trajectory();
  print_log_footer();

  executor.cancel();
}

//=============================================================================
// TEST 7: Return motion with effort profile
//=============================================================================
TEST_F(TrajectoryInterpolationTest, return_motion_with_effort)
{
  command_interface_types_ = {"position", "velocity", "effort"};
  state_interface_types_ = {"position", "velocity", "effort"};
  test_config_name_ = "RETURN_MOTION_EFFORT";
  interpolation_method_ = "splines";

  rclcpp::executors:: MultiThreadedExecutor executor;

  std::vector<rclcpp::Parameter> params;
  params.emplace_back("open_loop_control", true);

  SetUpAndActivateTrajectoryController(executor, params);

  // Go to position and return
  input_times_ = {0.0, 1.0, 2.0};
  input_positions_ = {
    {0.0, 0.0, 0.0},
    {1.0, 1.0, 1.0},
    {0.0, 0.0, 0.0}
  };
  input_velocities_ = {
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0},   // Zero velocity at peak
    {0.0, 0.0, 0.0}
  };
  input_efforts_ = {
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0},   // Zero effort at peak (gravity only)
    {0.0, 0.0, 0.0}
  };

  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.joint_names = joint_names_;
  traj_msg.header.stamp = rclcpp::Time(0, 0);

  for (size_t i = 0; i < input_positions_.size(); ++i)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = input_positions_[i];
    point. velocities = input_velocities_[i];
    if (i < input_efforts_.size() && !input_efforts_[i].empty())
    {
      point.effort = input_efforts_[i];
    }
    point.time_from_start = rclcpp::Duration:: from_seconds(input_times_[i]);
    traj_msg.points.push_back(point);
  }

  trajectory_publisher_->publish(traj_msg);
  traj_controller_->wait_for_trajectory(executor);

  print_log_header();
  print_input_trajectory();
  run_trajectory_with_logging(executor, 2.2, 150);
  print_sampled_trajectory();
  print_log_footer();

  executor.cancel();
}
//=============================================================================
// TEST 8: Position + Effort Command Interface (No Velocity Command)
// Checks if controller handles missing velocity command interface correctly
//=============================================================================
TEST_F(TrajectoryInterpolationTest, position_effort_command_only)
{
  command_interface_types_ = {"position", "effort"};
  state_interface_types_ = {"position", "velocity", "effort"};
  test_config_name_ = "POS_EFFORT_COMMAND_ONLY";
  interpolation_method_ = "splines";

  rclcpp::executors::MultiThreadedExecutor executor;
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("open_loop_control", true);

  SetUpAndActivateTrajectoryController(executor, params);

  input_times_ = {0.0, 1.0, 2.0};
  input_positions_ = {
    {0.0, 0.0, 0.0},
    {1.0, 1.0, 1.0},
    {2.0, 2.0, 2.0}
  };
  input_velocities_ = {
    {0.0, 0.0, 0.0},
    {1.0, 1.0, 1.0},
    {0.0, 0.0, 0.0}
  };
  input_efforts_ = {
    {0.0, 0.0, 0.0},
    {5.0, 5.0, 5.0},
    {0.0, 0.0, 0.0}
  };

  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.joint_names = joint_names_;
  traj_msg.header.stamp = rclcpp::Time(0, 0);

  for (size_t i = 0; i < input_positions_.size(); ++i)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = input_positions_[i];
    point.velocities = input_velocities_[i];
    point.effort = input_efforts_[i];
    point.time_from_start = rclcpp::Duration::from_seconds(input_times_[i]);
    traj_msg.points.push_back(point);
  }

  trajectory_publisher_->publish(traj_msg);
  traj_controller_->wait_for_trajectory(executor);

  print_log_header();
  print_input_trajectory();
  // Symulacja trwa 2.2s. Próbka w połowie to 1.1s.
  run_trajectory_with_logging(executor, 2.2, NUM_SAMPLES);
  print_sampled_trajectory();
  print_log_footer();

  // POPRAWKA: W czasie 1.1s (10% drogi między t=1 a t=2) moment spada z 5.0 do 0.0.
  // Wartość oczekiwana: 5.0 - (5.0 * 0.1) = 4.5
  EXPECT_NEAR(4.5, logged_data_[NUM_SAMPLES/2].command_efforts[0], 0.1); 
  
  executor.cancel();
}

//=============================================================================
// TEST 9: Sparse Input (Position + Effort only, No Velocity in MSG)
// Checks linear interpolation fallback for Position and linear for Effort
//=============================================================================
TEST_F(TrajectoryInterpolationTest, input_pos_effort_no_vel_msg)
{
  command_interface_types_ = {"position", "velocity", "effort"};
  state_interface_types_ = {"position", "velocity", "effort"};
  test_config_name_ = "INPUT_POS_EFF_NO_VEL";
  interpolation_method_ = "splines"; // Powinno spaść do liniowej

  rclcpp::executors::MultiThreadedExecutor executor;
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("open_loop_control", true);

  SetUpAndActivateTrajectoryController(executor, params);

  input_times_ = {0.0, 2.0};
  input_positions_ = {
    {0.0, 0.0, 0.0},
    {2.0, 2.0, 2.0}
  };
  // CELOWO BRAK VELOCITIES
  input_velocities_ = {}; 
  input_efforts_ = {
    {0.0, 0.0, 0.0},
    {10.0, 10.0, 10.0}
  };

  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.joint_names = joint_names_;
  traj_msg.header.stamp = rclcpp::Time(0, 0);

  for (size_t i = 0; i < input_positions_.size(); ++i)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = input_positions_[i];
    point.effort = input_efforts_[i];
    point.time_from_start = rclcpp::Duration::from_seconds(input_times_[i]);
    traj_msg.points.push_back(point);
  }

  trajectory_publisher_->publish(traj_msg);
  traj_controller_->wait_for_trajectory(executor);

  print_log_header();
  print_input_trajectory();
  // Symulacja trwa 2.2s. Próbka w połowie to 1.1s.
  run_trajectory_with_logging(executor, 2.2, NUM_SAMPLES);
  print_sampled_trajectory();
  print_log_footer();

  // POPRAWKA: Interpolacja liniowa od 0 do 10 w czasie 2s.
  // W czasie 1.1s wartość powinna wynosić: 10 * (1.1 / 2.0) = 5.5
  EXPECT_NEAR(5.5, logged_data_[NUM_SAMPLES/2].command_efforts[0], 0.1);

  executor.cancel();
}
