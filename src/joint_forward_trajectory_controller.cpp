// Copyright (c) 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "joint_forward_trajectory_controller/joint_forward_trajectory_controller.hpp"

#include <chrono>
#include <functional>
#include <memory>

#include <string>
#include <vector>

#include "angles/angles.h"
#include "builtin_interfaces/msg/duration.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joint_forward_trajectory_controller/trajectory.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace joint_forward_trajectory_controller
{
JointTrajectoryController::JointTrajectoryController()
: controller_interface::ControllerInterface(), dof_(0)
{
}

controller_interface::CallbackReturn JointTrajectoryController::on_init()
{
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();

    // Set interpolation method from string parameter
    interpolation_method_ = interpolation_methods::from_string(params_.interpolation_method);
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  // TODO(christophfroehlich): remove deprecation warning
  if (params_.allow_nonzero_velocity_at_trajectory_end)
  {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "[Deprecated]: \"allow_nonzero_velocity_at_trajectory_end\" is set to "
      "true. The default behavior will change to false.");
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
JointTrajectoryController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  if (dof_ == 0)
  {
    fprintf(
      stderr,
      "During ros2_control interface configuration, degrees of freedom is not valid;"
      " it should be positive. Actual DOF is %zu\n",
      dof_);
    std::exit(EXIT_FAILURE);
  }
  conf.names.reserve(dof_ * params_.command_interfaces.size());
  for (const auto & joint_name : params_.joints)
  {
    for (const auto & interface_type : params_.command_interfaces)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }
  return conf;
}

controller_interface::InterfaceConfiguration
JointTrajectoryController::state_interface_configuration() const
{
	controller_interface::InterfaceConfiguration conf;
	conf.type = controller_interface::interface_configuration_type::NONE;
	return conf;
}


controller_interface::return_type JointTrajectoryController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    return controller_interface::return_type::OK;
  }
  auto logger = this->get_node()->get_logger();
  // update dynamic parameters
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
  }

  // don't update goal after we sampled the trajectory to avoid any racecondition
  const auto active_goal = *rt_active_goal_.readFromRT();

  // Check if a new external message has been received from nonRT threads
  auto current_external_msg = traj_external_point_ptr_->get_trajectory_msg();
  auto new_external_msg = traj_msg_external_point_ptr_.readFromRT();
  // Discard, if a goal is pending but still not active (somewhere stuck in goal_handle_timer_)
  if (current_external_msg != *new_external_msg && (rt_has_pending_goal_ && !active_goal) == false)
  {
    fill_partial_goal(*new_external_msg);
    sort_to_local_joint_order(*new_external_msg);
    // TODO(denis): Add here integration of position and velocity
    traj_external_point_ptr_->update(*new_external_msg);
  }

  // TODO(anyone): can I here also use const on joint_interface since the reference_wrapper is not
  // changed, but its value only?
  auto assign_interface_from_point =
    [&](auto & joint_interface, const std::vector<double> & trajectory_point_interface)
  {
    for (size_t index = 0; index < dof_; ++index)
    {
      joint_interface[index].get().set_value(trajectory_point_interface[index]);
    }
  };


  // currently carrying out a trajectory
  if (has_active_trajectory())
  {
    bool first_sample = false;
    // if sampling the first time, set the point before you sample
    if (!traj_external_point_ptr_->is_sampled_already())
    {
      first_sample = true;
    }

    // find segment for current timestamp
    TrajectoryPointConstIter start_segment_itr, end_segment_itr;
    const bool valid_point = traj_external_point_ptr_->sample(
      time, interpolation_method_, state_desired_, start_segment_itr, end_segment_itr);
    state_desired_.time_from_start = 
      time - traj_external_point_ptr_->time_from_start();

    if (valid_point)
    {
      const rclcpp::Time traj_start = traj_external_point_ptr_->time_from_start();
      // this is the time instance
      // - started with the first segment: when the first point will be reached (in the future)
      // - later: when the point of the current segment was reached
      const rclcpp::Time segment_time_from_start = traj_start + start_segment_itr->time_from_start;
      // time_difference is
      // - negative until first point is reached
      // - counting from zero to time_from_start of next point
      double time_difference = time.seconds() - segment_time_from_start.seconds();
      const bool before_last_point = end_segment_itr != traj_external_point_ptr_->end();

      // have we reached the end, are not holding position, and is a timeout configured?
      // Check independently of other tolerances
      if (
        !before_last_point && !rt_is_holding_ && cmd_timeout_ > 0.0 &&
        time_difference > cmd_timeout_)
      {
        RCLCPP_WARN(logger, "Aborted due to command timeout");

        traj_msg_external_point_ptr_.reset();
        traj_msg_external_point_ptr_.initRT(set_hold_position());
      }


	  // set values for next hardware write()
		if (has_position_command_interface_)
		{
		  assign_interface_from_point(joint_command_interface_[0], state_desired_.positions);
		}
		if (has_velocity_command_interface_)
		{
			assign_interface_from_point(joint_command_interface_[1], state_desired_.velocities);
		}
		if (has_effort_command_interface_)
		{
		  assign_interface_from_point(joint_command_interface_[2], state_desired_.effort);
    	}

	// store the previous command. Used in open-loop control mode
	  last_commanded_state_ = state_desired_;

	  
      if (active_goal)
      {
        // send feedback
        auto feedback = std::make_shared<FollowJTrajAction::Feedback>();
        feedback->header.stamp = time;
        feedback->joint_names = params_.joints;

        feedback->actual = state_desired_;
        feedback->desired = state_desired_;
        active_goal->setFeedback(feedback);

        // check abort
		auto result = std::make_shared<FollowJTrajAction::Result>();
		result->set__error_code(FollowJTrajAction::Result::SUCCESSFUL);
		result->set__error_string("Goal successfully reached!");
		active_goal->setSucceeded(result);
		// TODO(matthew-reynolds): Need a lock-free write here
		// See https://github.com/ros-controls/ros2_controllers/issues/168
		rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
		rt_has_pending_goal_ = false;

		RCLCPP_INFO(logger, "Goal reached, success!");

		traj_msg_external_point_ptr_.reset();
		traj_msg_external_point_ptr_.initRT(set_success_trajectory_point());
	  }
    }
  }

  return controller_interface::return_type::OK;
}

void JointTrajectoryController::query_state_service(
  const std::shared_ptr<control_msgs::srv::QueryTrajectoryState::Request> request,
  std::shared_ptr<control_msgs::srv::QueryTrajectoryState::Response> response)
{
  const auto logger = get_node()->get_logger();
  // Preconditions
  if (get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    RCLCPP_ERROR(logger, "Can't sample trajectory. Controller is not active.");
    response->success = false;
    return;
  }
  const auto active_goal = *rt_active_goal_.readFromRT();
  response->name = params_.joints;
  trajectory_msgs::msg::JointTrajectoryPoint state_requested = last_commanded_state_;
  if (has_active_trajectory())
  {
    TrajectoryPointConstIter start_segment_itr, end_segment_itr;
    response->success = traj_external_point_ptr_->sample(
      static_cast<rclcpp::Time>(request->time), interpolation_method_, state_requested,
      start_segment_itr, end_segment_itr);
    // If the requested sample time precedes the trajectory finish time respond as failure
    if (response->success)
    {
      if (end_segment_itr == traj_external_point_ptr_->end())
      {
        RCLCPP_ERROR(logger, "Requested sample time precedes the current trajectory end time.");
        response->success = false;
      }
    }
    else
    {
      RCLCPP_ERROR(
        logger, "Requested sample time is earlier than the current trajectory start time.");
    }
  }
  else
  {
    RCLCPP_ERROR(logger, "Currently there is no valid trajectory instance.");
    response->success = false;
  }
  response->position = state_requested.positions;
  response->velocity = state_requested.velocities;
  // response->effort = state_requested.effort;
}

controller_interface::CallbackReturn JointTrajectoryController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  if (!param_listener_)
  {
    RCLCPP_ERROR(logger, "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  // get degrees of freedom
  dof_ = params_.joints.size();

  // TODO(destogl): why is this here? Add comment or move
  if (!reset())
  {
    return CallbackReturn::FAILURE;
  }

  if (params_.joints.empty())
  {
    // TODO(destogl): is this correct? Can we really move-on if no joint names are not provided?
    RCLCPP_WARN(logger, "'joints' parameter is empty.");
  }

  if (params_.command_interfaces.empty())
  {
    RCLCPP_ERROR(logger, "'command_interfaces' parameter is empty.");
    return CallbackReturn::FAILURE;
  }

  // Check if only allowed interface types are used and initialize storage to avoid memory
  // allocation during activation
  joint_command_interface_.resize(allowed_interface_types_.size());
  for (auto & itf : joint_command_interface_)
  {
    itf.reserve(params_.joints.size());
  }

  has_position_command_interface_ =
    contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_POSITION);
  has_velocity_command_interface_ =
    contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_VELOCITY);
  has_effort_command_interface_ =
    contains_interface_type(params_.command_interfaces, hardware_interface::HW_IF_EFFORT);



  // Validation of combinations of state and velocity together have to be done
  // here because the parameter validators only deal with each parameter
  // separately.

  if (has_velocity_command_interface_ && 
		  !has_position_command_interface_)
  {
    RCLCPP_ERROR(
      logger,
      "'velocity' command interface can only be with 'position' "
      "command interface");
    return CallbackReturn::FAILURE;
  }

  if (has_effort_command_interface_ &&
     !has_position_command_interface_)
  {
    RCLCPP_ERROR(
      logger,
      "'effort' command interface can only be used if "
      "'position' command interface is present");
    return CallbackReturn::FAILURE;
  }


  auto get_interface_list = [](const std::vector<std::string> & interface_types)
  {
    std::stringstream ss_interfaces;
    for (size_t index = 0; index < interface_types.size(); ++index)
    {
      if (index != 0)
      {
        ss_interfaces << " ";
      }
      ss_interfaces << interface_types[index];
    }
    return ss_interfaces.str();
  };

  // Print output so users can be sure the interface setup is correct
  RCLCPP_INFO(
    logger, "Command interfaces are [%s].",
    get_interface_list(params_.command_interfaces).c_str()
	);

  // parse remaining parameters
  const std::string interpolation_string =
    get_node()->get_parameter("interpolation_method").as_string();
  interpolation_method_ = interpolation_methods::from_string(interpolation_string);
  RCLCPP_INFO(
    logger, "Using '%s' interpolation method.",
    interpolation_methods::InterpolationMethodMap.at(interpolation_method_).c_str());

  // prepare hold_position_msg
  init_hold_position_msg();

  // create subscriber
  joint_command_subscriber_ =
    get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "~/joint_forward_trajectory", rclcpp::SystemDefaultsQoS(),
      std::bind(&JointTrajectoryController::topic_callback, this, std::placeholders::_1));

  // action server configuration
  if (params_.allow_partial_joints_goal)
  {
    RCLCPP_INFO(logger, "Goals with partial set of joints are allowed");
  }

  RCLCPP_INFO(
    logger, "Action status changes will be monitored at %.2f Hz.", params_.action_monitor_rate);
  action_monitor_period_ = rclcpp::Duration::from_seconds(1.0 / params_.action_monitor_rate);

  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<FollowJTrajAction>(
    get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
    get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
    std::string(get_node()->get_name()) + "/follow_joint_forward_trajectory",
    std::bind(&JointTrajectoryController::goal_received_callback, this, _1, _2),
    std::bind(&JointTrajectoryController::goal_cancelled_callback, this, _1),
    std::bind(&JointTrajectoryController::goal_accepted_callback, this, _1));

  resize_joint_forward_trajectory_point_command(state_desired_, dof_);
  resize_joint_forward_trajectory_point_command(last_commanded_state_, dof_);

  query_state_srv_ = get_node()->create_service<control_msgs::srv::QueryTrajectoryState>(
    std::string(get_node()->get_name()) + "/query_state",
    std::bind(&JointTrajectoryController::query_state_service, this, _1, _2));

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointTrajectoryController::on_activate(
  const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();


  // order all joints in the storage
  for (const auto & interface : params_.command_interfaces)
  {
    auto it =
      std::find(allowed_interface_types_.begin(), allowed_interface_types_.end(), interface);
    auto index = static_cast<size_t>(std::distance(allowed_interface_types_.begin(), it));
    if (!controller_interface::get_ordered_interfaces(
          command_interfaces_, params_.joints, interface, joint_command_interface_[index]))
    {
      RCLCPP_ERROR(
        logger, "Expected %zu '%s' command interfaces, got %zu.", dof_, interface.c_str(),
        joint_command_interface_[index].size());
      return CallbackReturn::ERROR;
    }
  }

  traj_external_point_ptr_ = std::make_shared<Trajectory>();
  traj_msg_external_point_ptr_.writeFromNonRT(
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory>());

  subscriber_is_active_ = true;

  resize_joint_forward_trajectory_point_command(last_commanded_state_, dof_);

  // The controller should start by holding position at the beginning of active state
  add_new_trajectory_msg(set_hold_position());
  rt_is_holding_ = true;

  // parse timeout parameter
  if (params_.cmd_timeout > 0.0)
  {
	  cmd_timeout_ = params_.cmd_timeout;
  }
  else
  {
    cmd_timeout_ = 0.0;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointTrajectoryController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal)
  {
    rt_has_pending_goal_ = false;
    auto action_res = std::make_shared<FollowJTrajAction::Result>();
    action_res->set__error_code(FollowJTrajAction::Result::INVALID_GOAL);
    action_res->set__error_string("Current goal cancelled during deactivate transition.");
    active_goal->setAborted(action_res);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  }

  for (size_t index = 0; index < dof_; ++index)
  {
    if (has_position_command_interface_)
    {
      joint_command_interface_[0][index].get().set_value(
        joint_command_interface_[0][index].get().get_value());
    }

    if (has_velocity_command_interface_)
    {
      joint_command_interface_[1][index].get().set_value(0.0);
    }

    if (has_effort_command_interface_)
    {
      joint_command_interface_[2][index].get().set_value(0.0);
    }
  }

  for (size_t index = 0; index < allowed_interface_types_.size(); ++index)
  {
    joint_command_interface_[index].clear();
  }
  release_interfaces();

  subscriber_is_active_ = false;

  traj_external_point_ptr_.reset();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointTrajectoryController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointTrajectoryController::on_error(
  const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

bool JointTrajectoryController::reset()
{
  subscriber_is_active_ = false;
  joint_command_subscriber_.reset();

  traj_external_point_ptr_.reset();

  return true;
}

void JointTrajectoryController::topic_callback(
  const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> msg)
{
  if (!validate_trajectory_msg(*msg))
  {
    return;
  }
  // http://wiki.ros.org/joint_forward_trajectory_controller/UnderstandingTrajectoryReplacement
  // always replace old msg with new one for now
  if (subscriber_is_active_)
  {
    add_new_trajectory_msg(msg);
    rt_is_holding_ = false;
  }
};

rclcpp_action::GoalResponse JointTrajectoryController::goal_received_callback(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJTrajAction::Goal> goal)
{
  RCLCPP_INFO(get_node()->get_logger(), "Received new action goal");

  // Precondition: Running controller
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Can't accept new action goals. Controller is not running.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (!validate_trajectory_msg(goal->trajectory))
  {
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_node()->get_logger(), "Accepted new action goal");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse JointTrajectoryController::goal_cancelled_callback(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
{
  RCLCPP_INFO(get_node()->get_logger(), "Got request to cancel goal");

  // Check that cancel request refers to currently active goal (if any)
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal && active_goal->gh_ == goal_handle)
  {
    RCLCPP_INFO(
      get_node()->get_logger(), "Canceling active action goal because cancel callback received.");

    // Mark the current goal as canceled
    rt_has_pending_goal_ = false;
    auto action_res = std::make_shared<FollowJTrajAction::Result>();
    active_goal->setCanceled(action_res);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());

    // Enter hold current position mode
    add_new_trajectory_msg(set_hold_position());
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void JointTrajectoryController::goal_accepted_callback(
  std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJTrajAction>> goal_handle)
{
  // mark a pending goal
  rt_has_pending_goal_ = true;

  // Update new trajectory
  {
    preempt_active_goal();
    auto traj_msg =
      std::make_shared<trajectory_msgs::msg::JointTrajectory>(goal_handle->get_goal()->trajectory);

    add_new_trajectory_msg(traj_msg);
    rt_is_holding_ = false;
  }

  // Update the active goal
  RealtimeGoalHandlePtr rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);
  rt_goal->preallocated_feedback_->joint_names = params_.joints;
  rt_goal->execute();
  rt_active_goal_.writeFromNonRT(rt_goal);

  // Set smartpointer to expire for create_wall_timer to delete previous entry from timer list
  goal_handle_timer_.reset();

  // Setup goal status checking timer
  goal_handle_timer_ = get_node()->create_wall_timer(
    action_monitor_period_.to_chrono<std::chrono::nanoseconds>(),
    std::bind(&RealtimeGoalHandle::runNonRealtime, rt_goal));
}

void JointTrajectoryController::fill_partial_goal(
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg) const
{
  // joint names in the goal are a subset of existing joints, as checked in goal_callback
  // so if the size matches, the goal contains all controller joints
  if (dof_ == trajectory_msg->joint_names.size())
  {
    return;
  }

  trajectory_msg->joint_names.reserve(dof_);

  for (size_t index = 0; index < dof_; ++index)
  {
    {
      if (
        std::find(
          trajectory_msg->joint_names.begin(), trajectory_msg->joint_names.end(),
          params_.joints[index]) != trajectory_msg->joint_names.end())
      {
        // joint found on msg
        continue;
      }
      trajectory_msg->joint_names.push_back(params_.joints[index]);

      for (auto & it : trajectory_msg->points)
      {
        // Assume hold position with 0 velocity and effort for missing joints
        if (!it.positions.empty())
        {
          if (
            has_position_command_interface_ &&
            !std::isnan(joint_command_interface_[0][index].get().get_value()))
          {
            // copy last command if cmd interface exists
            it.positions.push_back(joint_command_interface_[0][index].get().get_value());
          }
        }
        if (!it.velocities.empty())
        {
          it.velocities.push_back(0.0);
        }
        if (!it.effort.empty())
        {
          it.effort.push_back(0.0);
        }
      }
    }
  }
}

void JointTrajectoryController::sort_to_local_joint_order(
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg)
{
  // rearrange all points in the trajectory message based on mapping
  std::vector<size_t> mapping_vector = mapping(trajectory_msg->joint_names, params_.joints);
  auto remap = [this](
                 const std::vector<double> & to_remap,
                 const std::vector<size_t> & mapping) -> std::vector<double>
  {
    if (to_remap.empty())
    {
      return to_remap;
    }
    if (to_remap.size() != mapping.size())
    {
      RCLCPP_WARN(
        get_node()->get_logger(), "Invalid input size (%zu) for sorting", to_remap.size());
      return to_remap;
    }
    static std::vector<double> output(dof_, 0.0);
    // Only resize if necessary since it's an expensive operation
    if (output.size() != mapping.size())
    {
      output.resize(mapping.size(), 0.0);
    }
    for (size_t index = 0; index < mapping.size(); ++index)
    {
      auto map_index = mapping[index];
      output[map_index] = to_remap[index];
    }
    return output;
  };

  for (size_t index = 0; index < trajectory_msg->points.size(); ++index)
  {
    trajectory_msg->points[index].positions =
      remap(trajectory_msg->points[index].positions, mapping_vector);

    trajectory_msg->points[index].velocities =
      remap(trajectory_msg->points[index].velocities, mapping_vector);

    trajectory_msg->points[index].effort =
      remap(trajectory_msg->points[index].effort, mapping_vector);

    trajectory_msg->points[index].effort =
      remap(trajectory_msg->points[index].effort, mapping_vector);
  }
}

bool JointTrajectoryController::validate_trajectory_point_field(
  size_t joint_names_size, const std::vector<double> & vector_field,
  const std::string & string_for_vector_field, size_t i, bool allow_empty) const
{
  if (allow_empty && vector_field.empty())
  {
    return true;
  }
  if (joint_names_size != vector_field.size())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Mismatch between joint_names size (%zu) and %s (%zu) at point #%zu.", joint_names_size,
      string_for_vector_field.c_str(), vector_field.size(), i);
    return false;
  }
  return true;
}

bool JointTrajectoryController::validate_trajectory_msg(
  const trajectory_msgs::msg::JointTrajectory & trajectory) const
{
  // If partial joints goals are not allowed, goal should specify all controller joints
  if (!params_.allow_partial_joints_goal)
  {
    if (trajectory.joint_names.size() != dof_)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Joints on incoming trajectory don't match the controller joints.");
      return false;
    }
  }

  if (trajectory.joint_names.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Empty joint names on incoming trajectory.");
    return false;
  }

  if (trajectory.points.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Empty trajectory received.");
    return false;
  }

  const auto trajectory_start_time = static_cast<rclcpp::Time>(trajectory.header.stamp);
  // If the starting time it set to 0.0, it means the controller should start it now.
  // Otherwise we check if the trajectory ends before the current time,
  // in which case it can be ignored.
  if (trajectory_start_time.seconds() != 0.0)
  {
    auto const trajectory_end_time =
      trajectory_start_time + trajectory.points.back().time_from_start;
    if (trajectory_end_time < get_node()->now())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Received trajectory with non-zero start time (%f) that ends in the past (%f)",
        trajectory_start_time.seconds(), trajectory_end_time.seconds());
      return false;
    }
  }

  for (size_t i = 0; i < trajectory.joint_names.size(); ++i)
  {
    const std::string & incoming_joint_name = trajectory.joint_names[i];

    auto it = std::find(params_.joints.begin(), params_.joints.end(), incoming_joint_name);
    if (it == params_.joints.end())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Incoming joint %s doesn't match the controller's joints.",
        incoming_joint_name.c_str());
      return false;
    }
  }

  if (!params_.allow_nonzero_velocity_at_trajectory_end)
  {
    for (size_t i = 0; i < trajectory.points.back().velocities.size(); ++i)
    {
      if (fabs(trajectory.points.back().velocities.at(i)) > std::numeric_limits<float>::epsilon())
      {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Velocity of last trajectory point of joint %s is not zero: %.15f",
          trajectory.joint_names.at(i).c_str(), trajectory.points.back().velocities.at(i));
        return false;
      }
    }
  }

  rclcpp::Duration previous_traj_time(0ms);
  for (size_t i = 0; i < trajectory.points.size(); ++i)
  {
    if ((i > 0) && (rclcpp::Duration(trajectory.points[i].time_from_start) <= previous_traj_time))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Time between points %zu and %zu is not strictly increasing, it is %f and %f respectively",
        i - 1, i, previous_traj_time.seconds(),
        rclcpp::Duration(trajectory.points[i].time_from_start).seconds());
      return false;
    }
    previous_traj_time = trajectory.points[i].time_from_start;

    const size_t joint_count = trajectory.joint_names.size();
    const auto & points = trajectory.points;
    // This currently supports only position, velocity and effort inputs
    if (params_.allow_integration_in_goal_trajectories)
    {
      if (
        points[i].positions.empty() && points[i].velocities.empty() &&
        points[i].effort.empty())
      {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "The given trajectory has no position, velocity, or effort points.");
        return false;
      }
      const bool position_error =
        !points[i].positions.empty() &&
        !validate_trajectory_point_field(joint_count, points[i].positions, "positions", i, false);
      const bool velocity_error =
        !points[i].velocities.empty() &&
        !validate_trajectory_point_field(joint_count, points[i].velocities, "velocities", i, false);
      const bool effort_error =
        !points[i].effort.empty() &&
        !validate_trajectory_point_field(
          joint_count, points[i].effort, "effort", i, false);
      if (position_error || velocity_error || effort_error)
      {
        return false;
      }
    }
    else if (
      !validate_trajectory_point_field(joint_count, points[i].positions, "positions", i, false) ||
      !validate_trajectory_point_field(joint_count, points[i].velocities, "velocities", i, true) ||
      !validate_trajectory_point_field(
        joint_count, points[i].effort, "effort", i, true))
    {
      return false;
    }
  }
  return true;
}

void JointTrajectoryController::add_new_trajectory_msg(
  const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> & traj_msg)
{
  traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
}

void JointTrajectoryController::preempt_active_goal()
{
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal)
  {
    auto action_res = std::make_shared<FollowJTrajAction::Result>();
    action_res->set__error_code(FollowJTrajAction::Result::INVALID_GOAL);
    action_res->set__error_string("Current goal cancelled due to new incoming action.");
    active_goal->setCanceled(action_res);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  }
}

std::shared_ptr<trajectory_msgs::msg::JointTrajectory>
JointTrajectoryController::set_hold_position()
{
  // Command to stay at current position
  hold_position_msg_ptr_->points[0].positions = last_commanded_state_.positions;

  rt_is_holding_ = true;

  return hold_position_msg_ptr_;
}

std::shared_ptr<trajectory_msgs::msg::JointTrajectory>
JointTrajectoryController::set_success_trajectory_point()
{
  // set last command to be repeated at success, no matter if it has nonzero velocity or
  // effort
  hold_position_msg_ptr_->points[0] = traj_external_point_ptr_->get_trajectory_msg()->points.back();
  hold_position_msg_ptr_->points[0].time_from_start = rclcpp::Duration(0, 0);

  rt_is_holding_ = true;

  return hold_position_msg_ptr_;
}

bool JointTrajectoryController::contains_interface_type(
  const std::vector<std::string> & interface_type_list, const std::string & interface_type)
{
  return std::find(interface_type_list.begin(), interface_type_list.end(), interface_type) !=
         interface_type_list.end();
}

void JointTrajectoryController::resize_joint_forward_trajectory_point_command(
  trajectory_msgs::msg::JointTrajectoryPoint & point, size_t size)
{
  if (has_position_command_interface_)
  {
    point.positions.resize(size, 0.0);
  }
  if (has_velocity_command_interface_)
  {
    point.velocities.resize(size, 0.0);
  }
  if (has_effort_command_interface_)
  {
    point.effort.resize(size, 0.0);
  }
}

bool JointTrajectoryController::has_active_trajectory() const
{
  return traj_external_point_ptr_ != nullptr && traj_external_point_ptr_->has_trajectory_msg();
}

void JointTrajectoryController::init_hold_position_msg()
{
  hold_position_msg_ptr_ = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  hold_position_msg_ptr_->header.stamp =
    rclcpp::Time(0.0, 0.0, get_node()->get_clock()->get_clock_type());  // start immediately
  hold_position_msg_ptr_->joint_names = params_.joints;
  hold_position_msg_ptr_->points.resize(1);  // a trivial msg only
  hold_position_msg_ptr_->points[0].velocities.clear();
  hold_position_msg_ptr_->points[0].effort.clear();
  if (has_velocity_command_interface_)
  {
    // add velocity, so that trajectory sampling returns velocity points in any case
    hold_position_msg_ptr_->points[0].velocities.resize(dof_, 0.0);
  }
}

}  // namespace joint_forward_trajectory_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_forward_trajectory_controller::JointTrajectoryController, controller_interface::ControllerInterface)
