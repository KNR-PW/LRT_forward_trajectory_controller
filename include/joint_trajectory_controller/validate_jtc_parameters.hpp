// Copyright (c) 2022 ros2_control Development Team
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

#ifndef JOINT_TRAJECTORY_CONTROLLER__VALIDATE_JTC_PARAMETERS_HPP_
#define JOINT_TRAJECTORY_CONTROLLER__VALIDATE_JTC_PARAMETERS_HPP_

#include <string>
#include <vector>

#include "parameter_traits/parameter_traits.hpp"
#include "rclcpp/parameter.hpp"
#include "rsl/algorithm.hpp"
#include "tl_expected/expected.hpp"

namespace joint_trajectory_controller
{
tl::expected<void, std::string> command_interface_type_combinations(
  rclcpp::Parameter const & parameter)
{
  auto const & interface_types = parameter.as_string_array();

  // Check if command interfaces combination is valid. Valid combinations are:
  // 1. position 
  // 2. position velocity
  // 3. position velocity efort
  // 4. position effort 
  if (
    rsl::contains<std::vector<std::string>>(interface_types, "velocity") &&!(
	interface_types.size() == 1 ||
    !rsl::contains<std::vector<std::string>>(interface_types, "position")))
  {
    return tl::make_unexpected(
      "'velocity' command interface can be used only if 'position' "
      "command interface has to be present");
  }

  if (
    rsl::contains<std::vector<std::string>>(interface_types, "effort") &&!(
    interface_types.size() == 1 ||
	!rsl::contains<std::vector<std::string>>(interface_types,"position")))
  {
    return tl::make_unexpected("'effort' command interface can be used only if 'position' command interface is present");
  }

  return {};
}

tl::expected<void, std::string> state_interface_type_combinations(
  rclcpp::Parameter const & parameter)
{
  auto const & interface_types = parameter.as_string_array();


  const bool has_position =
    rsl::contains<std::vector<std::string>>(interface_types, "position");
  const bool has_velocity =
    rsl::contains<std::vector<std::string>>(interface_types, "velocity");
  const bool has_effort =
    rsl::contains<std::vector<std::string>>(interface_types, "effort");
  const bool has_acceleration =
    rsl::contains<std::vector<std::string>>(interface_types, "acceleration");
  if (has_acceleration)
  {
    return tl::make_unexpected(
      "'acceleration' state interface is not allowed. Required: [position, velocity, effort].");
  }

  if (!has_position || !has_velocity || !has_effort)
  {
    return tl::make_unexpected(
      "State interfaces must contain exactly: [position, velocity, effort].");
  }
  if (interface_types.size() != 3)
  {
    return tl::make_unexpected(
      "Invalid number of state interfaces. Required exactly 3: [position, velocity, effort].");
  }
  return {};
}

}  // namespace joint_trajectory_controller

#endif  // JOINT_TRAJECTORY_CONTROLLER__VALIDATE_JTC_PARAMETERS_HPP_
