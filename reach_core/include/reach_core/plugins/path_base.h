/*
 * Copyright 2022 PickNik, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/* Authors: Lovro Ivanov, @livanov93
   Desc:
*/
#ifndef REACH_CORE_PLUGINS_PATH_GENERATOR_BASE_H
#define REACH_CORE_PLUGINS_PATH_GENERATOR_BASE_H

#include <Eigen/Dense>
#include <memory>
#include <optional>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace reach {
namespace plugins {

/**
 * @brief Base class generating PATH at a given reach study location
 */
class PathBase {
 public:
  PathBase() {}

  virtual ~PathBase() {}

  /**
   * @brief initialize
   * @param config
   * @return
   */
  virtual bool initialize(
      const std::string& name, rclcpp::Node::SharedPtr node,
      const std::shared_ptr<const moveit::core::RobotModel> model) = 0;

  /**
   * @brief solveIKFromSeed attempts to find a valid IK solution for the given
   * target pose starting from the input seed state. If a solution is found, the
   * resulting IK solution is saved, and the pose is scored according to the
   * specified cost function plugin
   * @param target pose defining the goal
   * @param start_state robot state in joint space at the start of trajectory
   * @param end_state robot state in joint space at the end of trajectory
   * @param fraction total generated fraction of the desired trajectory
   * [0.0, 1.0]
   * @param moveit_trajectory generated trajectory
   * @return a boost optional type indicating the success of the PATH generation

  virtual std::optional<double> solvePath(
      const Eigen::Isometry3d& target,
      const std::map<std::string, double>& start_state,
      std::map<std::string, double>& end_state, double& fraction,
      moveit_msgs::msg::RobotTrajectory& moveit_trajectory) = 0;
      */

  /**
   * @brief solveIKFromSeed attempts to find a valid IK solution for the given
   * target pose starting from the input seed state. If a solution is found, the
   * resulting IK solution is saved, and the pose is scored according to the
   * specified cost function plugin
   * @param start_state robot state in joint space at the start of trajectory
   * @param goal_state target defined as robot state in joint space - end of
   * trajectory
   * @param fraction total generated fraction of the desired trajectory
   * [0.0, 1.0]
   * @param moveit_trajectory generated trajectory
   * @return a boost optional type indicating the success of the PATH generation
   */
  virtual std::optional<double> solvePath(
      const std::map<std::string, double>& start_state,
      std::map<std::string, double>& end_state, double& fraction,
      moveit_msgs::msg::RobotTrajectory& moveit_trajectory) = 0;

  /**
   * @brief getJointNames
   * @return
   */
  virtual std::vector<std::string> getJointNames() const = 0;

 public:
  rclcpp::Node::SharedPtr node_;
};
typedef std::shared_ptr<PathBase> PathBasePtr;

}  // namespace plugins
}  // namespace reach

#endif  // REACH_CORE_PLUGINS_PATH_GENERATOR_BASE_H
