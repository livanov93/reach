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
#ifndef MOVEIT_REACH_PLUGINS_PATH_CARTESIAN_PATH_GENERATION_H
#define MOVEIT_REACH_PLUGINS_PATH_CARTESIAN_PATH_GENERATION_H

#include <pluginlib/class_loader.hpp>
#include <reach_core/plugins/path_base.h>

// PlanningScene
#include <moveit_msgs/msg/planning_scene.hpp>

// cartesian interpolator include
#include "tf2_eigen/tf2_eigen.hpp"

#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

namespace planning_scene {
class PlanningScene;
typedef std::shared_ptr<PlanningScene> PlanningScenePtr;
}  // namespace planning_scene

namespace moveit_reach_plugins {

namespace path {

class CartesianPathGeneration : public reach::plugins::PathBase {
 public:
  CartesianPathGeneration();

  ~CartesianPathGeneration() {}

  virtual bool initialize(
      const std::string& name, rclcpp::Node::SharedPtr node,
      const std::shared_ptr<const moveit::core::RobotModel> model) override;

  virtual std::optional<double> solvePath(
      const std::map<std::string, double>& start_state,
      std::map<std::string, double>& end_state, double& fraction,
      moveit_msgs::msg::RobotTrajectory& moveit_trajectory) override;

  virtual std::vector<std::string> getJointNames() const override;

  bool isIKSolutionValid(moveit::core::RobotState* state,
                         const moveit::core::JointModelGroup* jmg,
                         const double* ik_solution) const;

 protected:
  moveit::core::RobotModelConstPtr model_;
  planning_scene::PlanningScenePtr scene_;
  const moveit::core::JointModelGroup* jmg_;
  std::string name_;

  // distance to retrieve from ik solution in [m]
  double retrieval_path_length_;
  double jump_threshold_;
  double max_eef_step_;
  std::string tool_frame_;
  double max_velocity_scaling_factor_;
  double max_acceleration_scaling_factor_;
  std::string collision_mesh_package_;
  std::string collision_mesh_frame_;
  std::vector<std::string> touch_links_;
  // collision checking
  double distance_threshold_;

  // transformations
  Eigen::Isometry3d global_transformation_;
  Eigen::Isometry3d tool_transformation_;
};

}  // namespace path
}  // namespace moveit_reach_plugins

#endif  // MOVEIT_REACH_PLUGINS_PATH_CARTESIAN_PATH_GENERATION_H
