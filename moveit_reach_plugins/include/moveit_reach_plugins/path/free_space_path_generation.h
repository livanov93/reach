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
#ifndef MOVEIT_REACH_PLUGINS_PATH_FREE_SPACE_PATH_GENERATION_H
#define MOVEIT_REACH_PLUGINS_PATH_FREE_SPACE_PATH_GENERATION_H

#include <pluginlib/class_loader.hpp>
#include <reach_core/plugins/path_base.h>

// PlanningScene
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>

// matrix calculation
#include "tf2_eigen/tf2_eigen.h"

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

namespace planning_scene {
class PlanningScene;
typedef std::shared_ptr<PlanningScene> PlanningScenePtr;
}  // namespace planning_scene

namespace moveit_reach_plugins {

namespace path {

class FreeSpacePathGeneration : public reach::plugins::PathBase {
 public:
  // https://github.com/ros-planning/moveit_task_constructor/blob/60229db010ea305296bc1c90d04faa3e4dacd976/core/include/moveit/task_constructor/solvers/pipeline_planner.h#L58-L64
  struct PlanningSpecs {
    moveit::core::RobotModelConstPtr _robot_model;
    std::string _namespace{"move_group"};
    std::string _planning_pipeline{"ompl"};
    std::string _planning_adapter_param{"request_adapters"};
  };
  FreeSpacePathGeneration();

  ~FreeSpacePathGeneration() {}

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
  // https://github.com/ros-planning/moveit_task_constructor/blob/60229db010ea305296bc1c90d04faa3e4dacd976/core/include/moveit/task_constructor/solvers/pipeline_planner.h#L66-L72
  static planning_pipeline::PlanningPipelinePtr create(
      const rclcpp::Node::SharedPtr& node,
      const moveit::core::RobotModelConstPtr& model) {
    PlanningSpecs spec;
    spec._robot_model = model;
    return create(node, spec);
  }

  static planning_pipeline::PlanningPipelinePtr create(
      const rclcpp::Node::SharedPtr& node, const PlanningSpecs& spec);

  moveit::core::RobotModelConstPtr model_;
  planning_scene::PlanningScenePtr scene_;
  std::vector<std::string> touch_links_;
  const moveit::core::JointModelGroup* jmg_;
  std::string name_;

  // planner stuff
  bool display_motion_plans_;
  bool publish_planning_requests_;
  uint num_planning_attempts_;
  double max_velocity_scaling_factor_;
  double max_acceleration_scaling_factor_;
  moveit_msgs::msg::WorkspaceParameters workspace_parameter_;
  // planner id
  std::string planner_id_;
  double goal_joint_tolerance_;
  double goal_position_tolerance_;
  double goal_orientation_tolerance_;
  double allowed_planning_time_;
  std::string poses_frame_id_;

  std::string pipeline_name_;
  planning_pipeline::PlanningPipelinePtr planner_;

  std::string tool_frame_;

  std::string collision_mesh_package_;
  std::string collision_mesh_frame_;
  // collision checking
  double distance_threshold_;

  // transformations
  Eigen::Isometry3d global_transformation_;
  Eigen::Isometry3d tool_transformation_;

 protected:
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr
      robot_start_state_pub_;
};

}  // namespace path
}  // namespace moveit_reach_plugins

#endif  // MOVEIT_REACH_PLUGINS_PATH_FREE_SPACE_PATH_GENERATION_H
