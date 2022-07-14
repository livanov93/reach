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
#include "moveit_reach_plugins/path/cartesian_path_generation.h"

#include "moveit_reach_plugins/utils.h"

#include <algorithm>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <reach_core/utils/general_utils.h>

namespace {

template <typename T>
T clamp(const T& val, const T& low, const T& high) {
  return std::max(low, std::min(val, high));
}

}  // namespace

namespace moveit_reach_plugins {
namespace path {

CartesianPathGeneration::CartesianPathGeneration() : PathBase() {}

bool CartesianPathGeneration::initialize(
    const std::string& name, rclcpp::Node::SharedPtr node,
    const std::shared_ptr<const moveit::core::RobotModel> model) {
  // interface to outer ros params and descriptions
  node_ = node;
  model_ = model;
  name_ = name;

  auto LOGGER = rclcpp::get_logger(
      name_ + ".moveit_reach_plugins.CartesianPathGeneration");

  std::string param_prefix = "path_generation_config." + name_ + ".";
  // get parameters for cartesian path computation
  try {
    double global_tx = 0.0, global_ty = 0.0, global_tz = 0.0;
    double global_rx = 0.0, global_ry = 0.0, global_rz = 0.0;
    if (!node->get_parameter(param_prefix + "global_transformation.tx",
                             global_tx)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "global_transformation.tx").c_str());
      return false;
    }
    if (!node->get_parameter(param_prefix + "global_transformation.ty",
                             global_ty)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "global_transformation.ty").c_str());
      return false;
    }
    if (!node->get_parameter(param_prefix + "global_transformation.tz",
                             global_tz)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "global_transformation.tz").c_str());
      return false;
    }
    if (!node->get_parameter(param_prefix + "global_transformation.rx",
                             global_rx)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "global_transformation.rx").c_str());
      return false;
    }
    if (!node->get_parameter(param_prefix + "global_transformation.ry",
                             global_ry)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "global_transformation.ry").c_str());
      return false;
    }
    if (!node->get_parameter(param_prefix + "global_transformation.rz",
                             global_rz)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "global_transformation.rz").c_str());
      return false;
    }
    // init global transformation
    global_transformation_ =
        Eigen::Translation3d(global_tx, global_ty, global_tz);
    global_transformation_ =
        global_transformation_ *
        Eigen::AngleAxisd(global_rx * M_PI / 180.0, Eigen::Vector3d::UnitX());
    global_transformation_ =
        global_transformation_ *
        Eigen::AngleAxisd(global_ry * M_PI / 180.0, Eigen::Vector3d::UnitY());
    global_transformation_ =
        global_transformation_ *
        Eigen::AngleAxisd(global_rz * M_PI / 180.0, Eigen::Vector3d::UnitZ());

    double tool_tx = 0.0, tool_ty = 0.0, tool_tz = 0.0;
    double tool_rx = 0.0, tool_ry = 0.0, tool_rz = 0.0;
    if (!node->get_parameter(param_prefix + "tool_transformation.tx",
                             tool_tx)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "tool_transformation.tx").c_str());
      return false;
    }
    if (!node->get_parameter(param_prefix + "tool_transformation.ty",
                             tool_ty)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "tool_transformation.ty").c_str());
      return false;
    }
    if (!node->get_parameter(param_prefix + "tool_transformation.tz",
                             tool_tz)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "tool_transformation.tz").c_str());
      return false;
    }
    if (!node->get_parameter(param_prefix + "tool_transformation.rx",
                             tool_rx)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "tool_transformation.rx").c_str());
      return false;
    }
    if (!node->get_parameter(param_prefix + "tool_transformation.ry",
                             tool_ry)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "tool_transformation.ry").c_str());
      return false;
    }
    if (!node->get_parameter(param_prefix + "tool_transformation.rz",
                             tool_rz)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "tool_transformation.rz").c_str());
      return false;
    }

    // init tool transformation
    tool_transformation_ = Eigen::Translation3d(tool_tx, tool_ty, tool_tz);
    tool_transformation_ =
        tool_transformation_ *
        Eigen::AngleAxisd(tool_rx * M_PI / 180.0, Eigen::Vector3d::UnitX());
    tool_transformation_ =
        tool_transformation_ *
        Eigen::AngleAxisd(tool_ry * M_PI / 180.0, Eigen::Vector3d::UnitY());
    tool_transformation_ =
        tool_transformation_ *
        Eigen::AngleAxisd(tool_rz * M_PI / 180.0, Eigen::Vector3d::UnitZ());

    if (!node->get_parameter(param_prefix + "distance_threshold",
                             distance_threshold_)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "distance_threshold").c_str());
      return false;
    }

    if (!node->get_parameter(param_prefix + "max_eef_step", max_eef_step_)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "max_eef_step").c_str());
      return false;
    }
    if (!node->get_parameter(param_prefix + "jump_threshold",
                             jump_threshold_)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "jump_threshold").c_str());
      return false;
    }
    if (!node->get_parameter(param_prefix + "tool_frame", tool_frame_)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "tool_frame").c_str());
      return false;
    }
    if (!node->get_parameter(param_prefix + "max_velocity_scaling_factor",
                             max_velocity_scaling_factor_)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "max_velocity_scaling_factor").c_str());
      return false;
    }

    if (!node->get_parameter(param_prefix + "max_acceleration_scaling_factor",
                             max_acceleration_scaling_factor_)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "max_acceleration_scaling_factor").c_str());
      return false;
    }

    if (!node->get_parameter(param_prefix + "touch_links", touch_links_)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   (param_prefix + "touch_links").c_str());
      return false;
    }

    if (std::find(touch_links_.begin(), touch_links_.end(), "") !=
        touch_links_.end()) {
      touch_links_.clear();
    }

    if (!node->get_parameter("path_generation_config.collision_mesh_package",
                             collision_mesh_package_)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   "path_generation_config.collision_mesh_package");
      return false;
    }

    if (!node->get_parameter("path_generation_config.collision_mesh_frame",
                             collision_mesh_frame_)) {
      RCLCPP_ERROR(LOGGER, "No parameter defined by the name: '%s'",
                   "path_generation_config.collision_mesh_frame");
      return false;
    }

    // make sure it is positive to follow solvers logic
    retrieval_path_length_ = std::abs(double(retrieval_path_length_));
    max_eef_step_ = std::abs(double(max_eef_step_));
    jump_threshold_ = std::abs(double(jump_threshold_));
  } catch (const std::exception& ex) {
    RCLCPP_ERROR_STREAM(LOGGER, ex.what());
    return false;
  }

  // init planning scene
  scene_ = std::make_shared<planning_scene::PlanningScene>(model_);

  // Check that the input collision mesh frame exists
  if (!scene_->knowsFrameTransform(collision_mesh_frame_)) {
    RCLCPP_ERROR_STREAM(LOGGER, "Specified collision mesh frame '"
                                    << collision_mesh_frame_
                                    << "' does not exist");
    return false;
  }

  // Add the collision object to the planning scene
  const std::string object_name = "reach_object";
  moveit_msgs::msg::CollisionObject obj = utils::createCollisionObject(
      collision_mesh_package_, collision_mesh_frame_, object_name);
  if (!scene_->processCollisionObjectMsg(obj)) {
    RCLCPP_ERROR(LOGGER, "Failed to add collision mesh to planning scene");
    return false;
  } else {
    scene_->getAllowedCollisionMatrixNonConst().setEntry(object_name,
                                                         touch_links_, true);
  }

  std::string planning_group;
  if (!node->get_parameter("planning_group", planning_group)) {
    return false;
  }

  jmg_ = model_->getJointModelGroup(planning_group);
  if (!jmg_) {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to get joint model group for '"
                                    << planning_group << "'");
    return false;
  }

  // output message about successful initialization
  RCLCPP_INFO(LOGGER, "Successfully initialized '%s' plugin",
              (name_ + ".CartesianPathGeneration").c_str());
  return true;
}

std::optional<double> CartesianPathGeneration::solvePath(
    const std::map<std::string, double>& start_state,
    std::map<std::string, double>& end_state, double& fraction,
    moveit_msgs::msg::RobotTrajectory& moveit_trajectory) {
  moveit::core::RobotState state(model_);

  const bool end_state_empty = end_state.empty();

  const std::vector<std::string>& joint_names =
      jmg_->getActiveJointModelNames();

  std::vector<double> start_state_subset;
  if (!utils::transcribeInputMap(start_state, joint_names,
                                 start_state_subset)) {
    auto LOGGER = rclcpp::get_logger(
        name_ + ".moveit_reach_plugins.CartesianPathGeneration");
    RCLCPP_ERROR_STREAM(
        LOGGER, __FUNCTION__ << ": failed to transcribe input pose map");
    return {};
  }

  // set start state
  state.setJointGroupPositions(jmg_, start_state_subset);
  state.update();

  Eigen::Isometry3d final_target;

  // if end state is sent empty - generate goal as relative transform from start
  // state
  if (end_state_empty) {
    const Eigen::Isometry3d& target = state.getGlobalLinkTransform(tool_frame_);

    // transform in global and local frame
    final_target = global_transformation_ * target * tool_transformation_;
  } else {
    // if end state is not empty, take it as is and use it as goal
    std::vector<double> end_state_subset;
    if (!utils::transcribeInputMap(end_state, joint_names, end_state_subset)) {
      auto LOGGER = rclcpp::get_logger(
          name_ + ".moveit_reach_plugins.CartesianPathGeneration");
      RCLCPP_ERROR_STREAM(
          LOGGER, __FUNCTION__ << ": failed to transcribe input pose map");
      return {};
    }
    moveit::core::RobotState end_robot_state(model_);
    // set end state
    end_robot_state.setJointGroupPositions(jmg_, end_state_subset);
    end_robot_state.update();
    final_target = end_robot_state.getGlobalLinkTransform(tool_frame_);
  }

  // trajectory for path retrieval
  std::vector<std::shared_ptr<moveit::core::RobotState>> trajectory;

  double f = moveit::core::CartesianInterpolator::computeCartesianPath(
      &state, jmg_, trajectory, state.getLinkModel(tool_frame_), final_target,
      true, moveit::core::MaxEEFStep(max_eef_step_),
      moveit::core::JumpThreshold(jump_threshold_, jump_threshold_),
      std::bind(&CartesianPathGeneration::isIKSolutionValid, this,
                std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3));

  if (f != 0.0) {
    // moveit trajectory
    auto result =
        std::make_shared<robot_trajectory::RobotTrajectory>(model_, jmg_);
    for (const auto& waypoint : trajectory)
      result->addSuffixWayPoint(waypoint, 0.0);
    trajectory_processing::IterativeParabolicTimeParameterization timing;
    timing.computeTimeStamps(*result, max_velocity_scaling_factor_,
                             max_acceleration_scaling_factor_);
    result->getRobotTrajectoryMsg(moveit_trajectory);

    std::vector<double> end_state_vec;
    result->getLastWayPoint().copyJointGroupPositions(jmg_, end_state_vec);
    end_state = [&] {
      std::map<std::string, double> ret;
      for (size_t i = 0; i < joint_names.size(); ++i) {
        ret[joint_names[i]] = end_state_vec[i];
      }
      return ret;
    }();

    // set fraction
    fraction = f;
    return f;

  } else {
    fraction = 0.0;
    return {};
  }
}

std::vector<std::string> CartesianPathGeneration::getJointNames() const {
  return jmg_->getJointModelNames();
}

bool CartesianPathGeneration::isIKSolutionValid(
    moveit::core::RobotState* state, const moveit::core::JointModelGroup* jmg,
    const double* ik_solution) const {
  state->setJointGroupPositions(jmg, ik_solution);
  state->update();

  const bool colliding =
      scene_->isStateColliding(*state, jmg->getName(), false);
  const bool too_close =
      (scene_->distanceToCollision(
           *state, scene_->getAllowedCollisionMatrix()) < distance_threshold_);
  return (!colliding && !too_close);
}

}  // namespace path
}  // namespace moveit_reach_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(moveit_reach_plugins::path::CartesianPathGeneration,
                       reach::plugins::PathBase)
